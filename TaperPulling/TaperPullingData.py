# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingData class. This class is responsible for
acquiring (using TaperPullingDAQ) and processing the data of the fabrication.
Its main initial focus is to obtain the spectrogram for the transmitted optical
power, and assess the single mode-ness of the taper, characterized by the lack
of beating between modes, (clean spectrum, no significant peaks).
All length units in mm.
Wavelength parameter in µm.
"""
# =============================================================================

import os
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import windows
from scipy.fft import fft, fftfreq
from scipy.signal import savgol_filter
from threading import Timer
import cv2

from .TaperPullingDAQ import TaperPullingDAQ


class TaperPullingData:
    """
    This class is responsible for acquiring and processing the 
    fabrication process data.
    """
    
    class Loop(Timer):
        """
        Class that manages the data acquisition loops.
        """
        def run(self):
            while not self.finished.wait(self.interval):
                self.function(*self.args, **self.kwargs)
    
    daq = None
    
    daq_busy = False
    simulate = False
    sampling_rate = 1e3
    
    monitor_loop = None
    monitor_interval = 20  # ms
    monitor_val = 0.0
    
    continuous_loop = None
    continuous_interval = 100  # ms, buffer will be constructed in parts to allow for fast updating
    
    buffer_size = 1000
    buffer_slice = 100
    soft_buffer_ratio = 10
    soft_buffer = np.zeros(soft_buffer_ratio, buffer_slice)
    soft_buffer_i = 0
    buffer = np.zeros(buffer_size)
    buffering = False
    
    impedance = 1e4  # Ohms
    responsivity = 1.0  # A/W
    
    spectrogram_loop = None
    spectrogram_interval = 100  # ms
    spectrum_points = 512
    spectra_points = 1000000000  # A high value to disable this
    max_spectra_points = 2*spectra_points  # Will downsample when this value is reached
    
    spectrogram_running = False
    last_spectrum = np.zeros((2, spectrum_points))
    last_spectrum_data = np.zeros((2, 2*spectrum_points))
    spectra = []
    spectra_freqs = []
    cuton_f = 0.0
    cutoff_f = np.inf
       
    def __init__(self, sampling_rate: float=1e3, cuton_f: float=0.0, cutoff_f: float=1e6):
        """
        This class is responsible for acquiring and processing the 
        fabrication process data.
        
        Args:
            cuton_f: Crop results below this frequency. Default is 0.0 (no crop).
            cutoff_f: Crop results above this frequency. Default is 1e6 (hopefully no crop).
        """
        
        self.daq = TaperPullingDAQ()
        
        self.sampling_rate = sampling_rate
        self.cuton_f = cuton_f
        self.cutoff_f = cutoff_f
        
        self.spectrogram_loop = None
        self.monitor_loop = None
        self.continuous_loop = None
        
    def __del__(self):
        self.close()
        
    def close(self):
        self.stop_cont_loop()
        self.stop_monitor()
        self.stop_spectrogram()
        self.daq.stop_measuring()
        
    def init_daq_as_default(self, simulate=False):
        self.daq_busy = True
        self.set_buffer(self.buffer_size)
        self.daq.get_devices()
        if len(self.daq.devices) > 0:
            self.daq.get_ai_channels()
        if len(self.daq.channels) > 0 or simulate:
            self.daq.setup_daq(self.daq.sampling_rate,
                               self.daq.device_channel,
                               self.daq.scale,
                               self.daq.term_config,
                               self.daq.mode,
                               2*self.buffer_size,
                               simulate)
            self.sampling_rate = self.daq.sampling_rate
            
        if self.daq.mode.name == "CONTINUOUS":
            self.continuous_interval = self.buffer_size/self.daq.sampling_rate
            self.start_cont_loop()
        else:
            self.stop_cont_loop()
            
        self.daq_busy = False
            
    def init_daq(self, srate: float, dev_ch: str, scale: float, term="DEFAULT", 
                  mode="CONTINUOUS", buffer_size=5000, simulate=False):
        self.daq_busy = True
        self.set_buffer(self.buffer_size)
        self.daq.get_devices()
        if len(self.daq.devices) > 0:
            self.daq.get_ai_channels()
        if len(self.daq.channels) > 0 and (dev_ch in self.daq.channels_names):
            self.daq.setup_daq(srate, dev_ch, scale, term, mode, buffer_size, simulate)
            self.sampling_rate = self.daq.sampling_rate
        if mode == "CONTINUOUS":
            self.continuous_interval = self.buffer_size/srate
            self.start_cont_loop()
        else:
            self.stop_cont_loop()
        
        self.daq_busy = False
            
    def set_sampling_rate(self, sampling_rate: float):
        self.sampling_rate = sampling_rate
        self.daq.sampling_rate = sampling_rate
        
        self.clear_spectrogram()
        
        if self.daq.ok:
            self.init_daq_as_default(self.daq.simulate)
            
    def set_buffer(self, n: int=1000):
        self.soft_buffer_ratio = int(np.ceil((n/self.sampling_rate)/(self.continuous_interval/1000.0)))
        if self.soft_buffer_ratio < 1: self.soft_buffer_i = 1
        self.buffer_slice = int(np.ceil(n/self.soft_buffer_ratio))
        n = self.buffer_slice*self.soft_buffer_ratio
        self.buffer_size = n
        self.soft_buffer = np.zeros((self.soft_buffer_ratio, self.buffer_slice))
        self.buffer = np.zeros(n)
            
    def get_single_value(self):
        if not self.daq_busy:
            self.daq_busy = True
            data = self.daq.read_data()[0]
            self.daq_busy = False
            return data
        else:
            return -1
    
    def get_single_power(self, db=False):
        pwr = 1e3*self.get_single_value()/(self.impedance*self.responsivity)
        if db:
            if pwr <= 0: pwr = 1e-99
            return 10*np.log10(pwr)
        else:
            return pwr
        
    def get_monitor_power(self, db=False):
        pwr = 1e3*self.monitor_val/(self.impedance*self.responsivity)
        if db:
            if pwr <= 0: pwr = 1e-99
            return 10*np.log10(pwr)
        else:
            return pwr
    
    def get_points(self, n:int):
        if not self.daq_busy:
            self.daq_busy = True
            data = self.daq.read_data(n)
            self.daq_busy = False
            return data
        else:
            return -1*np.ones(n)       
    
    def perform_fft(self, x_data, y_data, smooth=0.01, window=False, sigma=0.15):
        yf = []
        n = len(y_data)
        y_data = np.array(y_data)
        y_data = y_data - y_data.mean()
        if window:
            if np.abs(sigma) > 1.0: sigma = 1.0
            if np.abs(sigma) < 0.0: sigma = 0.0
            sigma = len(y_data)*sigma
            w = windows.gaussian(n, sigma)
            yf = fft(y_data*w)
        else:
            yf = fft(y_data)

        delta = x_data[1] - x_data[0]
        xf = fftfreq(n, delta)
        
        yf_real = (2.0/n)*np.abs(yf[:n//2])
        smooth_f = int(len(yf_real)*smooth)
        if smooth_f < 3: smooth_f = 3
        
        yf_real = savgol_filter(yf_real, smooth_f, 1)

        return xf[:n//2], yf_real
    
    def get_spectrum(self, smooth=0.01, window=True, sigma=0.15):
        data_n = 2*self.spectrum_points

        if data_n > self.buffer_size: data_n = self.buffer_size
        data_y = self.buffer[-data_n:]
        data_x = np.linspace(0, data_n*self.monitor_interval/1000.0, data_n)
        
        spec_x, spec_y = self.perform_fft(data_x, data_y, smooth, window, sigma)
        
        self.last_spectrum_data[0] = data_x
        self.last_spectrum_data[1] = data_y
        self.last_spectrum[0] = spec_x
        self.last_spectrum[1] = spec_y
        return spec_x, spec_y
    
    def clear_spectrogram(self):
        self.spectra = []
        
    def start_cont_loop(self):
        self.stop_cont_loop()
        self.continuous_loop = self.Loop(self.continuous_interval/1000.0, self.continuous_update)
        self.set_buffer(self.buffer_size)
        self.continuous_loop.start()
        
    def continuous_update(self):
        if not self.daq_busy:
            self.daq_busy = True
            self.buffering = True
            self.buffer = self.daq.read_data(self.buffer_size)

            # Clear remaining DAQ buffer
            self.daq.read_all()
            
            self.buffering = False
            self.daq_busy = False
            
    def stop_cont_loop(self):
        try:
            self.continuous_loop.cancel()
            self.continuous_loop = None
        except:
            pass
            
    def start_monitor(self):
        self.stop_monitor()
        self.monitor_loop = self.Loop(self.monitor_interval/1000.0, self.monitor)
        self.monitor_loop.start()
    
    def monitor(self):
        if not self.daq_busy:
            self.monitor_val = self.get_single_value()
            
    def stop_monitor(self):
        try:
            self.monitor_loop.cancel()
            self.monitor_loop = None
        except:
            pass
            
    def start_spectrogram(self, smooth=0.01, window=True, sigma=0.15):
        self.spectrogram_running = True
        self.clear_spectrogram()
        self.stop_spectrogram()
        self.spectrogram_loop = self.Loop(self.spectrogram_interval/1000.0, self.build_spectrogram, 
                                          args=(smooth, window, sigma))
        self.spectrogram_loop.start()
        
    def build_spectrogram(self, smooth=0.01, window=True, sigma=0.15):
        spec_x, spec_y = self.get_spectrum(smooth, window, sigma)
        cuton_i = np.abs(spec_x - self.cuton_f).argmin()
        cutoff_i = np.abs(spec_x - self.cutoff_f).argmin()
        self.spectra.append(spec_y[cuton_i:cutoff_i])
        self.spectra_freqs = spec_x[cuton_i:cutoff_i]
        
        # Downsample data, creates issues with the x axis (newer data occupies more space).
        # It is better to downsample the entire dataset before plotting, not here
        if len(self.spectra) >= self.max_spectra_points:
            self.spectra = cv2.resize(np.array(self.spectra), (len(self.spectra_freqs), self.spectra_points)).tolist()
    
    def stop_spectrogram(self):
        self.spectrogram_running = False
        try:
            self.spectrogram_loop.cancel()
            self.spectrogram_loop = None
        except:
            pass
        
