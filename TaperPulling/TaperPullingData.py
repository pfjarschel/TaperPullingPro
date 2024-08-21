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
    monitor_interval = 1  # ms
    monitor_buffer_size = 10240
    monitor_buffer = np.zeros(monitor_buffer_size)
    
    impedance = 1e4  # Ohms
    responsivity = 1.0  # A/W
    
    spectrogram_loop = None
    spectrogram_interval = 100  # ms
    spectrum_points = 1024
    spectra_points = 1000000000  # A high value to disable this
    max_spectra_points = 2*spectra_points  # Will downsample when this value is reached
    
    spectrogram_from_buffer = True
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
        
    def init_daq_as_default(self, simulate=False):
        while self.daq_busy:
            pass
        self.daq_busy = True
        self.daq.get_devices()
        if len(self.daq.devices) > 0:
            self.daq.get_ai_channels()
        if len(self.daq.channels) > 0 or simulate:
            self.daq.setup_daq(self.daq.sampling_rate,
                               self.daq.device_channel,
                               self.daq.min_scale,
                               self.daq.max_scale,
                               self.daq.term_config,
                               self.daq.clock_source,
                               simulate)
            self.sampling_rate = self.daq.sampling_rate
        self.daq_busy = False
            
    def init_daq(self, srate: float, dev_ch: str, min_v: float, max_v: float, term: str = "default",
                 clock: str = "OnboardClock", simulate: bool = False):
        while self.daq_busy:
            pass
        self.daq_busy = True
        self.daq.get_devices()
        if len(self.daq.devices) > 0:
            self.daq.get_ai_channels()
        if len(self.daq.channels) > 0:
            self.daq.setup_daq(srate, dev_ch, min_v, max_v, term, clock, simulate)
            self.sampling_rate = self.daq.sampling_rate
        self.daq_busy = False
            
    def set_sampling_rate(self, sampling_rate: float):
        self.sampling_rate = sampling_rate
        self.daq.sampling_rate = sampling_rate
        
        self.clear_spectrogram()
        
        if self.daq.ok:
            self.init_daq_as_default()
            
    def set_monitor_buffer(self, n: int=10240):
        while self.daq_busy:
            pass
        self.monitor_buffer_size = n
        self.monitor_buffer = np.zeros(n)
            
    def get_single_transmission(self, wait=False):
        if wait:
            while self.daq_busy:
                pass
        if not self.daq_busy:
            self.daq_busy = True
            data = self.daq.read_data()[0]
            self.monitor_buffer[0] = data
            self.monitor_buffer = np.roll(self.monitor_buffer, -1)
            self.daq_busy = False
            return data
        else:
            return -1
        
    def get_last_monitor(self):
        return self.monitor_buffer[-1]
    
    def get_last_power(self, db=False):
        pwr = 1e3*self.get_last_monitor()/(self.impedance*self.responsivity)
        if db:
            if pwr <= 0: pwr = 1e-99
            return 10*np.log10(pwr)
        else:
            return pwr
    
    def get_transmission_points(self, n:int, wait=False):
        if wait:
            while self.daq_busy:
                pass
        if not self.daq_busy:
            self.daq_busy = True
            data = self.daq.read_data(n)
            self.daq_busy = False
            return data
        else:
            return -1*np.ones(n)
        
    def get_buffer_average(self, n):
        if n <= self.monitor_buffer_size:
            return self.monitor_buffer[-n:].mean()
        else:
            return self.monitor_buffer.mean()
        
    def get_buffer_power_average(self, n, db=False):
        pwr = 1e3*self.get_buffer_average(n)/(self.impedance*self.responsivity)
        if db:
            if pwr <= 0: pwr = 1e-99
            return 10*np.log10(pwr)
        else:
            return pwr
    
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
    
    def get_spectrum(self, smooth=0.01, window=True, sigma=0.15, wait = False):
        data_n = 2*self.spectrum_points
        
        if self.spectrogram_from_buffer:
            if data_n > self.monitor_buffer_size: data_n = self.monitor_buffer_size
            data_y = self.monitor_buffer[-data_n:]
            data_x = np.linspace(0, data_n*self.monitor_interval/1000.0, data_n)
        else:
            data_y = self.get_transmission_points(data_n, wait)
            data_x = np.linspace(0, data_n/self.sampling_rate, data_n)
        
        spec_x, spec_y = self.perform_fft(data_x, data_y, smooth, window, sigma)
        
        self.last_spectrum_data[0] = data_x
        self.last_spectrum_data[1] = data_y
        self.last_spectrum[0] = spec_x
        self.last_spectrum[1] = spec_y
        return spec_x, spec_y
    
    def clear_spectrogram(self):
        self.spectra = []
    
    def start_monitor(self):
        self.monitor_loop = None
        self.monitor_loop = self.Loop(self.monitor_interval/1000.0, self.monitor)
        self.set_monitor_buffer(self.monitor_buffer_size)
        self.monitor_loop.start()
    
    def monitor(self):
        if not self.daq_busy:
            self.get_single_transmission()
            
    def stop_monitor(self):
        self.monitor_loop.cancel()
        self.monitor_loop = None
            
    def start_spectrogram(self, from_buffer=True, smooth=0.01, window=True, sigma=0.15, wait=True):
        self.spectrogram_running = True
        self.spectrogram_from_buffer = from_buffer
        self.clear_spectrogram()
        self.spectrogram_loop = None
        self.spectrogram_loop = self.Loop(self.spectrogram_interval/1000.0, self.build_spectrogram, 
                                          args=(smooth, window, sigma, wait))
        self.spectrogram_loop.start()
        
    def build_spectrogram(self, smooth=0.01, window=True, sigma=0.15, wait=True):
        if wait:
            while self.daq_busy:
                pass
        if not self.daq_busy:
            spec_x, spec_y = self.get_spectrum(smooth, window, sigma, wait)
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
        self.spectrogram_loop.cancel()
        self.spectrogram_loop = None
        
