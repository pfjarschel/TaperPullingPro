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
from threading import Timer, Thread, Event, Lock

from .TaperPullingDAQ import TaperPullingDAQ
from .TaperPullingDAQ import AcquisitionType as AcqType
from .TaperPullingDAQ import TerminalConfiguration as TermConf

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
    daq_initing = False
    simulate = False
    sampling_rate = 1e3
    
    monitor_loop = None
    monitor_interval = 20  # ms
    monitor_val = 0.0
    
    continuous_loop = None       # Acquisition thread (continuous mode)
    continuous_stop = None       # Event used to stop the acquisition thread
    continuous_interval = 100  # ms, the stream is read in chunks of this duration
    continuous_interval0 = continuous_interval
    
    buffer_size = 1000
    buffer_slice = 100
    soft_buffer_ratio = 10
    soft_buffer = np.zeros((soft_buffer_ratio, buffer_slice))
    buffer = np.zeros(buffer_size)
    
    # Photodetector calibration and acquisition settings
    gain_v_mw = 10.0  # V/mW (Responsivity / Gain)
    offset_v = 0.0  # V (Dark offset voltage)
    max_power_mw = 1.0  # mW (Max expected optical power)
    auto_v_range = True  # Automatically set DAQ scale to optimal range
    
    # Legacy parameters (kept for backward compatibility)
    impedance = 1e4  # Ohms
    responsivity = 1.0  # A/W
    
    spectrogram_loop = None
    spectrogram_busy = False
    spectrogram_interval = 100  # ms
    spectrum_points = 500
    spectra_points = 1000000000  # A high value to disable downsampling
    max_spectra_points = 2*spectra_points  # Will downsample when this value is reached
    
    spectrogram_running = False
    spectrogram_pause = False
    last_spectrum = np.zeros((2, spectrum_points))
    last_spectrum_data = np.zeros((2, 2*spectrum_points))
    spectra = []
    spectra_freqs = []
    spectra_t = []      # Time (epoch, s) of each spectrum
    spectra_x = []      # Value given by x_provider for each spectrum (e.g. total pulled)
    spectra_v = []      # Second value given by x_provider (e.g. pulling velocity)
    x_provider = None   # Optional function returning (x, v) at the time it is called
    
    # Full data recording (continuous mode only). Every sample is kept, no gaps.
    recording = False
    rec_lock = None
    rec_chunks = []
    rec_chunk_end = []   # Index of the sample after the last one of each chunk
    rec_chunk_time = []  # Time (epoch, s) at which each chunk was received
    rec_n = 0
    rec_srate = 1e3
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
        self.continuous_stop = None
        self.rec_lock = Lock()

    def set_calibration(self, gain_v_mw: float = 10.0, offset_v: float = 0.0, max_power_mw: float = 1.0):
        """
        Sets photodetector calibration parameters.
        
        Args:
            gain_v_mw (float): Responsivity / Gain in V/mW.
            offset_v (float): Dark offset voltage in V.
            max_power_mw (float): Max expected power in mW.
        """
        self.gain_v_mw = gain_v_mw if gain_v_mw != 0 else 1e-9
        self.offset_v = offset_v
        self.max_power_mw = max_power_mw

    def get_optimal_voltage_range(self, max_power_mw: float = None, dev=None) -> float:
        """
        Calculates the optimal DAQ voltage range based on calibration and max expected power.
        
        Args:
            max_power_mw (float, optional): Max expected power in mW. Defaults to self.max_power_mw.
            dev: NI DAQ device object (optional).
            
        Returns:
            float: Smallest hardware bipolar voltage scale (e.g. 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0)
                   that covers the expected signal voltage without saturation.
        """
        if max_power_mw is None:
            max_power_mw = self.max_power_mw
            
        v_max = self.offset_v + max_power_mw * self.gain_v_mw
        v_min = self.offset_v
        v_peak = max(abs(v_min), abs(v_max))
        
        ranges = self.daq.get_volt_ranges(dev)
        if not ranges:
            ranges = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
        ranges = sorted(list(set(ranges)))
        
        for r in ranges:
            if r >= v_peak:
                return r
        return ranges[-1]
        
    def __del__(self):
        self.close()
        
    def close(self):
        self.stop_cont_loop()
        self.stop_monitor()
        self.stop_spectrogram()
        self.daq.stop_measuring()
        
    def init_daq_as_default(self, simulate=False):
        self.daq_busy = True
        self.daq_initing = True
        self.stop_cont_loop()
        
        if isinstance(self.daq.term_config, str):
            self.daq.term_config = eval(f"TermConf.{self.daq.term_config}")
        if isinstance(self.daq.mode, str):
            self.daq.mode = eval(f"AcqType.{self.daq.mode}")
        
        self.set_sampling_rate(self.sampling_rate)
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
            
        if self.daq.mode == AcqType.CONTINUOUS:
            self.start_cont_loop()
        else:
            self.stop_cont_loop()
        
        self.daq_initing = False
        self.daq_busy = False
            
    def init_daq(self, srate: float, dev_ch: str, scale: float, term=TermConf.DEFAULT, 
                  mode=AcqType.CONTINUOUS, buffer_size=5000, simulate=False):
        
        if isinstance(term, str):
            term = eval(f"TermConf.{term}")
        if isinstance(mode, str):
            mode = eval(f"AcqType.{mode}")
        
        self.daq_busy = True
        self.daq_initing = True
        self.stop_cont_loop()
        self.set_sampling_rate(srate)
        self.set_buffer(buffer_size)
        self.daq.get_devices()
        if len(self.daq.devices) > 0:
            self.daq.get_ai_channels()
        if len(self.daq.channels) > 0 and (dev_ch in self.daq.channels_names):
            self.daq.setup_daq(srate, dev_ch, scale, term, mode, buffer_size, simulate)
            self.sampling_rate = self.daq.sampling_rate
        if mode == AcqType.CONTINUOUS:
            self.start_cont_loop()
        else:
            self.stop_cont_loop()
        
        self.daq_initing = False
        self.daq_busy = False
            
    def set_sampling_rate(self, sampling_rate: float):
        self.sampling_rate = sampling_rate
        self.daq.sampling_rate = sampling_rate
        
        self.clear_spectrogram()
        
        if self.daq.ok and not self.daq_initing:
            self.init_daq_as_default(self.daq.simulate)
            
    def set_buffer(self, n: int=1000):
        self.continuous_interval = self.continuous_interval0
        self.soft_buffer_ratio = int(np.ceil((n/self.sampling_rate)/(self.continuous_interval/1000.0)))
        if self.soft_buffer_ratio < 1: 
            self.soft_buffer_ratio = 1
            self.continuous_interval = 1000.0*n/self.sampling_rate

        self.buffer_slice = int(np.ceil(n/self.soft_buffer_ratio))
        self.soft_buffer = np.zeros((self.soft_buffer_ratio, self.buffer_slice))
        self.buffer = np.zeros(n)
        self.buffer_size = n
        
        self.clear_spectrogram()
        
        if self.daq.ok and not self.daq_initing:
            self.init_daq_as_default(self.daq.simulate)
            
    def stream_active(self) -> bool:
        """
        True if the continuous acquisition thread is running. In that case, all
        values must be taken from the buffer, never read directly from the DAQ
        (that would steal samples from the stream).
        """
        return self.continuous_loop is not None and self.continuous_loop.is_alive()
    
    def get_single_value(self):
        if self.stream_active():
            return float(self.buffer[-1])
        elif not self.daq_busy:
            self.daq_busy = True
            data = self.daq.read_data()[0]
            self.daq_busy = False
            return data
        else:
            return -1
    
    def get_power_from_voltage(self, volt: float, db: bool = False) -> float:
        """
        Converts voltage from photodetector into optical power (mW or dBm) using calibration.
        
        Args:
            volt (float): Voltage measured in V.
            db (bool, optional): If True, returns power in dBm (10*log10(P_mW)). Defaults to False.
            
        Returns:
            float: Optical power in mW (or dBm if db=True).
        """
        gain = self.gain_v_mw if self.gain_v_mw != 0 else 1e-9
        pwr = (volt - self.offset_v) / gain
        if db:
            if pwr <= 0:
                pwr = 1e-99
            return 10.0 * np.log10(pwr)
        else:
            return pwr

    def get_single_power(self, db=False):
        return self.get_power_from_voltage(self.get_single_value(), db=db)
        
    def get_monitor_power(self, db=False):
        return self.get_power_from_voltage(self.monitor_val, db=db)
    
    def get_points(self, n:int):
        if self.stream_active():
            return np.array(self.buffer[-n:])
        elif not self.daq_busy:
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
        # Optional smoothing (fraction of the spectrum length). 0 disables it.
        if smooth > 0:
            smooth_f = int(len(yf_real)*smooth)
            if smooth_f < 3: smooth_f = 3
            yf_real = savgol_filter(yf_real, smooth_f, 1)

        return xf[:n//2], yf_real
    
    def get_spectrum(self, smooth=0.01, window=True, sigma=0.15):
        data_n = 2*self.spectrum_points

        if data_n > self.buffer_size: data_n = self.buffer_size
        data_y = self.buffer[-data_n:]
        data_x = np.linspace(0, data_n/self.sampling_rate, data_n)
        
        spec_x, spec_y = self.perform_fft(data_x, data_y, smooth, window, sigma)
        
        self.last_spectrum_data[0] = data_x
        self.last_spectrum_data[1] = data_y
        self.last_spectrum[0] = spec_x
        self.last_spectrum[1] = spec_y
        return spec_x, spec_y
    
    def clear_spectrogram(self):
        self.spectra = []
        self.spectra_t = []
        self.spectra_x = []
        self.spectra_v = []
        
    def start_cont_loop(self):
        self.stop_cont_loop()
        self.set_buffer(self.buffer_size)
        self.continuous_stop = Event()
        self.continuous_loop = Thread(target=self.continuous_update, args=(self.continuous_stop,), daemon=True)
        self.continuous_loop.start()
        
    def continuous_update(self, stop_event):
        """
        Acquisition thread. Reads the DAQ stream chunk by chunk, without gaps,
        keeping the last buffer_size samples available in self.buffer (for
        display and live analysis), and storing everything if recording.
        """
        while not stop_event.is_set():
            try:
                chunk = self.daq.read_stream(self.buffer_slice)
            except Exception as e:
                if not stop_event.is_set():
                    print(f"DAQ stream read error: {e}")
                    stop_event.wait(0.2)
                continue
            
            n = len(chunk)
            if n == 0:
                continue
            t_chunk = time.time()
            
            self.buffer = np.concatenate((self.buffer, chunk))[-self.buffer_size:]
            self.monitor_val = float(chunk.mean())
            
            if self.recording:
                with self.rec_lock:
                    self.rec_n += n
                    self.rec_chunks.append(chunk.astype(np.float32))
                    self.rec_chunk_end.append(self.rec_n)
                    self.rec_chunk_time.append(t_chunk)
            
    def stop_cont_loop(self):
        loop = self.continuous_loop
        if loop is not None:
            self.continuous_stop.set()
            if loop.is_alive():
                loop.join(2.0 + 2.0*self.continuous_interval/1000.0)
        self.continuous_loop = None
        
    def start_recording(self):
        """
        Starts keeping every acquired sample (continuous mode only).
        Any previous recording is discarded.
        """
        with self.rec_lock:
            self.rec_chunks = []
            self.rec_chunk_end = []
            self.rec_chunk_time = []
            self.rec_n = 0
            self.rec_srate = self.sampling_rate
            self.recording = True
            
    def stop_recording(self):
        self.recording = False
        
    def get_recording_duration(self) -> float:
        return self.rec_n/self.rec_srate
        
    def get_recording(self) -> dict:
        """
        Returns the recorded data (can be called during or after a recording):
            signal: all samples, in V (float32).
            sampling_rate: in Hz. Sample i was taken at t0 + i/sampling_rate.
            t0: time (epoch, s) of the first sample.
            chunk_end, chunk_time: for each chunk received from the DAQ, the
                index after its last sample and the time it arrived. Can be used 
                to verify the time axis.
        """
        with self.rec_lock:
            chunks = list(self.rec_chunks)
            chunk_end = np.array(self.rec_chunk_end, dtype=np.int64)
            chunk_time = np.array(self.rec_chunk_time, dtype=float)
            
        if len(chunks) > 0:
            signal = np.concatenate(chunks)
            # A chunk can only arrive late, never early. The earliest start time
            # implied by any chunk is thus the best estimate.
            t0 = float(np.min(chunk_time - chunk_end/self.rec_srate))
        else:
            signal = np.zeros(0, dtype=np.float32)
            t0 = 0.0
            
        return {"signal": signal, "sampling_rate": self.rec_srate, "t0": t0,
                "chunk_end": chunk_end, "chunk_time": chunk_time}
            
    def start_monitor(self):
        self.stop_monitor()
        self.monitor_loop = self.Loop(self.monitor_interval/1000.0, self.monitor)
        self.monitor_loop.start()
    
    def monitor(self):
        # In continuous mode, the acquisition thread updates monitor_val (chunk average)
        if not self.stream_active() and not self.daq_busy:
            self.monitor_val = self.get_single_value()
            
    def stop_monitor(self):
        try:
            self.monitor_loop.cancel()
            self.monitor_loop = None
        except:
            pass
            
    def start_spectrogram(self, smooth=0.01, window=True, sigma=0.15):
        self.clear_spectrogram()
        self.stop_spectrogram()
        self.spectrogram_running = True
        self.last_spectrum = np.zeros((2, self.spectrum_points))
        self.last_spectrum_data = np.zeros((2, 2*self.spectrum_points))
        self.spectrogram_loop = self.Loop(self.spectrogram_interval/1000.0, self.build_spectrogram, 
                                          args=(smooth, window, sigma))
        self.spectrogram_loop.start()
        
    def build_spectrogram(self, smooth=0.01, window=True, sigma=0.15):
        if not self.spectrogram_busy and not self.spectrogram_pause:
            self.spectrogram_busy = True
            spec_x, spec_y = self.get_spectrum(smooth, window, sigma)
            cuton_i = np.abs(spec_x - self.cuton_f).argmin()
            cutoff_i = np.abs(spec_x - self.cutoff_f).argmin()
            x, v = 0.0, 0.0
            if self.x_provider is not None:
                try:
                    x, v = self.x_provider()
                except Exception:
                    pass
            self.spectra_freqs = spec_x[cuton_i:cutoff_i]
            self.spectra.append(spec_y[cuton_i:cutoff_i].astype(np.float32))
            self.spectra_t.append(time.time())
            self.spectra_x.append(x)
            self.spectra_v.append(v)
            
            self.spectrogram_busy = False
    
    def stop_spectrogram(self):
        self.spectrogram_running = False
        self.spectrogram_busy = False
        try:
            self.spectrogram_loop.cancel()
            self.spectrogram_loop = None
        except:
            pass
        
