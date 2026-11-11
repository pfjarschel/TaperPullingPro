# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingDAQ class.
It is responsible for controlling the NI data acquisition board used in our setup.
It can read the voltage values of a certain channel, which is usually connected
to a photodetector.
Of course, this file is specific to our setup at IFGW. If you are using this
code elsewhere, you may need to change the DAQ code to adapt to your specific
board.
"""
# =============================================================================

import time
import numpy as np

import nidaqmx
from nidaqmx.constants import AcquisitionType, TerminalConfiguration, READ_ALL_AVAILABLE

class TaperPullingDAQ:
    """
    This class contains the code to connect to and use our NI DAQ.
    It can scan and select devices and channels for measurement
    """
    # Control
    local_system = None
    devices = []
    devices_names = []
    channels = []
    channels_names = []
    task = None
    
    # Measurement
    sampling_rate = 1000.0  # Hz
    device_channel = "Dev 1/ai0"
    term_config = TerminalConfiguration.DEFAULT
    scale = 10.0  # V
    mode = AcquisitionType.FINITE
    buffer_size = 10000
    paused = False
    task_running = False
    ok = False
    sys_ok = False
    simulate = False # Simulate DAQ acquisition
    
    # Optional auxiliary channel, acquired together with the main one in the
    # continuous stream (e.g. a trigger signal). Empty: not used.
    aux_channel = ""
    last_aux = None  # Auxiliary channel samples of the last read_stream() call
    sim_aux_function = None  # Same as sim_function, for the auxiliary channel
    
    # Simulated stream. sim_function, if set, receives an array of times (s, since
    # the stream started) and must return the simulated voltages for those times.
    sim_function = None
    sim_t0 = 0.0
    sim_n = 0
    
    def __init__(self):
        """
        This class contains the code to connect to and use our NI DAQ.
        Nothing is done directly at instantiation, except getting system info.
        """
        
        try:
            self.local_system = nidaqmx.system.System.local()
            self.sys_ok = True
        except Exception as e:
            print(e)
        
    def __del__(self):
        self.stop_measuring()
        try:
            self.task.close()
        except:
            pass
        
    def get_devices(self):
        """
        Gets all NI DAQ devices present in the system.
        """
        if self.sys_ok:
            self.local_system = nidaqmx.system.System.local()
            for device in self.local_system.devices:
                self.devices.append(device)
                self.devices_names.append(device.name)
                
            return self.devices
        else:
            return []
        
    def get_ai_channels(self, dev=None):
        """
        Gets all analog in channels from a device (or all).
        """
        if self.sys_ok:
            if dev != None:
                for channel in dev.ai_physical_chans:
                    self.channels.append(channel)
                    self.channels_names.append(channel.name)
            else:
                for dev in self.devices:
                    for channel in dev.ai_physical_chans:
                        self.channels.append(channel)
                        self.channels_names.append(channel.name)
            
        return self.channels
    
    def get_dev_from_name(self, dev_chan: str):
        if self.sys_ok:
            try:
                dev_str = dev_chan.split("/")[0]
                dev_idx = self.devices_names.index(dev_str)
                return self.devices[dev_idx]
            except:
                print(f"Device from {dev_chan} not found")
                return None
    
    def get_volt_ranges(self, dev=None):
        """
        Gets the AI voltage ranges for a device.
        """
        ranges = []
        
        if self.sys_ok and dev is not None:
            try:
                dev_ranges = dev.ai_voltage_rngs
                for i in range(len(dev_ranges)//2):
                    ranges.append(float(np.abs(dev_ranges[i*2])))
            except Exception as e:
                print(f"Error reading voltage ranges: {e}")
        
        if not ranges:
            ranges = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]
            
        return sorted(list(set(ranges)))
    
    def get_term_configs(self, channel):
        """
        Gets the terminal configs for a channel.
        """
        configs = []
        
        if self.sys_ok:
            chan_configs = channel.ai_term_cfgs
            for conf in chan_configs:
                configs.append(conf)
            
        return configs
        
    def setup_daq(self, srate: float, dev_ch: str, scale: float, term=TerminalConfiguration.DEFAULT, 
                  mode=AcquisitionType.CONTINUOUS, buffer_size=10000, simulate=False):
        """
        Sets up DAQ device and channel for measurement
        
        Args:
            srate (float): Sampling rate
            dev_ch (str): Device and channel in the form of "Dev #/ai#"
            scale (float): Voltage scale (from -X to X)
            term (TerminalConfiguration name, optional): Terminals configuration. Defaults to DEFAULT.
            mode (AcquisitionType name, optional): Acquisition mode. Defaults to FINITE.
            buffer_size (int, optional): Device buffer size Can't read more than this in one go.
            simulate (bool, optional): Simulate DAQ acquisition. Defaults to False.
        """
        self.sampling_rate = srate
        self.device_channel = dev_ch
        self.scale = scale
        self.buffer_size = buffer_size
        self.simulate = simulate
        
        if isinstance(term, str):
            term = eval(f"TerminalConfiguration.{term}")
        if isinstance(mode, str):
            mode = eval(f"AcquisitionType.{mode}")
        self.term_config = term
        self.mode = mode
        self.sim_t0 = time.time()
        self.sim_n = 0

        
        if not self.simulate and self.sys_ok:
            try:
                self.task.stop()
                self.task.close()
            except:
                pass
            try:
                self.task = nidaqmx.Task()
                self.task.ai_channels.add_ai_voltage_chan(dev_ch, min_val=-1*scale, max_val=scale,
                                                          terminal_config=self.term_config)
                if self.aux_channel != "" and self.aux_channel != dev_ch:
                    aux_range = max(self.get_volt_ranges(self.get_dev_from_name(self.aux_channel)))
                    self.task.ai_channels.add_ai_voltage_chan(self.aux_channel, min_val=-aux_range, max_val=aux_range,
                                                              terminal_config=self.term_config)
                self.task.timing.cfg_samp_clk_timing(srate, sample_mode=self.mode, 
                                                     samps_per_chan=buffer_size + 1)
                
                self.ok = True
                self.start_measuring()
            except Exception as e:
                print("Error starting DAQ task!")
                print(e)
                self.ok = False
            
    def read_single(self) -> float:
        """
        Read a single value from the DAQ
        Not recommended. Even if you want a single value, use read_data instead, and take the mean.
        
        Returns:
            float: Value read.
        """
        if self.ok and not self.paused:
            return float(np.atleast_1d(self.task.read())[0])
        elif self.simulate and not self.paused:
            avg = 0.0
            offs = 1.0
            span = 2*self.scale
            return offs + avg + np.random.randint(-int(np.abs(span)*1000), int(np.abs(span)*1000))/2000000.0
        else:
            return 0.0
        
    def read_data(self, n:int=1) -> np.ndarray:
        """
        Read multiple values from the DAQ, at the defined sampling rate. Blocking.
        Args:
            n (int): Number of samples to read.
            
        Returns:
            np.ndarray: Array of values read.
        """
        if self.ok and not self.paused:
            return self.main_channel(np.array(self.task.read(n)))
        elif self.simulate and not self.paused:
            avg = 0.0
            offs = 1.0
            span = 2*self.scale
            return offs + avg + np.random.randint(-int(np.abs(span)*1000), int(np.abs(span)*1000), n)/2000000.0
        else:
            return np.zeros(n)
        
    def main_channel(self, data: np.ndarray) -> np.ndarray:
        """
        Returns only the main channel of data read from the task.
        """
        if data.ndim == 2:
            return data[0]
        return np.atleast_1d(data)
    
    def read_stream(self, n: int=100) -> np.ndarray:
        """
        Reads the next samples of a continuous acquisition, without ever
        discarding any. Blocks until at least n samples are available, and also
        returns anything else already waiting in the device buffer, so that
        consecutive calls return a contiguous (gapless) stream.
        If the auxiliary channel is used, its samples are left in last_aux.
        In simulation, samples are generated at the real sampling rate.
        
        Args:
            n (int): Minimum number of samples to read.
            
        Returns:
            np.ndarray: Array of values read. Empty if nothing could be read.
        """
        if self.paused:
            time.sleep(0.05)
            return np.zeros(0)
        
        if self.ok and not self.simulate:
            # With the auxiliary channel, the data has one row per channel
            data = np.array(self.task.read(n), dtype=float)
            try:
                extra = self.task.in_stream.avail_samp_per_chan
            except Exception:
                extra = 0
            if extra > 0:
                more = np.array(self.task.read(extra), dtype=float)
                data = np.concatenate((data, more), axis=-1)
            
            if data.ndim == 2:
                self.last_aux = data[1]
                return data[0]
            else:
                self.last_aux = None
                return np.atleast_1d(data)
        elif self.simulate:
            # Wait until the n-th sample from now "exists"
            t_ready = self.sim_t0 + (self.sim_n + n)/self.sampling_rate
            wait = t_ready - time.time()
            if wait > 0:
                time.sleep(wait)
            
            # Return everything "acquired" so far (bounded, in case of long stalls)
            total = int((time.time() - self.sim_t0)*self.sampling_rate)
            count = int(np.clip(total - self.sim_n, n, max(n, 10*self.buffer_size)))
            t = (self.sim_n + np.arange(count))/self.sampling_rate
            self.sim_n += count
            
            if self.aux_channel != "" and self.sim_aux_function is not None:
                self.last_aux = np.asarray(self.sim_aux_function(t), dtype=float)
            else:
                self.last_aux = None
            
            if self.sim_function is not None:
                return np.asarray(self.sim_function(t), dtype=float)
            else:
                return 1.0 + 1e-3*self.scale*np.random.uniform(-1.0, 1.0, count)
        else:
            time.sleep(0.05)
            return np.zeros(0)
        
    def read_all(self) -> np.ndarray:
        """
        Read all values from the DAQ buffer.
            
        Returns:
            np.ndarray: Array of values read.
        """
        if self.ok and not self.paused:
            return self.main_channel(np.array(self.task.read(number_of_samples_per_channel=READ_ALL_AVAILABLE)))
        elif self.simulate and not self.paused:
            avg = 0.0
            offs = 1.0
            span = 2*self.scale
            return offs + avg + np.random.randint(-int(np.abs(span)*1000), int(np.abs(span)*1000), self.buffer_size)/2000000.0
        else:
            return np.zeros(self.buffer_size)
        
    def start_measuring(self):
        """
        Starts DAQ operation, if mode is continuous.
        """
        if self.ok:
            if self.mode == AcquisitionType.CONTINUOUS:
                self.task_running = True
                self.task.start()
                
    def stop_measuring(self):
        """
        Starts DAQ operation, if mode is continuous.
        """
        try:
            if self.task_running:
                self.task.stop()
        except:
            pass
    
    def pause(self):
        """
        Pauses DAQ operation.
        """
        if self.ok and not self.paused:
            self.task.stop()
            self.task_running = False
            self.paused = True
        elif self.simulate and not self.paused:
            self.paused = True
    
    def resume(self):
        """
        Resumes DAQ operation.
        """
        if self.ok and self.paused:
            if self.mode == AcquisitionType.CONTINUOUS:
                self.task.start()
                self.task_running = True
            self.paused = False
        elif self.simulate and self.paused:
            self.paused = False
        
    
        