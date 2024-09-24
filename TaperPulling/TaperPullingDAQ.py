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

import numpy as np

import nidaqmx
from nidaqmx.constants import AcquisitionType, TerminalConfiguration

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
    term_config = "Default"
    scale = 10.0  # V
    mode = AcquisitionType.FINITE
    buffer_size = 10000
    paused = False
    task_running = False
    ok = False
    simulate = False # Simulate DAQ acquisition
    
    def __init__(self):
        """
        This class contains the code to connect to and use our NI DAQ.
        Nothing is done directly at instantiation, except getting system info.
        """
        
        self.local_system = nidaqmx.system.System.local()
        
    def __del__(self):
        self.stop_measuring()
        
    def get_devices(self):
        """
        Gets all NI DAQ devices present in the system.
        """
        self.local_system = nidaqmx.system.System.local()
        for device in self.local_system.devices:
            self.devices.append(device)
            self.devices_names.append(device.name)
            
        return self.devices
        
    def get_ai_channels(self, dev=None):
        """
        Gets all analog in channels from a device (or all).
        """
        channels = []
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
    
    def get_volt_ranges(self, dev):
        """
        Gets the AI voltage ranges for a device.
        """
        ranges = []
        dev_ranges = dev.ai_voltage_rngs
        for i in range(len(dev_ranges)//2):
            ranges.append(np.abs(dev_ranges[i*2]))
            
        return ranges
    
    def get_term_configs(self, channel):
        """
        Gets the terminal configs for a channel.
        """
        configs = []
        chan_configs = channel.ai_term_cfgs
        for conf in chan_configs:
            configs.append(conf)
            
        return configs
        
    def setup_daq(self, srate: float, dev_ch: str, scale: float, term="DEFAULT", 
                  mode="FINITE", buffer_size=10000, simulate=False):
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
        self.term_config = TerminalConfiguration[term]
        self.mode = AcquisitionType[mode]
        self.buffer_size = buffer_size
        self.simulate = simulate
        
        if not self.simulate:
            try:
                if self.task_running:
                    self.task.stop()
            except:
                pass
            try:
                self.task = nidaqmx.Task()
                self.task.ai_channels.add_ai_voltage_chan(dev_ch, min_val=-1*scale, max_val=scale,
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
            return self.task.read()
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
            return np.array(self.task.read(n))
        elif self.simulate and not self.paused:
            avg = 0.0
            offs = 1.0
            span = 2*self.scale
            return offs + avg + np.random.randint(-int(np.abs(span)*1000), int(np.abs(span)*1000), n)/2000000.0
        else:
            return np.zeros(n)
        
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
        
    
        