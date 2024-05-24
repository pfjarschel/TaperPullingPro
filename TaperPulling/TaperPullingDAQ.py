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
#TODO: NI DAQ imports

class TaperPullingDAQ:
    """
    This class contains the code to connect to and use our NI DAQ.
    It can scan and select devices and channels for measurement
    """
    # Control
    devices = []
    channels = []
    
    # Measurement
    sampling_rate = 1000.0  # Hz
    device_channel = "Dev 1/ai0"
    term_config = "default"
    clock_source = "OnboardClock"
    min_scale = -10.0  # V
    max_scale = 10.0  # V
    paused = False
    ok = False
    simulate = False # Simulate DAQ acquisition
    
    def __init__(self):
        """
        This class contains the code to connect to and use our NI DAQ.
        Nothing is done directly at instantiation.
        """
        #TODO: code to initialize the class
        print("TaperDAQ loaded")
        
    def get_devices(self):
        """
        Gets all NI DAQ devices present in the system.
        """
        #TODO: code to get devices
        self.devices = ["Dev 1"]
        
    def get_ai_channels(self):
        """
        Gets all analog in channels from all NI DAQ boards present in the system.
        """
        #TODO: code to get devices
        self.channels = ["ai0"]
        
    def setup_daq(self, srate: float, dev_ch: str, min_v: float, max_v: float, term="default", clock="OnboardClock", simulate=False):
        """
        Sets up DAQ device and channel for measurement
        
        Args:
            srate (float): Sampling rate
            dev_ch (str): Device and channel in the form of "Dev #/ai#"
            min_v (float): Minmum voltage scale
            max_v (float): Maximum voltage scale
            term (str, optional): Terminals configuration. Defaults to "default".
            clock (str, optional): Clock source. Defaults to "OnboardClock".
            simulate (bool, optional): Simulate DAQ acquisition. Defaults to False.
        """
        self.sampling_rate = srate
        self.device_channel = dev_ch
        self.min_scale = min_v
        self.max_scale = max_v
        self.term_config = term
        self.clock_source = clock
        self.simulate = simulate
        
        if not self.simulate:
            #TODO: code to setup daq
            self.ok = True
            
    def read_single(self) -> float:
        """
        Read a single value from the DAQ
        Not recommended. Even if you want a single value, use read_data instead, and take the mean.
        
        Returns:
            float: Value read.
        """
        if self.ok and not self.paused:
            # TODO: code to read a single value
            return 0.0
        elif self.simulate and not self.paused:
            return np.random.randint(self.min_scale*1000.0, self.max_scale*1000.0)/1000.0
        else:
            return 0.0
        
    def read_data(self, n:int) -> np.ndarray:
        """
        Read multiple values from the DAQ, at the defined sampling rate. Blocking.
        Args:
            n (int): Number of samples to read.
            
        Returns:
            np.ndarray: Array of values read.
        """
        if self.ok and not self.paused:
            # TODO: code to read multiple values
            return np.zeros(n)
        elif self.simulate and not self.paused:
            return np.random.randint(self.min_scale*1000.0, self.max_scale*1000.0, n)/1000.0
        else:
            return np.zeros(n)
        
    def pause(self):
        """
        Pauses DAQ operation.
        """
        if self.ok and not self.paused:
            #TODO: code to pause DAQ
            pass
        elif self.simulate and not self.paused:
            self.paused = True
    
    def resume(self):
        """
        Resumes DAQ operation.
        """
        if self.ok and self.paused:
            #TODO: code to resume DAQ
            pass
        elif self.simulate and self.paused:
            self.paused = False
        
    
        