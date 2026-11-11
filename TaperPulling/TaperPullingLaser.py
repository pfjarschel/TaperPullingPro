# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingLaser class.
It controls a tunable laser module installed in an Agilent/Keysight 816x
mainframe (e.g. 81940A or 81980A in an 8163B), through the network or any
other VISA interface. The laser is used at a fixed wavelength to monitor the
transmission during fabrication, and swept to measure transmission spectra.
It can also be simulated, in which case it provides the transmission of a
fictitious loop resonator, to be used by the simulated DAQ.
Of course, this file is specific to our setup at IFGW. If you are using this
code elsewhere, you may need to change the commands to adapt to your laser.
All wavelengths in nm, power in mW, times in s.
"""
# =============================================================================

import time
import socket
import numpy as np


class TaperPullingLaser:
    """
    This class contains the code to connect to and use a tunable laser
    module in an Agilent/Keysight 816x mainframe.
    """
    # Connection
    address = ""      # VISA resource name, or "IP" / "IP:port" (raw socket, default port 5025)
    slot = 1          # Slot of the laser module in the mainframe
    timeout = 3.0
    conn = None       # Socket or VISA resource
    is_visa = False
    ok = False
    simulate = False
    idn = ""
    last_error = ""

    # State
    wavelength = 1550.0
    power = 1.0
    output = False

    # Allowed wavelength range. The instrument limits are read at connection,
    # but it is safer to stay away from them: set a narrower range here.
    wl_min = 1480.0
    wl_max = 1565.0
    hw_wl_min = 0.0
    hw_wl_max = 1e6

    # Sweep
    sweep_start = 1480.0
    sweep_stop = 1565.0
    sweep_speeds = [0.5, 1.0, 2.0, 5.0, 10.0]  # nm/s, values accepted by the laser
    sweep_speed = 2.0   # nm/s
    sweep_trigger_step = 0.0  # nm. Trigger output at every step. 0: only when the sweep starts
    sweep_t0 = 0.0      # Time (epoch) the sweep was started

    # Simulated loop resonator (transmission seen by the simulated DAQ)
    sim_loop_length = 0.3    # mm
    sim_ng = 1.5             # Group index
    sim_a = 0.85             # Round-trip amplitude
    sim_r = 0.80             # Self-coupling amplitude
    sim_lead_time = 0.3      # Delay between the sweep command and the sweep start (random at each sweep)
    sim_trigger_width = 0.005  # s

    def __init__(self):
        """
        Nothing is done at instantiation. Use connect().
        """
        pass

    def __del__(self):
        self.close()

    # Communication
    def connect(self, address: str="", slot: int=1, simulate: bool=False) -> bool:
        """
        Connects to the mainframe.

        Args:
            address (str): "IP" or "IP:port" for a raw socket connection (default
                port: 5025), or a VISA resource name ("TCPIP0::IP::inst0::INSTR",
                "GPIB0::20::INSTR", ...).
            slot (int): Slot of the laser module in the mainframe.
            simulate (bool, optional): Simulate the laser. Defaults to False.
        """
        self.close()
        self.address = address.strip()
        self.slot = int(slot)
        self.simulate = simulate
        self.last_error = ""

        if simulate:
            self.idn = "Simulated tunable laser"
            self.ok = True
            return True

        try:
            if "::" in self.address:
                import pyvisa
                self.conn = pyvisa.ResourceManager().open_resource(self.address)
                self.conn.timeout = int(1000*self.timeout)
                self.conn.read_termination = "\n"
                self.conn.write_termination = "\n"
                self.is_visa = True
            else:
                host, _, port = self.address.partition(":")
                self.conn = socket.create_connection((host, int(port) if port else 5025), self.timeout)
                self.conn.settimeout(self.timeout)
                self.is_visa = False

            self.ok = True
            self.idn = self.query("*IDN?")
            self.write("*CLS")
            self.write(":LOCK 0,1234")                   # Unlock the laser (default password)
            self.write(f"SOUR{self.slot}:POW:UNIT W")
            self.hw_wl_min = 1e9*float(self.query(f"SOUR{self.slot}:WAV? MIN"))
            self.hw_wl_max = 1e9*float(self.query(f"SOUR{self.slot}:WAV? MAX"))
            self.wavelength = 1e9*float(self.query(f"SOUR{self.slot}:WAV?"))
            self.power = 1e3*float(self.query(f"SOUR{self.slot}:POW?"))
            self.output = int(float(self.query(f"SOUR{self.slot}:POW:STAT?"))) == 1
        except Exception as e:
            self.last_error = str(e)
            print(f"Error connecting to the laser: {e}")
            self.close()

        return self.ok

    def close(self):
        try:
            if self.conn is not None:
                self.conn.close()
        except Exception:
            pass
        self.conn = None
        self.ok = False

    def write(self, cmd: str):
        if self.ok and not self.simulate:
            if self.is_visa:
                self.conn.write(cmd)
            else:
                self.conn.sendall((cmd + "\n").encode())

    def query(self, cmd: str) -> str:
        if self.ok and not self.simulate:
            if self.is_visa:
                return self.conn.query(cmd).strip()
            else:
                self.conn.sendall((cmd + "\n").encode())
                resp = b""
                while not resp.endswith(b"\n"):
                    part = self.conn.recv(4096)
                    if not part:
                        break
                    resp += part
                return resp.decode().strip()
        return ""

    def get_error(self) -> str:
        """
        Reads the oldest error in the instrument's error queue.
        """
        if self.ok and not self.simulate:
            return self.query("SYST:ERR?")
        return '0,"No error"'

    # Basic control
    def get_limits(self) -> tuple[float, float]:
        """
        Returns the wavelength range that can be used: the allowed range, limited
        to what the instrument can do.
        """
        return max(self.wl_min, self.hw_wl_min), min(self.wl_max, self.hw_wl_max)

    def set_limits(self, wl_min: float, wl_max: float):
        self.wl_min = min(wl_min, wl_max)
        self.wl_max = max(wl_min, wl_max)

    def clamp(self, wl: float) -> float:
        lo, hi = self.get_limits()
        return float(np.clip(wl, lo, hi))

    def set_wavelength(self, wl: float, wait: bool=True) -> float:
        """
        Sets the laser wavelength (limited to the allowed range).

        Args:
            wl (float): Wavelength in nm.
            wait (bool): Wait until the laser has finished tuning.

        Returns:
            float: Wavelength set.
        """
        wl = self.clamp(wl)
        if self.ok:
            self.write(f"SOUR{self.slot}:WAV {wl:.4f}NM")
            if wait:
                self.query("*OPC?")
            self.wavelength = wl
        return wl

    def get_wavelength(self) -> float:
        """
        Asks the laser for its current wavelength, in nm. During a sweep, it
        follows the sweep.
        """
        if self.ok and not self.simulate:
            return 1e9*float(self.query(f"SOUR{self.slot}:WAV?"))
        elif self.simulate:
            return float(self.sim_wavelength(time.time()))
        return self.wavelength

    def set_power(self, power: float):
        """
        Sets the output power, in mW.
        """
        if self.ok:
            self.write(f"SOUR{self.slot}:POW {1e-3*power:.6e}")
            self.power = power

    def set_output(self, on: bool):
        """
        Turns the laser output on or off.
        """
        if self.ok:
            self.write(f"SOUR{self.slot}:POW:STAT {1 if on else 0}")
            self.output = on

    # Continuous sweep
    def setup_sweep(self, start: float, stop: float, speed: float, trigger_step: float=0.0) -> str:
        """
        Configures a single continuous sweep.

        Args:
            start, stop (float): Wavelengths in nm, limited to the allowed range.
            speed (float): Sweep speed in nm/s. Only some values are accepted by
                the instrument (see sweep_speeds).
            trigger_step (float): If > 0, the laser outputs a trigger every time
                the wavelength advances by this much (nm). Otherwise, a single
                trigger is output when the sweep starts.

        Returns:
            str: Empty if everything is ok, or a description of the problem.
        """
        self.sweep_start = self.clamp(min(start, stop))
        self.sweep_stop = self.clamp(max(start, stop))
        self.sweep_speed = speed
        self.sweep_trigger_step = max(trigger_step, 0.0)
        if self.sweep_stop <= self.sweep_start:
            return "The sweep range is empty or outside the allowed wavelength range."

        if self.ok and not self.simulate:
            s = f"SOUR{self.slot}:WAV:SWE"
            self.write("*CLS")
            self.write(f"{s}:MODE CONT")
            self.write(f"{s}:STAR {self.sweep_start:.4f}NM")
            self.write(f"{s}:STOP {self.sweep_stop:.4f}NM")
            self.write(f"{s}:SPE {self.sweep_speed:.4f}NM/S")
            self.write(f"{s}:CYCL 1")
            self.write(f"{s}:REP ONEW")
            self.write(f"{s}:LLOG 0")
            # Trigger output, that can be recorded by the DAQ to calibrate the wavelengths
            if self.sweep_trigger_step > 0:
                self.write(f"{s}:STEP {self.sweep_trigger_step:.4f}NM")
                self.write(f"TRIG{self.slot}:OUTP STF")
            else:
                self.write(f"TRIG{self.slot}:OUTP SWST")

            check = self.query(f"{s}:CHEC?")
            if not check.startswith("0"):
                return f"Sweep parameters refused by the laser: {check}"
            err = self.get_error()
            if not err.lstrip("+").startswith("0"):
                return f"Laser error: {err}"
        elif not self.ok:
            return "Laser not connected."

        return ""

    def sweep_duration(self) -> float:
        return (self.sweep_stop - self.sweep_start)/self.sweep_speed

    def start_sweep(self):
        if self.ok:
            self.write(f"SOUR{self.slot}:WAV:SWE STAR")
            if self.simulate:
                self.sim_lead_time = np.random.uniform(0.2, 0.6)
            self.sweep_t0 = time.time()

    def stop_sweep(self):
        if self.ok:
            self.write(f"SOUR{self.slot}:WAV:SWE STOP")
            self.sweep_t0 = 0.0

    def sweep_running(self) -> bool:
        if self.ok and not self.simulate:
            return int(float(self.query(f"SOUR{self.slot}:WAV:SWE?"))) != 0
        elif self.simulate:
            return 0.0 <= (time.time() - self.sweep_t0) < (self.sim_lead_time + self.sweep_duration())
        return False

    # Simulation
    def sim_wavelength(self, t) -> np.ndarray:
        """
        Wavelength of the simulated laser at the times t (epoch, s).
        """
        t = np.asarray(t, dtype=float)
        ts = t - self.sweep_t0 - self.sim_lead_time
        sweeping = (ts >= 0.0) & (ts < self.sweep_duration())
        return np.where(sweeping, self.sweep_start + self.sweep_speed*ts, self.wavelength)

    def sim_trigger(self, t) -> np.ndarray:
        """
        Trigger output (V) of the simulated laser at the times t (epoch, s).
        """
        t = np.asarray(t, dtype=float)
        ts = t - self.sweep_t0 - self.sim_lead_time
        sweeping = (ts >= 0.0) & (ts < self.sweep_duration())
        if self.sweep_trigger_step > 0:
            # Time since the last step
            since = np.mod(ts, self.sweep_trigger_step/self.sweep_speed)
        else:
            since = ts
        return np.where(sweeping & (since < self.sim_trigger_width), 5.0, 0.0)

    def sim_transmission(self, t) -> np.ndarray:
        """
        Power (mW) transmitted by the simulated loop resonator at the times t
        (epoch, s), following the simulated laser wavelength.
        """
        t = np.asarray(t, dtype=float)
        if not self.output:
            return np.zeros(t.shape)
        wl = self.sim_wavelength(t)
        a, r = self.sim_a, self.sim_r
        phi = 2*np.pi*self.sim_ng*(1e6*self.sim_loop_length)/wl
        T = (a**2 - 2*a*r*np.cos(phi) + r**2)/(1 - 2*a*r*np.cos(phi) + (a*r)**2)
        return self.power*T
