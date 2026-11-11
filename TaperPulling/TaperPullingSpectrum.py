# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingSpectrum class. It measures transmission
spectra by sweeping the tunable laser (TaperPullingLaser) while the detector
signal is acquired (TaperPullingData).

Two sweep modes are available:
    "Continuous": the laser sweeps at constant speed while the signal is
        recorded without gaps. Fast. The wavelength of each sample is
        calculated from the time since the sweep started, so it relies on the
        timing between the computer and the laser (see time_offset).
    "Stepped": the laser is set to each wavelength, one by one. Slow (each
        step takes at least one acquisition chunk, about 0.1 s), but the
        wavelength of each point is the one set in the laser.

It does not block: call start(), then update() periodically (at least a few
times per second) until it returns False.
All wavelengths in nm, power in mW, times in s.
"""
# =============================================================================

import json
import time
from datetime import datetime
import numpy as np

from .TaperPullingData import TaperPullingData
from .TaperPullingLaser import TaperPullingLaser


class TaperPullingSpectrum:
    """
    This class measures transmission spectra using the tunable laser and
    the DAQ.
    """
    modes = ["Continuous", "Stepped"]

    # Settings
    mode = "Continuous"
    start_wl = 1480.0
    stop_wl = 1565.0
    speed = 5.0         # nm/s (continuous)
    step = 0.2          # nm (stepped)
    settle_time = 0.2   # s. Stepped: wait after each step. Continuous: wait at the start wavelength
    average_time = 0.05 # s. Stepped: the signal is averaged over this time at each step
    time_offset = 0.0   # s. Continuous: delay between the sweep command and the actual start of the sweep
    monitor_wl = 1550.0 # The laser goes back to this wavelength after the sweep
    pre_time = 0.3      # s. Continuous: signal recorded before the sweep
    post_time = 0.3     # s. Continuous: signal recorded after the sweep

    # State
    state = "idle"
    running = False
    progress = 0.0      # 0 to 1
    error = ""
    result = None       # Dictionary with the last spectrum, see get_result()
    metadata = {}
    t_state = 0.0
    t_command = 0.0
    t_end = 0.0
    step_wls = np.zeros(0)
    step_vals = np.zeros(0)
    step_i = 0
    step_set = False

    def __init__(self, data: TaperPullingData, laser: TaperPullingLaser):
        self.data = data
        self.laser = laser

    def start(self, metadata: dict|None=None) -> str:
        """
        Starts a sweep.

        Args:
            metadata (dict, optional): Anything that should be saved with the
                spectrum (motor positions, notes...). Must be json serializable.

        Returns:
            str: Empty if started, or the reason why it could not start.
        """
        if self.running:
            return "A sweep is already running."
        if not self.laser.ok:
            return "Laser not connected."
        if not self.laser.output:
            return "The laser output is off."
        if not self.data.stream_active():
            return "The DAQ must be in CONTINUOUS mode."

        self.start_wl, self.stop_wl = min(self.start_wl, self.stop_wl), max(self.start_wl, self.stop_wl)
        self.start_wl = self.laser.clamp(self.start_wl)
        self.stop_wl = self.laser.clamp(self.stop_wl)
        if self.stop_wl <= self.start_wl:
            return "The sweep range is empty or outside the allowed wavelength range."

        self.metadata = dict(metadata) if metadata is not None else {}
        self.error = ""
        self.progress = 0.0
        self.result = None

        try:
            if self.mode == "Stepped":
                n = int(np.floor((self.stop_wl - self.start_wl)/self.step + 1e-9)) + 1
                self.step_wls = self.start_wl + self.step*np.arange(n)
                self.step_vals = np.zeros(n)
                self.step_i = 0
                self.step_set = False
                self.state = "stepping"
            else:
                err = self.laser.setup_sweep(self.start_wl, self.stop_wl, self.speed)
                if err != "":
                    return err
                # Go to the start, so the laser does not need to travel when the sweep begins
                self.laser.set_wavelength(self.start_wl, wait=False)
                self.state = "positioning"
        except Exception as e:
            return f"Laser communication error: {e}"

        self.t_state = time.time()
        self.running = True
        return ""

    def update(self) -> bool:
        """
        Advances the sweep. Must be called periodically while it is running.

        Returns:
            bool: True while the sweep is running.
        """
        if not self.running:
            return False

        try:
            if self.mode == "Stepped":
                self.update_stepped()
            else:
                self.update_continuous()
        except Exception as e:
            self.fail(f"Sweep error: {e}")

        return self.running

    def update_continuous(self):
        now = time.time()
        duration = self.laser.sweep_duration()

        if self.state == "positioning":
            if now - self.t_state >= max(self.settle_time, 0.5):
                self.data.start_recording("sweep")
                self.state = "pre"
                self.t_state = now
        elif self.state == "pre":
            if now - self.t_state >= self.pre_time:
                t_a = time.time()
                self.laser.start_sweep()
                t_b = time.time()
                self.t_command = (t_a + t_b)/2
                self.state = "sweeping"
                self.t_state = t_b
        elif self.state == "sweeping":
            elapsed = now - self.t_command
            self.progress = float(np.clip((elapsed - self.time_offset)/duration, 0.0, 1.0))
            if elapsed > 0.5 and not self.laser.sweep_running():
                self.t_end = now
                self.state = "post"
                self.t_state = now
            elif elapsed > 1.5*duration + 20.0:
                self.laser.stop_sweep()
                self.fail("The laser did not finish the sweep in the expected time.")
        elif self.state == "post":
            if now - self.t_state >= self.post_time:
                self.data.stop_recording("sweep")
                rec = self.data.get_recording("sweep")
                raw_time = rec["t0"] + np.arange(len(rec["signal"]))/rec["sampling_rate"] - self.t_command
                self.finish(raw_time=raw_time, raw_signal=rec["signal"], sampling_rate=rec["sampling_rate"])

    def update_stepped(self):
        now = time.time()
        srate = self.data.sampling_rate

        if not self.step_set:
            self.laser.set_wavelength(self.step_wls[self.step_i], wait=True)
            self.step_set = True
            self.t_state = time.time()
        elif self.data.last_chunk_time >= self.t_state + self.settle_time + self.average_time:
            # The buffer now ends with samples taken after the laser settled
            n_avg = max(int(self.average_time*srate), 1)
            self.step_vals[self.step_i] = float(np.mean(self.data.get_points(n_avg)))
            self.step_i += 1
            self.step_set = False
            self.progress = self.step_i/len(self.step_wls)
            if self.step_i >= len(self.step_wls):
                self.finish(wavelength=self.step_wls, signal=self.step_vals, sampling_rate=srate)

    def map_wavelength(self, raw_time: np.ndarray, time_offset: float|None=None) -> np.ndarray:
        """
        Continuous sweeps: wavelength at each time (relative to the sweep command).
        """
        if time_offset is None:
            time_offset = self.time_offset
        return self.laser.sweep_start + self.laser.sweep_speed*(raw_time - time_offset)

    def finish(self, wavelength=None, signal=None, raw_time=None, raw_signal=None, sampling_rate=0.0):
        info = {
            "format_version": 1,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "units": {"wavelength": "nm", "signal": "V", "power": "mW", "time": "s"},
            "mode": self.mode,
            "start_wl": float(self.start_wl),
            "stop_wl": float(self.stop_wl),
            "monitor_wl": float(self.monitor_wl),
            "laser": self.laser.idn,
            "laser_power": float(self.laser.power),
            "sampling_rate": float(sampling_rate),
            "gain_V_per_mW": float(self.data.gain_v_mw),
            "offset_V": float(self.data.offset_v),
            "metadata": self.metadata,
        }

        if raw_time is not None:
            # Continuous: keep the samples inside the sweep range. The raw data is
            # also kept, so the wavelengths can be recalculated with another offset.
            wl_all = self.map_wavelength(raw_time)
            inside = (wl_all >= self.laser.sweep_start) & (wl_all <= self.laser.sweep_stop)
            wavelength = wl_all[inside]
            signal = np.asarray(raw_signal, dtype=float)[inside]
            info.update({"speed": float(self.laser.sweep_speed), "time_offset": float(self.time_offset),
                         "command_time": float(self.t_command),
                         "sweep_end_detected": float(self.t_end - self.t_command)})
        else:
            raw_time = np.zeros(0)
            raw_signal = np.zeros(0, dtype=np.float32)
            info.update({"step": float(self.step), "settle_time": float(self.settle_time),
                         "average_time": float(self.average_time)})

        self.result = {
            "wavelength": np.asarray(wavelength, dtype=float),
            "signal": np.asarray(signal, dtype=float),
            "power": self.data.get_power_from_voltage(np.asarray(signal, dtype=float)),
            "raw_time": np.asarray(raw_time, dtype=float),
            "raw_signal": np.asarray(raw_signal, dtype=np.float32),
            "info": info,
        }
        self.progress = 1.0
        self.end()

    def fail(self, message: str):
        self.error = message
        self.result = None
        self.end()

    def abort(self):
        """
        Cancels a running sweep.
        """
        if self.running:
            try:
                if self.mode != "Stepped":
                    self.laser.stop_sweep()
            except Exception:
                pass
            self.fail("Sweep aborted.")

    def end(self):
        self.data.stop_recording("sweep")
        self.running = False
        self.state = "idle"
        try:
            self.laser.set_wavelength(self.monitor_wl, wait=False)
        except Exception as e:
            print(f"Error sending the laser back to the monitor wavelength: {e}")

    def get_result(self) -> dict|None:
        """
        Returns the last spectrum, or None if there isn't one:
            wavelength (nm), signal (V), power (mW): the spectrum.
            raw_time (s, relative to the sweep command), raw_signal (V):
                continuous sweeps only, everything that was recorded.
            info: dictionary with the sweep settings and metadata.
        """
        return self.result

    @staticmethod
    def save(result: dict, filename: str):
        """
        Saves a spectrum in a .npz file.
        Use TaperPulling.TaperPullingLog.load_spectrum() to read it.
        """
        np.savez_compressed(filename,
                            info_json=np.array(json.dumps(result["info"])),
                            wavelength=result["wavelength"], signal=result["signal"], power=result["power"],
                            raw_time=result["raw_time"], raw_signal=result["raw_signal"])
