# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingSpectrum class. It measures transmission
spectra by sweeping the tunable laser (TaperPullingLaser) at constant speed,
while the detector signal is recorded without gaps (TaperPullingData).

The wavelength of each sample is calculated from the time since the sweep
started. The laser takes some time to start sweeping after it is told to, and
this time is different at every sweep. To find the real start, the laser is
asked for its wavelength continuously: the sweep started when it changes.

If the trigger output of the laser is connected to the auxiliary channel of
the DAQ, it is recorded and saved with the spectrum. It can then be used to
refine the wavelengths afterwards (see TaperPullingLog).

The laser is asked as little as possible while it sweeps: a few times per
second until the sweep is seen to start, then only now and then. Asking
continuously can stall the mainframe.

It does not block for long: call start(), then update() periodically (at least
10 times per second) until it returns False.
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
    # Settings
    start_wl = 1480.0
    stop_wl = 1565.0
    speed = 2.0         # nm/s
    trigger_step = 0.0  # nm. The laser outputs a trigger at every step. 0: only at the sweep start
    monitor_wl = 1550.0 # The laser goes back to this wavelength after the sweep
    settle_time = 1.0   # s. Wait at the first wavelength before sweeping
    pre_time = 0.3      # s. Signal recorded before the sweep command
    post_time = 0.3     # s. Signal recorded after the sweep
    wl_tolerance = 2e-4 # nm. The wavelength changed if it differs more than this
    poll_interval = 0.1       # s. Time between wavelength readings, until the sweep starts
    poll_interval_sweep = 2.0 # s. Time between wavelength readings, after it started
    state_interval = 0.5      # s. Time between "is it still sweeping?" questions, at the end
    start_timeout = 20.0      # s. Give up if the wavelength did not change in this time

    # State
    state = "idle"
    running = False
    progress = 0.0      # 0 to 1
    error = ""
    result = None       # Dictionary with the last spectrum, see get_result()
    metadata = {}
    t_state = 0.0
    t_command = 0.0     # Time (epoch) the sweep command was sent
    t_end = 0.0
    start_delay = None  # Time between the sweep command and the detected sweep start
    wl_before = 0.0     # Wavelength reported by the laser before the sweep
    polls = []          # (time since command, wavelength) of every wavelength reading
    t_poll_next = 0.0   # Time (epoch) of the next question to the laser

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
        self.start_delay = None
        self.polls = []

        try:
            err = self.laser.setup_sweep(self.start_wl, self.stop_wl, self.speed, self.trigger_step)
            if err != "":
                return err
            # Go to the start, so the laser does not need to travel when the sweep begins
            self.laser.set_wavelength(self.start_wl, wait=False)
        except Exception as e:
            return f"Laser communication error: {e}"

        self.state = "positioning"
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
            self.update_sweep()
        except Exception as e:
            self.fail(f"Sweep error: {e}")

        return self.running

    def update_sweep(self):
        now = time.time()
        duration = self.laser.sweep_duration()

        if self.state == "positioning":
            if now - self.t_state >= self.settle_time:
                # The laser must have arrived at the first wavelength before anything else
                if not self.laser.wait_ready():
                    self.fail(f"The laser did not get ready to sweep ({self.laser.last_error}).")
                    return
                self.wl_before = self.laser.get_wavelength()
                self.data.start_recording("sweep")
                self.state = "pre"
                self.t_state = time.time()

        elif self.state == "pre":
            if now - self.t_state >= self.pre_time:
                t_a = time.time()
                self.laser.start_sweep()
                t_b = time.time()
                self.t_command = (t_a + t_b)/2
                self.polls = [(0.0, self.wl_before)]
                self.t_poll_next = t_b + self.poll_interval
                self.state = "sweeping"

        elif self.state == "sweeping":
            elapsed_cmd = now - self.t_command

            if self.start_delay is None:
                # Has it started? Ask where the laser is, a few times per second
                if now >= self.t_poll_next:
                    t_poll, wl = self.poll_wavelength()
                    self.t_poll_next = time.time() + self.poll_interval
                    if wl > self.wl_before + self.wl_tolerance:
                        # First change: the sweep started between the previous reading and
                        # this one. Assume the wavelength just read is exact at this time.
                        self.start_delay = max(t_poll - (wl - self.laser.sweep_start)/self.laser.sweep_speed,
                                               self.polls[-2][0])
                        self.t_poll_next = time.time() + self.poll_interval_sweep
                    elif t_poll > self.start_timeout:
                        self.laser.stop_sweep()
                        self.fail("The laser wavelength did not change after the sweep command.")
            else:
                # Sweeping: the progress comes from the clock, the laser is left alone
                elapsed = elapsed_cmd - self.start_delay
                self.progress = float(np.clip(elapsed/duration, 0.0, 1.0))
                if elapsed < duration:
                    if now >= self.t_poll_next:
                        self.poll_wavelength()
                        self.t_poll_next = time.time() + self.poll_interval_sweep
                elif now >= self.t_poll_next:
                    # It should have finished: ask, now and then, until it says so
                    self.t_poll_next = time.time() + self.state_interval
                    if not self.laser.sweep_running():
                        self.t_end = time.time()
                        self.state = "post"
                        self.t_state = self.t_end
                    elif elapsed > 1.5*duration + 20.0:
                        self.laser.stop_sweep()
                        self.fail("The laser did not finish the sweep in the expected time.")

        elif self.state == "post":
            if now - self.t_state >= self.post_time:
                self.data.stop_recording("sweep")
                self.finish()

    def poll_wavelength(self) -> tuple[float, float]:
        """
        Asks the laser for its wavelength.

        Returns:
            tuple: time since the sweep command (s), wavelength (nm).
        """
        t_a = time.time()
        wl = self.laser.get_wavelength()
        t_b = time.time()
        t_poll = (t_a + t_b)/2 - self.t_command
        self.polls.append((t_poll, wl))
        return t_poll, wl

    def finish(self):
        rec = self.data.get_recording("sweep")
        raw_signal = rec["signal"]
        raw_time = rec["t0"] + np.arange(len(raw_signal))/rec["sampling_rate"] - self.t_command

        # Keep the samples inside the sweep range. The raw data is also kept,
        # so the wavelengths can be recalculated afterwards.
        wl_all = self.laser.sweep_start + self.laser.sweep_speed*(raw_time - self.start_delay)
        inside = (wl_all >= self.laser.sweep_start) & (wl_all <= self.laser.sweep_stop)
        signal = np.asarray(raw_signal, dtype=float)[inside]
        polls = np.array(self.polls, dtype=float).reshape(-1, 2)

        info = {
            "format_version": 2,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "units": {"wavelength": "nm", "signal": "V", "power": "mW", "time": "s"},
            "start_wl": float(self.laser.sweep_start),
            "stop_wl": float(self.laser.sweep_stop),
            "speed": float(self.laser.sweep_speed),
            "trigger_step": float(self.laser.sweep_trigger_step),
            "start_delay": float(self.start_delay),
            "command_time": float(self.t_command),
            "monitor_wl": float(self.monitor_wl),
            "laser": self.laser.idn,
            "laser_power": float(self.laser.power),
            "sampling_rate": float(rec["sampling_rate"]),
            "gain_V_per_mW": float(self.data.gain_v_mw),
            "offset_V": float(self.data.offset_v),
            "metadata": self.metadata,
        }

        self.result = {
            "wavelength": wl_all[inside],
            "signal": signal,
            "power": self.data.get_power_from_voltage(signal),
            "raw_time": raw_time,
            "raw_signal": np.asarray(raw_signal, dtype=np.float32),
            "raw_trigger": np.asarray(rec["aux"], dtype=np.float32),
            "poll_time": polls[:, 0],
            "poll_wavelength": polls[:, 1],
            "info": info,
        }
        self.progress = 1.0
        self.end()

    def fail(self, message: str):
        self.error = message
        self.result = None
        print(f"Sweep failed: {message}")
        self.laser.print_log()
        self.end()

    def abort(self):
        """
        Cancels a running sweep.
        """
        if self.running:
            try:
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
                everything that was recorded, before, during and after the sweep.
            raw_trigger (V): the auxiliary DAQ channel (laser trigger output),
                for the same times. Empty if not used.
            poll_time (s, relative to the sweep command), poll_wavelength (nm):
                the wavelengths reported by the laser during the sweep.
            info: dictionary with the sweep settings and metadata.
        """
        return self.result

    @staticmethod
    def save(result: dict, filename: str):
        """
        Saves a spectrum in a .npz file.
        Use TaperPulling.TaperPullingLog.load_spectrum() to read it.
        """
        arrays = {key: value for key, value in result.items() if key != "info"}
        np.savez_compressed(filename, info_json=np.array(json.dumps(result["info"])), **arrays)
