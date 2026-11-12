# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module reads the files saved by the taper pulling program, so that
analysis code does not need to know how they are organized:
    - the full process log ("*_full_log.npz") saved after each fabrication;
    - the transmission spectra ("spectrum_*.npz") measured with the tunable laser.

It only depends on numpy, and can be copied and used on its own:

    from TaperPullingLog import load_pull_log

    log = load_pull_log("auto_15_04_32_full_log.npz")
    plt.plot(log.total_pulled, log.power)     # Transmission vs. pulled distance
    freqs, x, spec = log.spectrogram(1000)    # Recalculate the spectrogram

    spectra = load_spectra("loop_15_20_11")   # All spectra in a folder
    for sp in spectra:
        plt.plot(sp.wavelength, sp.power, label=f"{sp.metadata['slack']:.3f} mm")

All lengths in mm, times in s, velocities in mm/s, power in mW, wavelengths in nm.
"""
# =============================================================================

import os
import glob
import json
import numpy as np


class PullLog:
    """
    Contents of a full process log. Times are relative to the first sample of
    the detector signal (use t0 to convert to absolute time).

    Detector signal (every sample acquired during the process, no gaps):
        time            Time of each sample (s)
        signal          Detector voltage (V)
        power           Optical power (mW), from the detector calibration
        total_pulled    Total pulled distance at each sample (mm)
        stage           Process stage at each sample (see stages)
        sampling_rate   (Hz)

    Motors and process state (one entry per control loop update, ~20 Hz):
        motors          Dictionary of arrays: "time", "stage", "total_pulled",
                        "brusher_pos", "flame_io_pos", "puller_left_pos",
                        "puller_right_pos", "hotzone_target", "pull_vel_left",
                        "pull_vel_right"
        stages          Meaning of the stage numbers

    Spectrogram, as calculated live during the process:
        stft            Amplitudes, one row per spectrum
        stft_freqs      Frequencies (Hz)
        stft_time       Time of each spectrum (s)
        stft_total_pulled, stft_pull_vel

    Others:
        hotzone_design      Designed hotzone size vs. pulled distance
        hotzone_measured    Hotzone size measured at each flame sweep
        events              List of (time, total pulled, note) marked by the user
        parameters          All program settings used in the process
        info                Other information (units, date, ...)
    """

    def __init__(self, filename: str):
        f = np.load(filename, allow_pickle=False)

        self.info = json.loads(str(f["info_json"]))
        self.parameters = self.info.get("parameters", {})
        self.stages = {int(k): v for k, v in self.info.get("stages", {}).items()}

        # Detector signal
        self.sampling_rate = float(f["signal_rate"])
        self.t0 = float(f["signal_t0"])
        self.signal = f["signal"].astype(float)
        self.time = np.arange(len(self.signal))/self.sampling_rate
        self.gain = float(f["signal_gain_V_per_mW"])
        self.offset = float(f["signal_offset_V"])
        self.power = (self.signal - self.offset)/self.gain
        self.chunk_end = f["signal_chunk_end"]
        self.chunk_time = f["signal_chunk_time"] - self.t0

        # Motors and process state
        self.motors = {}
        for key in f.files:
            if key.startswith("log_"):
                self.motors[key[4:]] = f[key]
        self.motors["time"] = self.motors["time"] - self.t0

        # Total pulled and stage for each sample of the signal
        if len(self.motors["time"]) > 0:
            self.total_pulled = np.interp(self.time, self.motors["time"], self.motors["total_pulled"])
            idxs = np.clip(np.searchsorted(self.motors["time"], self.time, side="right") - 1,
                           0, len(self.motors["time"]) - 1)
            self.stage = self.motors["stage"][idxs].astype(int)
        else:
            self.total_pulled = np.zeros(len(self.time))
            self.stage = np.zeros(len(self.time), dtype=int)

        # Live spectrogram
        self.stft = f["stft"]
        self.stft_freqs = f["stft_freqs"]
        self.stft_time = f["stft_time"] - self.t0
        self.stft_total_pulled = f["stft_total_pulled"]
        self.stft_pull_vel = f["stft_pull_vel"]

        # Others
        self.hotzone_design = f["hotzone_design"]
        self.hotzone_measured = f["hotzone_measured"]
        self.events = [(float(t) - self.t0, float(x), str(label)) for t, x, label in
                       zip(f["event_time"], f["event_total_pulled"], f["event_label"])]

    def pulling(self) -> np.ndarray:
        """
        Returns a mask that selects the samples acquired while actually pulling:
            log.power[log.pulling()]
        """
        return self.stage == 3

    def spectrogram(self, window: int=1000, step: int|None=None, sigma: float=0.15):
        """
        Calculates the spectrogram (short-time Fourier transform) of the detector
        signal, using a gaussian window.

        Args:
            window (int): Number of samples in each spectrum.
            step (int): Number of samples between spectra. Defaults to window/4.
            sigma (float): Width of the gaussian window, relative to its length.

        Returns:
            freqs: Frequencies (Hz).
            total_pulled: Total pulled distance at the center of each spectrum (mm).
            spec: Amplitudes (V). spec[i, j] is frequency i, spectrum j.
        """
        if step is None:
            step = max(window//4, 1)

        n = np.arange(window)
        w = np.exp(-0.5*((n - (window - 1)/2)/(sigma*window))**2)

        starts = np.arange(0, len(self.signal) - window + 1, step)
        spec = np.zeros((window//2 + 1, len(starts)))
        for j, i0 in enumerate(starts):
            y = self.signal[i0:i0 + window]
            spec[:, j] = (2.0/w.sum())*np.abs(np.fft.rfft((y - y.mean())*w))

        freqs = np.fft.rfftfreq(window, 1.0/self.sampling_rate)
        total_pulled = self.total_pulled[starts + window//2] if len(starts) > 0 else np.zeros(0)
        return freqs, total_pulled, spec

    def __repr__(self):
        return (f"PullLog: {len(self.signal)} samples at {self.sampling_rate:g} Hz "
                f"({len(self.signal)/self.sampling_rate:.1f} s), "
                f"{self.motors['total_pulled'].max() if len(self.motors['time']) > 0 else 0.0:.3f} mm pulled, "
                f"{len(self.events)} events")


def load_pull_log(filename: str) -> PullLog:
    """
    Reads a full process log file ("*_full_log.npz").
    """
    return PullLog(filename)


def find_trigger_times(raw_time: np.ndarray, raw_trigger: np.ndarray, threshold: float|None=None,
                       min_amplitude: float=1.0) -> np.ndarray:
    """
    Returns the times of the triggers sent by the laser (rising edges of the
    recorded trigger signal) that came after the sweep command (time > 0).

    Args:
        raw_time: Time of every sample (s, relative to the sweep command).
        raw_trigger: Trigger signal, for the same samples (V).
        threshold (float): Voltage above which the trigger is considered high.
            Defaults to half way between its lowest and highest values.
        min_amplitude (float): If the signal changes less than this (V), there
            are no triggers in it (nothing connected), only noise.
    """
    if len(raw_trigger) == 0 or len(raw_trigger) != len(raw_time):
        return np.zeros(0)
    if threshold is None:
        if np.ptp(raw_trigger) < min_amplitude:
            return np.zeros(0)
        threshold = (raw_trigger.min() + raw_trigger.max())/2
    high = raw_trigger > threshold
    rising = np.flatnonzero(high[1:] & ~high[:-1]) + 1
    times = raw_time[rising]
    return times[times > 0]


def wavelengths_from_triggers(raw_time: np.ndarray, trigger_times: np.ndarray, start_wl: float,
                              stop_wl: float, speed: float, trigger_step: float) -> tuple[np.ndarray, int]:
    """
    Calculates the wavelength of every sample from the triggers sent by the laser.
    If the laser sent a trigger at every wavelength step (trigger_step > 0), the
    first trigger is the start wavelength and each following one is one step
    further, with the wavelengths in between interpolated. Otherwise, the first
    trigger is the sweep start, and the wavelength follows the nominal speed.

    Returns:
        tuple: wavelength of every sample (nm), number of triggers used.
    """
    if len(trigger_times) == 0:
        raise ValueError("No triggers found.")

    if trigger_step > 0 and len(trigger_times) > 1:
        # No more triggers than fit in the sweep range
        expected = int(round((stop_wl - start_wl)/trigger_step)) + 1
        t_trig = trigger_times[:max(expected, 2)]
        wl_trig = start_wl + trigger_step*np.arange(len(t_trig))
        wl = np.interp(raw_time, t_trig, wl_trig)
        # Outside the triggers, continue at the nominal speed
        before = raw_time < t_trig[0]
        after = raw_time > t_trig[-1]
        wl[before] = wl_trig[0] + speed*(raw_time[before] - t_trig[0])
        wl[after] = wl_trig[-1] + speed*(raw_time[after] - t_trig[-1])
        return wl, len(t_trig)

    return start_wl + speed*(raw_time - trigger_times[0]), 1


class Spectrum:
    """
    A transmission spectrum measured by sweeping the tunable laser.

        wavelength      (nm)
        signal          Detector voltage (V)
        power           Optical power (mW), from the detector calibration
        info            Sweep settings (range, speed, start delay, laser...)
        metadata        What was saved with the spectrum by the program (motor
                        positions, notes...). Same as info["metadata"]

    Raw data, to recalculate the wavelengths if needed:
        raw_time        Time of every recorded sample, relative to the moment
                        the laser was told to sweep (s)
        raw_signal      Detector voltage of every recorded sample (V)
        raw_trigger     Trigger output of the laser, for the same samples (V).
                        Empty if it was not connected
        poll_time, poll_wavelength
                        Wavelengths reported by the laser during the sweep, and
                        when (s, relative to the sweep command)

    How the wavelengths were calculated is in info["wavelength_source"]:
        "triggers"       from the triggers sent by the laser at every wavelength
                         step (info["trigger_step"]), interpolating in between
        "trigger_start"  from the time since the single trigger sent by the
                         laser when the sweep started
        "polling"        from the time since the sweep started, found from the
                         first change in the wavelength reported by the laser:
                         wavelength = start_wl + speed*(raw_time - start_delay)
    They can be recalculated with remap() and remap_from_triggers().
    """

    def __init__(self, filename: str):
        f = np.load(filename, allow_pickle=False)

        self.filename = filename
        self.info = json.loads(str(f["info_json"]))
        self.metadata = self.info.get("metadata", {})
        self.wavelength = f["wavelength"]
        self.signal = f["signal"]
        self.power = f["power"]
        self.raw_time = f["raw_time"]
        self.raw_signal = f["raw_signal"].astype(float)
        self.raw_trigger = f["raw_trigger"].astype(float)
        self.poll_time = f["poll_time"]
        self.poll_wavelength = f["poll_wavelength"]

    def set_wavelengths(self, raw_wavelength: np.ndarray):
        """
        Replaces the spectrum (wavelength, signal and power) by the raw samples
        that are inside the sweep range, given the wavelength of every raw sample.
        """
        inside = (raw_wavelength >= self.info["start_wl"]) & (raw_wavelength <= self.info["stop_wl"])
        self.wavelength = raw_wavelength[inside]
        self.signal = self.raw_signal[inside]
        self.power = (self.signal - self.info["offset_V"])/self.info["gain_V_per_mW"]

    def remap(self, start_delay: float):
        """
        Recalculates the spectrum from the raw data, assuming a different delay
        (s) between the sweep command and the actual start of the sweep.
        """
        self.set_wavelengths(self.info["start_wl"] + self.info["speed"]*(self.raw_time - start_delay))
        self.info["start_delay"] = start_delay
        self.info["wavelength_source"] = "manual"

    def trigger_times(self, threshold: float|None=None) -> np.ndarray:
        """
        Returns the times (s, relative to the sweep command) of the triggers
        sent by the laser (rising edges of raw_trigger) after the sweep command.

        Args:
            threshold (float): Voltage above which the trigger is considered
                high. Defaults to half way between its lowest and highest values.
        """
        return find_trigger_times(self.raw_time, self.raw_trigger, threshold)

    def remap_from_triggers(self, threshold: float|None=None) -> int:
        """
        Recalculates the spectrum from the raw data, using the triggers sent by
        the laser (see wavelengths_from_triggers).

        Returns:
            int: The number of triggers used.
        """
        t_trig = self.trigger_times(threshold)
        if len(t_trig) == 0:
            raise ValueError("No triggers found in this spectrum.")
        wl, n = wavelengths_from_triggers(self.raw_time, t_trig, self.info["start_wl"], self.info["stop_wl"],
                                          self.info["speed"], self.info.get("trigger_step", 0.0))
        self.set_wavelengths(wl)
        self.info["start_delay"] = float(t_trig[0])
        self.info["wavelength_source"] = "triggers" if n > 1 else "trigger_start"
        return n

    def __repr__(self):
        if len(self.wavelength) == 0:
            return "Spectrum: empty"
        return (f"Spectrum: {self.wavelength[0]:.2f} to {self.wavelength[-1]:.2f} nm, "
                f"{len(self.wavelength)} points, {self.info['speed']:g} nm/s")


def load_spectrum(filename: str) -> Spectrum:
    """
    Reads a transmission spectrum file ("spectrum_*.npz").
    """
    return Spectrum(filename)


def load_spectra(folder: str) -> list[Spectrum]:
    """
    Reads all transmission spectra in a folder, in the order they were measured.
    """
    return [Spectrum(f) for f in sorted(glob.glob(os.path.join(folder, "spectrum_*.npz")))]
