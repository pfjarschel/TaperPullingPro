# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module reads the full process log ("*_full_log.npz") saved by the taper
pulling program after each fabrication, so that analysis code does not need to
know how the file is organized.

It only depends on numpy, and can be copied and used on its own:

    from TaperPullingLog import load_pull_log

    log = load_pull_log("auto_15_04_32_full_log.npz")
    plt.plot(log.total_pulled, log.power)     # Transmission vs. pulled distance
    freqs, x, spec = log.spectrogram(1000)    # Recalculate the spectrogram

All lengths in mm, times in s, velocities in mm/s, power in mW.
"""
# =============================================================================

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
