# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingCamera class.
It reads images from a standard USB camera (anything OpenCV can open, like a
USB microscope) in a separate thread, keeping the most recent one available.
It is used to watch the taper, e.g. while forming a loop. It can save
snapshots and record video. It can also be simulated.
"""
# =============================================================================

import sys
import time
from threading import Thread, Event, Lock
import numpy as np
import cv2


class TaperPullingCamera:
    """
    This class reads images from a USB camera, using OpenCV.
    """
    index = 0
    cap = None
    ok = False
    simulate = False
    flip_h = False
    flip_v = False
    last_error = ""

    thread = None
    stop_event = None
    lock = None
    frame = None        # Most recent image (BGR)
    frame_count = 0     # Increases with every new image
    fps = 0.0           # Measured

    writer = None       # Video being recorded
    recording = False

    # Simulated image: a loop of this diameter, relative to the image height
    sim_loop_size = 0.4
    sim_fps = 15.0

    def __init__(self):
        """
        Nothing is done at instantiation. Use open().
        """
        self.lock = Lock()

    def __del__(self):
        self.close()

    def open(self, index: int=0, simulate: bool=False) -> bool:
        """
        Opens the camera and starts reading images.

        Args:
            index (int): Camera number, as seen by the system (0 is the first).
            simulate (bool, optional): Simulate the camera. Defaults to False.
        """
        self.close()
        self.index = int(index)
        self.simulate = simulate
        self.last_error = ""

        if not simulate:
            try:
                # DirectShow opens much faster on Windows
                if sys.platform.startswith("win"):
                    self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
                else:
                    self.cap = cv2.VideoCapture(self.index)
                if not self.cap.isOpened():
                    raise Exception(f"Camera {self.index} could not be opened.")
            except Exception as e:
                self.last_error = str(e)
                print(f"Error opening the camera: {e}")
                self.cap = None
                return False

        self.ok = True
        self.frame = None
        self.frame_count = 0
        self.stop_event = Event()
        self.thread = Thread(target=self.read_loop, args=(self.stop_event,), daemon=True)
        self.thread.start()
        return True

    def close(self):
        self.stop_recording()
        if self.thread is not None:
            self.stop_event.set()
            if self.thread.is_alive():
                self.thread.join(2.0)
            self.thread = None
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None
        self.ok = False

    def read_loop(self, stop_event):
        t_last = time.time()
        while not stop_event.is_set():
            if self.simulate:
                stop_event.wait(1.0/self.sim_fps)
                frame = self.sim_frame()
            else:
                try:
                    got, frame = self.cap.read()
                except Exception:
                    got = False
                if not got or frame is None:
                    stop_event.wait(0.05)
                    continue

            now = time.time()
            if now > t_last:
                self.fps = 0.9*self.fps + 0.1/(now - t_last)
            t_last = now

            with self.lock:
                if self.flip_h and self.flip_v:
                    frame = cv2.flip(frame, -1)
                elif self.flip_h:
                    frame = cv2.flip(frame, 1)
                elif self.flip_v:
                    frame = cv2.flip(frame, 0)
                self.frame = frame
                self.frame_count += 1
                if self.recording and self.writer is not None:
                    try:
                        self.writer.write(frame)
                    except Exception as e:
                        print(f"Error recording video: {e}")
                        self.recording = False

    def set_flip_h(self, flip: bool):
        """Flips the camera image horizontally."""
        flip = bool(flip)
        with self.lock:
            if self.flip_h != flip:
                self.flip_h = flip
                if self.frame is not None:
                    self.frame = cv2.flip(self.frame, 1)
                    self.frame_count += 1

    def set_flip_v(self, flip: bool):
        """Flips the camera image vertically."""
        flip = bool(flip)
        with self.lock:
            if self.flip_v != flip:
                self.flip_v = flip
                if self.frame is not None:
                    self.frame = cv2.flip(self.frame, 0)
                    self.frame_count += 1

    def get_frame(self) -> np.ndarray|None:
        """
        Returns the most recent image (BGR, as used by OpenCV), or None if
        there isn't one yet.
        """
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def save_snapshot(self, filename: str) -> bool:
        """
        Saves the most recent image. The format is given by the file extension.
        """
        frame = self.get_frame()
        if frame is None:
            return False
        # imencode + tofile, instead of imwrite, to accept any character in the file name
        got, buffer = cv2.imencode("." + filename.rsplit(".", 1)[-1], frame)
        if got:
            buffer.tofile(filename)
        return bool(got)

    def start_recording(self, filename: str) -> bool:
        """
        Starts recording video (.avi) of everything the camera sees.
        """
        self.stop_recording()
        frame = self.get_frame()
        if frame is None:
            return False

        fps = float(np.clip(self.fps, 5.0, 60.0))
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"MJPG"), fps, (width, height))
        if not writer.isOpened():
            return False
        with self.lock:
            self.writer = writer
            self.recording = True
        return True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            if self.writer is not None:
                try:
                    self.writer.release()
                except Exception:
                    pass
            self.writer = None

    def sim_frame(self) -> np.ndarray:
        """
        Simulated image: a fiber making a loop.
        """
        h, w = 480, 640
        img = np.full((h, w, 3), 30, dtype=np.uint8)
        r = int(max(self.sim_loop_size, 0.02)*h/2)
        cx, cy = w//2, h//2 - r//2
        color = (215, 225, 235)
        cv2.circle(img, (cx, cy), r, color, 2, cv2.LINE_AA)
        # The two arms leave from the bottom of the loop, crossing each other
        cv2.line(img, (cx, cy + r), (0, cy + r + h//6), color, 2, cv2.LINE_AA)
        cv2.line(img, (cx, cy + r), (w - 1, cy + r + h//6), color, 2, cv2.LINE_AA)
        noise = np.random.randint(0, 12, (h, w, 1), dtype=np.uint8)
        return cv2.add(img, np.repeat(noise, 3, axis=2))
