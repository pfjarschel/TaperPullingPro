# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingSim class. It simulates the fabrication of a
fiber taper by flame brushing, before pulling it, and animates the process: the
flame sweeping along the fiber, and the fiber getting thinner under it.

It is useful as a design tool, alongside the TaperShape class: for a hotzone
function (hotzone size vs. total pulled distance), it gives the shape of the
fiber at each moment of the process, as the setup is expected to make it.

The simulation uses the same model as the estimate shown while pulling
(TaperPullingEstimate): the flame heats the fiber with a gaussian profile, the
elongation is shared along the fiber in proportion to the heating, and volume is
conserved. The flame moves as the motors of the setup were seen to do it, in
both brushing modes (standard: the flame sweeps along a fiber that is pulled
symmetrically; relative: the flame is fixed, and the pullers move the fiber in
sinusoidal sweeps). Everything is shown in the frame of the centre of the fiber.

    sim = TaperPullingSim()
    sim.relative = False
    z, r_arrs, total_pulled = sim.simulate([[0, 33], [3.5, 3.5]])
    sim.show_last_sim_animation()                  # Live window
    sim.show_last_sim_animation("taper.gif")       # Saved (also .mp4, with ffmpeg)

All length units in mm, unless noted.
"""
# =============================================================================

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter

try:
    from TaperPulling.TaperPullingEstimate import (TaperEstimator, planned_flame_motion,
                                                   BRUSHER_REVERSAL_DECEL, RELATIVE_SWEEP_GAIN)
except ImportError:
    from TaperPullingEstimate import (TaperEstimator, planned_flame_motion,
                                      BRUSHER_REVERSAL_DECEL, RELATIVE_SWEEP_GAIN)


class TaperPullingSim:
    """
    Simulates and animates the fabrication of a taper by flame brushing.
    """

    # Fabrication parameters
    vp = 0.125  # Pulling velocity, each puller (mm/s)
    vf = 2.4  # Flame brushing velocity (mm/s)
    flame_size = 0.6  # Flame size: width (FWHM) of the heated region (mm)
    min_span = 1.5  # Sweeps shorter than this are not made: the flame stays at the centre (mm)
    relative = False  # Brushing mode: False for standard, True for relative
    r0 = 0.0625  # Initial fiber radius (mm)

    # Simulation parameters
    time_step = 0.02  # Time step (s)
    grid_step = 0.01  # Step of the grid along the fiber (mm)
    max_frames = 1024  # Upper limit of stored frames (profiles)

    # Last simulation results (one value, or profile, for each frame)
    z_arr = np.zeros(11)  # Positions along the fiber, from its centre
    r_arrs = np.zeros((1, 11))  # Radius profiles
    x_arr = np.zeros(1)  # Total pulled
    fpos_arr = np.zeros(1)  # Flame positions, from the fiber centre
    hz_arr = np.zeros(1)  # Hotzone sizes
    d_arr = np.zeros(1)  # Waist (thinnest) diameters (µm)

    def __init__(self, r0: float=0.0625):
        """
        Args:
            r0: Initial fiber radius in mm. Default is 0.0625.
        """
        self.r0 = r0
        self._anim = None

    def simulate(self, hotzone_function: list[list[float]]|np.ndarray, x_end: float|None=None):
        """
        Simulates the pull of a taper with a hotzone function.

        Args:
            hotzone_function: 2D list or array: total pulled distances, and the hotzone
                size at each of them (as given by TaperShape.hz_from_profile()).
            x_end (float): Pull up to here. Defaults to the end of the hotzone function
                (beyond it, its last value is kept).

        Returns:
            z_arr: Positions along the fiber, from its centre.
            r_arrs: Radius profiles, one for each stored frame (their total pulled
                distances, flame positions... are kept in x_arr, fpos_arr, hz_arr, d_arr).
            total_pulled: Total pulled distance at the end.
        """
        try:
            hz = np.array(hotzone_function, dtype=float)
        except Exception:
            raise ValueError("hotzone_function must be a 2D list or numpy array.")
        if hz.ndim != 2 or hz.shape[0] < 2 or hz.shape[1] < 2:
            raise ValueError("hotzone_function must be a 2D list or numpy array.")
        hz_x, hz_y = hz[0], hz[1]
        if x_end is None:
            x_end = float(hz_x.max())

        # The grid has to hold all the fiber that gets tapered: half of the pulled
        # distance, plus how far the flame reaches, on each side
        sweep = max(float(hz_y.max()) - self.flame_size, 0.0)
        reach = max(RELATIVE_SWEEP_GAIN*sweep/2, sweep/2 + self.vf**2/(2*BRUSHER_REVERSAL_DECEL))
        half = x_end/2 + reach + 3*self.flame_size + 1.0
        est = TaperEstimator(self.r0, self.flame_size, ds=self.grid_step, margin=half)
        keep = np.abs(est.s) <= half
        self.z_arr = est.s[keep]

        steps = int(x_end/(2*self.vp*self.time_step)) + 1
        every = max(1, int(np.ceil(steps/self.max_frames)))
        r_arrs, x_arr, fpos_arr, d_arr = [], [], [], []
        motion = planned_flame_motion([hz_x, hz_y], x_end, self.flame_size, self.min_span, self.vf,
                                      self.vp, self.relative, dt=self.time_step)
        t0 = t1 = time.time()
        i, x, u, stored = 0, 0.0, 0.0, False
        for i, (x, u) in enumerate(motion):
            est.add(x, u)
            stored = i % every == 0
            if stored:
                r_arrs.append(est.radius()[keep])
                x_arr.append(x)
                fpos_arr.append(u)
                d_arr.append(est.diameter)
            if time.time() - t1 >= 1.0:
                t1 = time.time()
                ratio = max(i, 1)/steps
                print(f"Simulation progress: {100*ratio:.1f}%. {(t1 - t0)*(1/ratio - 1):.0f} s remaining      ", end="\r")
        if not stored:  # The end of the pull is always kept
            r_arrs.append(est.radius()[keep])
            x_arr.append(x)
            fpos_arr.append(u)
            d_arr.append(est.diameter)
        print("Simulation done                                           ")

        self.r_arrs = np.array(r_arrs)
        self.x_arr = np.array(x_arr)
        self.fpos_arr = np.array(fpos_arr)
        self.d_arr = np.array(d_arr)
        self.hz_arr = np.interp(self.x_arr, hz_x, hz_y)

        return self.z_arr, self.r_arrs, float(self.x_arr[-1])

    def _flame_image(self) -> np.ndarray:
        # Glow of the flame: gaussian along the fiber (FWHM = flame size, over a width of
        # 3 flame sizes), brighter near the fiber. Pale yellow where hottest, orange outside.
        xi = np.linspace(-1.5, 1.5, 121)[None, :]
        yi = np.linspace(-1.0, 1.0, 81)[:, None]
        heat = np.exp(-0.5*(xi/0.4247)**2)*(0.4 + 0.6*np.exp(-0.5*(yi/0.45)**2))
        img = np.empty(heat.shape + (4,))
        cold, hot = np.array([1.0, 0.35, 0.05]), np.array([1.0, 0.93, 0.65])
        img[..., :3] = cold + (hot - cold)*heat[..., None]**1.5
        img[..., 3] = 0.9*heat
        return img

    def show_last_sim_animation(self, save_path: str|None=None, fps: int=30, frame_step: int=1,
                                zoom_margin: float=2.5, show: bool|None=None, dpi: int=110):
        """
        Animates the last simulation: the flame sweeping, and the fiber getting thinner.
        Above, the whole taper; below, the region around the hotzone, with a radius scale
        that follows the fiber as it gets thinner (so the process can be seen until the end).

        Args:
            save_path (str): If given, the animation is saved to this file (.gif, or
                .mp4 if ffmpeg is installed).
            fps (int): Frames per second.
            frame_step (int): Shows only each frame_step-th stored frame (faster, smaller files).
            zoom_margin (float): The view below goes this far beyond the largest hotzone, on each side.
            show (bool): Open a window with the animation. Defaults to True, or False when saving.
            dpi (int): Resolution of the saved file.

        Returns:
            The matplotlib FuncAnimation.
        """
        if len(self.r_arrs) < 2:
            raise RuntimeError("Nothing to show: run simulate() first.")
        if show is None:
            show = save_path is None

        z, um = self.z_arr, 1000.0
        r0_um = self.r0*um
        frames = list(range(0, len(self.r_arrs), max(1, int(frame_step))))
        if frames[-1] != len(self.r_arrs) - 1:
            frames.append(len(self.r_arrs) - 1)
        zoom_half = min(self.hz_arr.max()/2 + zoom_margin, z[-1])
        in_zoom = np.abs(z) <= zoom_half
        flame_half = 1.5*self.flame_size

        bg, panel, glass, edge, text, grid, green = ("#0b0f14", "#111821", "#7fb8e6", "#dff1ff", "#c9d4df",
                                                     "#243242", "#5fd38d")
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), facecolor=bg,
                                       gridspec_kw={"height_ratios": [1, 1.5], "hspace": 0.32})
        flame_img = self._flame_image()
        fibers, flames, hz_lines = [], [], []
        for ax in (ax1, ax2):
            ax.set_facecolor(panel)
            ax.tick_params(colors=text, labelsize=9)
            for spine in ax.spines.values():
                spine.set_color(grid)
            ax.grid(True, color=grid, linewidth=0.6)
            ax.set_axisbelow(True)
            ax.set_ylabel("Radius (µm)", color=text)
            fibers.append(ax.fill(z, z*0, facecolor=glass, edgecolor=edge, linewidth=0.8, zorder=2)[0])
            flames.append(ax.imshow(flame_img, extent=(-1, 1, -1, 1), aspect="auto", origin="lower",
                                    interpolation="bilinear", zorder=3))
            hz_lines.append([ax.axvline(0.0, linestyle="--", color=green, linewidth=1.0, alpha=0.7, zorder=4)
                             for _ in range(2)])
        ax2.set_xlabel("Position along the fiber (mm)", color=text)
        ax1.set_title("Whole taper", color=text, fontsize=10, loc="left")
        ax2.set_title("Around the hotzone (radius scale follows the fiber)", color=text, fontsize=10, loc="left")
        ax1.axvspan(-zoom_half, zoom_half, color="white", alpha=0.04, zorder=1)
        ax1.set_xlim(z[0], z[-1])
        ax1.set_ylim(-1.7*r0_um, 1.7*r0_um)
        ax2.set_xlim(-zoom_half, zoom_half)
        mode = "relative" if self.relative else "standard"
        readout = fig.text(0.5, 0.955, "", color=edge, ha="center", va="center", family="monospace", fontsize=12)
        fig.text(0.985, 0.012, f"{mode} brushing  ·  flame {self.flame_size:g} mm", color=text, alpha=0.6,
                 ha="right", va="bottom", fontsize=8)
        fig.subplots_adjust(left=0.09, right=0.975, top=0.89, bottom=0.09)

        def draw(i):
            r = self.r_arrs[i]*um
            outline = np.column_stack([np.concatenate([z, z[::-1]]), np.concatenate([r, -r[::-1]])])
            y2 = 1.35*r[in_zoom].max()
            ax2.set_ylim(-y2, y2)
            u, hz_now = self.fpos_arr[i], self.hz_arr[i]
            for ax, fiber, flame, lines in zip((ax1, ax2), fibers, flames, hz_lines):
                fiber.set_xy(outline)
                y = ax.get_ylim()[1]
                flame.set_extent((u - flame_half, u + flame_half, -y, y))
                lines[0].set_xdata([-hz_now/2, -hz_now/2])
                lines[1].set_xdata([hz_now/2, hz_now/2])
            ax1.set_xlim(z[0], z[-1])  # (imshow's set_extent can change the limits)
            ax2.set_xlim(-zoom_half, zoom_half)
            d = self.d_arr[i]
            d_txt = f"{d:6.2f} µm" if d >= 10 else f"{d:6.3f} µm"
            readout.set_text(f"pulled {self.x_arr[i]:6.2f} mm     waist ⌀ {d_txt}     hotzone {hz_now:5.2f} mm")
            return ()

        draw(frames[0])
        self._anim = FuncAnimation(fig, draw, frames=frames, interval=1000.0/fps, blit=False, repeat=False)
        if save_path is not None:
            writer = PillowWriter(fps=fps) if save_path.lower().endswith(".gif") else FFMpegWriter(fps=fps, bitrate=4000)
            t0 = time.time()
            self._anim.save(save_path, writer=writer, dpi=dpi, savefig_kwargs={"facecolor": bg})
            print(f"Animation saved to {save_path} ({len(frames)} frames, {time.time() - t0:.0f} s)")
        if show:
            plt.show()
        else:
            plt.close(fig)
        return self._anim
