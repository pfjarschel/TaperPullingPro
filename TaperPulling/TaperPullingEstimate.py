# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
Estimates the shape of the taper while it is pulled, from the measured motion of
the flame relative to the fiber.

The classic model (constant or slowly changing hotzone L, r = r0*exp(-x/2L))
assumes that the whole hotzone is heated uniformly. With brushing, it is not:
the heat each point of the fiber receives depends on how long the flame stays
near it, which depends on how the flame moves (uniform sweeps slow down at the
edges; the sinusoidal sweeps of the relative pulling mode stay much longer at
the edges than at the centre), and on the real amplitude of the motion (which
can differ from the commanded one).

Model:
    - The flame heats the fiber with a gaussian profile, of FWHM equal to the
      flame size, centred at the flame position u(t) relative to the fiber centre.
    - The elongation is shared along the fiber in proportion to that heating:
      while the fiber is pulled by dX, the point at s stretches by
      dX*g(s - u)/integral(g), with g the gaussian.
    - Volume is conserved, and the fiber is pulled symmetrically, so the centre
      stays in place and everything else moves outwards.
The profile is calculated on a fixed grid along the fiber (in the frame of its
centre), by integrating the mass conservation equation for ln(r^2).

With a static, uniform hotzone this is exactly the classic model. Calibrated
with the point where real tapers became single mode (d ~ 1.1 µm at 1550 nm),
it predicts the diameter within ~10% in both brushing modes (the classic model,
with the commanded hotzone, was 20% off in the standard mode, and 10x off in the
relative mode).

It only depends on numpy, and can be copied and used on its own:

    from TaperPullingLog import load_pull_log
    from TaperPullingEstimate import estimate_from_log

    log = load_pull_log("pull_full_log.npz")
    x, d, est = estimate_from_log(log, flame_fwhm=0.6)
    plt.plot(x, d)                  # Thinnest diameter (µm) vs. total pulled (mm)
    plt.plot(*est.profile())        # Final half profile (mm, mm)

Lengths in mm, unless noted.
"""
# =============================================================================

import numpy as np


class TaperEstimator:
    """
    Calculates the taper profile from the total pulled distance and the position
    of the flame relative to the fiber centre, fed as they are measured (add()).
    """

    def __init__(self, r0: float=0.0625, flame_fwhm: float=0.6, ds: float=0.01, margin: float=10.0):
        """
        Args:
            r0 (float): Initial fiber radius (mm).
            flame_fwhm (float): Width (FWHM) of the region heated by the flame (mm).
            ds (float): Grid step along the fiber (mm).
            margin (float): Grid length beyond the pulled region, on each side (mm).
        """
        self.ds = ds
        self.margin = margin
        self.reset(r0, flame_fwhm)

    def reset(self, r0: float|None=None, flame_fwhm: float|None=None):
        """
        Starts a new taper (untapered fiber).
        """
        if r0 is not None:
            self.r0 = r0
        if flame_fwhm is not None:
            self.flame_fwhm = flame_fwhm
        self.sigma = max(self.flame_fwhm, 4*self.ds)/(2*np.sqrt(2*np.log(2)))
        self.half = 2*self.margin
        self.s = np.arange(-self.half, self.half + self.ds/2, self.ds)
        self.lnA = np.full(len(self.s), 2*np.log(self.r0))  # ln(r^2)
        self.last_x = None
        self.last_u = None
        self.last_new_t = -np.inf

    def _extend(self, half: float):
        # Untapered fiber added at both ends of the grid
        n = int(np.ceil((half - self.half)/self.ds))
        if n > 0:
            pad = np.full(n, 2*np.log(self.r0))
            self.lnA = np.concatenate([pad, self.lnA, pad])
            self.half += n*self.ds
            self.s = np.arange(len(self.lnA))*self.ds - self.half

    def _step(self, dx: float, u: float):
        # Stretches the fiber by dx, heated by the flame at u
        g = np.exp(-0.5*((self.s - u)/self.sigma)**2)
        e = g*(dx/(g.sum()*self.ds))  # Local strain (sums to dx along the fiber)
        v = np.cumsum(e)*self.ds      # Displacement of each point...
        i0 = len(self.s)//2
        v -= v[i0]                    # ... the centre stays in place
        grad = np.empty_like(self.lnA)
        grad[i0 + 1:] = np.diff(self.lnA[i0:])/self.ds        # Upwind, for s > 0
        grad[:i0] = np.diff(self.lnA[:i0 + 1])/self.ds        # Upwind, for s < 0
        grad[i0] = 0.0
        self.lnA -= v*grad + e

    def add(self, total_pulled: float, flame_pos: float):
        """
        Adds a new measurement: total pulled distance (mm) and flame position
        relative to the fiber centre (mm). Between measurements, both are
        interpolated linearly.
        """
        if self.last_x is None or total_pulled < self.last_x:
            self.last_x, self.last_u = total_pulled, flame_pos
            return
        dx = total_pulled - self.last_x
        if dx > 0:
            if total_pulled/2 + self.margin > self.half:
                self._extend(total_pulled/2 + 2*self.margin)
            # Sub steps: the flame moves less than sigma/2, and the fiber less than ds/2
            du = flame_pos - self.last_u
            n = int(min(max(np.ceil(abs(du)/(self.sigma/2)), np.ceil(dx/self.ds), 1), 500))
            for k in range(n):
                self._step(dx/n, self.last_u + du*(k + 0.5)/n)
        self.last_x, self.last_u = total_pulled, flame_pos

    def add_logged(self, t: float, total_pulled: float, flame_pos: float, max_repeat: float=0.3):
        """
        Same as add(), for a row of the process log (t in s). The motor positions are
        polled less often than the log is written, so a position equal to the previous
        one is not a new measurement, and is skipped (unless it lasted max_repeat s:
        then the flame is really still).
        """
        if flame_pos == self.last_u and t - self.last_new_t < max_repeat:
            return
        self.last_new_t = t
        self.add(total_pulled, flame_pos)

    def radius(self) -> np.ndarray:
        """
        Radius (mm) at each point of the grid (self.s, mm from the centre).
        """
        return np.exp(0.5*self.lnA)

    @property
    def diameter(self) -> float:
        """
        Thinnest diameter (µm).
        """
        return 2000*np.exp(0.5*self.lnA.min())

    @property
    def waist_position(self) -> float:
        """
        Position of the thinnest point, from the centre (mm).
        """
        return self.s[np.argmin(self.lnA)]

    def profile(self, rel_threshold: float=1e-3) -> tuple[np.ndarray, np.ndarray]:
        """
        Half profile, in the same format as TaperShape.profile_from_hz(): z from
        the start of the transition (where the radius is r0) to the centre (mm),
        and radius (mm).
        """
        r = self.radius()
        i0 = len(self.s)//2
        tapered = np.flatnonzero(r[i0:] < self.r0*(1 - rel_threshold))
        if len(tapered) == 0:
            return np.array([0.0, 1.0]), np.array([self.r0, self.r0])
        i1 = i0 + min(tapered[-1] + 2, len(self.s) - i0 - 1)
        s, r = self.s[i0:i1 + 1][::-1], r[i0:i1 + 1][::-1]
        return s[0] - s, r


def flame_positions_from_log(log) -> np.ndarray:
    """
    Flame position relative to the fiber centre (mm), for each row of the motors
    log of a PullLog. Recorded since the estimate was introduced; for older logs,
    it is calculated from the motor positions, centred on their mean while pulling.
    """
    m = log.motors
    if "flame_rel_pos" in m:
        return m["flame_rel_pos"]
    if log.info.get("relative_pull", False):
        u = (m["puller_right_pos"] - m["puller_left_pos"])/2
    else:
        u = m["brusher_pos"].copy()
    pulling = m["stage"] == 3
    return u - (u[pulling].mean() if pulling.any() else u.mean())


def estimate_from_log(log, flame_fwhm: float=0.6, r0: float|None=None):
    """
    Runs the estimate over a saved process (PullLog, from TaperPullingLog).

    Returns:
        x: Total pulled (mm), for each update while pulling.
        d: Thinnest diameter (µm) at each x.
        est: The TaperEstimator, at the end of the process (profile(), radius()...).
    """
    if r0 is None:
        r0 = log.parameters.get("Profile", {}).get("Initial_diameter", 125.0)/2000
    est = TaperEstimator(r0, flame_fwhm)
    m = log.motors
    u_all = flame_positions_from_log(log)
    xs, ds = [], []
    for i in np.flatnonzero(m["stage"] == 3):
        est.add_logged(m["time"][i], m["total_pulled"][i], u_all[i])
        xs.append(m["total_pulled"][i])
        ds.append(est.diameter)
    return np.array(xs), np.array(ds), est
