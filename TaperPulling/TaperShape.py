# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperShape class. This class is useful not only for
the fabrication of fiber tapers, but also as a design tool, due to the many
functions available that can be used to study and explore the shape of fiber
tapers.
It can output a hotzone function to be used in fabrication (by the
TaperPullingCore module) given desired parameters, such as final waist radius
and length, and adiabaticity criteria.
All length units in mm.
Wavelength parameter in µm.
"""
# =============================================================================

import os
import numpy as np
from enum import Enum
from scipy.signal import savgol_filter
from scipy.optimize import minimize

# Get relevant paths
thispath = os.path.dirname(os.path.realpath(__file__)).replace("\\", "/")
rootpath = thispath
respath = f"{rootpath}/resources"


class Mode(Enum):
        LP01 = 0
        LP11 = 1
        LP21 = 2
        LP02 = 3
        LP31 = 4
        LP41 = 5
        LP51 = 6


class TaperShape:
    """
    This class contains all the tools related to the shape of fiber tapers.
    It can calculate pulling parameters for a desired simple shape, obtain an
    optimal adiabatic shape, and calculate the hotzone function for any tapering
    shape. It also calculates the effective indexes and takes dispersion into
    account.
    """
    # Basic parameters
    wavelength = 1.55  # µm
    initial_r = 62.5e-3  # mm
    r_core = 4.1e-3  # mm
    n_core_ratio = 1.0036
    n_medium = 1.0
    n_points = 1001
        
    # Modal effective index difference
    dneffs = np.zeros(11)
    r_array = np.zeros(11)
    z_r0 = np.zeros(11)
    
    def __init__(self, wl: float=1.55,
                       r0: float=62.5e-3,
                       r_core: float=4.1e-3,
                       n_core_ratio: float=1.0036,
                       n_medium: float=1.0,
                       n_points: int=1001
                       ):
        """
        This class contains all the tools related to the shape of fiber tapers.
        
        Args:
        wl: Wavelength of operation in µm. Default is 1.55.
        r0: Initial fiber radius in mm. Default is 0.0625.
        r_core: Fiber core radius, in mm. Default is 0.0041.
        n_core_ratio: n_core/n_claddin ration. Default is 1.0036.
        n_medium: Medium refractive index. Default is 1.0 (vacuum/air).
        n_points: Number of points for the profile/hotzone function. Default is 1001.
        """
        
        # Load default modal effective index difference if parameters are default
        if (wl == self.wavelength and r0 == self.initial_r and r_core == self.r_core and 
                  n_core_ratio == self.n_core_ratio and n_medium == self.n_medium):
            self.load_dneffs(f"{respath}/dneffs_SMF28_FB_1550.txt")
        
        self.set_parameters(wl, r0, r_core, n_core_ratio, n_medium, n_points)
        
    def set_parameters(self,  wl: float=1.55,
                       r0: float=62.5e-3,
                       r_core: float=4.1e-3,
                       n_core_ratio: float=1.0036,
                       n_medium: float=1.0,
                       n_points: int=1001
                       ):

        self.wavelength = wl
        self.initial_r = r0
        self.r_core = r_core
        self.n_core_ratio = n_core_ratio
        self.n_medium = n_medium
        self.n_points = n_points
        self.n_cladding = self.cladding_refractive_index(wl)
        self.n_core = self.core_refractive_index(wl)
        
    def calculate_approx_dneffs(self):
        # This is wrong. Do not use!
        # Will be updated shortly.
        
        self.r_array = np.logspace(np.log10(5e-5), np.log10(self.initial_r), self.n_points)
        
        n = self.n_points
        n_cl = self.n_cladding
        n_co = self.n_core
        n_med = self.n_medium
        wl = self.wavelength

        neffs_cl_1 = np.zeros(n)
        neffs_cl_2 = np.zeros(n)
        neffs_co_1 = np.zeros(n)
        neffs_co_2 = np.zeros(n)

        for i in range(n):
            clad_r = 1e3*self.r_array[i]
            core_r = self.r_core*(clad_r/self.initial_r)
            neffs_co_1[i] = self.mode_neff(core_r, n_co, n_cl, wl, Mode.LP01)
            neffs_co_2[i] = self.mode_neff(core_r, n_co, n_cl, wl, Mode.LP02)
            neffs_cl_1[i] = self.mode_neff(clad_r, n_cl, n_med, wl, Mode.LP01)
            neffs_cl_2[i] = self.mode_neff(clad_r, n_cl, n_med, wl, Mode.LP02)

        thresh_lp01_idx = np.nonzero(np.diff(neffs_cl_1))[0][-1]
        thresh_lp01_r = self.r_array[thresh_lp01_idx]
        thresh_lp02_idx = np.nonzero(np.diff(neffs_cl_2))[0][-1]
        thresh_lp02_r = self.r_array[thresh_lp02_idx]
        neffs_lp01 = np.zeros(n)
        neffs_lp02 = np.zeros(n)
        a = 0.5
        for i in range(n):
            if i <= thresh_lp01_idx:
                neffs_lp01[i] = neffs_cl_1[i]
            else:
                x = 1e3*(self.r_array[i] - thresh_lp01_r)
                co_weight = x/(x + a)
                cl_weight = 1 - co_weight
                neffs_lp01[i] = cl_weight*neffs_cl_1[i] + co_weight*neffs_co_1[i]
        for i in range(n):
            if i <= thresh_lp02_idx:
                neffs_lp02[i] = neffs_cl_2[i]
            else:
                x = 1e3*(self.r_array[i] - thresh_lp02_r)
                co_weight = x/(x + a)
                cl_weight = 1 - co_weight
                neffs_lp02[i] = cl_weight*neffs_cl_2[i] + co_weight*neffs_co_2[i]

        self.dneffs = np.zeros(n)
        for i in range(n):
            self.dneffs[i] = neffs_lp01[i] - neffs_lp02[i]
            if self.dneffs[i] < 1e-5:
                self.dneffs[i] = 1e-5
        
    def load_dneffs(self, file):
        dneffs_txt = np.loadtxt(file)
        txt_r_array = dneffs_txt[:, 0]
        txt_dneffs = dneffs_txt[:, 1]

        self.r_array = np.logspace(np.log10(txt_r_array.max()), np.log10(txt_r_array.min()), self.n_points)
        self.dneffs = np.interp(self.r_array, txt_r_array, txt_dneffs)
    
    def cladding_refractive_index(self, wl):
        """
        Calculates the refractive index of fused silica (SiO2) at a given wavelength
        using the Sellmeier equation.
        
        Args:
            wl (float): Wavelength in micrometers (μm)
            
        Returns:
            float: Refractive index of fused silica at the given wavelength
        """
        
        # Sellmeier coefficients for fused silica
        A1 = 0.6961663
        A2 = 0.4079426
        A3 = 0.8974794
        
        lambda1_sq = 0.0684043**2
        lambda2_sq = 0.1162414**2
        lambda3_sq = 9.896161**2
        
        lambda_sq = wl**2
        
        n_sq = 1 + A1 * lambda_sq / (lambda_sq - lambda1_sq) + \
                A2 * lambda_sq / (lambda_sq - lambda2_sq) + \
                A3 * lambda_sq / (lambda_sq - lambda3_sq)
        
        return np.sqrt(n_sq)

    def core_refractive_index(self, wl, n_core_ratio=1.0036):
        """
        Calculates the refractive index of SMF core at a given wavelength
        using the standard refractive index difference of 0.36%.
        
        Args:
            wl (float): Wavelength in micrometers (μm)
            
        Returns:
            float: Refractive index of SMF core at the given wavelength
        """
        
        return n_core_ratio*self.cladding_refractive_index(wl)

    def calculate_v_number(self, r_core, n_co, n_cl, wl):
        """
        Calculates the V-number for a step-index fiber.

        Args:
            n_co (float): Refractive index of the fiber core
            n_cl (float): Refractive index of the fiber cladding
            r_core (float): Radius of the fiber core
            wl (float): Wavelength of light in micrometers

        Returns:
            float: V-number for the step-index fiber
        """
        NA = np.sqrt(n_co**2 - n_cl**2)
        V = 2*np.pi*r_core*NA/wl
        return V
    
    def calculate_single_mode_rcore(self, n_co, n_cl, wl):
        V = 2.405
        NA = np.sqrt(n_co**2 - n_cl**2)
        r_core = V*wl/(2*np.pi*NA)
        return r_core
    
    def calculate_single_mode_wl(self, r_core, n_co, n_cl):
        V = 2.405
        NA = np.sqrt(n_co**2 - n_cl**2)
        wl = 2*np.pi*r_core*NA/V
        return wl
    
    def uniform_hz(self, hz0: float, alpha: float, pull: float) -> np.ndarray:
        hz_function = np.array([np.linspace(0.0, pull, self.n_points), 
                                np.linspace(hz0, hz0 + pull*alpha, self.n_points)])
        return hz_function
    
    def calc_rw_uniform_hz(self, hz0: float, alpha: float, pull: float, r0: float) -> float:
        if np.abs(alpha) < 1e-6: alpha = 1e-6  # Avoid numerical indetermination
        rw = r0*((1 + alpha*pull/hz0)**(-1/(2*alpha)))
        return rw
    
    def calc_pull_uniform_hz(self, hz0: float, alpha: float, rw: float, r0: float) -> float:
        if np.abs(alpha) < 1e-6: alpha = 1e-6  # Avoid numerical indetermination
        pull = 2*((r0/rw)**(2*alpha) - 1)*hz0/(2*alpha)
        return pull
    
    def parametric_hz(self, xt, x0, y0, a, min_l, n):
        x = np.linspace(0, xt, n)
        y = y0 - np.exp(a*(x - x0)) + a*(x - x0) + 1
        y[y < min_l] = min_l
        y = savgol_filter(y, int(n/30), 1)
    
        return x, y
    
    def hz_from_profile(self, z_array: np.ndarray|list[float],
                        r_array: np.ndarray|list[float], lw: float):
        # Calculate hotzone size from z (l(z))
        rw = np.min(r_array)
        l_z = []
        for i in range(len(z_array)):
            l_zi = lw*((rw/r_array[i])**2) + (2/((r_array[i])**2))*np.trapz((r_array[i:])**2, z_array[i:])
            l_z.append(l_zi)
        l_z = np.array(l_z)
        
        # Finally, calculate l(x)
        x_array = 2*z_array + l_z - l_z[0]
        
        return x_array, l_z
    
    def profile_from_hz(self, x_arr, l_arr):
        n = len(x_arr)
        r_arr = np.zeros(n)
        for i in range(n):
            r_arr[i] = self.initial_r*np.exp(-0.5*np.trapz(1/l_arr[:i + 1], x_arr[:i + 1]))
        z_arr = (x_arr - l_arr + l_arr[0])/2
        
        return z_arr, r_arr
    
    def profile_angles(self, z_arr, r_arr):
        n = len(r_arr)
        angles = np.zeros(n)
        for i in range(1, n):
            delta_rz = r_arr[i - 1] - r_arr[i]
            angles[i - 1] = np.arctan(delta_rz/(z_arr[i] - z_arr[i - 1]))
        angles[-1] = angles[-2]
        
        return angles
    
    def profile_angles_deg(self, z_arr, r_arr):
        angles = self.profile_angles(z_arr, r_arr)
        return angles*(180/np.pi)
    
    def ideal_adiabatic_profile(self, rw: float, f: float=0.5):
        rw_idx = np.abs(self.r_array - rw).argmin() + 1
        if rw_idx > self.n_points: rw_idx = self.n_points
        
        z_r = np.zeros(self.n_points)
        
        # Calculate adiabaticity z(r) integral for every r
        # Tim Birks method
        # for i in range(self.n_points):
        #     z_r[i] = 1e-3*self.wavelength*np.trapz(1/(self.r_array[i:]*self.dneffs[i:]), self.r_array[i:])
        
        # Simpler angle method (Love)
        for i in range(1, self.n_points):
            beat_l = 1e-3*self.wavelength/self.dneffs[i]
            delta_rz = self.r_array[i - 1] - self.r_array[i]
            z_r[i] = z_r[i - 1] + delta_rz*beat_l/self.r_array[i]
        
        z_rw = z_r[:rw_idx]/f
        adiab_r_array = self.r_array[:rw_idx]
        
        return z_rw, adiab_r_array
    
    def profile_optimization_function(self, p, min_l, n, z0, r0):
        x_arr, l_arr = self.parametric_hz(p[0], p[1], p[2], p[3], min_l, n)
        z_arr, r_arr = self.profile_from_hz(x_arr, l_arr)
        angles = self.profile_angles(z_arr, r_arr)
        angles0 = self.profile_angles(z0, r0)
        new_r_arr = np.linspace(r_arr.max(), r_arr.min(), n)
        angles_interp = np.flip(np.interp(np.flip(new_r_arr), np.flip(r_arr), np.flip(angles)))
        angles0_interp = np.flip(np.interp(np.flip(new_r_arr), np.flip(r0), np.flip(angles0)))
        d_angles =  (angles_interp - angles0_interp)
        
        d_angles[d_angles < 0] = 0
        val_angles = np.sum(d_angles/(angles0_interp))
        
        val_r = 1e3*np.abs(r_arr.min() - r0.min())
        
        val_x = x_arr.max()
        
        return np.sqrt(val_angles**2 + val_r**2 + val_x)
    
    def extend_profile_until_rw(self, z_arr, r_arr, rw, lf):
        while np.abs(r_arr.min() - rw) > np.abs(r_arr[-1] - r_arr[-2]):
            if r_arr.min() < rw:
                rw_idx = np.abs(r_arr - rw).argmin()
                r_arr = r_arr[:rw_idx + 1]
                z_arr = z_arr[:rw_idx + 1]
            elif r_arr.min() > rw:
                x_arr, l_arr = self.hz_from_profile(z_arr, r_arr, lf)
                dx = x_arr[-1] - x_arr[-2]
                x_conc = np.arange(x_arr[-1] + dx, x_arr[-1] + 1.0 + dx, dx)
                x_arr = np.concatenate([x_arr, x_conc])
                l_arr = np.concatenate([l_arr, [lf]*len(x_conc)])
                z_arr, r_arr = self.profile_from_hz(x_arr, l_arr)
            
        return z_arr, r_arr
    
    def real_adiabatic_profile(self, rw: float, min_hz: float=2.0, start_f: float=1.0,
                               f_step: float=0.01, limit_r: float=20e-3):        
        print("Trying to find the best feasible profile for the current parameters...")
        
        # Find f factor that allows starting hz to be feasible
        f = 1.0  # First attempt
        start_hz_ok = False
        while not start_hz_ok:
            z_rw, adiab_r_array = self.ideal_adiabatic_profile(rw, f)
            
            # Get hot-zone to adjust for real flame sizes
            x_hz, hz = self.hz_from_profile(z_rw, adiab_r_array, min_hz)
            
            # Find point where hz <= min_hz
            min_hz_diff = hz[1:] - min_hz
            min_hz_idx = len(hz) - 1
            for i in range(1, len(min_hz_diff)):
                if np.sign(min_hz_diff[i - 1]) != np.sign(min_hz_diff[i]):
                    min_hz_idx = i
                    break
            r0 = adiab_r_array[min_hz_idx]
            
            if hz[0] >= min_hz and r0 <= limit_r:
                start_hz_ok = True
            else:
                f -= f_step
                if f <= 0:
                    raise Exception("Error: Could not find a feasible profile that matches the desired final radius.")
            
        if hz.min() >= min_hz or min_hz_idx >= len(hz) - 1:          
            # Profile is already feasible, min_hz is not broken
            print("Found a feasible profile!")
            return z_rw, adiab_r_array, f
        else:          
            # Perform optimization using parametric hot-zone
            curr_n = len(adiab_r_array)
            x_min = x_hz.max()
            x_max = 100.0
            x0_min = 0.0
            x0_max = x_max
            l0_min = min_hz
            l0_max = 50.0
            a_min = -10.0
            a_max = 10.0
            p0 = [x_hz.max(), x_hz[hz.argmax()], hz.max(), 0.5]
            bounds = [[x_min, x_max], [x0_min, x0_max], [l0_min, l0_max], [a_min, a_max]]
            
            # Nelder-Mead is a little slow but gives good results.
            p1 = minimize(self.profile_optimization_function, p0, bounds=bounds,
                          args=(min_hz, curr_n, z_rw, adiab_r_array), method='Nelder-Mead').x
            x_arr_opt0, l_arr_opt0 = self.parametric_hz(p1[0], p1[1], p1[2], p1[3], min_hz, curr_n)
            z_arr_opt0, r_arr_opt0 = self.profile_from_hz(x_arr_opt0, l_arr_opt0)
            z_arr_opt0, r_arr_opt0 = self.extend_profile_until_rw(z_arr_opt0, r_arr_opt0, rw, l_arr_opt0[-1])
            
            print("Found an optimized profile!")
            return z_arr_opt0, r_arr_opt0
    
    
        
