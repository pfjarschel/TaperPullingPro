# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains the TaperPullingSim class. This class is useful not only for
the fabrication of fiber tapers, but also as a design tool alongside, the TaperShape
class, due to the many functions available that can be used to study and explore
the shape of fiber tapers.
It can simulate the taper fabrication process (currently, only flame brushing or
an unrealistic hot-zone that can change length), outputting the fiber shape during
each step of the process (hot-zone size or flame position).
It can also aid in monitoring the fabrication process, showin valuable inferred
data from the pulling stats (total pulled, etc.)
All length units in mm.
Wavelength parameter in µm.
"""
# =============================================================================

import os
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter


# Get relevant paths
thispath = os.path.dirname(os.path.realpath(__file__)).replace("\\", "/")
rootpath = thispath
respath = f"{rootpath}/resources"

class TaperPullingSim:
    """
    This class contains simulation routines for the flame brushing method.
    It can also simulate an unrealistic growing/shrinking flame, which gives a result
    that usually matches the designed shape (but with smoothed edges).
    """
    
    # Fabrication parameters
    vp = 0.125  # Pulling velocity (mm/s)
    vf = 2.0  # Flame velocity (mm/s)
    af0 = 4.5  # Flame acceleration (mm/s2)
       
    # Physical parameters
    min_flame_size = 1.0  # Flame real length (mm)
    r0 = 0.0625  # Initial fiber radius (mm)

    # Flame brushing parameters
    predict_reverse_direction = True  # Flame starts decelerating before reaching hotzone limit
    turn_off_at_edges = False  # Turn pulling off at edges (compensates for low acceleration)
    end_brush_at_edge = False  # Wait until brushing reaches next edge to end
    compensate_flame_size = True  # Subtract flame size from hotzones
    
    # Simulation parameters
    time_step = 0.01  # Time step used in simulation (s)
    move_flame = True  # If false, will consider a growing flame
    smooth_inside_flame = True  # Smooth out all points inside flame
    heat_reach = 0.0  # Exponentially decaying hotzone reach beyond flame, 0 to disable. WIP, can dramatically change the results.
    min_heat_reach = 0.001  # Cut-off reach that is below this value, helps speed up simulation
    reduce_frames_factor = 1  # Capture each Xth frame
    max_frames = 1024  # Set an upper frame limit
    pos_0 = 0.0   # Flame initial position (middle)
    
    # Last simulation results
    z_arr = np.zeros(11)  # Array to keep z positions
    r_arrs = np.zeros((11, 11))  # Array to keep all profile arrays
    fpos_arr = np.zeros(11)  # Array to keep flame positions
    hz_arr = np.zeros(11)  # Array to keep hotzones
    fsize_arr = np.zeros(11)  # Array to keep flame size when flame grows
    
    def __init__(self, r0: float=0.0625):
        """
        This class contains taper fabrication simulation routines.
        
        Args:
        r0: Initial fiber radius in mm. Default is 0.0625.
        """
        
        self.r0 = r0
        
    def simulate(self, hotzone_function: list[list[float]]|np.ndarray):
         # Set hotzone function
        try:
            hotzone_function = np.array(hotzone_function)
        except:
            raise Exception("Error: hotzone_function must be a 2D list or numpy array.")
        if hotzone_function.shape[0] < 2:
            raise Exception("Error: hotzone_function must be a 2D list or numpy array.")
        
        hz_x = hotzone_function[0]
        hz_y = hotzone_function[1]
        
        # Definitions
        total_to_pull = hz_x.max()  # Distance to pull, two sides
        total_pulled = 0.0  # Pulled distance, two sides
        hz0 = hz_y[0]  # Initial hotzone
        lw = hz_y[-1]  # Waist length
        half_pull = total_to_pull/2.0  # Distance to pull, one side
        fiber_l = 1.5*(total_to_pull + lw)  # Total simulated fiber length
        dx = self.vp*self.time_step  # Pulling step
        dx_f = self.vf*self.time_step  # Flame position step
        dt = self.time_step
        pts = int(np.round(fiber_l/dx))
        steps = int(np.round(half_pull/dx))
        
        # Simulation preparations
        if steps/self.reduce_frames_factor > self.max_frames:
            self.reduce_frames_factor = int(steps/self.max_frames)
        pos_1 = self.pos_0  # Flame position
        prev_pos = self.pos_0  # Flame previous position
        hz1 = hz0  # Hot Zone initial size
        flame_size = self.min_flame_size  # Flame initial size
        cur_vf = 0  # Current flame velocity
        af = self.af0  # Current acceleration
        self.z_arr = np.linspace(-fiber_l/2.0, fiber_l/2.0, pts)  # Position array
        self.fpos_arr = np.zeros(steps)  # Array to keep flame positions
        self.hz_arr = np.zeros(steps)  # Array to keep hotzones
        self.fsize_arr = np.zeros(steps)  # Array to keep flame size when flame grows
        delta_x = (self.z_arr[-1] - self.z_arr[0])/(pts - 1)
        r_arr = self.r0*np.ones(pts)  # Radius array
        self.r_arrs = np.zeros((int(steps/self.reduce_frames_factor) + 1, pts))  # Array to keep all r arrays
        self.r_arrs[0] = r_arr
        compensate_l = self.compensate_flame_size*(flame_size)/2
        
        t0 = time.time()
        t1 = t0
        i = 1
        done = False
        last_sweep = False
        
        while not done:
            turn_off_condition = np.abs(np.abs(cur_vf) - self.vf) > 0.10*self.vf
            if not self.turn_off_at_edges or (self.turn_off_at_edges and not turn_off_condition) or not self.move_flame:
                # Store flame position and size, hotzone size, and mean waist radius
                if i < len(self.fpos_arr):
                    self.fpos_arr[i] = pos_1
                    self.hz_arr[i] = hz1
                    self.fsize_arr[i] = flame_size
                else:
                    self.fpos_arr = np.concatenate([self.fpos_arr, [pos_1]])
                    self.fsize_arr = np.concatenate([self.fsize_arr, [flame_size]])
                    self.hz_arr = np.concatenate([self.hz_arr, [hz1]])
                
                # Get indexes of the flame limits, before and after moving 
                jfl = np.abs(self.z_arr - (np.min([pos_1, prev_pos]) - flame_size/2.0)).argmin()
                jfr = np.abs(self.z_arr - (np.max([pos_1, prev_pos]) + flame_size/2.0)).argmin() + 1
                flame_n = jfr - jfl
                
                # Smooth out hot zone
                if (not self.move_flame) or self.smooth_inside_flame:
                    r_arr[jfl:jfr] = np.mean(r_arr[jfl:jfr])

                # Decrease radius and create one transition point at each side
                if self.heat_reach == 0.0 or (not self.move_flame):
                    decrease_factors = np.ones(flame_n)*np.sqrt(delta_x/(delta_x + 2*dx/flame_n))  # Simple volume conservation
                    jl = jfl
                    jr = jfr
                else:
                    # Gaussian decaying reach beyond flame
                    gauss_l = np.exp(-((self.z_arr[:jfl] - self.z_arr[jfl])**2)/(2*(self.heat_reach**2)))
                    gauss_r = np.exp(-((self.z_arr[jfr + 1:] - self.z_arr[jfr])**2)/(2*(self.heat_reach**2)))
                    jl = np.abs(gauss_l - self.min_heat_reach).argmin()
                    jr = -np.abs(np.flip(gauss_r) -  self.min_heat_reach).argmin()
                    heat_function = np.concatenate([gauss_l[jl:], np.ones(flame_n), gauss_r[:jr + 1]])
                    effective_n = flame_n + np.sum(gauss_l[jl:]) + np.sum(gauss_r[:jr + 1])  # Effective number of dxs
                    decrease_factors = np.sqrt(delta_x/(delta_x + heat_function*2*dx/effective_n))  # Decaying volume conservation

                r_left = r_arr[0:jl]
                r_right = r_arr[jr:]
                r_arr[jl:jr] = r_arr[jl:jr]*decrease_factors
                r_left = np.roll(r_left, -1)
                r_left[-1] = (r_arr[jl - 1] + r_arr[jl])/2.0
                r_right = np.roll(r_right, 1)
                r_right[0] = (r_arr[jr - 1] + r_arr[jr])/2.0

                r_arr[0:jl] = r_left
                r_arr[jr:] = r_right

                # Store result (not all frames, reduced by the defined factor)
                if i % self.reduce_frames_factor == 0:
                    idx = int(i/self.reduce_frames_factor)
                    if idx <  len(self.r_arrs):
                        self.r_arrs[idx] = r_arr
                    else:
                        self.r_arrs = np.concatenate([self.r_arrs, [r_arr]])
                
                # Change flame/hotzone
                total_pulled = 2*dx*(i)               
                hz1 = np.interp(total_pulled, hz_x, hz_y)

                i += 1
                if i >= steps:
                    if self.end_brush_at_edge and self.move_flame:
                        last_sweep = True
                    else:
                        done = True
            
            if self.move_flame:  
                # Check if flame reached end of hotzone (reverse direction)
                hz_l = self.pos_0 - hz1/2 + compensate_l
                hz_r = self.pos_0 + hz1/2 - compensate_l
                
                # Calculate flame velocity
                cur_vf += af*self.time_step
                if af >= 0:
                    if cur_vf > self.vf: cur_vf = self.vf
                else:
                    if cur_vf < -self.vf: cur_vf = -self.vf
                dx_f = cur_vf*self.time_step

                if self.predict_reverse_direction and (not self.turn_off_at_edges):
                    decel_t = np.abs(cur_vf/af)
                    decel_x = np.abs(af)*(decel_t**2)/2
                    decel_steps = np.round(decel_t/dt)
                    decel_total_pulled = 2*dx*(i + decel_steps)
                    decel_hz = np.interp(decel_total_pulled, hz_x, hz_y)
                    decel_hz_l = self.pos_0 - decel_hz/2 + compensate_l
                    decel_hz_r = self.pos_0 + decel_hz/2 - compensate_l
                    
                    if last_sweep:
                        if (af > 0 and (pos_1 >= hz_r)) or (af < 0 and (pos_1 <= hz_l)):
                            done = True
                    else:
                        if (af > 0 and (pos_1 + decel_x >= decel_hz_r)):
                            af = -self.af0
                        elif (af < 0 and (pos_1 - decel_x <= decel_hz_l)):
                            af = self.af0
                else:
                    if (af > 0 and (pos_1 >= hz_r)) or (af < 0 and (pos_1 <= hz_l)):
                        af = -af
                        if last_sweep:
                            done = True

                # Move flame
                prev_pos = pos_1
                pos_1 += dx_f
            
            else:
                # If the flame grows, the hotzone is the flamesize, and position remains the same
                flame_size = hz1
                dx_f = 0
            if flame_size < self.min_flame_size:
                flame_size = self.min_flame_size

            # Print progress each second
            t2 = time.time() - t1
            if t2 >= 1.0:
                ratio = float(i)/float(steps)
                t1 = time.time()
                t_tot = t1 - t0
                t_left = t_tot/ratio - t_tot
                print(f"Simulation progress: {(100*i/steps):.1f}%. {t_left:.0f} s remaining      ", end="\r")
        print("Simulation done                                           ")
        self.r_arrs[-1] = r_arr  # Safety
        
        return self.z_arr, self.r_arrs, total_pulled
    
    def show_last_sim_animation(self, smooth_window: float=0.02):
        smooth_w = int(smooth_window*len(self.r_arrs[0]))
        
        plt.ion()
        fig1, ax1 = plt.subplots(1, 1)
        line1, = ax1.plot(self.z_arr, self.r_arrs[0]*1000.0, color='tab:blue')
        line2, = ax1.plot(self.z_arr, -self.r_arrs[0]*1000.0, color='tab:blue')
        line3, = ax1.plot([self.fpos_arr[0] - self.fsize_arr[0]/2, self.fpos_arr[0] - self.fsize_arr[0]/2], 
                          [-self.r0*1000, self.r0*1000], color="tab:red")
        line4, = ax1.plot([self.fpos_arr[0] + self.fsize_arr[0]/2, self.fpos_arr[0] + self.fsize_arr[0]/2],
                          [-self.r0*1000, self.r0*1000], color="tab:red")
        line5, = ax1.plot([self.pos_0 - self.hz_arr[0]/2, self.pos_0 - self.hz_arr[0]/2], 
                          [-self.r0*1000, self.r0*1000], '--', color="tab:green")
        line6, = ax1.plot([self.pos_0 + self.hz_arr[0]/2, self.pos_0 + self.hz_arr[0]/2], 
                          [-self.r0*1000, self.r0*1000], '--', color="tab:green")

        t0 = time.time()
        for i in range(len(self.r_arrs)):
            r_data = savgol_filter(self.r_arrs[i], smooth_w, 1)
            line1.set_ydata(r_data*1000.0)
            line2.set_ydata(-r_data*1000.0)
            
            j = int(i*self.reduce_frames_factor)
            if j >= len(self.fpos_arr): 
                j = len(self.fpos_arr) - 1
            line3.set_xdata([self.fpos_arr[j] - self.fsize_arr[j]/2, self.fpos_arr[j] - self.fsize_arr[j]/2])
            line4.set_xdata([self.fpos_arr[j] + self.fsize_arr[j]/2, self.fpos_arr[j] + self.fsize_arr[j]/2])
            line5.set_xdata([self.pos_0 - self.hz_arr[j]/2, self.pos_0 - self.hz_arr[j]/2])
            line6.set_xdata([self.pos_0 + self.hz_arr[j]/2, self.pos_0 + self.hz_arr[j]/2])
            fig1.canvas.draw()
            fig1.canvas.flush_events()
            if not plt.fignum_exists(fig1.number):
                break
        print(f"Animation FPS: {(len(self.r_arrs)/(time.time() - t0)):.2f}")
        plt.ioff()
        
        r_data = savgol_filter(self.r_arrs[-1], smooth_w, 1)
        line1.set_ydata(r_data*1000.0)
        line2.set_ydata(-r_data*1000.0)
        fig1.canvas.draw()
        fig1.canvas.flush_events()
        plt.show()