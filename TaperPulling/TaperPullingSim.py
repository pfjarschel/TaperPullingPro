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
from enum import Enum


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
        if steps/reduce_frames_factor > self.max_frames:
            reduce_frames_factor = int(steps/self.max_frames)
        pos_0 = 0.0  # Flame initial position (middle)
        pos_1 = pos_0  # Flame position
        prev_pos = pos_0  # Flame previous position
        hz1 = hz0  # Hot Zone initial size
        flame_size = self.min_flame_size  # Flame initial size
        cur_vf = 0  # Current flame velocity
        af = self.af0  # Current acceleration
        x_array = np.linspace(-fiber_l/2.0, fiber_l/2.0, pts)  # Position array
        delta_x = (x_array[-1] - x_array[0])/(pts - 1)
        r_array = self.r0*np.ones(pts)  # Radius array
        r_array_time = np.zeros((int(steps/reduce_frames_factor) + 1, pts))  # Array to keep all r arrays
        tpulled_array = np.zeros((steps))  # Array to keep total pulled values
        r_array_time[0] = r_array
        tpulled_array[0] = 0.0
        compensate_l = self.compensate_flame_size*(flame_size)/2 
        
        t0 = time.time()
        t1 = t0
        i = 1
        done = False
        last_sweep = False
        
        while not done:
            if not self.turn_off_at_edges or (self.turn_off_at_edges and (np.abs(cur_vf) == self.vf)) or not self.move_flame:
                # Get indexes of the flame limits, before and after moving 
                jfl = np.abs(x_array - (np.min([pos_1, prev_pos]) - flame_size/2.0)).argmin()
                jfr = np.abs(x_array - (np.max([pos_1, prev_pos]) + flame_size/2.0)).argmin() + 1
                flame_n = jfr - jfl
                
                # Smooth out hot zone
                if (not self.move_flame) or self.smooth_inside_flame:
                    r_array[jfl:jfr] = np.mean(r_array[jfl:jfr])

                # Decrease radius and create one transition point at each side
                if self.heat_reach == 0.0 or (not self.move_flame):
                    decrease_factors = np.ones(flame_n)*np.sqrt(delta_x/(delta_x + 2*dx/flame_n))  # Simple volume conservation
                    jl = jfl
                    jr = jfr
                else:
                    # Gaussian decaying reach beyond flame
                    gauss_l = np.exp(-((x_array[:jfl] - x_array[jfl])**2)/(2*(self.heat_reach**2)))
                    gauss_r = np.exp(-((x_array[jfr + 1:] - x_array[jfr])**2)/(2*(self.heat_reach**2)))
                    jl = np.abs(gauss_l - self.min_heat_reach).argmin()
                    jr = -np.abs(np.flip(gauss_r) -  self.min_heat_reach).argmin()
                    heat_function = np.concatenate([gauss_l[jl:], np.ones(flame_n), gauss_r[:jr + 1]])
                    effective_n = flame_n + np.sum(gauss_l[jl:]) + np.sum(gauss_r[:jr + 1])  # Effective number of dxs
                    decrease_factors = np.sqrt(delta_x/(delta_x + heat_function*2*dx/effective_n))  # Decaying volume conservation

                r_left = r_array[0:jl]
                r_right = r_array[jr + 1:]
                r_array[jl:jr] = r_array[jl:jr]*decrease_factors
                r_left = np.roll(r_left, -1)
                r_left[-1] = (r_array[jl - 1] + r_array[jl])/2.0
                r_right = np.roll(r_right, 1)
                r_right[0] = (r_array[jr] + r_array[jr - 1])/2.0

                r_array[0:jl] = r_left
                r_array[jr+1:] = r_right

                # Store result (not all frames, reduced by the defined factor)
                if i % reduce_frames_factor == 0:
                    if not last_sweep:
                        r_array_time[int(i/reduce_frames_factor)] = r_array
                    else:
                        r_array_time = np.concatenate([r_array_time, [r_array]])
                
                # Change flame/hotzone
                total_pulled = 2*dx*(i)
                if not last_sweep:
                    tpulled_array[i] = total_pulled
                else:
                    tpulled_array = np.concatenate([tpulled_array, [total_pulled]])
                
                hz1 = np.interp(total_pulled, hz_x, hz_y)

                i += 1
                if i >= steps:
                    if self.end_brush_at_edge:
                        last_sweep = True
                    else:
                        done = True
            
            if self.move_flame:  
                # Check if flame reached end of hotzone (reverse direction)
                hz_l = pos_0 - hz1/2 + compensate_l
                hz_r = pos_0 + hz1/2 - compensate_l
                
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
                    decel_hz_l = pos_0 - decel_hz/2 + compensate_l
                    decel_hz_r = pos_0 + decel_hz/2 - compensate_l
                    
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
        
        return tpulled_array, r_array_time