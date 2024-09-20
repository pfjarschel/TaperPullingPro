# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains a more comples example to run the taper fabrication routine
using the TaperPullingCore and TaperPullingData modules, using a hotzone function
obtained by the TaperShape module.
The process is monitored using the TaperPulling Data and the TaperPullingSim
modules. 
Change the simulate=True lines to False to actually use this in the real setup.
The parameters used are the standard for SMF-28. For other values, the TaperShape
module needs to calculate the effective indexes, which can take some time. 
"""
# =============================================================================

# Imports
import time
import numpy as np
import matplotlib.pyplot as plt
from TaperPulling.TaperShape import TaperShape
from TaperPulling.TaperPullingSim import TaperPullingSim
from TaperPulling.TaperPullingCore import TaperPullingCore
from TaperPulling.TaperPullingData import TaperPullingData
from scipy.signal import savgol_filter


# Initialize design modules
shape_man = TaperShape()  # Defaults
sim_man = TaperPullingSim()

# Design parameters
wl = 1.55  # Wavelength, µm
r0 = 62.5e-3  # Fiber radius, mm
r_core = 4.1e-3  # Radius core, mm
n_core_ratio = 1.0036  # Core to cladding refractive index ratio
n_medium = 1.0  # Usually air
min_l = 2  # Minimum hotzone size, mm
flame_size = 1.0  # Estimated flame size, mm
rw = 0.4e-3  # Desired final radius, mm
lw = 3  # Desired waist length, mm
f = 0.9  # Adiabaticity factor. 0 to 1. Above 1 it is NOT adiabatic.

# Calculate ideal profile
z_arr, r_arr = shape_man.real_adiabatic_profile(rw, min_l, f)
x_arr, l_arr = shape_man.hz_from_profile(z_arr, r_arr, lw)
print(f"Total to pull: {x_arr[-1]:.2f} mm.")
print(f"Estimated time: {0.5*x_arr[-1]/sim_man.vp:.2f} s.")

# Create hotzone function
hz_function = np.array([x_arr, l_arr])

# Set up fabrication simulation with real values
sim_man.vp = 0.125  # Pulling speed, mm/s
sim_man.vf = 2.0  # Flame brushing speed, mm/s
sim_man.af0 = 45  # Flame acceleration, mm/s2
sim_man.min_flame_size = flame_size
sim_man.compensate_flame_size = True
sim_man.predict_reverse_direction = True
sim_man.max_frames = 10000
sim_man.reduce_frames_factor = 1

# Simulate fabrication in advance
sim_z_arr, sim_r_arrs, sim_total_pulled = sim_man.simulate(hz_function)
sim_total_pulled_arr = np.linspace(0.0, sim_total_pulled, len(sim_r_arrs))
# Smooth the resulting profile a little
final_shape = savgol_filter(sim_r_arrs[-1], int(len(sim_r_arrs[-1])/50), 1)

# Initialize core
core = TaperPullingCore()
core.init_brusher_as_default(simulate=True)
core.init_flameio_as_default(simulate=True)
core.init_puller_l_as_default(simulate=True)
core.init_puller_r_as_default(simulate=True)


# Initialize Data acquisition and processing
data_manager = TaperPullingData()
data_manager.set_sampling_rate(1e3)
data_manager.init_daq_as_default(simulate=True)

# Lists to hold fabrication data
b_pos = []
fio_pos = []
lp_pos = []
rp_pos = []
total_pulled = []
times = []
vals = []

# Initialize list to hold measured hz lengths
hz_points = [[0.0, hz_function[1][0]]]
hz = np.array(hz_points).T

# Start monitoring
data_manager.start_monitor()

# Start process
core.start_process(hz_function)
while not core.pulling:  # Wait for pulling to start before plotting
    time.sleep(0.1)

# Set up plots
plt.ion()  # To dinamically update plot
figure, (ax, ax2) = plt.subplots(1, 2)
b_line, = ax.plot(total_pulled, b_pos, label="Brusher")
fio_line, = ax.plot(total_pulled, fio_pos, label="In-Out")
lp_line, = ax.plot(total_pulled, lp_pos, label="Left")
rp_line, = ax.plot(total_pulled, rp_pos, label="Right")
hz_line, = ax.plot(hz_function[0], hz_function[1], label="Intended HZ")
rhz_line, = ax.plot(hz[0], hz[1], 'o', label="Real HZ")
vals_line, = ax2.plot(total_pulled, vals)
vals_dummy_line, = ax2.plot(hz_function[0], np.zeros(len(hz_function[0])))
vals_dummy_line.set_visible(False)
ax.set_xlabel("Total Pulled (mm)")
ax.set_ylabel("Position (mm)")
ax2.set_xlabel("Total Pulled (mm)")
ax2.set_ylabel("Transmission (a.u.)")
ax.legend(loc='upper right')
figure.tight_layout()

fig_sim, ax_sim = plt.subplots(1, 1)
line_sim_top, = ax_sim.plot(sim_z_arr, sim_r_arrs[0]*1000.0, color='tab:blue')
line_sim_bot, = ax_sim.plot(sim_z_arr, -sim_r_arrs[0]*1000.0, color='tab:blue')
# line_sim_fl, = ax_sim.plot([-flame_size/2, -flame_size/2], 
#                     [-r0*1000, r0*1000], color="tab:red")
# line_sim_fr, = ax_sim.plot([flame_size/2, flame_size/2],
#                     [-r0*1000, r0*1000], color="tab:red")
# line_sim_hzl, = ax_sim.plot([-hz_function[1][0]/2, -hz_function[1][0]/2], 
#                     [-r0*1000, r0*1000], '--', color="tab:green")
# line_sim_hzr, = ax_sim.plot([hz_function[1][0]/2, hz_function[1][0]/2], 
#                     [-r0*1000, r0*1000], '--', color="tab:green")


# Monitor process
t0 = time.time()
last_t = t0
i = 0
tsleep = 0.001
prev_b_dir_change = core.brusher_x0
prev_b_dir = core.brusher_dir
while not core.standby:
    # Get data
    b_pos.append(core.motors.brusher.pos)
    fio_pos.append(core.motors.flame_io.pos)
    lp_pos.append(core.motors.left_puller.pos)
    rp_pos.append(core.motors.right_puller.pos)
    vals.append(data_manager.get_last_monitor())
    total_pulled.append(core.total_pulled)
    times.append(time.time() - t0)
    
    # Measure hotzone (at each change of direction)
    if i > 1:
        if core.brusher_dir != prev_b_dir:
            real_hz = np.abs(b_pos[-2] - prev_b_dir_change)
            prev_b_dir_change = b_pos[-2]
            hz_points.append([core.total_pulled, real_hz])
            prev_b_dir = core.brusher_dir
            hz = np.array(hz_points).T
    
    # Update plots   
    b_line.set_xdata(total_pulled)
    b_line.set_ydata(b_pos)
    fio_line.set_xdata(total_pulled)
    fio_line.set_ydata(fio_pos)
    lp_line.set_xdata(total_pulled)
    lp_line.set_ydata(lp_pos)
    rp_line.set_xdata(total_pulled)
    rp_line.set_ydata(rp_pos)
    rhz_line.set_xdata(hz[0])
    rhz_line.set_ydata(hz[1])
    vals_line.set_xdata(total_pulled)
    vals_line.set_ydata(vals)
    ax.relim()
    ax.autoscale()
    ax2.relim()
    ax2.autoscale()
    figure.canvas.draw()
    figure.canvas.flush_events()
    
    
    frame_idx = np.abs(sim_total_pulled_arr - core.total_pulled).argmin()
    # instant_hz = np.interp(core.total_pulled, hz_function[0], hz_function[1])
    line_sim_top.set_ydata(savgol_filter(sim_r_arrs[frame_idx], int(len(sim_r_arrs[frame_idx])/50), 1)*1000.0)
    line_sim_bot.set_ydata(-savgol_filter(sim_r_arrs[frame_idx], int(len(sim_r_arrs[frame_idx])/50), 1)*1000.0)
    # line_sim_fl.set_xdata([b_pos[-1] - core.brusher_x0 - flame_size/2, b_pos[-1] - core.brusher_x0 - flame_size/2])
    # line_sim_fr.set_xdata([b_pos[-1] - core.brusher_x0 + flame_size/2, b_pos[-1] - core.brusher_x0 + flame_size/2])
    # line_sim_hzl.set_ydata([b_pos[-1] - core.brusher_x0 - instant_hz/2, b_pos[-1] - core.brusher_x0 - instant_hz/2])
    # line_sim_hzr.set_ydata([b_pos[-1] - core.brusher_x0 + instant_hz/2, b_pos[-1] - core.brusher_x0 + instant_hz/2])
    ax_sim.relim()
    ax_sim.autoscale()
    fig_sim.canvas.draw()
    fig_sim.canvas.flush_events()
    
    # Just some progress info
    if time.time() - last_t >= 1.0:
        last_t = time.time()
        el_time = time.time() - t0
        ratio = core.total_pulled/hz_function[0][-1]
        total_time = el_time/ratio
        etr = total_time - el_time
        print("                                         ", end='\r', flush=True)
        if etr > 1.0:
            print(f"{(100*ratio):.2f}%, ETR: {etr:.2f} s", end='\r', flush=True)
    i += 1
    time.sleep(tsleep)

# Show final plot and block execution
plt.ioff()
plt.show()  

# Shut down
data_manager.stop_monitor()
core.stop_pulling()
core.close()
