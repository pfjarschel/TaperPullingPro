# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# First release: May 31, 2024
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
This module contains a basic example to run the taper fabrication routine using
the TaperPullingCore module, using all the default parameters. A hotzone
function (2 point line) is the only input.
Change the simulate=True lines to False to actually use this in the real setup.
"""
# =============================================================================

# Imports
import time
import numpy as np
import matplotlib.pyplot as plt
from TaperPulling.TaperPullingCore import TaperPullingCore
from TaperPulling.TaperPullingData import TaperPullingData

# Initialize core
core = TaperPullingCore()
if core.init_all_motors_as_default(simulate=True):
    core.start_update()

# Initialize Data acquisition and processing
data_manager = TaperPullingData()
data_manager.set_sampling_rate(1e3)
data_manager.init_daq_as_default(simulate=True)

# Lists to hold data
b_pos = []
fio_pos = []
lp_pos = []
rp_pos = []
total_pulled = []
times = []
vals = []

# Hotzone function: a 2 point line
# Will pull 20 mm (10 on each side)
# Hot zone will grow from 2 mm to 10 mm
hz_function = [[0.0, 20.0], [2.0, 10.0]]

# Initialize list to hold measured hz lengths
hz_points = [[0.0, hz_function[1][0]]]
hz = np.array(hz_points).T

# Start monitoring
data_manager.start_monitor()
print("Homing...")
core.motors.brusher.home()
core.motors.flame_io.home()
core.motors.left_puller.home()
core.motors.right_puller.home()

core.motors.left_puller.wait_for_home()
core.motors.right_puller.wait_for_home()
core.motors.flame_io.wait_for_home()
core.motors.brusher.wait_for_home()
print("Homed")

core.brusher_x0 = 33.0
core.flame_io_x0 = 14.1
core.left_puller_x0 = 84.0
core.right_puller_x0 = 84.0

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

# Monitor process
t0 = time.time()
i = 0
tsleep = 0.1
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
    
    # Just some progress info
    if i % int(1/tsleep) == 0 and core.total_pulled > 0.0:
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
