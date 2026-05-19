#
# Simple PID controller for a serial-controlled water pump experiment
#
# This application communicates over a USB serial interface to an Arduino, which in turn
# controls the pump motor and reads out the current water level.
#
# The serial protocol is as follows:
# - The application sends a byte (value 0-255) to set the pump speed (S)
# It then receives a byte (value 0-255) corresponding to the current water level (L).
#
# The graphical interface includes sliders for setting:
# - The target water level (T)
# - The controller coefficients (P, I, and D)
#
# After receiving a new value of L from the pump system, the application updates three
# internal floating-point variables: DIFF, INTEGRAL and DERIVATIVE:
#
# DIFF = T-L  
# INTEGRAL = INTEGRAL+DIFF (cumulative sum of DIFF, which can be manually reset to zero by the user
# DERIVATIVE is the difference between the current and previous values of DIFF
#
# These variables are multipled by their respective coefficients to calculate
# a new value of S to set the pump speed:
#
# S_DIF = P*DIFF
# S_INT = I*INTEGRAL 
# S_DRV = D*DERIVATIVE  
# S = integer(S_DIF+S_INT+S_DRV)  
#
# The following libraries are required to run the application:
# pip install pyserial matplotlib
#

import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import time
import serial
import serial.tools.list_ports
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
plt.rcParams.update({'font.size': 5})

SERIAL_AVAILABLE = False


def find_arduino():
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if "Arduino" in port.description or "USB" in port.description:
            return port.device
    
    return None

arduino_port = find_arduino()

if arduino_port:
    print(f"Found Arduino on {arduino_port}")
    SERIAL_AVAILABLE = True
else:
    print("Arduino not found")

# -----------------------
# CONFIG
# -----------------------
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
UPDATE_INTERVAL = 0.5

# -----------------------
# PID STATE
# -----------------------
INTEGRAL = 0.0
PREV_DIFF = 0.0
PREV_L = 0.0
running = False

# Simulation state
sim_level = 50.0

# History
history_L = deque(maxlen=150)
history_S = deque(maxlen=150)
history_SDIF = deque(maxlen=150)
history_SINT = deque(maxlen=150)
history_SDRV = deque(maxlen=150)

# -----------------------
# SERIAL SETUP
# -----------------------
ser = None
if SERIAL_AVAILABLE:
    try:
        ser = serial.Serial(arduino_port, BAUD_RATE, timeout=UPDATE_INTERVAL)
    except:
        ser = None

# -----------------------
# GUI SETUP
# -----------------------
root = tk.Tk()
root.title("Pump PID Controller")

T_var = tk.DoubleVar(value=400)
P_var = tk.DoubleVar(value=1.0)
I_var = tk.DoubleVar(value=0.0)
D_var = tk.DoubleVar(value=0.0)
Lmax_var = tk.DoubleVar(value=600)
sim_mode_var = tk.BooleanVar(value=True)
pause_mode_var = tk.BooleanVar(value=False)

prev_sim_mode = True

# -----------------------
# UI HELPERS
# -----------------------
def create_slider(label, var, from_, to):
    frame = ttk.Frame(root)
    frame.pack(fill='x', padx=5, pady=2)

    ttk.Label(frame, text=label, width=18).pack(side='left')
    ttk.Scale(frame, from_=from_, to=to, variable=var).pack(side='left', fill='x', expand=True)
    ttk.Label(frame, textvariable=var, width=6).pack(side='right')

create_slider("Target level (T)", T_var, 0, 800)
create_slider("Max water level", Lmax_var, 0, 800)
create_slider("P Coefficent", P_var, 0, 10)
create_slider("I Coefficent", I_var, 0, 4)
create_slider("D Coefficent", D_var, 0, 4)


# -----------------------
# Toggle buttons
# -----------------------

# Create a frame to hold the checkbuttons
frame = ttk.Frame(root)
frame.pack(padx=25, pady=5)

# -----------------------
# MODE TOGGLE
# -----------------------
def toggle_mode():
    if sim_mode_var.get():
        mode_label.config(text="SIMULATION MODE", foreground="blue")
    else:
        mode_label.config(text="HARDWARE MODE", foreground="black")

ttk.Checkbutton(frame, text="Simulation Mode",
                variable=sim_mode_var,
                command=toggle_mode).pack(side=tk.LEFT,padx=10,pady=5)



# -----------------------
# PAUSE TOGGLE
# -----------------------
def toggle_pause():
    if pause_mode_var.get():
        pause_label.config(text="GRAPH PAUSED", foreground="blue")
    else:
        pause_label.config(text="GRAPH RUNNING", foreground="green")

ttk.Checkbutton(frame, text="Pause graph",
                variable=pause_mode_var,
                command=toggle_pause).pack(side=tk.LEFT,padx=10,pady=2)

# Create a frame to hold the checkbutton status
frame2 = ttk.Frame(root)
frame2.pack(padx=25, pady=2)


mode_label = ttk.Label(frame2, text="SIMULATION MODE", foreground="blue")
mode_label.pack(side=tk.LEFT,padx=10,pady=2)
pause_label = ttk.Label(frame2, text="GRAPH RUNNING", foreground="green")
pause_label.pack(side=tk.LEFT,padx=10,pady=2)



# -----------------------
# BUTTONS
# -----------------------
def toggle_running():
    global running
    running = not running
    btn_start.config(text="Stop PID control" if running else "Start PID control")
    status_label.config(
        text="PID CONTROLLER RUNNING" if running else "PID CONTROLLER NOT RUNNING",
        foreground="green" if running else "red"
    )

def reset_integral():
    global INTEGRAL
    INTEGRAL = 0.0


ttk.Button(root, text="Reset Integral", command=reset_integral)\
    .pack(fill='x', padx=5, pady=2)

btn_start = ttk.Button(root, text="Start PID control", command=toggle_running)
btn_start.pack(fill='x', padx=5, pady=2)


status_label = ttk.Label(root, text="PID CONTROLLER NOT RUNNING", foreground="red")
status_label.pack(pady=2)

# -----------------------
# DISPLAYS
# -----------------------
labels = {}

def create_display(parent, name, row, col):
    frame = ttk.Frame(parent)
    frame.grid(row=row, column=col, padx=10, pady=2, sticky="w")

    ttk.Label(frame, text=name + ":", width=18).pack(side='left')
    val = ttk.Label(frame, text="0")
    val.pack(side='right')
    labels[name] = val


container = ttk.Frame(root)
container.pack(padx=10, pady=10)

#names = ["DIFF", "S_DIF", "INTEGRAL",
#         "S_INT", "DERIVATIVE", "S_DRV", "L", "S"]

names = ["DIFF", "S_P", "INTEGRAL",
         "S_I", "DERIVATIVE", "S_D", "L", "S_TOT"]

    
for i, name in enumerate(names):
    row = i // 2      # integer division → new row every 2 items
    col = i % 2       # 0 or 1 → left or right column
    create_display(container, name, row, col)

# -----------------------
# PLOT SETUP (SIMPLE)
# -----------------------

fig, ax = plt.subplots(1,3,figsize=(7, 3),gridspec_kw={'width_ratios': [3, 2, 2]} )
#fig.tight_layout()

line, = ax[0].plot([], [], marker='o', markersize=1, linestyle='-', label="Level (L)")
target_line, = ax[0].plot([], [], linestyle='dashed', linewidth=1.5, label="Target (T)")
limit_line, = ax[0].plot([], [], 'r', linestyle='dashed', linewidth=1.5, label="Max level")
ax[0].set_ylim(0, 1023)
ax[0].set_title("Water Level (L)")
ax[0].legend()

line2, = ax[1].plot([], [], marker='o', markersize=1, linestyle='-', label="Level (L)")
target_line2, = ax[1].plot([], [], linestyle='dashed', linewidth=1.5, label="Target (T)")
ax[1].set_ylim(0, 1023)
ax[1].set_title("Water Level (L)")
ax[1].grid(True)

S_line, = ax[2].plot([], [], 'b', label="S_TOT")
S_DIFF_line, = ax[2].plot([], [], 'r',linestyle='dashed', linewidth=1.5, label="S_P")
S_INTG_line, = ax[2].plot([], [], 'c',linestyle='dashed', linewidth=1.5, label="S_I")
S_DERV_line, = ax[2].plot([], [], 'y', linestyle='dashed', linewidth=1.5, label="S_D")
ax[2].set_ylim(-200, 1010)
ax[2].set_title("Pump setting (S)")
ax[2].legend()
ax[2].grid(True)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill='both', expand=True)

# -----------------------
# SIMULATION MODEL
# -----------------------
def simulate_step(S):
    global sim_level

    pump_gain = 1.2
    leak = 0.45
    noise = 0   # Ignore noise in the simulation for now

    inflow = pump_gain * S
    outflow = leak * sim_level

    sim_level += inflow - outflow

    import random
    sim_level += random.uniform(-noise, noise)

    sim_level = max(0, min(1000, sim_level))
    return int(sim_level)

# -----------------------
# CONTROL LOOP
# -----------------------
def control_loop():
    global INTEGRAL, PREV_DIFF, PREV_L
    global prev_sim_mode, ser, sim_level

    S = 0  # keep last output for simulation
    next_time = time.time()
    while True:
        next_time += UPDATE_INTERVAL
        current_mode = sim_mode_var.get()

        if current_mode != prev_sim_mode:
            print("Switching mode")

            # -----------------------
            # SWITCH → SIMULATION
            # -----------------------
            if current_mode:
                print("Entering SIMULATION mode")

                # Turn off pump safely
                if ser:
                    try:
                        ser.write(b"0\n")
                    except:
                        pass

                # Close serial
                if ser:
                    ser.close()
                    ser = None

                # Reset simulation + PID
                sim_level = 300.0
                INTEGRAL = 0.0
                PREV_DIFF = 0.0

            # -----------------------
            # SWITCH → HARDWARE
            # -----------------------
            else:
                print("Entering HARDWARE mode")

                try:
                    port = find_arduino() or SERIAL_PORT
                    ser = serial.Serial(port, BAUD_RATE, timeout=UPDATE_INTERVAL)
                    print(f"Connected to {port}")
                except Exception as e:
                    print("Failed to connect:", e)
                    ser = None

                # Reset PID (important)
                INTEGRAL = 0.0
                PREV_DIFF = 0.0

            prev_sim_mode = current_mode

        try:
            # --- Get L ---
            if current_mode:
                L = simulate_step(S if running else 0)
            else:
                #print("writing:", S)

                if not ser:
                    print("No serial connection")
                    L = history_L[-1] if history_L else 0
                else:
                    ser.write(f"{S}\n".encode('utf-8'))
                    line = ser.readline()
                    if not line:
                        print("No data received")
                        L = history_L[-1] if history_L else 0
                    else:
                        try:
                            L = int(line.strip())
                        except ValueError:
                            print("Bad data:", line)
                            L = history_L[-1] if history_L else 0

            T = T_var.get()
            P = P_var.get()
            I = I_var.get()
            D = D_var.get()
            Lmax = Lmax_var.get()

            T = min (T, Lmax)
            
            if running:
                DIFF = T - L

                INTEGRAL += DIFF
                # DERIVATIVE = DIFF - PREV_DIFF
                DERIVATIVE = L - PREV_L

                INTEGRAL = max(-50000, min(50000, INTEGRAL))

                S_DIF = P * DIFF
                S_INT = I * INTEGRAL
                S_DRV = -1 * D * DERIVATIVE

                S = int(S_DIF + S_INT + S_DRV)

                if L >= Lmax:
                    S = 0

                S = max(0, min(1000, S))
                # PREV_DIFF = DIFF
                PREV_L = L

            else:
                DIFF = 0
                DERIVATIVE = 0
                S_DIF = S_INT = S_DRV = 0
                S = 0

            history_L.append(L)
            history_S.append(S)
            history_SDIF.append(S_DIF)
            history_SINT.append(S_INT)
            history_SDRV.append(S_DRV)

            root.after(0, update_gui,
                       DIFF, INTEGRAL, DERIVATIVE,
                       S_DIF, S_INT, S_DRV, S, L, T)

        except Exception as e:
            print("Error:", e)

        # Wait until next cycle
            

        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            # we're lagging behind
            next_time = time.time()

# -----------------------
# GUI UPDATE
# -----------------------
def update_gui(DIFF, INTEGRAL_val, DERIVATIVE,
               S_DIF, S_INT, S_DRV, S, L, T):

    labels["DIFF"].config(text=f"{DIFF:.2f}")
    labels["INTEGRAL"].config(text=f"{INTEGRAL_val:.2f}")
    labels["DERIVATIVE"].config(text=f"{DERIVATIVE:.2f}")

    labels["S_P"].config(text=f"{S_DIF:.2f}")
    labels["S_I"].config(text=f"{S_INT:.2f}")
    labels["S_D"].config(text=f"{S_DRV:.2f}")
    labels["S_TOT"].config(text=f"{S}")

    labels["L"].config(text=f"{L}")

    x_data = range(len(history_L))
    y_data = list(history_L)
    x_data2 = x_data[-30:]
    y_data2 = y_data[-30:]


    line.set_data(x_data, y_data)
    line2.set_data(x_data2, y_data2)

    S_data = list(history_S)
    SDIF_data = list(history_SDIF)
    SINT_data = list(history_SINT)
    SDRV_data = list(history_SDRV)
    
    S_line.set_data(x_data2, S_data[-30:])
    S_DIFF_line.set_data(x_data2, SDIF_data[-30:])
    S_INTG_line.set_data(x_data2, SINT_data[-30:])
    S_DERV_line.set_data(x_data2, SDRV_data[-30:])
    
    # Set X limits first
    xmin = 0
    xmax = max(50, len(history_L))
    ax[0].set_xlim(xmin, xmax)

    
    xmin2 = min(x_data2)
    xmax2 = max(20, xmax)
    ax[1].set_xlim(xmin2, xmax2)
    ax[1].xaxis.set_ticks(np.arange(xmin2, xmax2+1, 5.0))

    ax[2].set_xlim(xmin2, xmax2)
    
    # --- TARGET LINE ---
    # T = T_var.get()
    LIMIT = Lmax_var.get()

    target_line.set_data([xmin, xmax], [T, T])
    limit_line.set_data([xmin, xmax], [LIMIT, LIMIT])
    target_line2.set_data([xmin2, xmax2], [T, T])
  
    if history_L:
        ymin_data = min(history_L)
        ymax_data = max(history_L)
        ymin_data2 = min(y_data2)
        ymax_data2 = max(y_data2)
    else:
        ymin_data = T
        ymax_data = T
        ymin_data2 = T
        ymax_data2 = T

    ymin = min(ymin_data, T) - 30
    ymax = max(ymax_data, T) + 30

    #ymin2 = min(ymin_data2, T) - 15
    #ymax2 = max(ymax_data2, T) + 15

    ymin2 = ymin_data2 - 15
    ymax2 = ymax_data2 + 15

    
    # Clamp to valid range
    ymin = max(0, ymin)
    ymax = min(1023, ymax)

    ymin2 = max(0, ymin2)
    ymax2 = min(1023, ymax2)
    ymax3 = max(S_data[-20:])+50
    ymin3 = min(-20,(min(SDIF_data[-20:])-10),(min(SINT_data[-10:])-20),(min(SDRV_data[-10:])-20))

    # Prevent collapse
    if ymax - ymin < 10:
        ymax = ymin + 10
    if ymax2 - ymin2 < 10:
        ymax2 = ymin2 + 10
    
    ax[0].set_ylim(ymin, ymax)
    ax[1].set_ylim(ymin2, ymax2)
    ax[2].set_ylim(ymin3, ymax3)
    
    if pause_mode_var.get():
        zz=0
    else:
        canvas.draw_idle()
# -----------------------
# START THREAD
# -----------------------
thread = threading.Thread(target=control_loop, daemon=True)
thread.start()

root.mainloop()
 
