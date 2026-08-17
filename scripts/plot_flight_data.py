#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flight Data Plotter for Quadrotor MPC
Usage:
    python3 plot_flight_data.py [path_to_csv_file]
If no file is provided, it automatically picks the latest CSV in the data/ folder.
"""

import sys
import glob
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def find_latest_csv(data_dir):
    csv_files = glob.glob(os.path.join(data_dir, "flight_log_*.csv"))
    if not csv_files:
        return None
    latest_file = max(csv_files, key=os.path.getctime)
    return latest_file

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data_dir = os.path.join(script_dir, "..", "data")

    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = find_latest_csv(default_data_dir)
        if csv_path is None:
            print(f"[Error] No CSV logs found in {default_data_dir}")
            sys.exit(1)

    print(f"Loading flight log: {csv_path}")
    df = pd.read_csv(csv_path)

    t = df["time_sec"]

    plt.figure(figsize=(14, 10))

    # 1. 3D Trajectory / XY Plane
    plt.subplot(2, 2, 1)
    plt.plot(df["ref_x"], df["ref_y"], "r--", label="Reference Circle", linewidth=1.5)
    plt.plot(df["pos_x"], df["pos_y"], "b-", label="Real Flight", linewidth=1.5)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("XY Flight Trajectory vs Reference")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    # 2. Position Tracking Error vs Time
    plt.subplot(2, 2, 2)
    plt.plot(t, df["err_x"], label="Error X (m)", alpha=0.8)
    plt.plot(t, df["err_y"], label="Error Y (m)", alpha=0.8)
    plt.plot(t, df["err_z"], label="Error Z (m)", alpha=0.8)
    plt.plot(t, df["err_pos_norm"], "k-", label="3D Error Norm (m)", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.title("Position Tracking Errors")
    plt.grid(True)
    plt.legend()

    # 3. Control Inputs (Angular Rates & Specific Thrust)
    plt.subplot(2, 2, 3)
    plt.plot(t, df["ctrl_wx"], label="wx (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_wy"], label="wy (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_wz"], label="wz (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_acc_z"], label="Thrust Acc T (m/s^2)", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Commands")
    plt.title("MPC Control Outputs (Body Rates & Thrust Acc)")
    plt.grid(True)
    plt.legend()

    # 4. Thrust & Throttle Estimation
    plt.subplot(2, 2, 4)
    plt.plot(t, df["cmd_thrust"], label="Command Throttle (0~1)", linewidth=1.5)
    plt.plot(t, df["estimated_hover_thrust"], "r--", label="Estimated Hover Thrust (0~1)", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Throttle (0.0 ~ 1.0)")
    plt.title("Throttle & Adaptive Hover Thrust Convergence")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
