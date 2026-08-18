#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flight Data Plotter for Quadrotor MPC (Position, Attitude, Control & ESO)
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

    plt.figure(figsize=(16, 11))

    # 1. 3D Trajectory / XY Plane
    plt.subplot(2, 3, 1)
    plt.plot(df["ref_x"], df["ref_y"], "r--", label="Reference Circle", linewidth=1.5)
    plt.plot(df["pos_x"], df["pos_y"], "b-", label="Real Flight", linewidth=1.5)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("XY Flight Trajectory vs Reference")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.axis("equal")
    plt.legend()

    # 2. Position Tracking Error vs Time
    plt.subplot(2, 3, 2)
    plt.plot(t, df["err_x"], label="Err X (m)", alpha=0.8)
    plt.plot(t, df["err_y"], label="Err Y (m)", alpha=0.8)
    plt.plot(t, df["err_z"], label="Err Z (m)", alpha=0.8)
    plt.plot(t, df["err_pos_norm"], "k-", label="3D Error Norm (m)", linewidth=1.8)
    plt.axvspan(45.0, 75.0, color="orange", alpha=0.2, label="Wind Gust Window (45~75s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.title("Position Tracking Errors")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    # 3. Attitude Tracking (Roll, Pitch, Yaw)
    plt.subplot(2, 3, 3)
    if "roll_deg" in df.columns:
        plt.plot(t, df["roll_deg"], "r-", label="Roll (°)", alpha=0.8)
        plt.plot(t, df["pitch_deg"], "g-", label="Pitch (°)", alpha=0.8)
        plt.plot(t, df["yaw_deg"], "b-", label="Yaw (°)", alpha=0.6)
        if "ref_pitch_deg" in df.columns:
            plt.plot(t, df["ref_pitch_deg"], "g--", label="Ref Pitch (°)", alpha=0.5)
    plt.axvspan(45.0, 75.0, color="orange", alpha=0.2)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Euler Angles (Roll, Pitch, Yaw)")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    # 4. MPC Control Outputs (Body Rates & Specific Thrust)
    plt.subplot(2, 3, 4)
    plt.plot(t, df["ctrl_wx"], label="wx (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_wy"], label="wy (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_wz"], label="wz (rad/s)", alpha=0.8)
    plt.plot(t, df["ctrl_acc_z"], label="Thrust Acc (m/s²)", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Commands")
    plt.title("MPC Control Outputs (Body Rates & Acc)")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    # 5. Throttle & Hover Thrust Convergence
    plt.subplot(2, 3, 5)
    plt.plot(t, df["cmd_thrust"], label="Cmd Throttle (0~1)", linewidth=1.5)
    plt.plot(t, df["estimated_hover_thrust"], "r--", label="Estimated Hover Throttle", linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Throttle (0.0 ~ 1.0)")
    plt.title("Throttle & Adaptive Hover Thrust")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    # 6. ESO Disturbance Estimation
    plt.subplot(2, 3, 6)
    if "eso_dx" in df.columns and (df["eso_dx"]**2 + df["eso_dy"]**2).sum() > 1e-4:
        plt.plot(t, df["eso_dx"], "r-", label="d_hat_x (m/s²)", linewidth=1.5)
        plt.plot(t, df["eso_dy"], "g-", label="d_hat_y (m/s²)", linewidth=1.5)
        plt.plot(t, df["eso_dz"], "b-", label="d_hat_z (m/s²)", linewidth=1.5)
        d_norm = np.sqrt(df["eso_dx"]**2 + df["eso_dy"]**2 + df["eso_dz"]**2)
        plt.plot(t, d_norm, "k--", label="|d_hat| (m/s²)", linewidth=1.5)
        plt.axvspan(45.0, 75.0, color="orange", alpha=0.2, label="Wind Gust Window (45~75s)")
        plt.title("ESO Wind Disturbance Estimation")
    else:
        plt.text(0.5, 0.5, "No ESO Disturbance Data", ha="center", va="center")
        plt.title("ESO Disturbance (N/A for Pure NMPC/PID)")
    plt.xlabel("Time (s)")
    plt.ylabel("Disturbance Acc (m/s²)")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
