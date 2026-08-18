#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Flight Data & Angular Rate Performance Analyzer for Quadrotor MPC
Features:
    1. Comprehensive 6-panel flight performance visualization
    2. Detailed 3-axis Angular Rate (Roll, Pitch, Yaw) tracking response & error analysis
    3. Quantitative statistical metrics (RMSE, Max Error, Control Smoothness)
    4. Adaptive hover thrust and position tracking analysis

Usage:
    python3 plot_flight_data.py [path_to_csv_file]
    python3 plot_flight_data.py --rates [path_to_csv_file]  # Dedicated angular rate focus mode
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

def print_performance_summary(df):
    t = df["time_sec"]
    # Filter valid tracking phase (after 3s transition)
    track_mask = t >= 3.0 if t.max() > 5.0 else t >= 0.0
    df_eval = df[track_mask].copy()

    # 1. Position Errors
    pos_rmse_x = np.sqrt(np.mean(df_eval["err_x"]**2))
    pos_rmse_y = np.sqrt(np.mean(df_eval["err_y"]**2))
    pos_rmse_z = np.sqrt(np.mean(df_eval["err_z"]**2))
    pos_rmse_3d = np.sqrt(np.mean(df_eval["err_pos_norm"]**2))
    pos_max_3d = np.max(df_eval["err_pos_norm"])

    # 2. Angular Rates Analysis (rad/s -> deg/s)
    has_imu_rates = "imu_wx" in df_eval.columns and (df_eval["imu_wx"]**2 + df_eval["imu_wy"]**2).sum() > 1e-4
    
    print("\n" + "=" * 68)
    print("           QUADROTOR MPC FLIGHT PERFORMANCE EVALUATION           ")
    print("=" * 68)
    print(f" Flight Duration : {t.max():.2f} s | Valid Samples : {len(df_eval)} pts")
    print("-" * 68)
    print(" [1] POSITION TRACKING METRICS:")
    print(f"   - 3D Position RMSE  : {pos_rmse_3d * 100:.2f} cm (Max: {pos_max_3d * 100:.2f} cm)")
    print(f"   - Axis-X RMSE       : {pos_rmse_x * 100:.2f} cm")
    print(f"   - Axis-Y RMSE       : {pos_rmse_y * 100:.2f} cm")
    print(f"   - Axis-Z RMSE       : {pos_rmse_z * 100:.2f} cm")

    if has_imu_rates:
        err_wx = df_eval["ctrl_wx"] - df_eval["imu_wx"]
        err_wy = df_eval["ctrl_wy"] - df_eval["imu_wy"]
        err_wz = df_eval["ctrl_wz"] - df_eval["imu_wz"]
        err_rate_norm = np.sqrt(err_wx**2 + err_wy**2 + err_wz**2)

        rate_rmse_wx = np.sqrt(np.mean(err_wx**2)) * 180.0 / np.pi
        rate_rmse_wy = np.sqrt(np.mean(err_wy**2)) * 180.0 / np.pi
        rate_rmse_wz = np.sqrt(np.mean(err_wz**2)) * 180.0 / np.pi
        rate_rmse_3d = np.sqrt(np.mean(err_rate_norm**2)) * 180.0 / np.pi
        rate_max_3d = np.max(err_rate_norm) * 180.0 / np.pi

        # Control rate variation / smoothness (Jitter Metric: mean absolute derivative)
        dt = np.diff(t[track_mask])
        dt[dt <= 0] = 0.02
        dwx_dt = np.abs(np.diff(df_eval["ctrl_wx"])) / dt
        dwy_dt = np.abs(np.diff(df_eval["ctrl_wy"])) / dt
        smoothness_index = np.mean(dwx_dt + dwy_dt)

        print("-" * 68)
        print(" [2] ANGULAR VELOCITY TRACKING & ATTITUDE RESPONSE:")
        print(f"   - Total Rate RMSE   : {rate_rmse_3d:.2f} deg/s (Max: {rate_max_3d:.2f} deg/s)")
        print(f"   - Roll Rate (wx)    : RMSE = {rate_rmse_wx:.2f} deg/s | Max = {np.max(np.abs(err_wx))*180/np.pi:.2f} deg/s")
        print(f"   - Pitch Rate (wy)   : RMSE = {rate_rmse_wy:.2f} deg/s | Max = {np.max(np.abs(err_wy))*180/np.pi:.2f} deg/s")
        print(f"   - Yaw Rate (wz)     : RMSE = {rate_rmse_wz:.2f} deg/s | Max = {np.max(np.abs(err_wz))*180/np.pi:.2f} deg/s")
        print(f"   - Rate Jitter Metric: {smoothness_index:.2f} rad/s^2 (Lower = Smoother control)")
    else:
        print("-" * 68)
        print(" [2] ANGULAR VELOCITY METRICS: IMU rates not found or idle.")

    if "cmd_thrust" in df_eval.columns and "estimated_hover_thrust" in df_eval.columns:
        mean_thr = np.mean(df_eval["cmd_thrust"])
        last_hov = df_eval["estimated_hover_thrust"].iloc[-1]
        print("-" * 68)
        print(" [3] THRUST & ADAPTIVE ESTIMATION:")
        print(f"   - Mean Commanded Throttle : {mean_thr * 100:.1f} %")
        print(f"   - Estimated Hover Throttle: {last_hov * 100:.1f} % (Converged)")
    print("=" * 68 + "\n")

def plot_comprehensive(df, csv_path):
    t = df["time_sec"]
    has_imu = "imu_wx" in df.columns and (df["imu_wx"]**2 + df["imu_wy"]**2).sum() > 1e-4
    has_eso = "eso_dx" in df.columns and (df["eso_dx"]**2 + df["eso_dy"]**2).sum() > 1e-4

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"Flight Performance & Angular Rate Analysis [{os.path.basename(csv_path)}]", fontsize=14, fontweight='bold')

    # 1. 2D / XY Trajectory
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(df["ref_x"], df["ref_y"], "r--", label="Reference Circle", linewidth=1.8)
    ax1.plot(df["pos_x"], df["pos_y"], "b-", label="Real Flight", linewidth=1.5, alpha=0.9)
    ax1.scatter([df["pos_x"].iloc[0]], [df["pos_y"].iloc[0]], color='green', marker='o', s=60, label='Start')
    ax1.scatter([df["pos_x"].iloc[-1]], [df["pos_y"].iloc[-1]], color='purple', marker='x', s=60, label='End')
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_title("XY Flight Trajectory vs Reference")
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.axis("equal")
    ax1.legend(loc='best')

    # 2. Position Tracking Error
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(t, df["err_x"] * 100, label="Err X", color='#1f77b4', alpha=0.8)
    ax2.plot(t, df["err_y"] * 100, label="Err Y", color='#ff7f0e', alpha=0.8)
    ax2.plot(t, df["err_z"] * 100, label="Err Z", color='#2ca02c', alpha=0.8)
    ax2.plot(t, df["err_pos_norm"] * 100, "k-", label="3D Error Norm", linewidth=1.8)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error (cm)")
    ax2.set_title("Position Tracking Errors (cm)")
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='best')

    # 3. Roll & Pitch Angular Rates (Command vs Real)
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(t, np.rad2deg(df["ctrl_wx"]), "r--", label="Roll Rate Cmd (wx_cmd)", linewidth=1.5)
    if has_imu:
        ax3.plot(t, np.rad2deg(df["imu_wx"]), "r-", label="Roll Rate Real (wx_real)", linewidth=1.2, alpha=0.7)
    ax3.plot(t, np.rad2deg(df["ctrl_wy"]), "b--", label="Pitch Rate Cmd (wy_cmd)", linewidth=1.5)
    if has_imu:
        ax3.plot(t, np.rad2deg(df["imu_wy"]), "b-", label="Pitch Rate Real (wy_real)", linewidth=1.2, alpha=0.7)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angular Velocity (deg/s)")
    ax3.set_title("Roll & Pitch Rates (Cmd vs Real)")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='best')

    # 4. Yaw Rate & Angular Rate Error Norm
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(t, np.rad2deg(df["ctrl_wz"]), "g--", label="Yaw Rate Cmd (wz_cmd)", linewidth=1.5)
    if has_imu:
        ax4.plot(t, np.rad2deg(df["imu_wz"]), "g-", label="Yaw Rate Real (wz_real)", linewidth=1.2, alpha=0.7)
        err_rate = np.sqrt((df["ctrl_wx"] - df["imu_wx"])**2 + 
                           (df["ctrl_wy"] - df["imu_wy"])**2 + 
                           (df["ctrl_wz"] - df["imu_wz"])**2)
        ax4.plot(t, np.rad2deg(err_rate), "k-", label="Rate Error Norm |e_w|", linewidth=1.2, alpha=0.85)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Angular Velocity (deg/s)")
    ax4.set_title("Yaw Rate & Total Rate Error (deg/s)")
    ax4.grid(True, linestyle='--', alpha=0.6)
    ax4.legend(loc='best')

    # 5. Thrust Acceleration & Throttle Convergence
    ax5 = plt.subplot(2, 3, 5)
    ax5_twin = ax5.twinx()
    p1 = ax5.plot(t, df["cmd_thrust"], "m-", label="Cmd Throttle (0~1)", linewidth=1.5)
    p2 = ax5.plot(t, df["estimated_hover_thrust"], "r--", label="Est Hover Throttle", linewidth=1.8)
    p3 = ax5_twin.plot(t, df["ctrl_acc_z"], "c-.", label="Thrust Acc T (m/s^2)", linewidth=1.2, alpha=0.7)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Throttle (0.0 ~ 1.0)", color='m')
    ax5_twin.set_ylabel("Thrust Acc (m/s^2)", color='c')
    ax5.set_title("Thrust Acc & Hover Throttle Estimation")
    ax5.grid(True, linestyle='--', alpha=0.6)
    plots = p1 + p2 + p3
    labs = [l.get_label() for l in plots]
    ax5.legend(plots, labs, loc='best')

    # 6. Disturbance / Velocity Tracking
    ax6 = plt.subplot(2, 3, 6)
    if has_eso:
        ax6.plot(t, df["eso_dx"], "r-", label="ESO Disturbance X (m/s^2)", linewidth=1.4)
        ax6.plot(t, df["eso_dy"], "b-", label="ESO Disturbance Y (m/s^2)", linewidth=1.4)
        ax6.plot(t, df["eso_dz"], "g-", label="ESO Disturbance Z (m/s^2)", linewidth=1.4)
        ax6.set_title("ESO Estimated Disturbance (m/s^2)")
        ax6.set_ylabel("Disturbance Acc (m/s^2)")
    else:
        ax6.plot(t, df["vel_x"], "r-", label="Vel X (m/s)", linewidth=1.2)
        ax6.plot(t, df["vel_y"], "b-", label="Vel Y (m/s)", linewidth=1.2)
        ax6.plot(t, df["vel_z"], "g-", label="Vel Z (m/s)", linewidth=1.2)
        ax6.set_title("Linear Velocities (m/s)")
        ax6.set_ylabel("Velocity (m/s)")
    ax6.set_xlabel("Time (s)")
    ax6.grid(True, linestyle='--', alpha=0.6)
    ax6.legend(loc='best')

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)

def plot_rates_focused(df, csv_path):
    t = df["time_sec"]
    has_imu = "imu_wx" in df.columns and (df["imu_wx"]**2 + df["imu_wy"]**2).sum() > 1e-4

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fig.suptitle(f"3-Axis Angular Velocity Tracking Detailed Analysis [{os.path.basename(csv_path)}]", fontsize=14, fontweight='bold')

    axes[0].plot(t, np.rad2deg(df["ctrl_wx"]), "r--", label="Roll Rate Cmd wx_cmd", linewidth=1.8)
    if has_imu:
        axes[0].plot(t, np.rad2deg(df["imu_wx"]), "r-", label="Roll Rate IMU wx_real", linewidth=1.4, alpha=0.8)
        axes[0].plot(t, np.rad2deg(df["ctrl_wx"] - df["imu_wx"]), "k:", label="Error e_wx", linewidth=1.0)
    axes[0].set_ylabel("Roll Rate (deg/s)")
    axes[0].set_title("Roll Axis (X) Angular Rate Tracking")
    axes[0].grid(True, linestyle='--', alpha=0.6)
    axes[0].legend(loc='upper right')

    axes[1].plot(t, np.rad2deg(df["ctrl_wy"]), "b--", label="Pitch Rate Cmd wy_cmd", linewidth=1.8)
    if has_imu:
        axes[1].plot(t, np.rad2deg(df["imu_wy"]), "b-", label="Pitch Rate IMU wy_real", linewidth=1.4, alpha=0.8)
        axes[1].plot(t, np.rad2deg(df["ctrl_wy"] - df["imu_wy"]), "k:", label="Error e_wy", linewidth=1.0)
    axes[1].set_ylabel("Pitch Rate (deg/s)")
    axes[1].set_title("Pitch Axis (Y) Angular Rate Tracking")
    axes[1].grid(True, linestyle='--', alpha=0.6)
    axes[1].legend(loc='upper right')

    axes[2].plot(t, np.rad2deg(df["ctrl_wz"]), "g--", label="Yaw Rate Cmd wz_cmd", linewidth=1.8)
    if has_imu:
        axes[2].plot(t, np.rad2deg(df["imu_wz"]), "g-", label="Yaw Rate IMU wz_real", linewidth=1.4, alpha=0.8)
        axes[2].plot(t, np.rad2deg(df["ctrl_wz"] - df["imu_wz"]), "k:", label="Error e_wz", linewidth=1.0)
    axes[2].set_ylabel("Yaw Rate (deg/s)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("Yaw Axis (Z) Angular Rate Tracking")
    axes[2].grid(True, linestyle='--', alpha=0.6)
    axes[2].legend(loc='upper right')

    plt.tight_layout()
    plt.subplots_adjust(top=0.92)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data_dir = os.path.join(script_dir, "..", "data")

    rates_mode = False
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]

    if "--rates" in flags or "-r" in flags:
        rates_mode = True

    if len(args) > 0:
        csv_path = args[0]
    else:
        csv_path = find_latest_csv(default_data_dir)
        if csv_path is None:
            print(f"[Error] No CSV logs found in {default_data_dir}")
            sys.exit(1)

    print(f"[DataLoader] Reading flight log: {csv_path}")
    df = pd.read_csv(csv_path)

    # 1. Print statistical analysis
    print_performance_summary(df)

    # 2. Plotting
    if rates_mode:
        plot_rates_focused(df, csv_path)
    else:
        plot_comprehensive(df, csv_path)

    plt.show()

if __name__ == "__main__":
    main()
