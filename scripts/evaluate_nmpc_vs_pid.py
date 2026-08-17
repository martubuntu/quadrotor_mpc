#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Dimensional Performance Evaluation: NMPC vs PID Baseline
Simulation under Wind Gust Disturbance (90s ~ 120s, 3.0 m/s +X)

Usage:
    python3 evaluate_nmpc_vs_pid.py [nmpc_log.csv] [pid_log.csv]
If arguments are omitted, it automatically picks the latest NMPC and PID logs in data/.
"""

import sys
import glob
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

GUST_START = 90.0
GUST_END = 120.0

def find_latest_log(data_dir, prefix):
    pattern = os.path.join(data_dir, f"flight_log_{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        if prefix == "NMPC":
            all_files = glob.glob(os.path.join(data_dir, "flight_log_*.csv"))
            non_pid = [f for f in all_files if "PID" not in f]
            if non_pid:
                return max(non_pid, key=os.path.getctime)
        return None
    return max(files, key=os.path.getctime)

def calculate_metrics(df, label=""):
    # Filter valid tracking data (elapsed time >= 15.0s, steady tracking)
    df_track = df[df["time_sec"] >= 15.0].copy()
    if df_track.empty:
        df_track = df.copy()

    # Time segments
    calm = df_track[(df_track["time_sec"] >= 20.0) & (df_track["time_sec"] < GUST_START)]
    gust = df_track[(df_track["time_sec"] >= GUST_START) & (df_track["time_sec"] <= GUST_END)]
    transient = df_track[(df_track["time_sec"] >= GUST_START) & (df_track["time_sec"] <= GUST_START + 5.0)]

    # 1. Tracking Errors
    rmse_3d = float(np.sqrt(np.mean(df_track["err_pos_norm"]**2)))
    rmse_x = float(np.sqrt(np.mean(df_track["err_x"]**2)))
    rmse_y = float(np.sqrt(np.mean(df_track["err_y"]**2)))
    rmse_z = float(np.sqrt(np.mean(df_track["err_z"]**2)))
    max_err = float(np.max(df_track["err_pos_norm"]))
    mean_err = float(np.mean(df_track["err_pos_norm"]))

    # 2. Disturbance Rejection Metrics
    rmse_calm = float(np.sqrt(np.mean(calm["err_pos_norm"]**2))) if not calm.empty else rmse_3d
    rmse_gust = float(np.sqrt(np.mean(gust["err_pos_norm"]**2))) if not gust.empty else rmse_3d
    gust_amp_ratio = float(rmse_gust / rmse_calm) if rmse_calm > 1e-4 else 1.0
    transient_peak = float(np.max(transient["err_pos_norm"])) if not transient.empty else max_err

    # 3. Control Smoothness (Body Rates)
    # Prefer measured IMU angular velocity; fallback to ctrl_wx/wy/wz or velocity gradient
    if "imu_wx" in df_track.columns and (df_track["imu_wx"]**2 + df_track["imu_wy"]**2).sum() > 1e-3:
        body_rate_norm = np.sqrt(df_track["imu_wx"]**2 + df_track["imu_wy"]**2 + df_track["imu_wz"]**2)
    elif "ctrl_wx" in df_track.columns and (df_track["ctrl_wx"]**2 + df_track["ctrl_wy"]**2 + df_track["ctrl_wz"]**2).sum() > 1e-3:
        body_rate_norm = np.sqrt(df_track["ctrl_wx"]**2 + df_track["ctrl_wy"]**2 + df_track["ctrl_wz"]**2)
    else:
        # Approximate body angular rate from velocity lateral acceleration (rad/s)
        vx = df_track["vel_x"].values
        vy = df_track["vel_y"].values
        dt_approx = np.median(np.diff(df_track["time_sec"])) if len(df_track) > 1 else 0.05
        ax = np.gradient(vx, dt_approx)
        ay = np.gradient(vy, dt_approx)
        body_rate_norm = np.sqrt(ax**2 + ay**2) / 9.8066

    rate_rms = float(np.sqrt(np.mean(body_rate_norm**2)))

    # 4. Throttle Series & Variance
    thrust_series = df_track["cmd_thrust"].copy().astype(float)
    if thrust_series.sum() < 1e-3 and "imu_acc_z" in df_track.columns:
        # Reconstruct throttle for PID from IMU specific force
        hover_ratio = 9.8066 / 0.58
        thrust_series = np.clip(df_track["imu_acc_z"] / hover_ratio, 0.15, 0.95)
    
    thrust_var = float(np.var(thrust_series))

    # 5. Energy Metrics (Power proxy integral: T^1.5 dt, control effort: (T^2 + 0.05 * w^2) dt)
    dt = float(np.median(np.diff(df_track["time_sec"]))) if len(df_track) > 1 else 0.05
    energy_power_proxy = float(np.sum(thrust_series**1.5) * dt)
    energy_effort = float(np.sum(thrust_series**2 + 0.05 * (body_rate_norm**2)) * dt)

    return {
        "label": label,
        "rmse_3d": rmse_3d,
        "rmse_x": rmse_x,
        "rmse_y": rmse_y,
        "rmse_z": rmse_z,
        "max_err": max_err,
        "mean_err": mean_err,
        "rmse_calm": rmse_calm,
        "rmse_gust": rmse_gust,
        "gust_amp_ratio": gust_amp_ratio,
        "transient_peak": transient_peak,
        "rate_rms": rate_rms,
        "thrust_var": thrust_var,
        "energy_power": energy_power_proxy,
        "energy_effort": energy_effort,
        "thrust_series": thrust_series,
        "body_rate_norm": body_rate_norm
    }

def print_comparison_table(m_nmpc, m_pid):
    print("\n" + "="*80)
    print("      QUADROTOR TRAJECTORY TRACKING MULTI-DIMENSIONAL EVALUATION REPORT")
    print("      Wind Gust: 90s - 120s (+X 3.0 m/s) | Circle Radius: 1.5m, Vel: 0.8m/s")
    print("="*80)
    print(f"{'Performance Metric':<35} | {'NMPC':<12} | {'PID Baseline':<12} | {'Improvement':<12}")
    print("-"*80)

    comparisons = [
        ("3D Position RMSE (m)", "rmse_3d", True),
        ("  - X-axis RMSE (m)", "rmse_x", True),
        ("  - Y-axis RMSE (m)", "rmse_y", True),
        ("  - Z-axis RMSE (m)", "rmse_z", True),
        ("Max 3D Tracking Error (m)", "max_err", True),
        ("Mean 3D Tracking Error (m)", "mean_err", True),
        ("Calm Phase RMSE (20-90s) (m)", "rmse_calm", True),
        ("Gust Phase RMSE (90-120s) (m)", "rmse_gust", True),
        ("Gust Amplification Ratio", "gust_amp_ratio", True),
        ("Gust Onset Peak Error (m)", "transient_peak", True),
        ("Body Rate RMS (rad/s)", "rate_rms", True),
        ("Throttle Variance (Var)", "thrust_var", True),
        ("Aerodynamic Power Proxy (J)", "energy_power", True),
        ("Control Effort Metric", "energy_effort", True),
    ]

    for title, key, lower_is_better in comparisons:
        v_nmpc = m_nmpc[key]
        v_pid = m_pid[key] if m_pid else None
        if v_pid is not None:
            if v_pid != 0:
                pct = (v_pid - v_nmpc) / v_pid * 100.0 if lower_is_better else (v_nmpc - v_pid) / v_pid * 100.0
                pct_str = f"{pct:+.1f}%" if abs(pct) > 0.01 else "0.0%"
            else:
                pct_str = "0.0%"
            print(f"{title:<35} | {v_nmpc:<12.4f} | {v_pid:<12.4f} | {pct_str:<12}")
        else:
            print(f"{title:<35} | {v_nmpc:<12.4f} | {'N/A':<12} | {'N/A':<12}")
    print("="*80 + "\n")

def plot_comparison(df_nmpc, df_pid, m_nmpc, m_pid, save_path):
    fig = plt.figure(figsize=(18, 12))

    # 1. XY Flight Trajectory Comparison
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(df_nmpc["ref_x"], df_nmpc["ref_y"], "k--", label="Reference Circle", linewidth=1.5)
    ax1.plot(df_nmpc["pos_x"], df_nmpc["pos_y"], "b-", label=f"NMPC (RMSE={m_nmpc['rmse_3d']:.3f}m)", linewidth=1.8)
    if df_pid is not None:
        ax1.plot(df_pid["pos_x"], df_pid["pos_y"], "r-.", label=f"PID Baseline (RMSE={m_pid['rmse_3d']:.3f}m)", linewidth=1.5, alpha=0.8)
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_title("XY Flight Trajectory Comparison")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.axis("equal")
    ax1.legend(loc="upper right")

    # 2. Tracking Error vs Time with Shaded Wind Gust Window
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axvspan(GUST_START, GUST_END, color="orange", alpha=0.2, label="Wind Gust Window (3.0 m/s +X)")
    ax2.plot(df_nmpc["time_sec"], df_nmpc["err_pos_norm"], "b-", label="NMPC 3D Error", linewidth=1.8)
    if df_pid is not None:
        ax2.plot(df_pid["time_sec"], df_pid["err_pos_norm"], "r-.", label="PID Baseline 3D Error", linewidth=1.5, alpha=0.8)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("3D Position Error (m)")
    ax2.set_title("Position Tracking Error vs Time (With Wind Gust 90~120s)")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right")

    # 3. Control Throttle & Dynamic Response
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axvspan(GUST_START, GUST_END, color="orange", alpha=0.2, label="Wind Gust")
    nmpc_track = df_nmpc[df_nmpc["time_sec"] >= 15.0]
    ax3.plot(nmpc_track["time_sec"], m_nmpc["thrust_series"], "b-", label="NMPC Thrust", linewidth=1.5)
    if df_pid is not None:
        pid_track = df_pid[df_pid["time_sec"] >= 15.0]
        ax3.plot(pid_track["time_sec"], m_pid["thrust_series"], "r--", label="PID Equivalent Thrust", linewidth=1.5, alpha=0.7)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Thrust (Normalized 0~1)")
    ax3.set_title("Throttle Response under Wind Disturbance")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper right")

    # 4. Multi-Dimensional Performance Bar Chart
    ax4 = fig.add_subplot(2, 2, 4)
    if df_pid is not None:
        categories = ["3D RMSE (m)", "Gust RMSE (m)", "Peak Err (m)", "Rate RMS (rad/s)"]
        nmpc_vals = [m_nmpc["rmse_3d"], m_nmpc["rmse_gust"], m_nmpc["transient_peak"], m_nmpc["rate_rms"]]
        pid_vals = [m_pid["rmse_3d"], m_pid["rmse_gust"], m_pid["transient_peak"], m_pid["rate_rms"]]

        x = np.arange(len(categories))
        width = 0.35
        rects1 = ax4.bar(x - width/2, nmpc_vals, width, label="NMPC", color="royalblue")
        rects2 = ax4.bar(x + width/2, pid_vals, width, label="PID Baseline", color="salmon")

        ax4.set_ylabel("Metric Value")
        ax4.set_title("Key Performance Comparison Across Dimensions")
        ax4.set_xticks(x)
        ax4.set_xticklabels(categories)
        ax4.legend()
        ax4.grid(True, axis="y", linestyle="--", alpha=0.6)

        for rect in rects1 + rects2:
            h = rect.get_height()
            ax4.annotate(f'{h:.3f}',
                         xy=(rect.get_x() + rect.get_width() / 2, h),
                         xytext=(0, 3), textcoords="offset points",
                         ha='center', va='bottom', fontsize=9)
    else:
        ax4.text(0.5, 0.5, "PID log not provided for comparative bar chart", ha="center", va="center")

    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"[Plot] Evaluation figure saved to: {save_path}")
    plt.show()

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "..", "data")

    nmpc_file = None
    pid_file = None

    if len(sys.argv) >= 3:
        nmpc_file = sys.argv[1]
        pid_file = sys.argv[2]
    elif len(sys.argv) == 2:
        nmpc_file = sys.argv[1]
    else:
        nmpc_file = find_latest_log(data_dir, "NMPC")
        pid_file = find_latest_log(data_dir, "PID")

    if nmpc_file is None:
        print(f"[Error] No NMPC flight log found in {data_dir}")
        sys.exit(1)

    print(f"Loading NMPC Log: {nmpc_file}")
    df_nmpc = pd.read_csv(nmpc_file)
    m_nmpc = calculate_metrics(df_nmpc, "NMPC")

    df_pid = None
    m_pid = None
    if pid_file and os.path.exists(pid_file):
        print(f"Loading PID Log : {pid_file}")
        df_pid = pd.read_csv(pid_file)
        m_pid = calculate_metrics(df_pid, "PID")

    print_comparison_table(m_nmpc, m_pid)

    save_plot_path = os.path.join(data_dir, "simulation_evaluation_report.png")
    plot_comparison(df_nmpc, df_pid, m_nmpc, m_pid, save_plot_path)

if __name__ == "__main__":
    main()
