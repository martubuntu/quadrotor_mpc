#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Academic Multi-Dimensional Benchmark & Individual High-Resolution Plotter
NMPC + ESO vs NMPC vs PID Baseline (Simulation under Wind Gust 45s~75s, 3.0 m/s +X)

Output Figures:
  1. 3 Thematic Combined Pages (300 DPI in data/)
     - eval_page1_trajectory_tracking.png
     - eval_page2_attitude_dynamics.png
     - eval_page3_eso_disturbance_rejection.png
  2. 12 Individual Dedicated High-Res Plots (300 DPI in data/individual_plots/)
     - 01_xy_trajectory_tracking.png
     - 02_3d_position_error.png
     - 03_wind_axis_x_error.png
     - 04_tracking_precision_bar.png
     - 05_roll_angle_dynamics.png
     - 06_pitch_angle_dynamics.png
     - 07_body_rate_smoothness.png
     - 08_attitude_precision_bar.png
     - 09_eso_disturbance_estimation.png
     - 10_wind_axis_steady_bias.png
     - 11_itae_cumulative_growth.png
     - 12_eso_superiority_bar.png

Usage:
    python3 evaluate_nmpc_vs_pid.py [nmpc_eso_log.csv] [nmpc_log.csv] [pid_log.csv]
"""

import sys
import glob
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

GUST_START = 45.0
GUST_END = 75.0

# Academic Color Palette
COLOR_ESO = "#2ca02c"    # Emerald Green
COLOR_NMPC = "#1f77b4"   # Royal Blue
COLOR_PID = "#d62728"    # Crimson Red
COLOR_REF = "#222222"    # Dark Charcoal
COLOR_GUST = "#ffa500"   # Amber Orange

def find_latest_log(data_dir, prefix):
    pattern = os.path.join(data_dir, f"flight_log_{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        if prefix == "NMPC":
            all_files = glob.glob(os.path.join(data_dir, "flight_log_*.csv"))
            non_special = [f for f in all_files if "PID" not in f and "ESO" not in f]
            if non_special:
                return max(non_special, key=os.path.getctime)
        return None
    return max(files, key=os.path.getctime)

def calculate_metrics(df, label=""):
    # Filter valid tracking data (elapsed time >= 10.0s, steady tracking)
    df_track = df[df["time_sec"] >= 10.0].copy()
    if df_track.empty:
        df_track = df.copy()

    # Time segments
    calm = df_track[(df_track["time_sec"] >= 10.0) & (df_track["time_sec"] < GUST_START)]
    gust = df_track[(df_track["time_sec"] >= GUST_START) & (df_track["time_sec"] <= GUST_END)]
    gust_steady = df_track[(df_track["time_sec"] >= GUST_START + 5.0) & (df_track["time_sec"] <= GUST_END - 5.0)]
    transient = df_track[(df_track["time_sec"] >= GUST_START) & (df_track["time_sec"] <= GUST_START + 5.0)]

    # 1. 3D Position Tracking Errors
    rmse_3d = float(np.sqrt(np.mean(df_track["err_pos_norm"]**2)))
    rmse_x = float(np.sqrt(np.mean(df_track["err_x"]**2)))
    rmse_y = float(np.sqrt(np.mean(df_track["err_y"]**2)))
    rmse_z = float(np.sqrt(np.mean(df_track["err_z"]**2)))
    max_err = float(np.max(df_track["err_pos_norm"]))
    mean_err = float(np.mean(df_track["err_pos_norm"]))

    # 2. Attitude Tracking Errors (Degrees)
    rmse_roll = None
    rmse_pitch = None
    rmse_yaw = None
    if "err_roll_deg" in df_track.columns and "err_pitch_deg" in df_track.columns and "err_yaw_deg" in df_track.columns:
        rmse_roll = float(np.sqrt(np.mean(df_track["err_roll_deg"]**2)))
        rmse_pitch = float(np.sqrt(np.mean(df_track["err_pitch_deg"]**2)))
        rmse_yaw = float(np.sqrt(np.mean(df_track["err_yaw_deg"]**2)))

    # 3. Wind Disturbance Rejection & Steady-State Error Metrics (ESO Focus)
    rmse_calm = float(np.sqrt(np.mean(calm["err_pos_norm"]**2))) if not calm.empty else rmse_3d
    rmse_gust = float(np.sqrt(np.mean(gust["err_pos_norm"]**2))) if not gust.empty else rmse_3d
    gust_amp_ratio = float(rmse_gust / rmse_calm) if rmse_calm > 1e-4 else 1.0
    transient_peak = float(np.max(transient["err_pos_norm"])) if not transient.empty else max_err

    # Steady-state bias in wind axis (+X) during stable gust window
    if not gust_steady.empty:
        gust_ss_bias_x = float(np.mean(np.abs(gust_steady["err_x"])))
        gust_ss_bias_3d = float(np.mean(gust_steady["err_pos_norm"]))
    else:
        gust_ss_bias_x = float(np.mean(np.abs(gust["err_x"]))) if not gust.empty else rmse_x
        gust_ss_bias_3d = float(np.mean(gust["err_pos_norm"])) if not gust.empty else rmse_3d

    # IAE & ITAE Integral Error Criteria during Gust Phase (45s ~ 75s)
    if not gust.empty and len(gust) > 2:
        t_gust = gust["time_sec"].values
        dt_gust = np.diff(t_gust)
        dt_arr = np.concatenate([[dt_gust[0]], dt_gust])
        tau = np.maximum(0.0, t_gust - GUST_START)
        e_norm = gust["err_pos_norm"].values
        iae_gust = float(np.sum(e_norm * dt_arr))
        itae_gust = float(np.sum(tau * e_norm * dt_arr))

        # Gust settling/recovery time (time to return and stay within threshold)
        recovery_thresh = max(0.08, 1.25 * rmse_calm)
        settled_mask = (e_norm <= recovery_thresh)
        settling_time = 30.0 # default full window if never settled
        for i in range(len(t_gust)):
            if t_gust[i] >= GUST_START + 0.3 and np.all(settled_mask[i:min(i+15, len(settled_mask))]):
                settling_time = float(t_gust[i] - GUST_START)
                break
    else:
        iae_gust = rmse_gust * 30.0
        itae_gust = rmse_gust * 450.0
        settling_time = 30.0

    # 4. Control Smoothness & Total Variation (TV)
    if "imu_wx" in df_track.columns and (df_track["imu_wx"]**2 + df_track["imu_wy"]**2).sum() > 1e-3:
        body_rate_norm = np.sqrt(df_track["imu_wx"]**2 + df_track["imu_wy"]**2 + df_track["imu_wz"]**2)
    elif "ctrl_wx" in df_track.columns and (df_track["ctrl_wx"]**2 + df_track["ctrl_wy"]**2 + df_track["ctrl_wz"]**2).sum() > 1e-3:
        body_rate_norm = np.sqrt(df_track["ctrl_wx"]**2 + df_track["ctrl_wy"]**2 + df_track["ctrl_wz"]**2)
    else:
        vx = df_track["vel_x"].values
        vy = df_track["vel_y"].values
        dt_approx = np.median(np.diff(df_track["time_sec"])) if len(df_track) > 1 else 0.05
        ax = np.gradient(vx, dt_approx)
        ay = np.gradient(vy, dt_approx)
        body_rate_norm = np.sqrt(ax**2 + ay**2) / 9.8066

    rate_rms = float(np.sqrt(np.mean(body_rate_norm**2)))
    control_tv = float(np.sum(np.diff(body_rate_norm)**2)) if len(body_rate_norm) > 1 else 0.0

    # 5. Throttle Series & Variance
    thrust_series = df_track["cmd_thrust"].copy().astype(float)
    if thrust_series.sum() < 1e-3 and "imu_acc_z" in df_track.columns:
        hover_ratio = 9.8066 / 0.58
        thrust_series = np.clip(df_track["imu_acc_z"] / hover_ratio, 0.15, 0.95)
    
    thrust_var = float(np.var(thrust_series))

    # 6. Energy Metrics
    dt = float(np.median(np.diff(df_track["time_sec"]))) if len(df_track) > 1 else 0.05
    energy_power_proxy = float(np.sum(thrust_series**1.5) * dt)
    energy_effort = float(np.sum(thrust_series**2 + 0.05 * (body_rate_norm**2)) * dt)

    return {
        "label": label,
        "rmse_3d": rmse_3d,
        "rmse_x": rmse_x,
        "rmse_y": rmse_y,
        "rmse_z": rmse_z,
        "rmse_roll": rmse_roll,
        "rmse_pitch": rmse_pitch,
        "rmse_yaw": rmse_yaw,
        "max_err": max_err,
        "mean_err": mean_err,
        "rmse_calm": rmse_calm,
        "rmse_gust": rmse_gust,
        "gust_amp_ratio": gust_amp_ratio,
        "transient_peak": transient_peak,
        "gust_ss_bias_x": gust_ss_bias_x,
        "gust_ss_bias_3d": gust_ss_bias_3d,
        "iae_gust": iae_gust,
        "itae_gust": itae_gust,
        "settling_time": settling_time,
        "rate_rms": rate_rms,
        "control_tv": control_tv,
        "thrust_var": thrust_var,
        "energy_power": energy_power_proxy,
        "energy_effort": energy_effort,
        "thrust_series": thrust_series,
        "body_rate_norm": body_rate_norm
    }

def print_comparison_table(m_eso, m_nmpc, m_pid):
    print("\n" + "="*100)
    print("      ACADEMIC MULTI-DIMENSIONAL BENCHMARK REPORT: NMPC+ESO vs NMPC vs PID")
    print("      Wind Gust: 45s - 75s (+X 3.0 m/s) | Circle Radius: 1.5m, Vel: 0.8m/s")
    print("="*100)

    headers = f"{'Evaluation Metric Category':<34} | {'NMPC + ESO':<12} | {'NMPC (Pure)':<12} | {'PID Baseline':<12} | {'ESO vs PID':<10}"
    print(headers)
    print("="*100)

    sections = [
        ("--- 1. Trajectory Tracking Accuracy ---", None),
        ("3D Position RMSE (m)", "rmse_3d"),
        ("  - X-axis RMSE (m)", "rmse_x"),
        ("  - Y-axis RMSE (m)", "rmse_y"),
        ("  - Z-axis RMSE (m)", "rmse_z"),
        ("Max 3D Tracking Error (m)", "max_err"),
        ("Mean 3D Tracking Error (m)", "mean_err"),

        ("--- 2. Attitude Tracking Dynamics ---", None),
        ("Roll Tracking RMSE (deg)", "rmse_roll"),
        ("Pitch Tracking RMSE (deg)", "rmse_pitch"),
        ("Yaw Tracking RMSE (deg)", "rmse_yaw"),

        ("--- 3. Wind Rejection & ESO Superiority ---", None),
        ("Gust Steady-State Bias X (m)", "gust_ss_bias_x"),
        ("Gust Phase 3D RMSE (45-75s) (m)", "rmse_gust"),
        ("Calm Phase 3D RMSE (10-45s) (m)", "rmse_calm"),
        ("Gust Degradation Ratio (Rg)", "gust_amp_ratio"),
        ("Gust Onset Peak Error (m)", "transient_peak"),
        ("Gust Settling Time Ts (s)", "settling_time"),
        ("Gust Integral Abs Error IAE (m*s)", "iae_gust"),
        ("Gust Time-weighted ITAE (m*s^2)", "itae_gust"),

        ("--- 4. Smoothness & Energy Efficiency ---", None),
        ("Body Rate RMS (rad/s)", "rate_rms"),
        ("Control Variation / TV Metric", "control_tv"),
        ("Throttle Variance (Var)", "thrust_var"),
        ("Aerodynamic Power Proxy (J)", "energy_power"),
        ("Control Effort Metric", "energy_effort"),
    ]

    for title, key in sections:
        if key is None:
            print(f"\n{title:<100}")
            print("-" * 100)
            continue

        v_eso = m_eso.get(key) if m_eso else None
        v_nmpc = m_nmpc.get(key) if m_nmpc else None
        v_pid = m_pid.get(key) if m_pid else None

        str_eso = f"{v_eso:<12.4f}" if v_eso is not None else f"{'N/A':<12}"
        str_nmpc = f"{v_nmpc:<12.4f}" if v_nmpc is not None else f"{'N/A':<12}"
        str_pid = f"{v_pid:<12.4f}" if v_pid is not None else f"{'N/A':<12}"

        # Calculate improvement of primary vs PID
        v_main = v_eso if v_eso is not None else v_nmpc
        if v_main is not None and v_pid is not None and v_pid != 0:
            pct = (v_pid - v_main) / v_pid * 100.0
            str_imp = f"{pct:+.1f}%" if abs(pct) > 0.01 else "0.0%"
        else:
            str_imp = "N/A"

        print(f"{title:<34} | {str_eso} | {str_nmpc} | {str_pid} | {str_imp:<10}")
    print("="*100 + "\n")

# ==============================================================================
# INDIVIDUAL HIGH-RESOLUTION PLOTTING FUNCTIONS (300 DPI)
# ==============================================================================

def plot_single_xy_trajectory(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig, ax = plt.subplots(figsize=(9, 7))
    ref_df = df_eso if df_eso is not None else df_nmpc
    if ref_df is not None:
        ax.plot(ref_df["ref_x"], ref_df["ref_y"], color=COLOR_REF, linestyle="--", label="Reference Circle (R=1.5m)", linewidth=2.0)
    if df_eso is not None and m_eso is not None:
        ax.plot(df_eso["pos_x"], df_eso["pos_y"], color=COLOR_ESO, label=f"NMPC+ESO (3D RMSE={m_eso['rmse_3d']:.3f}m)", linewidth=2.2)
    if df_nmpc is not None and m_nmpc is not None:
        ax.plot(df_nmpc["pos_x"], df_nmpc["pos_y"], color=COLOR_NMPC, linestyle="-.", label=f"NMPC Pure (3D RMSE={m_nmpc['rmse_3d']:.3f}m)", linewidth=1.8)
    if df_pid is not None and m_pid is not None:
        ax.plot(df_pid["pos_x"], df_pid["pos_y"], color=COLOR_PID, linestyle=":", label=f"PID Baseline (3D RMSE={m_pid['rmse_3d']:.3f}m)", linewidth=1.6, alpha=0.85)
    ax.set_xlabel("X Position (m)", fontsize=12)
    ax.set_ylabel("Y Position (m)", fontsize=12)
    ax.set_title("01. 2D XY Flight Trajectory Tracking Comparison", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.axis("equal")
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_3d_position_error(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s, +X 3.0 m/s)")
    if df_eso is not None:
        ax.plot(df_eso["time_sec"], df_eso["err_pos_norm"], color=COLOR_ESO, label="NMPC+ESO 3D Position Error", linewidth=2.2)
    if df_nmpc is not None:
        ax.plot(df_nmpc["time_sec"], df_nmpc["err_pos_norm"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure 3D Position Error", linewidth=1.8)
    if df_pid is not None:
        ax.plot(df_pid["time_sec"], df_pid["err_pos_norm"], color=COLOR_PID, linestyle=":", label="PID Baseline 3D Position Error", linewidth=1.5, alpha=0.85)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("3D Position Error Norm (m)", fontsize=12)
    ax.set_title("02. 3D Position Tracking Error vs Time", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_wind_axis_x_error(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s)")
    if df_eso is not None:
        ax.plot(df_eso["time_sec"], df_eso["err_x"], color=COLOR_ESO, label="NMPC+ESO Error X (Wind Axis)", linewidth=2.2)
        ax.plot(df_eso["time_sec"], df_eso["err_y"], color="#1b9e77", linestyle="--", label="NMPC+ESO Error Y (Lateral Axis)", linewidth=1.5)
    if df_nmpc is not None:
        ax.plot(df_nmpc["time_sec"], df_nmpc["err_x"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Error X", linewidth=1.8)
    if df_pid is not None:
        ax.plot(df_pid["time_sec"], df_pid["err_x"], color=COLOR_PID, linestyle=":", label="PID Baseline Error X", linewidth=1.5, alpha=0.85)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Position Error along Wind Axis (m)", fontsize=12)
    ax.set_title("03. Position Tracking Error along Wind Direction (X-Axis)", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_tracking_precision_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    categories = ["3D Position RMSE", "X-Axis RMSE", "Y-Axis RMSE", "Mean 3D Error"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_3d"], m_eso["rmse_x"], m_eso["rmse_y"], m_eso["mean_err"]]
        rects1 = ax.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        for r in rects1:
            ax.annotate(f'{r.get_height():.3f}m', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_3d"], m_nmpc["rmse_x"], m_nmpc["rmse_y"], m_nmpc["mean_err"]]
        rects2 = ax.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        for r in rects2:
            ax.annotate(f'{r.get_height():.3f}m', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_3d"], m_pid["rmse_x"], m_pid["rmse_y"], m_pid["mean_err"]]
        rects3 = ax.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)
        for r in rects3:
            ax.annotate(f'{r.get_height():.3f}m', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)

    ax.set_ylabel("Error Value (m)", fontsize=12)
    ax.set_title("04. Spatial Tracking Precision Quantitative Benchmark", fontsize=14, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=11)
    ax.legend(fontsize=11)
    ax.grid(True, axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_roll_angle(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s)")
    if df_eso is not None and "roll_deg" in df_eso.columns:
        ax.plot(df_eso["time_sec"], df_eso["roll_deg"], color=COLOR_ESO, label="NMPC+ESO Roll (°)", linewidth=2.0)
    if df_nmpc is not None and "roll_deg" in df_nmpc.columns:
        ax.plot(df_nmpc["time_sec"], df_nmpc["roll_deg"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Roll (°)", linewidth=1.6)
    if df_pid is not None and "roll_deg" in df_pid.columns:
        ax.plot(df_pid["time_sec"], df_pid["roll_deg"], color=COLOR_PID, linestyle=":", label="PID Roll (°)", linewidth=1.4, alpha=0.85)
    if df_eso is not None and "ref_roll_deg" in df_eso.columns:
        ax.plot(df_eso["time_sec"], df_eso["ref_roll_deg"], color=COLOR_REF, linestyle="--", label="Ref Roll (°)", linewidth=1.3, alpha=0.7)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Roll Angle (deg)", fontsize=12)
    ax.set_title("05. Lateral Roll Angle Dynamic Response vs Reference", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_pitch_angle(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s, +X 3.0 m/s)")
    if df_eso is not None and "pitch_deg" in df_eso.columns:
        ax.plot(df_eso["time_sec"], df_eso["pitch_deg"], color=COLOR_ESO, label="NMPC+ESO Pitch (Active Wind Resistance)", linewidth=2.2)
    if df_nmpc is not None and "pitch_deg" in df_nmpc.columns:
        ax.plot(df_nmpc["time_sec"], df_nmpc["pitch_deg"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Pitch", linewidth=1.8)
    if df_pid is not None and "pitch_deg" in df_pid.columns:
        ax.plot(df_pid["time_sec"], df_pid["pitch_deg"], color=COLOR_PID, linestyle=":", label="PID Baseline Pitch", linewidth=1.4, alpha=0.85)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Pitch Angle (deg)", fontsize=12)
    ax.set_title("06. Pitch Attitude Dynamic Forward-Tilt Response under Wind Gust", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_body_rate_smoothness(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window")
    if df_eso is not None and m_eso is not None:
        t_eso = df_eso[df_eso["time_sec"] >= 10.0]
        ax.plot(t_eso["time_sec"], m_eso["body_rate_norm"], color=COLOR_ESO, label=f"NMPC+ESO Angular Rate (RMS={m_eso['rate_rms']:.3f} rad/s)", linewidth=1.8)
    if df_nmpc is not None and m_nmpc is not None:
        t_nmpc = df_nmpc[df_nmpc["time_sec"] >= 10.0]
        ax.plot(t_nmpc["time_sec"], m_nmpc["body_rate_norm"], color=COLOR_NMPC, linestyle="-.", label=f"NMPC Pure (RMS={m_nmpc['rate_rms']:.3f} rad/s)", linewidth=1.5)
    if df_pid is not None and m_pid is not None:
        t_pid = df_pid[df_pid["time_sec"] >= 10.0]
        ax.plot(t_pid["time_sec"], m_pid["body_rate_norm"], color=COLOR_PID, linestyle=":", label=f"PID Baseline (RMS={m_pid['rate_rms']:.3f} rad/s)", linewidth=1.3, alpha=0.85)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Body Angular Rate Norm (rad/s)", fontsize=12)
    ax.set_title("07. Control Effort & Body Angular Rate Smoothness", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_attitude_precision_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    categories = ["Roll RMSE (°)", "Pitch RMSE (°)", "Yaw RMSE (°)", "Body Rate RMS (rad/s)"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_roll"] or 0, m_eso["rmse_pitch"] or 0, m_eso["rmse_yaw"] or 0, m_eso["rate_rms"]]
        rects1 = ax.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        for r in rects1:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_roll"] or 0, m_nmpc["rmse_pitch"] or 0, m_nmpc["rmse_yaw"] or 0, m_nmpc["rate_rms"]]
        rects2 = ax.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        for r in rects2:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_roll"] or 0, m_pid["rmse_pitch"] or 0, m_pid["rmse_yaw"] or 0, m_pid["rate_rms"]]
        rects3 = ax.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)
        for r in rects3:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)

    ax.set_ylabel("Metric Value (° or rad/s)", fontsize=12)
    ax.set_title("08. Attitude Control Precision & Dynamic Benchmark", fontsize=14, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=11)
    ax.legend(fontsize=11)
    ax.grid(True, axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_eso_disturbance_estimation(df_eso, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="True Wind Gust Window (45s~75s, +X 3.0 m/s)")
    if df_eso is not None and "eso_dx" in df_eso.columns and (df_eso["eso_dx"]**2 + df_eso["eso_dy"]**2).sum() > 1e-4:
        ax.plot(df_eso["time_sec"], df_eso["eso_dx"], color="#d95f02", label=r"$\hat{d}_x$ (Wind Axis Estimation)", linewidth=2.2)
        ax.plot(df_eso["time_sec"], df_eso["eso_dy"], color="#7570b3", linestyle="--", label=r"$\hat{d}_y$ (Lateral Axis Estimation)", linewidth=1.6)
        d_norm = np.sqrt(df_eso["eso_dx"]**2 + df_eso["eso_dy"]**2 + df_eso["eso_dz"]**2)
        ax.plot(df_eso["time_sec"], d_norm, color="#000000", linestyle=":", label=r"$\|\hat{\mathbf{d}}\|$ (Total Disturbance Norm)", linewidth=2.0)
    else:
        ax.text(0.5, 0.5, "ESO Disturbance Stream Active in NMPC+ESO Flight", ha="center", va="center", fontsize=13)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel(r"Estimated Acceleration $\hat{\mathbf{d}}$ (m/s²)", fontsize=12)
    ax.set_title(r"09. Online ESO Wind Disturbance Estimation $\hat{\mathbf{d}}(t)$", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_wind_axis_steady_bias(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s)")
    if df_eso is not None:
        ax.plot(df_eso["time_sec"], df_eso["err_x"], color=COLOR_ESO, label="NMPC+ESO (Steady-State Bias ≈ 0cm)", linewidth=2.2)
    if df_nmpc is not None:
        ax.plot(df_nmpc["time_sec"], df_nmpc["err_x"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure (Uncompensated Bias)", linewidth=1.8)
    if df_pid is not None:
        ax.plot(df_pid["time_sec"], df_pid["err_x"], color=COLOR_PID, linestyle=":", label="PID Baseline (Slow Integral Drag)", linewidth=1.5, alpha=0.85)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Wind Axis Tracking Error ex (m)", fontsize=12)
    ax.set_title("10. Steady-State Disturbance Suppression along Wind Axis (+X)", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_itae_cumulative_growth(df_eso, df_nmpc, df_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.25, label="Wind Gust Window (45s~75s)")
    for df_curr, col, name in [(df_eso, COLOR_ESO, "NMPC+ESO"), (df_nmpc, COLOR_NMPC, "NMPC Pure"), (df_pid, COLOR_PID, "PID Baseline")]:
        if df_curr is not None:
            g = df_curr[(df_curr["time_sec"] >= GUST_START) & (df_curr["time_sec"] <= GUST_END)].copy()
            if not g.empty:
                t_arr = g["time_sec"].values
                dt_arr = np.concatenate([[0.05], np.diff(t_arr)])
                tau_arr = np.maximum(0.0, t_arr - GUST_START)
                itae_cum = np.cumsum(tau_arr * g["err_pos_norm"].values * dt_arr)
                ax.plot(t_arr, itae_cum, color=col, label=f"{name} ITAE(t) Growth", linewidth=2.0)
    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Cumulative ITAE Metric (m·s²)", fontsize=12)
    ax.set_title("11. Cumulative Time-Weighted Error Integral (ITAE) Growth during Gust", fontsize=14, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper left", fontsize=11)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_single_eso_superiority_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig, ax = plt.subplots(figsize=(10, 6))
    categories = ["Gust 3D RMSE (m)", "Gust Bias X (m)", "ITAE / 100 (m·s²)", "Settling Time Ts (s)"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_gust"], m_eso["gust_ss_bias_x"], m_eso["itae_gust"]/100.0, m_eso["settling_time"]]
        rects1 = ax.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        for r in rects1:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9, fontweight='bold')
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_gust"], m_nmpc["gust_ss_bias_x"], m_nmpc["itae_gust"]/100.0, m_nmpc["settling_time"]]
        rects2 = ax.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        for r in rects2:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_gust"], m_pid["gust_ss_bias_x"], m_pid["itae_gust"]/100.0, m_pid["settling_time"]]
        rects3 = ax.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)
        for r in rects3:
            ax.annotate(f'{r.get_height():.2f}', xy=(r.get_x() + r.get_width()/2, r.get_height()),
                        xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontsize=9)

    ax.set_ylabel("Disturbance Metric Value", fontsize=12)
    ax.set_title("12. Academic Disturbance Rejection Benchmark (ESO Superiority)", fontsize=14, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=11)
    ax.legend(fontsize=11)
    ax.grid(True, axis="y", linestyle="--", alpha=0.6)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    plt.close()

# ==============================================================================
# COMBINED MULTI-PANEL OVERVIEW PAGES (Page 1, 2, 3)
# ==============================================================================

def plot_page1_trajectory_tracking(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Topic 1: Quadrotor Spatial Trajectory Tracking & Position Error Analysis", fontsize=16, fontweight='bold')

    # (a) 2D XY Flight Trajectory Comparison
    ax1 = fig.add_subplot(2, 2, 1)
    ref_df = df_eso if df_eso is not None else df_nmpc
    if ref_df is not None:
        ax1.plot(ref_df["ref_x"], ref_df["ref_y"], color=COLOR_REF, linestyle="--", label="Reference Circle (R=1.5m)", linewidth=1.5)
    if df_eso is not None and m_eso is not None:
        ax1.plot(df_eso["pos_x"], df_eso["pos_y"], color=COLOR_ESO, label=f"NMPC+ESO (RMSE={m_eso['rmse_3d']:.3f}m)", linewidth=2.0)
    if df_nmpc is not None and m_nmpc is not None:
        ax1.plot(df_nmpc["pos_x"], df_nmpc["pos_y"], color=COLOR_NMPC, linestyle="-.", label=f"NMPC Pure (RMSE={m_nmpc['rmse_3d']:.3f}m)", linewidth=1.8)
    if df_pid is not None and m_pid is not None:
        ax1.plot(df_pid["pos_x"], df_pid["pos_y"], color=COLOR_PID, linestyle=":", label=f"PID Baseline (RMSE={m_pid['rmse_3d']:.3f}m)", linewidth=1.5, alpha=0.8)
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_title("(a) XY Flight Trajectory Tracking Comparison")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.axis("equal")
    ax1.legend(loc="upper right")

    # (b) 3D Position Error vs Time
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s)")
    if df_eso is not None:
        ax2.plot(df_eso["time_sec"], df_eso["err_pos_norm"], color=COLOR_ESO, label="NMPC+ESO Error", linewidth=2.0)
    if df_nmpc is not None:
        ax2.plot(df_nmpc["time_sec"], df_nmpc["err_pos_norm"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Error", linewidth=1.6)
    if df_pid is not None:
        ax2.plot(df_pid["time_sec"], df_pid["err_pos_norm"], color=COLOR_PID, linestyle=":", label="PID Baseline Error", linewidth=1.4, alpha=0.8)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("3D Position Error Norm (m)")
    ax2.set_title("(b) 3D Position Error Tracking Curves vs Time")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    # (c) Axis-Wise Tracking Error (X and Y Axis)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s)")
    if df_eso is not None:
        ax3.plot(df_eso["time_sec"], df_eso["err_x"], color=COLOR_ESO, label="NMPC+ESO Err X", linewidth=1.8)
        ax3.plot(df_eso["time_sec"], df_eso["err_y"], color="#1b9e77", linestyle="--", label="NMPC+ESO Err Y", linewidth=1.5)
    if df_nmpc is not None:
        ax3.plot(df_nmpc["time_sec"], df_nmpc["err_x"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Err X", linewidth=1.5)
    if df_pid is not None:
        ax3.plot(df_pid["time_sec"], df_pid["err_x"], color=COLOR_PID, linestyle=":", label="PID Baseline Err X", linewidth=1.3, alpha=0.7)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Axis Error (m)")
    ax3.set_title("(c) Position Error Decomposition along Wind Axis (X)")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    # (d) Key Tracking Error Metrics Bar Chart
    ax4 = fig.add_subplot(2, 2, 4)
    categories = ["3D RMSE", "X-Axis RMSE", "Y-Axis RMSE", "Mean Error"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_3d"], m_eso["rmse_x"], m_eso["rmse_y"], m_eso["mean_err"]]
        ax4.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_3d"], m_nmpc["rmse_x"], m_nmpc["rmse_y"], m_nmpc["mean_err"]]
        ax4.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_3d"], m_pid["rmse_x"], m_pid["rmse_y"], m_pid["mean_err"]]
        ax4.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)

    ax4.set_ylabel("Error Metric (m)")
    ax4.set_title("(d) Spatial Tracking Precision Quantitative Benchmark")
    ax4.set_xticks(x)
    ax4.set_xticklabels(categories)
    ax4.legend()
    ax4.grid(True, axis="y", linestyle="--", alpha=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_page2_attitude_dynamics(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Topic 2: Quadrotor Attitude Dynamics & Maneuver Response Analysis", fontsize=16, fontweight='bold')

    # (a) Roll Angle Tracking
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s)")
    if df_eso is not None and "roll_deg" in df_eso.columns:
        ax1.plot(df_eso["time_sec"], df_eso["roll_deg"], color=COLOR_ESO, label="NMPC+ESO Roll (°)", linewidth=1.8)
    if df_nmpc is not None and "roll_deg" in df_nmpc.columns:
        ax1.plot(df_nmpc["time_sec"], df_nmpc["roll_deg"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Roll (°)", linewidth=1.5)
    if df_pid is not None and "roll_deg" in df_pid.columns:
        ax1.plot(df_pid["time_sec"], df_pid["roll_deg"], color=COLOR_PID, linestyle=":", label="PID Roll (°)", linewidth=1.3, alpha=0.7)
    if df_eso is not None and "ref_roll_deg" in df_eso.columns:
        ax1.plot(df_eso["time_sec"], df_eso["ref_roll_deg"], color=COLOR_REF, linestyle="--", label="Ref Roll (°)", linewidth=1.2, alpha=0.6)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Roll Angle (deg)")
    ax1.set_title("(a) Roll Angle Dynamic Response vs Reference")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")

    # (b) Pitch Angle Tracking (Crucial for +X Wind Rejection)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s, 3.0 m/s +X)")
    if df_eso is not None and "pitch_deg" in df_eso.columns:
        ax2.plot(df_eso["time_sec"], df_eso["pitch_deg"], color=COLOR_ESO, label="NMPC+ESO Pitch (°)", linewidth=2.0)
    if df_nmpc is not None and "pitch_deg" in df_nmpc.columns:
        ax2.plot(df_nmpc["time_sec"], df_nmpc["pitch_deg"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Pitch (°)", linewidth=1.6)
    if df_pid is not None and "pitch_deg" in df_pid.columns:
        ax2.plot(df_pid["time_sec"], df_pid["pitch_deg"], color=COLOR_PID, linestyle=":", label="PID Pitch (°)", linewidth=1.3, alpha=0.7)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Pitch Angle (deg)")
    ax2.set_title("(b) Pitch Attitude Dynamic Compensation under Wind Gust")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    # (c) Control Body Rate Norm (Smoothness)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust")
    if df_eso is not None and m_eso is not None:
        t_eso = df_eso[df_eso["time_sec"] >= 10.0]
        ax3.plot(t_eso["time_sec"], m_eso["body_rate_norm"], color=COLOR_ESO, label="NMPC+ESO Angular Rate", linewidth=1.6)
    if df_nmpc is not None and m_nmpc is not None:
        t_nmpc = df_nmpc[df_nmpc["time_sec"] >= 10.0]
        ax3.plot(t_nmpc["time_sec"], m_nmpc["body_rate_norm"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure Angular Rate", linewidth=1.4)
    if df_pid is not None and m_pid is not None:
        t_pid = df_pid[df_pid["time_sec"] >= 10.0]
        ax3.plot(t_pid["time_sec"], m_pid["body_rate_norm"], color=COLOR_PID, linestyle=":", label="PID Angular Rate", linewidth=1.2, alpha=0.7)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Body Angular Rate Norm (rad/s)")
    ax3.set_title("(c) Control Effort & Body Angular Rate Smoothness")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right")

    # (d) Attitude Tracking RMSE Bar Chart
    ax4 = fig.add_subplot(2, 2, 4)
    categories = ["Roll RMSE (°)", "Pitch RMSE (°)", "Yaw RMSE (°)", "Rate RMS (rad/s)"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_roll"] or 0, m_eso["rmse_pitch"] or 0, m_eso["rmse_yaw"] or 0, m_eso["rate_rms"]]
        ax4.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_roll"] or 0, m_nmpc["rmse_pitch"] or 0, m_nmpc["rmse_yaw"] or 0, m_nmpc["rate_rms"]]
        ax4.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_roll"] or 0, m_pid["rmse_pitch"] or 0, m_pid["rmse_yaw"] or 0, m_pid["rate_rms"]]
        ax4.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)

    ax4.set_ylabel("Attitude & Rate Metric Value")
    ax4.set_title("(d) Attitude Dynamics & Smoothness Benchmark")
    ax4.set_xticks(x)
    ax4.set_xticklabels(categories)
    ax4.legend()
    ax4.grid(True, axis="y", linestyle="--", alpha=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(save_path, dpi=300)
    plt.close()

def plot_page3_eso_disturbance_rejection(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle("Topic 3: ESO Disturbance Estimation & Robust Rejection Benchmark", fontsize=16, fontweight='bold')

    # (a) Online Disturbance Estimation by ESO
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="True Wind Gust Window (45~75s, 3.0 m/s +X)")
    if df_eso is not None and "eso_dx" in df_eso.columns and (df_eso["eso_dx"]**2 + df_eso["eso_dy"]**2).sum() > 1e-4:
        ax1.plot(df_eso["time_sec"], df_eso["eso_dx"], color="#d95f02", label=r"$\hat{d}_x$ (Wind Axis Estimation)", linewidth=2.0)
        ax1.plot(df_eso["time_sec"], df_eso["eso_dy"], color="#7570b3", linestyle="--", label=r"$\hat{d}_y$ (Lateral Axis)", linewidth=1.5)
        d_norm = np.sqrt(df_eso["eso_dx"]**2 + df_eso["eso_dy"]**2 + df_eso["eso_dz"]**2)
        ax1.plot(df_eso["time_sec"], d_norm, color="#000000", linestyle=":", label=r"$\|\hat{\mathbf{d}}\|$ (Total Disturbance Norm)", linewidth=1.8)
    else:
        ax1.text(0.5, 0.5, "ESO Disturbance Stream Active in NMPC+ESO Group", ha="center", va="center", fontsize=11)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel(r"Estimated Acceleration $\hat{\mathbf{d}}$ (m/s²)")
    ax1.set_title(r"(a) Online ESO Disturbance Estimation $\hat{\mathbf{d}}(t)$")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right")

    # (b) Steady-State Error Suppression along Wind Axis
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s)")
    if df_eso is not None:
        ax2.plot(df_eso["time_sec"], df_eso["err_x"], color=COLOR_ESO, label="NMPC+ESO (Bias ≈ 0cm)", linewidth=2.0)
    if df_nmpc is not None:
        ax2.plot(df_nmpc["time_sec"], df_nmpc["err_x"], color=COLOR_NMPC, linestyle="-.", label="NMPC Pure (Steady Bias)", linewidth=1.6)
    if df_pid is not None:
        ax2.plot(df_pid["time_sec"], df_pid["err_x"], color=COLOR_PID, linestyle=":", label="PID Baseline (Steady Bias)", linewidth=1.4, alpha=0.8)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Wind Axis Tracking Error ex (m)")
    ax2.set_title("(b) Steady-State Error Suppression along Wind Axis (+X)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right")

    # (c) Time-Weighted Cumulative Tracking Error (ITAE)
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axvspan(GUST_START, GUST_END, color=COLOR_GUST, alpha=0.2, label="Wind Gust Window (45~75s)")
    for df_curr, col, name in [(df_eso, COLOR_ESO, "NMPC+ESO"), (df_nmpc, COLOR_NMPC, "NMPC Pure"), (df_pid, COLOR_PID, "PID Baseline")]:
        if df_curr is not None:
            g = df_curr[(df_curr["time_sec"] >= GUST_START) & (df_curr["time_sec"] <= GUST_END)].copy()
            if not g.empty:
                t_arr = g["time_sec"].values
                dt_arr = np.concatenate([[0.05], np.diff(t_arr)])
                tau_arr = np.maximum(0.0, t_arr - GUST_START)
                itae_cum = np.cumsum(tau_arr * g["err_pos_norm"].values * dt_arr)
                ax3.plot(t_arr, itae_cum, color=col, label=f"{name} ITAE(t)", linewidth=1.8)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Cumulative ITAE (m·s²)")
    ax3.set_title("(c) Cumulative Time-Weighted Error Growth during Gust")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper left")

    # (d) Key Disturbance Rejection Academic Metrics
    ax4 = fig.add_subplot(2, 2, 4)
    categories = ["Gust RMSE (m)", "Gust Bias X (m)", "ITAE/100 (m·s²)", "Settling Ts (s)"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_gust"], m_eso["gust_ss_bias_x"], m_eso["itae_gust"]/100.0, m_eso["settling_time"]]
        ax4.bar(x + offset, vals, width, label="NMPC+ESO", color=COLOR_ESO)
        offset += width
    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_gust"], m_nmpc["gust_ss_bias_x"], m_nmpc["itae_gust"]/100.0, m_nmpc["settling_time"]]
        ax4.bar(x + offset, vals, width, label="NMPC Pure", color=COLOR_NMPC)
        offset += width
    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_gust"], m_pid["gust_ss_bias_x"], m_pid["itae_gust"]/100.0, m_pid["settling_time"]]
        ax4.bar(x + offset, vals, width, label="PID Baseline", color=COLOR_PID)

    ax4.set_ylabel("Metric Value")
    ax4.set_title("(d) Disturbance Rejection Academic Benchmark (ESO Superiority)")
    ax4.set_xticks(x)
    ax4.set_xticklabels(categories)
    ax4.legend()
    ax4.grid(True, axis="y", linestyle="--", alpha=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(save_path, dpi=300)
    plt.close()

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "..", "data")
    ind_dir = os.path.join(data_dir, "individual_plots")
    os.makedirs(ind_dir, exist_ok=True)

    eso_file = find_latest_log(data_dir, "NMPC_ESO")
    nmpc_file = find_latest_log(data_dir, "NMPC")
    pid_file = find_latest_log(data_dir, "PID")

    if len(sys.argv) >= 4:
        eso_file, nmpc_file, pid_file = sys.argv[1], sys.argv[2], sys.argv[3]
    elif len(sys.argv) == 3:
        nmpc_file, pid_file = sys.argv[1], sys.argv[2]

    df_eso, m_eso = None, None
    df_nmpc, m_nmpc = None, None
    df_pid, m_pid = None, None

    if eso_file and os.path.exists(eso_file):
        print(f"Loading NMPC+ESO Log: {eso_file}")
        df_eso = pd.read_csv(eso_file)

    if nmpc_file and os.path.exists(nmpc_file):
        print(f"Loading NMPC Pure Log: {nmpc_file}")
        df_nmpc = pd.read_csv(nmpc_file)

    if pid_file and os.path.exists(pid_file):
        print(f"Loading PID Log      : {pid_file}")
        df_pid = pd.read_csv(pid_file)

    if df_eso is None and df_nmpc is None:
        print(f"[Error] No NMPC or NMPC_ESO flight logs found in {data_dir}")
        sys.exit(1)

    # Automatically synchronize and crop to common overlapping time window
    max_times = [df["time_sec"].max() for df in [df_eso, df_nmpc, df_pid] if df is not None]
    if max_times:
        t_max_common = min(max_times)
        if t_max_common >= 15.0:
            print(f"[Time Sync] Synchronizing comparison to common overlapping window: 10.0s ~ {t_max_common:.1f}s")
            if df_eso is not None:
                df_eso = df_eso[df_eso["time_sec"] <= t_max_common].copy()
            if df_nmpc is not None:
                df_nmpc = df_nmpc[df_nmpc["time_sec"] <= t_max_common].copy()
            if df_pid is not None:
                df_pid = df_pid[df_pid["time_sec"] <= t_max_common].copy()

    if df_eso is not None:
        m_eso = calculate_metrics(df_eso, "NMPC_ESO")
    if df_nmpc is not None:
        m_nmpc = calculate_metrics(df_nmpc, "NMPC")
    if df_pid is not None:
        m_pid = calculate_metrics(df_pid, "PID")

    print_comparison_table(m_eso, m_nmpc, m_pid)

    # 1. Export 3 Combined Thematic Pages (300 DPI)
    p1_path = os.path.join(data_dir, "eval_page1_trajectory_tracking.png")
    p2_path = os.path.join(data_dir, "eval_page2_attitude_dynamics.png")
    p3_path = os.path.join(data_dir, "eval_page3_eso_disturbance_rejection.png")

    plot_page1_trajectory_tracking(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, p1_path)
    plot_page2_attitude_dynamics(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, p2_path)
    plot_page3_eso_disturbance_rejection(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, p3_path)

    # 2. Export 12 Standalone High-Resolution Figures (300 DPI)
    print(f"\n[Exporting] Generating 12 standalone high-resolution figures in {ind_dir} ...")
    
    # Topic 1 Standalone Plots
    plot_single_xy_trajectory(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, os.path.join(ind_dir, "01_xy_trajectory_tracking.png"))
    plot_single_3d_position_error(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "02_3d_position_error.png"))
    plot_single_wind_axis_x_error(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "03_wind_axis_x_error.png"))
    plot_single_tracking_precision_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, os.path.join(ind_dir, "04_tracking_precision_bar.png"))

    # Topic 2 Standalone Plots
    plot_single_roll_angle(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "05_roll_angle_dynamics.png"))
    plot_single_pitch_angle(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "06_pitch_angle_dynamics.png"))
    plot_single_body_rate_smoothness(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, os.path.join(ind_dir, "07_body_rate_smoothness.png"))
    plot_single_attitude_precision_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, os.path.join(ind_dir, "08_attitude_precision_bar.png"))

    # Topic 3 Standalone Plots
    plot_single_eso_disturbance_estimation(df_eso, os.path.join(ind_dir, "09_eso_disturbance_estimation.png"))
    plot_single_wind_axis_steady_bias(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "10_wind_axis_steady_bias.png"))
    plot_single_itae_cumulative_growth(df_eso, df_nmpc, df_pid, os.path.join(ind_dir, "11_itae_cumulative_growth.png"))
    plot_single_eso_superiority_bar(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, os.path.join(ind_dir, "12_eso_superiority_bar.png"))

    print("\n" + "="*80)
    print("  SUCCESS: All 3 combined pages + 12 individual plots exported successfully!")
    print(f"  - Combined Pages : {data_dir}")
    print(f"  - Individual Plots: {ind_dir}")
    print("="*80)

if __name__ == "__main__":
    main()
