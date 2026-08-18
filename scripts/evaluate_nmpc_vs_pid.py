#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Dimensional Academic Performance Evaluation: NMPC + ESO vs NMPC vs PID Baseline
Simulation under Wind Gust Disturbance (90s ~ 120s, 3.0 m/s +X)

Metrics include:
  1. 3D Position Tracking RMSE & Max/Mean Error
  2. Attitude Tracking RMSE (Roll, Pitch, Yaw)
  3. ESO Disturbance Rejection Metrics (Steady-State Bias, IAE, ITAE, Settling Time, Amplification)
  4. Control Smoothness, Chattering Index (Total Variation) & Energy Efficiency

Usage:
    python3 evaluate_nmpc_vs_pid.py [nmpc_eso_log.csv] [nmpc_log.csv] [pid_log.csv]
If arguments are omitted, it automatically picks the latest logs from data/.
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
            non_special = [f for f in all_files if "PID" not in f and "ESO" not in f]
            if non_special:
                return max(non_special, key=os.path.getctime)
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

    # IAE & ITAE Integral Error Criteria during Gust Phase (90s ~ 120s)
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
    print("      Wind Gust: 90s - 120s (+X 3.0 m/s) | Circle Radius: 1.5m, Vel: 0.8m/s")
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
        ("Gust Phase 3D RMSE (90-120s) (m)", "rmse_gust"),
        ("Calm Phase 3D RMSE (20-90s) (m)", "rmse_calm"),
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

def plot_comparison(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_path):
    fig = plt.figure(figsize=(18, 12))

    # 1. XY Flight Trajectory Comparison
    ax1 = fig.add_subplot(2, 2, 1)
    ref_df = df_eso if df_eso is not None else df_nmpc
    if ref_df is not None:
        ax1.plot(ref_df["ref_x"], ref_df["ref_y"], "k--", label="Reference Circle", linewidth=1.5)
    
    if df_eso is not None and m_eso is not None:
        ax1.plot(df_eso["pos_x"], df_eso["pos_y"], "g-", label=f"NMPC+ESO (RMSE={m_eso['rmse_3d']:.3f}m)", linewidth=2.0)
    if df_nmpc is not None and m_nmpc is not None:
        ax1.plot(df_nmpc["pos_x"], df_nmpc["pos_y"], "b-.", label=f"NMPC Pure (RMSE={m_nmpc['rmse_3d']:.3f}m)", linewidth=1.8)
    if df_pid is not None and m_pid is not None:
        ax1.plot(df_pid["pos_x"], df_pid["pos_y"], "r:", label=f"PID Baseline (RMSE={m_pid['rmse_3d']:.3f}m)", linewidth=1.5, alpha=0.8)
    
    ax1.set_xlabel("X Position (m)")
    ax1.set_ylabel("Y Position (m)")
    ax1.set_title("XY Flight Trajectory Comparison")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.axis("equal")
    ax1.legend(loc="upper right")

    # 2. Tracking Error vs Time with Shaded Wind Gust Window
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.axvspan(GUST_START, GUST_END, color="orange", alpha=0.2, label="Wind Gust Window (3.0 m/s +X)")
    if df_eso is not None:
        ax2.plot(df_eso["time_sec"], df_eso["err_pos_norm"], "g-", label="NMPC+ESO Error", linewidth=2.0)
    if df_nmpc is not None:
        ax2.plot(df_nmpc["time_sec"], df_nmpc["err_pos_norm"], "b-.", label="NMPC Pure Error", linewidth=1.6)
    if df_pid is not None:
        ax2.plot(df_pid["time_sec"], df_pid["err_pos_norm"], "r:", label="PID Baseline Error", linewidth=1.4, alpha=0.8)
    
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("3D Position Error (m)")
    ax2.set_title("Position Tracking Error vs Time (With Wind Gust 90~120s)")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right")

    # 3. Attitude Response (Pitch Angle vs Time) - Direct visual of wind resistance
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.axvspan(GUST_START, GUST_END, color="orange", alpha=0.2, label="Wind Gust Window")
    has_att_plot = False
    if df_eso is not None and "pitch_deg" in df_eso.columns:
        ax3.plot(df_eso["time_sec"], df_eso["pitch_deg"], "g-", label="NMPC+ESO Pitch (°)", linewidth=1.8)
        has_att_plot = True
    if df_nmpc is not None and "pitch_deg" in df_nmpc.columns:
        ax3.plot(df_nmpc["time_sec"], df_nmpc["pitch_deg"], "b-.", label="NMPC Pure Pitch (°)", linewidth=1.5)
        has_att_plot = True
    if df_pid is not None and "pitch_deg" in df_pid.columns:
        ax3.plot(df_pid["time_sec"], df_pid["pitch_deg"], "r:", label="PID Pitch (°)", linewidth=1.3, alpha=0.7)
        has_att_plot = True

    if not has_att_plot:
        if df_eso is not None and m_eso is not None:
            t_eso = df_eso[df_eso["time_sec"] >= 15.0]
            ax3.plot(t_eso["time_sec"], m_eso["thrust_series"], "g-", label="NMPC+ESO Thrust", linewidth=1.8)
        if df_nmpc is not None and m_nmpc is not None:
            t_nmpc = df_nmpc[df_nmpc["time_sec"] >= 15.0]
            ax3.plot(t_nmpc["time_sec"], m_nmpc["thrust_series"], "b-.", label="NMPC Pure Thrust", linewidth=1.5)
        if df_pid is not None and m_pid is not None:
            t_pid = df_pid[df_pid["time_sec"] >= 15.0]
            ax3.plot(t_pid["time_sec"], m_pid["thrust_series"], "r:", label="PID Equivalent Thrust", linewidth=1.3, alpha=0.7)
        ax3.set_ylabel("Thrust (Normalized 0~1)")
        ax3.set_title("Throttle Response under Wind Disturbance")
    else:
        ax3.set_ylabel("Pitch Angle (deg)")
        ax3.set_title("Attitude Pitch Angle Response under Wind Disturbance")

    ax3.set_xlabel("Time (s)")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper right")

    # 4. Multi-Dimensional Performance Bar Chart
    ax4 = fig.add_subplot(2, 2, 4)
    categories = ["3D RMSE (m)", "Gust RMSE (m)", "Gust Bias X (m)", "ITAE/100 (m*s²)"]
    x = np.arange(len(categories))
    width = 0.25

    offset = -width if df_eso is not None and df_nmpc is not None and df_pid is not None else -width/2
    if df_eso is not None and m_eso is not None:
        vals = [m_eso["rmse_3d"], m_eso["rmse_gust"], m_eso["gust_ss_bias_x"], m_eso["itae_gust"]/100.0]
        ax4.bar(x + offset, vals, width, label="NMPC+ESO", color="mediumseagreen")
        offset += width

    if df_nmpc is not None and m_nmpc is not None:
        vals = [m_nmpc["rmse_3d"], m_nmpc["rmse_gust"], m_nmpc["gust_ss_bias_x"], m_nmpc["itae_gust"]/100.0]
        ax4.bar(x + offset, vals, width, label="NMPC Pure", color="royalblue")
        offset += width

    if df_pid is not None and m_pid is not None:
        vals = [m_pid["rmse_3d"], m_pid["rmse_gust"], m_pid["gust_ss_bias_x"], m_pid["itae_gust"]/100.0]
        ax4.bar(x + offset, vals, width, label="PID Baseline", color="salmon")

    ax4.set_ylabel("Metric Value")
    ax4.set_title("Key Disturbance Rejection Comparison (ESO Highlights)")
    ax4.set_xticks(x)
    ax4.set_xticklabels(categories)
    ax4.legend()
    ax4.grid(True, axis="y", linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"[Plot] Evaluation figure saved to: {save_path}")
    plt.show()

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(script_dir, "..", "data")

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
        if t_max_common >= 20.0:
            print(f"[Time Sync] Synchronizing comparison to common overlapping window: 15.0s ~ {t_max_common:.1f}s")
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

    save_plot_path = os.path.join(data_dir, "simulation_evaluation_report.png")
    plot_comparison(df_eso, df_nmpc, df_pid, m_eso, m_nmpc, m_pid, save_plot_path)

if __name__ == "__main__":
    main()
