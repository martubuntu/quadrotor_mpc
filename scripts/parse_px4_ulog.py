#!/usr/bin/env python3
"""
PX4 ULog Parser & Flight Dynamics Analyzer for UAV NMPC Experiments.
Extracts steady-state hover thrust, motor throttle balance, battery voltage, current, and hover power.
"""
import struct
import os
import math
import statistics
import sys

def parse_ulog(filepath):
    filename = os.path.basename(filepath)
    print("=" * 70)
    print(f">> Analyzing PX4 ULog: {filename}")
    print("=" * 70)
    
    with open(filepath, 'rb') as f:
        header = f.read(16)
        if not header.startswith(b'ULog'):
            print(f"Error: {filename} is not a valid ULog binary file.")
            return None
            
        params = {}
        formats = {}
        subscriptions = {}
        
        pos_list = []
        thrust_list = []
        battery_list = []
        motors_list = []
        
        while True:
            h = f.read(3)
            if len(h) < 3:
                break
            msg_size, msg_type = struct.unpack('<HB', h)
            payload = f.read(msg_size)
            if len(payload) < msg_size:
                break
            char_type = chr(msg_type)
            if char_type == 'P':
                klen = payload[0]
                k = payload[1:1+klen].decode('ascii', errors='ignore')
                if len(payload[1+klen:]) == 4:
                    params[k] = struct.unpack('<f', payload[1+klen:])[0]
            elif char_type == 'A':
                multi = payload[0]
                mid = struct.unpack('<H', payload[1:3])[0]
                name = payload[3:].decode('ascii', errors='ignore')
                subscriptions[mid] = (name, multi)
            elif char_type == 'D':
                mid = struct.unpack('<H', payload[:2])[0]
                d = payload[2:]
                if mid in subscriptions:
                    name, multi = subscriptions[mid]
                    if name == 'vehicle_local_position' and multi == 0 and len(d) >= 76:
                        t_us = struct.unpack('<Q', d[:8])[0]
                        x, y, z = struct.unpack('<fff', d[40:52])
                        vx, vy, vz = struct.unpack('<fff', d[64:76])
                        pos_list.append((t_us, x, y, z, vx, vy, vz))
                    elif name == 'vehicle_thrust_setpoint' and multi == 0 and len(d) >= 28:
                        t_us, t_sample, tx, ty, tz = struct.unpack('<QQfff', d[:28])
                        thrust_list.append((t_us, tx, ty, tz))
                    elif name == 'battery_status' and multi == 0 and len(d) >= 20:
                        t_us, v_v, i_a, i_avg = struct.unpack('<Qfff', d[:20])
                        battery_list.append((t_us, v_v, i_a))
                    elif name == 'actuator_motors' and multi == 0 and len(d) >= 32:
                        t_us = struct.unpack('<Q', d[:8])[0]
                        motors = struct.unpack('<4f', d[16:32])
                        motors_list.append((t_us, motors))

    if not pos_list or not thrust_list:
        print(f"Warning: No valid position or thrust telemetry in {filename}")
        return None

    t0 = pos_list[0][0]
    pos_t = [(p[0] - t0) * 1e-6 for p in pos_list]
    ground_z = pos_list[0][3]
    rel_alt = [ground_z - p[3] for p in pos_list]
    
    # Identify in-air steady hover window (between 25s and 65s or automatic)
    hover_thrusts = []
    for row in thrust_list:
        t = (row[0] - t0) * 1e-6
        if 25.0 <= t <= 65.0:
            hover_thrusts.append(-row[3])
            
    hover_bats = []
    for row in battery_list:
        t = (row[0] - t0) * 1e-6
        if 25.0 <= t <= 65.0:
            hover_bats.append((row[1], row[2], row[1] * row[2]))
            
    hover_motors = []
    for row in motors_list:
        t = (row[0] - t0) * 1e-6
        if 25.0 <= t <= 65.0:
            hover_motors.append(row[1])

    if not hover_thrusts:
        print(f"Warning: Could not isolate hover window for {filename}")
        return None

    mean_th = statistics.mean(hover_thrusts)
    std_th = statistics.stdev(hover_thrusts) if len(hover_thrusts) > 1 else 0.0
    mean_v = statistics.mean([b[0] for b in hover_bats]) if hover_bats else 0.0
    mean_i = statistics.mean([b[1] for b in hover_bats]) if hover_bats else 0.0
    mean_p = statistics.mean([b[2] for b in hover_bats]) if hover_bats else 0.0
    
    m1 = statistics.mean([m[0] for m in hover_motors]) if hover_motors else 0.0
    m2 = statistics.mean([m[1] for m in hover_motors]) if hover_motors else 0.0
    m3 = statistics.mean([m[2] for m in hover_motors]) if hover_motors else 0.0
    m4 = statistics.mean([m[3] for m in hover_motors]) if hover_motors else 0.0

    print(f"  * Steady Hover Thrust:  {mean_th:.4f} (+/- {std_th:.4f})")
    print(f"  * Average Hover Power:  {mean_p:.1f} W ({mean_i:.2f} A @ {mean_v:.2f} V)")
    print(f"  * Motor Throttle Split: M1={m1:.3f}, M2={m2:.3f}, M3={m3:.3f}, M4={m4:.3f}")
    
    return {
        'filename': filename,
        'mean_thrust': mean_th,
        'std_thrust': std_th,
        'power': mean_p,
        'voltage': mean_v,
        'current': mean_i,
        'm1': m1, 'm2': m2, 'm3': m3, 'm4': m4
    }

def main():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(pkg_dir, "data", "px4_flyrecord")
    
    if len(sys.argv) > 1:
        data_dir = sys.argv[1]
        
    print(f"Scanning for ULog files in: {data_dir}")
    if not os.path.exists(data_dir):
        print(f"Directory not found: {data_dir}")
        return

    ulog_files = [os.path.join(data_dir, f) for f in sorted(os.listdir(data_dir)) if f.endswith('.ulg')]
    if not ulog_files:
        print("No .ulg files found.")
        return

    results = []
    for f in ulog_files:
        res = parse_ulog(f)
        if res:
            results.append(res)
            
    if results:
        overall_thrust = statistics.mean([r['mean_thrust'] for r in results])
        overall_power = statistics.mean([r['power'] for r in results])
        overall_voltage = statistics.mean([r['voltage'] for r in results])
        overall_current = statistics.mean([r['current'] for r in results])
        
        print("\n" + "=" * 70)
        print(">>> COMBINED PX4 FLIGHT CALIBRATION SUMMARY <<<")
        print("=" * 70)
        print(f"  * Recommended YAML 'hover_thrust': {overall_thrust:.4f} (Round: {round(overall_thrust, 2)})")
        print(f"  * Average Steady Hover Power:     {overall_power:.2f} W")
        print(f"  * Average Steady Hover Current:   {overall_current:.2f} A @ {overall_voltage:.2f} V")
        print("=" * 70)

if __name__ == '__main__':
    main()
