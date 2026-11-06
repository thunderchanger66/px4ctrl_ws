#!/usr/bin/env python3
"""Summarize a run_performance.py CSV into engineering metrics."""

import argparse
import csv
import json
import math
import statistics


def f(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else math.nan
    except (KeyError, ValueError, TypeError):
        return math.nan


def mean(values):
    values = [v for v in values if math.isfinite(v)]
    return statistics.fmean(values) if values else math.nan


def percentile(values, p):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return math.nan
    return values[min(len(values) - 1, round((len(values) - 1) * p))]


def quat_error(row):
    a = [f(row, f"att_q_{k}") for k in "wxyz"]
    b = [f(row, f"sp_q_{k}") for k in "wxyz"]
    if not all(math.isfinite(v) for v in a + b):
        return math.nan
    na = math.sqrt(sum(v*v for v in a)); nb = math.sqrt(sum(v*v for v in b))
    if na < 1e-8 or nb < 1e-8:
        return math.nan
    dot = abs(sum(x*y for x, y in zip(a, b)) / (na*nb))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True)
    p.add_argument("--events", required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args()
    with open(args.csv, newline="", encoding="utf-8") as s:
        rows = list(csv.DictReader(s))
    with open(args.events, encoding="utf-8") as s:
        events = json.load(s)
    dt = [f(b, "elapsed_s") - f(a, "elapsed_s") for a, b in zip(rows, rows[1:])]
    path_errors, xy_errors, z_errors, attitude_errors, speeds = [], [], [], [], []
    for row in rows:
        if all(math.isfinite(f(row, k)) for k in ("cmd_x", "cmd_y", "cmd_z", "odom_x", "odom_y", "odom_z")):
            dx = f(row, "cmd_x") - f(row, "odom_x")
            dy = f(row, "cmd_y") - f(row, "odom_y")
            dz = f(row, "cmd_z") - f(row, "odom_z")
            path_errors.append(math.sqrt(dx*dx + dy*dy + dz*dz))
            xy_errors.append(math.hypot(dx, dy))
            z_errors.append(abs(dz))
        attitude_errors.append(quat_error(row))
        speeds.append(math.sqrt(sum(f(row, k)**2 for k in ("odom_vx_body", "odom_vy_body", "odom_vz_body"))))
    max_speed = max((v for v in speeds if math.isfinite(v)), default=math.nan)
    thrust = [abs(f(r, "thrust_z")) for r in rows]
    valid_thrust = [v for v in thrust if math.isfinite(v)]
    thrust_sat = sum(v >= 0.949 for v in valid_thrust) / len(valid_thrust) if valid_thrust else math.nan
    max_thrust = max(valid_thrust, default=math.nan)
    attitude_valid = [v for v in attitude_errors if math.isfinite(v)]
    failures = sum(f(r, "failsafe") == 1 or f(r, "offboard_lost") == 1 for r in rows)
    landed_midair = any(f(r, "landed") == 1 and f(r, "arming_state") != 1 for r in rows)
    emergency_count = max((f(r, "planner_emergency_count") for r in rows if math.isfinite(f(r, "planner_emergency_count"))), default=0)
    hold_rows = [r for r in rows if rows and f(r, "elapsed_s") >= f(rows[-1], "elapsed_s") - 5.0]
    hold_xyz = [[f(r, key) for r in hold_rows] for key in ("odom_x", "odom_y", "odom_z")]
    hold_speeds = [math.sqrt(sum(f(r, k)**2 for k in ("odom_vx_body", "odom_vy_body", "odom_vz_body"))) for r in hold_rows]
    resets = {}
    for key in ("xy_reset", "z_reset", "heading_reset"):
        vals = [f(r, key) for r in rows if math.isfinite(f(r, key))]
        resets[key] = int(max(vals) - min(vals)) if vals else None

    lines = ["# PX4Ctrl SITL 性能测试结果", "", f"采样数：{len(rows)}，总时长：{f(rows[-1], 'elapsed_s'):.1f} s" if rows else "采样数：0", "",
             "## 全程指标", "", "| 指标 | 实测 |", "|---|---:|",
             f"| 采样频率中位数 | {1 / statistics.median(dt):.2f} Hz |" if dt and statistics.median(dt) > 0 else "| 采样频率中位数 | N/A |",
             f"| 采样间隔 P95 | {percentile(dt, .95)*1000:.1f} ms |" if dt else "| 采样间隔 P95 | N/A |",
             f"| 命令到 FAST-LIO 路径误差 RMSE | {math.sqrt(mean([v*v for v in path_errors])):.3f} m |" if path_errors else "| 命令到 FAST-LIO 路径误差 RMSE | N/A |",
             f"| 路径误差 P95 / 最大值 | {percentile(path_errors, .95):.3f} / {max(path_errors):.3f} m |" if path_errors else "| 路径误差 P95 / 最大值 | N/A |",
             f"| 水平 / 垂直误差 RMSE | {math.sqrt(mean([v*v for v in xy_errors])):.3f} / {math.sqrt(mean([v*v for v in z_errors])):.3f} m |" if xy_errors else "| 水平 / 垂直误差 RMSE | N/A |",
             f"| FAST-LIO 速度峰值 | {max_speed:.3f} m/s |" if math.isfinite(max_speed) else "| FAST-LIO 速度峰值 | N/A |",
             f"| 最后 5 s 悬停位置标准差 X/Y/Z | {statistics.pstdev(hold_xyz[0]):.3f} / {statistics.pstdev(hold_xyz[1]):.3f} / {statistics.pstdev(hold_xyz[2]):.3f} m |" if len(hold_rows) > 1 else "| 最后 5 s 悬停位置标准差 X/Y/Z | N/A |",
             f"| 最后 5 s 悬停速度 RMS / 峰值 | {math.sqrt(mean([v*v for v in hold_speeds])):.3f} / {max(hold_speeds):.3f} m/s |" if hold_speeds else "| 最后 5 s 悬停速度 | N/A |",
             f"| 姿态指令跟踪角误差均值 / P95 / 最大值 | {mean(attitude_errors):.2f} / {percentile(attitude_errors,.95):.2f} / {max(attitude_valid):.2f} deg |" if attitude_valid else "| 姿态指令跟踪角误差 | N/A |",
             f"| 推力饱和代理（|thrust_z| >= 0.949） | {thrust_sat*100:.2f}% 样本，峰值 {max_thrust:.3f} |" if valid_thrust else "| 推力饱和代理 | N/A |",
             f"| PX4 failsafe / Offboard lost 样本 | {failures} |", f"| Planner emergency 计数最大值 | {emergency_count:.0f} |",
             f"| EKF reset 计数变化 XY / Z / heading | {resets['xy_reset']} / {resets['z_reset']} / {resets['heading_reset']} |", "",
             "## 航点结果", "", "| 航段 | 目标 (lidar) | 结果 | 到点时间 | 到点误差 | 到点速度 | 超调 |", "|---|---|---|---:|---:|---:|---:|"]
    for e in events.get("goals", []):
        if "target_lidar" not in e:
            continue
        target = ", ".join(f"{x:.2f}" for x in e["target_lidar"])
        elapsed = f"{e['done_elapsed_s'] - e['sent_elapsed_s']:.1f} s" if e.get("result") == "reached" else "超时/中止"
        error = f"{e.get('final_error_m', math.nan):.3f} m" if e.get("result") == "reached" else "-"
        speed = f"{e.get('final_speed_mps', math.nan):.3f} m/s" if e.get("result") == "reached" else "-"
        overshoot = math.nan
        if e.get("result") == "reached":
            start, goal = e["start_lidar"], e["target_lidar"]
            delta = [b - a for a, b in zip(start, goal)]
            length = math.sqrt(sum(v*v for v in delta))
            segment = [r for r in rows if e["sent_elapsed_s"] <= f(r, "elapsed_s") <= e["done_elapsed_s"]]
            if length > 1e-6 and segment:
                projections = [sum((f(r, k) - a) * v for k, a, v in zip(("odom_x", "odom_y", "odom_z"), start, delta)) / length for r in segment]
                overshoot = max(0.0, max(projections) - length)
        over_text = f"{overshoot:.3f} m" if math.isfinite(overshoot) else "-"
        e["overshoot_m"] = overshoot
        lines.append(f"| {e['index']} | ({target}) | {e.get('result','unknown')} | {elapsed} | {error} | {speed} | {over_text} |")
    all_reached = bool(events.get("goals")) and all(e.get("result") == "reached" for e in events["goals"] if "target_lidar" in e)
    tracking_pass = bool(path_errors) and math.sqrt(mean([v*v for v in path_errors])) <= 0.25 and percentile(path_errors, .95) <= 0.40 and max(path_errors) <= 0.60
    attitude_pass = bool(attitude_valid) and percentile(attitude_errors, .95) <= 10.0
    thrust_pass = math.isfinite(thrust_sat) and thrust_sat < 0.01
    passed = all_reached and failures == 0 and not landed_midair and emergency_count == 0 and tracking_pass and attitude_pass and thrust_pass
    lines += ["", "## 判定", "", f"**{'通过' if passed else '未通过/未完成'}**：航点需在 0.25 m 内、速度不超过 0.20 m/s 并持续稳定 2 s；路径误差 RMSE/P95/最大值门槛为 0.25/0.40/0.60 m；姿态误差 P95 不超过 10 deg；推力饱和代理低于 1%；且无 failsafe、Offboard 丢失、空中落地或 planner emergency。",
              "", "路径误差是同一 lidar 坐标系下 `pos_cmd` 与 FAST-LIO odometry 的位置差；姿态误差是 PX4 实际姿态与 attitude setpoint 的四元数角距离。推力饱和是按配置最大推力 0.95 计算的代理值。", "",
              "原始数据与事件时间分别见同目录 CSV 和 JSON。单次 SITL 试验只描述本次仿真状态，不构成实机性能保证。", ""]
    with open(args.out, "w", encoding="utf-8") as output:
        output.write("\n".join(lines))
    print(f"wrote {args.out}; {'PASS' if passed else 'NOT PASS'}")


if __name__ == "__main__":
    main()
