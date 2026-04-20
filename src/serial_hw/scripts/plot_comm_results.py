#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 rosbag 生成通信实验用图（延迟时间序列、延迟直方图、cmd_vel 分量）。
用法:
  python3 plot_comm_results.py comm_test.bag
  rosrun serial_hw plot_comm_results.py comm_test.bag
依赖: rosbag, matplotlib, numpy
"""

from __future__ import print_function

import argparse
import os
import sys

try:
    import rosbag
except ImportError as e:
    print("需要 ROS 环境: source /opt/ros/<distro>/setup.bash && source <ws>/devel/setup.bash", file=sys.stderr)
    raise e

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    print("请安装 matplotlib 与 numpy: pip3 install matplotlib numpy", file=sys.stderr)
    raise e


def _bag_time_to_sec(t):
    return t.to_sec() if hasattr(t, "to_sec") else float(t)


def load_bag(path):
    lat_t = []
    lat_v = []
    cv_t = []
    cv_lx = []
    cv_ly = []
    cv_az = []

    t0 = None
    with rosbag.Bag(path, "r") as bag:
        for topic, msg, t in bag.read_messages(
            topics=["/comm_latency", "/cmd_vel", "/robot2/cmd_vel"]
        ):
            if t0 is None:
                t0 = t
            ts = _bag_time_to_sec(t - t0)
            if topic == "/comm_latency":
                lat_t.append(ts)
                lat_v.append(float(msg.data))
            elif topic in ("/cmd_vel", "/robot2/cmd_vel"):
                cv_t.append(ts)
                cv_lx.append(float(msg.linear.x))
                cv_ly.append(float(msg.linear.y))
                cv_az.append(float(msg.angular.z))

    return (
        np.array(lat_t),
        np.array(lat_v),
        np.array(cv_t),
        np.array(cv_lx),
        np.array(cv_ly),
        np.array(cv_az),
    )


def main():
    ap = argparse.ArgumentParser(description="Plot serial comm test results from a rosbag.")
    ap.add_argument("bag", help="path to .bag file")
    ap.add_argument(
        "-o",
        "--output-prefix",
        default="comm_test",
        help="output PNG basename prefix (default: comm_test -> comm_test_latency_ts.png ...)",
    )
    args = ap.parse_args()

    if not os.path.isfile(args.bag):
        print("文件不存在:", args.bag, file=sys.stderr)
        sys.exit(1)

    lat_t, lat_v, cv_t, cv_lx, cv_ly, cv_az = load_bag(args.bag)

    out1 = args.output_prefix + "_latency_timeseries.png"
    out2 = args.output_prefix + "_latency_hist.png"
    out3 = args.output_prefix + "_cmd_vel.png"

    # 图1：延迟时间序列
    fig1, ax1 = plt.subplots(figsize=(8, 3.5))
    if lat_t.size:
        ax1.plot(lat_t, lat_v * 1000.0, lw=0.8)
        ax1.set_xlabel("Time (s) from bag start")
        ax1.set_ylabel("Latency (ms)")
        ax1.set_title("Serial send latency (callback enqueue to write complete)")
        ax1.grid(True, alpha=0.3)
    else:
        ax1.text(0.5, 0.5, "No /comm_latency in bag", ha="center", va="center")
    fig1.tight_layout()
    fig1.savefig(out1, dpi=200)
    plt.close(fig1)

    # 图2：延迟直方图
    fig2, ax2 = plt.subplots(figsize=(6, 4))
    if lat_v.size:
        ax2.hist(lat_v * 1000.0, bins=40, color="steelblue", edgecolor="white")
        ax2.set_xlabel("Latency (ms)")
        ax2.set_ylabel("Count")
        ax2.set_title("Latency distribution")
        ax2.grid(True, alpha=0.3)
    else:
        ax2.text(0.5, 0.5, "No /comm_latency in bag", ha="center", va="center")
    fig2.tight_layout()
    fig2.savefig(out2, dpi=200)
    plt.close(fig2)

    # 图3：cmd_vel（若 bag 里是 /robot2/cmd_vel 亦可）
    fig3, ax3 = plt.subplots(figsize=(8, 3.5))
    if cv_t.size:
        ax3.plot(cv_t, cv_lx, label="linear.x", lw=0.8)
        ax3.plot(cv_t, cv_ly, label="linear.y", lw=0.8)
        ax3.plot(cv_t, cv_az, label="angular.z", lw=0.8)
        ax3.set_xlabel("Time (s) from bag start")
        ax3.set_ylabel("cmd_vel")
        ax3.set_title("Velocity commands in bag")
        ax3.legend(loc="upper right", fontsize=8)
        ax3.grid(True, alpha=0.3)
    else:
        ax3.text(0.5, 0.5, "No /cmd_vel or /robot2/cmd_vel in bag", ha="center", va="center")
    fig3.tight_layout()
    fig3.savefig(out3, dpi=200)
    plt.close(fig3)

    print("Wrote:", out1, out2, out3)
    if lat_v.size:
        print(
            "Latency (ms): mean={:.3f} std={:.3f} min={:.3f} max={:.3f} n={}".format(
                float(np.mean(lat_v) * 1000.0),
                float(np.std(lat_v) * 1000.0),
                float(np.min(lat_v) * 1000.0),
                float(np.max(lat_v) * 1000.0),
                int(lat_v.size),
            )
        )


if __name__ == "__main__":
    main()
