#!/usr/bin/env python3
"""Measure the rate of every topic a generated scenario should publish and flag missing or slow ones.

  python3 scripts/check_sensors.py generated/arena_solo/scenario_info.yaml [seconds] [--agent blimp]
"""
import argparse
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("info")
    ap.add_argument("seconds", nargs="?", type=float, default=5.0)
    ap.add_argument("--agent")
    a = ap.parse_args(rclpy.utilities.remove_ros_args()[1:])
    with open(a.info) as f:
        info = yaml.safe_load(f)
    expected = {}
    for ag in info["agents"]:
        if a.agent and ag["name"] != a.agent:
            continue
        expected.update(ag.get("expected_topics", {}))
    rclpy.init()
    n = Node("check_sensors")
    counts = {t: 0 for t in expected}
    for t, spec in expected.items():
        n.create_subscription(get_message(spec["type"]), t, lambda m, t=t: counts.__setitem__(t, counts[t] + 1),
                              qos_profile_sensor_data)
    t_end = time.time() + 1.0
    while time.time() < t_end:  # let discovery settle
        rclpy.spin_once(n, timeout_sec=0.05)
    for t in counts:
        counts[t] = 0
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
    dur = time.time() - t0
    bad = 0
    print(f"{'topic':<46} {'expected':>9} {'measured':>9}")
    for t, spec in sorted(expected.items()):
        hz = counts[t] / dur
        flag = "" if hz > 0.6 * spec["rate"] else ("  <-- NO DATA" if hz == 0 else "  <-- LOW (sim slower than real time?)")
        bad += bool(flag)
        print(f"{t:<46} {spec['rate']:>8.0f}  {hz:>8.1f}{flag}")
    print("(rates are wall-clock; with real_time_factor < 1 everything reads low)")
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
