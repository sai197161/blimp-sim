#!/usr/bin/env python3
"""Score an estimated trajectory against ground truth (numpy only; no ROS needed).

  python3 scripts/eval_traj.py results/run1/gt.tum results/run1/est.tum \
      --label rtabmap_stereo --stack full_stack --summary results/summary.csv [--plot]
  python3 scripts/eval_traj.py --table results/summary.csv          # print the comparison table

Metrics:
  ate_rmse / ate_max  absolute trajectory error after aligning est to gt (--align se3 | sim3 | yaw | none)
  rpe_trans / rpe_yaw relative error over --delta seconds (drift rate, independent of alignment)
  z_rmse              vertical error (baro / ToF quality shows up here)
  final_err           position error at the last estimate after alignment
  coverage            fraction of the gt duration with an estimate (tracking loss shows up here)
  cpu_mean / rss_max  from cpu.csv next to est.tum, if traj_recorder logged it
The same numbers are what `evo_ape tum gt.tum est.tum -a` reports for SE3 ATE; use evo for fancier plots.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np


def load_tum(p):
    a = np.loadtxt(p, ndmin=2)
    if a.size == 0:
        sys.exit(f"{p} is empty: did the estimator publish anything?")
    return a[np.argsort(a[:, 0])]


def associate(gt, est, max_dt):
    idx = np.searchsorted(gt[:, 0], est[:, 0])
    idx = np.clip(idx, 1, len(gt) - 1)
    left, right = gt[idx - 1, 0], gt[idx, 0]
    idx = np.where(np.abs(est[:, 0] - left) < np.abs(est[:, 0] - right), idx - 1, idx)
    ok = np.abs(gt[idx, 0] - est[:, 0]) < max_dt
    return gt[idx[ok]], est[ok]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def umeyama(src, dst, scale=False, yaw_only=False):
    """R, t, s minimising |dst - (s R src + t)|."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    if yaw_only:  # rotation about z only (gravity is observable, so roll/pitch should already agree)
        H = xs[:, :2].T @ xd[:, :2]
        th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
        return R, mu_d - R @ mu_s, 1.0
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / xs.var(0).sum() if scale else 1.0
    return R, mu_d - s * R @ mu_s, s


def evaluate(gt_all, est_all, align="se3", delta=1.0, max_dt=0.05):
    gt, est = associate(gt_all, est_all, max_dt)
    if len(est) < 10:
        sys.exit(f"only {len(est)} associated poses (clock mismatch? use_sim_time on both?)")
    if align == "none":
        R, t, s = np.eye(3), np.zeros(3), 1.0
    else:
        R, t, s = umeyama(est[:, 1:4], gt[:, 1:4], scale=align == "sim3", yaw_only=align == "yaw")
    p_est = (s * (R @ est[:, 1:4].T)).T + t
    err = np.linalg.norm(p_est - gt[:, 1:4], axis=1)
    # RPE over `delta` seconds: compare relative motion (in each trajectory's own frame)
    rt, ry = [], []
    j = 0
    for i in range(len(est)):
        while j < len(est) and est[j, 0] - est[i, 0] < delta:
            j += 1
        if j >= len(est):
            break
        Rg_i, Re_i = quat_to_R(gt[i, 4:8]), quat_to_R(est[i, 4:8])
        dg = Rg_i.T @ (gt[j, 1:4] - gt[i, 1:4])
        de = Re_i.T @ (est[j, 1:4] - est[i, 1:4]) * s
        rt.append(np.linalg.norm(dg - de))
        dyg = yaw_of(gt[j, 4:8]) - yaw_of(gt[i, 4:8])
        dye = yaw_of(est[j, 4:8]) - yaw_of(est[i, 4:8])
        ry.append(abs((dyg - dye + math.pi) % (2 * math.pi) - math.pi))
    path = np.linalg.norm(np.diff(gt_all[:, 1:4], axis=0), axis=1).sum()
    dur_gt = gt_all[-1, 0] - gt_all[0, 0]
    gaps = np.diff(est[:, 0])
    covered = gaps[gaps < 1.0].sum()
    return dict(n=len(est), ate_rmse=float(np.sqrt((err ** 2).mean())), ate_max=float(err.max()),
                rpe_trans=float(np.sqrt(np.mean(np.square(rt)))) if rt else float("nan"),
                rpe_yaw_deg=float(np.degrees(np.sqrt(np.mean(np.square(ry))))) if ry else float("nan"),
                z_rmse=float(np.sqrt(((p_est[:, 2] - gt[:, 3]) ** 2).mean())), final_err=float(err[-1]),
                coverage=float(min(1.0, covered / dur_gt)) if dur_gt > 0 else 0.0, path_m=float(path),
                scale=float(s)), (gt, p_est, err)


COLS = ["label", "stack", "scenario", "pattern", "ate_rmse", "ate_max", "rpe_trans", "rpe_yaw_deg", "z_rmse",
        "final_err", "coverage", "path_m", "cpu_mean", "rss_max", "n", "run_dir"]


def print_table(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return print("no rows")
    rows.sort(key=lambda r: (r["scenario"], r["pattern"], float(r["ate_rmse"] or 1e9)))
    hdr = ["label", "stack", "pattern", "ate_rmse", "rpe_trans", "rpe_yaw_deg", "z_rmse", "coverage", "cpu_mean"]
    w = [max(len(h), *(len(r.get(h, "")) for r in rows)) for h in hdr]
    print("  ".join(h.ljust(x) for h, x in zip(hdr, w)))
    for r in rows:
        print("  ".join(str(r.get(h, "")).ljust(x) for h, x in zip(hdr, w)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("gt", nargs="?")
    ap.add_argument("est", nargs="?")
    ap.add_argument("--align", default="se3", choices=["se3", "sim3", "yaw", "none"])
    ap.add_argument("--delta", type=float, default=1.0)
    ap.add_argument("--max-dt", type=float, default=0.05)
    ap.add_argument("--label", default="")
    ap.add_argument("--stack", default="")
    ap.add_argument("--scenario", default="")
    ap.add_argument("--pattern", default="")
    ap.add_argument("--summary", help="append a row to this CSV")
    ap.add_argument("--plot", action="store_true", help="save traj.png next to est.tum (needs matplotlib)")
    ap.add_argument("--table", help="print a comparison table from a summary CSV and exit")
    a = ap.parse_args()
    if a.table:
        return print_table(a.table)
    if not (a.gt and a.est):
        ap.error("gt and est are required")
    res, (gt, p_est, err) = evaluate(load_tum(a.gt), load_tum(a.est), a.align, a.delta, a.max_dt)
    run_dir = os.path.dirname(os.path.abspath(a.est))
    cpu_path = os.path.join(run_dir, "cpu.csv")
    res["cpu_mean"] = res["rss_max"] = ""
    if os.path.isfile(cpu_path):
        c = np.genfromtxt(cpu_path, delimiter=",", names=True, ndmin=1)
        if c.size:
            res["cpu_mean"], res["rss_max"] = round(float(np.mean(c["cpu_percent"])), 1), round(float(np.max(c["rss_mb"])), 1)
    for k, v in res.items():
        print(f"  {k:<12} {v:.4f}" if isinstance(v, float) else f"  {k:<12} {v}")
    if a.summary:
        new = not os.path.isfile(a.summary)
        os.makedirs(os.path.dirname(os.path.abspath(a.summary)), exist_ok=True)
        with open(a.summary, "a", newline="") as f:
            w = csv.DictWriter(f, COLS)
            if new:
                w.writeheader()
            row = dict(res, label=a.label, stack=a.stack, scenario=a.scenario, pattern=a.pattern, run_dir=run_dir)
            w.writerow({k: (f"{row[k]:.4f}" if isinstance(row.get(k), float) else row.get(k, "")) for k in COLS})
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        ax[0].plot(gt[:, 1], gt[:, 2], "k-", lw=1, label="ground truth")
        ax[0].plot(p_est[:, 0], p_est[:, 1], "-", lw=1, label=f"{a.label or 'estimate'} (aligned)")
        ax[0].set_aspect("equal"); ax[0].legend(); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
        ax[1].plot(gt[:, 0] - gt[0, 0], err, lw=1); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("position error [m]")
        ax[1].set_title(f"ATE rmse {res['ate_rmse']:.3f} m")
        fig.tight_layout(); fig.savefig(os.path.join(run_dir, "traj.png"), dpi=120)
        print(f"  plot         {os.path.join(run_dir, 'traj.png')}")


if __name__ == "__main__":
    main()
