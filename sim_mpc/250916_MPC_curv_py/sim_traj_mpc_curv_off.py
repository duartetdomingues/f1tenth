#!/usr/bin/env python3
"""
Python reimplementation of the MATLAB script `sim_traj_mpc_curv_off.m`,
now wired to load your controller/vehicle parameters from a simple YAML-like
config file (exactly like the block you sent).

It uses your existing acados-based model & OCP setup in:
  - acados_settings.py
  - bicycle_model.py

Key features:
- Loads track CSV: [s, x, y, kappa, (nl), (nr)] and allows a constant width override.
- Builds the OCP with `acados_settings(...)` and runs a closed-loop MPC.
- Supports input/actuation delay via `steps_delay` + `t_delay` (rounded to whole steps).
- Soft constraints with Zl/Zu/zl/zu as in your params.
- CLI flag `--config` to read the params file; if omitted, sensible defaults are used.

Run (example):
  python sim_traj_mpc_curv_off_v2.py \
      --csv ../../traj/centerline_test_map.csv \
      --config params.yaml \
      --sim-time 80

Where `params.yaml` is the text you posted.
"""
from __future__ import annotations
import argparse
import sys
from types import SimpleNamespace
from pathlib import Path
import re
from collections import deque
import numpy as np
import casadi as cs
import time
from mpc_plots import plot_all, plot_track_and_boundaries, init_sim_plot, update_sim_plot
from load_params import load_params_file, load_track_csv, make_configs
import matplotlib.pyplot as plt

# acados OCP builder from your project
from acados_settings import acados_settings, get_parameters

# ---------------------------- SIM ----------------------------

def run_sim(track_csv: Path, sim_time: float, cfg_overrides: dict | None,
            track_width: float | None):
    # load track
    tr = load_track_csv(track_csv, track_width_override=track_width)
    
    s0 = tr["s"].copy()
    kappa = tr["kappa"].copy()
    d_left = tr["nl"].copy()
    d_right = tr["nr"].copy()
    x_s = tr["x"].copy()
    y_s = tr["y"].copy()
    
    plot_track_and_boundaries(tr, d_left, d_right)

    # configs
    stmpc, car, tire = make_configs(cfg_overrides)

    # build OCP & solver
    # returns: constraint, model, acados_solver, params(ns)
    constraint, model, solver, params_ns = acados_settings(
        s0, kappa, d_left, d_right, stmpc, car, tire
    )

    # param vector used online (kept constant here)
    p_vec = get_parameters(stmpc).astype(float)
    # set parameters for every stage (0..N)
    for stage in range(stmpc.N + 1):
        solver.set(stage, "p", p_vec)

    # initial state (near zero)
    x = np.array([15.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0], dtype=float)

    # logging
    dt = 1.0 / float(stmpc.MPC_freq)
    n_steps = int(sim_time * stmpc.MPC_freq)
    hist_x = np.zeros((n_steps + 1, model.n_x))
    hist_u = np.zeros((n_steps, model.n_u))
    status_hist = np.zeros(n_steps, dtype=int)
    solve_ms = np.zeros(n_steps)
    
    fig = init_sim_plot(tr, d_left, d_right, x)

    hist_x[0, :] = x

    # effective discrete delay in steps (steps_delay + t_delay rounded)
    extra_steps = int(round(stmpc.t_delay * stmpc.MPC_freq)) if hasattr(stmpc, "t_delay") else 0
    delay_steps = max(0, int(getattr(stmpc, "steps_delay", 0)) + extra_steps)
    u_buffer: deque[np.ndarray] = deque([np.zeros(model.n_u) for _ in range(delay_steps)], maxlen=max(delay_steps, 1))

    # last control (for fallback integration)
    u_prev = np.zeros(model.n_u)

    for k in range(n_steps):
        # provide initial condition
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)

        t0 = time.perf_counter()
        ret = solver.solve()
        solve_ms[k] = float((time.perf_counter() - t0) * 1e3)
        status = ret
        status_hist[k] = status

        if status != 0:
            # Failure: integrate one step with previous applied control
            sys.stderr.write(f"[WARN] Solver failed at step {k} with status {status}. Integrating 1 step.\n")
            u_cmd = u_prev.copy()
            exit(1)
            return {
                "x_hist": hist_x,
                "u_hist": hist_u,
                "status": status_hist,
                "solve_ms": solve_ms,
                "dt": dt,
                "freq": float(stmpc.MPC_freq),
            }
        else:
            print(f"[INFO] Step {k} solved in {solve_ms[k]:.2f} ms.")
            x_N = []
            for s in range(stmpc.N + 1):
                x_N.append(solver.get(s, "x").reshape(-1))
                
            u_N = []
            for s in range(stmpc.N):
                u_N.append(solver.get(s, "u").reshape(-1))
                
        fig = update_sim_plot(fig, tr, d_left, d_right, np.array(x_N))

        # enqueue commanded u, decide applied u (with delay)
        if delay_steps > 0:
            u_buffer.append(u_N[0])
            u_applied = u_buffer[0]
        else:
            u_applied = u_N[0]

        # state update
        if status == 0 and delay_steps == 0:
            # use optimizer's predicted next state for speed
            x_next = x_N[1].astype(float)
        else:
            # integrate one step with (possibly delayed) control
            print("[INFO] Integrating one step with applied control.")
            f = model.f_expl_func(
                x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7],
                float(u_applied[0]), float(u_applied[1]), p_vec
            ).full().reshape(-1)
            x_next = (x + dt * f).astype(float)

        # log
        hist_x[k + 1, :] = x_next
        hist_u[k, :] = u_applied
        u_prev = u_applied
        x = x_next

    return {
        "x_hist": hist_x,
        "u_hist": hist_u,
        "status": status_hist,
        "solve_ms": solve_ms,
        "dt": dt,
        "freq": float(stmpc.MPC_freq),
    }


# ---------------------------- MAIN ----------------------------

def main():
    
    traj_default = "./traj/centerline_test_map_v2.csv"
    
    ap = argparse.ArgumentParser(description="Closed-loop MPC sim (acados)")
    ap.add_argument("--csv", type=Path, default=traj_default,
                    help="Path to track CSV [s,x,y,kappa,(nl),(nr)]")
    ap.add_argument("--mpc_config", type=Path, default="./sim_mpc/250916_MPC_curv_py/mpc_config.yaml",
                    help="YAML-like MPC params file (use the block you sent)")
    ap.add_argument("--car_config", type=Path, default="./sim_mpc/250916_MPC_curv_py/car_model.yaml",
                    help="YAML-like car params file (use the block you sent)")
    ap.add_argument("--sim-time", type=float, default=80.0,
                    help="Total sim time [s]")
    ap.add_argument("--track-width", type=float, default=None,
                    help="Override track half-widths nl,nr with this value [m]")
    ap.add_argument("--no-plot", action="store_true", help="Disable plotting")
    args = ap.parse_args()

    overrides = []
    overrides.append(load_params_file(args.car_config)) if args.car_config.exists() else None
    overrides.append(load_params_file(args.mpc_config)) if args.mpc_config.exists() else None

    res = run_sim(
    track_csv=args.csv,
    sim_time=args.sim_time,
    cfg_overrides=overrides,
    track_width=args.track_width,
    )

    x_hist = res["x_hist"]
    u_hist = res["u_hist"]
    print(f"Done. Steps: {u_hist.shape[0]}, avg solve time: {res['solve_ms'].mean():.2f} ms at {res['freq']:.1f} Hz")

    if not args.no_plot:
        try:
            print("Plotting...")
            plot_xy_from_csv(x_hist, args.csv, track_width=args.track_width, show=True)
            plot_all(x_hist, u_hist, res["dt"], save_prefix=None, show=True)
        except Exception as e:
            print("plotting failed:", e)
            sys.exit(0)


if __name__ == "__main__":
    main()
