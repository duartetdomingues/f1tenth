#!/usr/bin/env python3
"""
Reusable plotting utilities for MPC simulations.

Usage (from another script):
    from mpc_plots import plot_all
    plot_all(x_hist, u_hist, dt, save_prefix=None, show=True)

CLI (optional): load arrays from an .npz and plot:
    python mpc_plots.py --npz results.npz

Expected arrays:
- x_hist: (T+1, nx)
- u_hist: (T, nu)
- dt: scalar time step [s]
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from aux_func import frenet_to_global

def _ensure_2d(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a)
    if a.ndim == 1:
        a = a[:, None]
    return a

def plot_track_and_boundaries(tr: dict, d_left: np.ndarray, d_right: np.ndarray):

    plt.figure(figsize=(10, 6))
    plt.plot(tr["x"], tr["y"], label="Centerline", color="blue")
    plt.plot(tr["x"] + d_left * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] + d_left * np.sin(tr["psi"] + np.pi / 2),
             label="Left Boundary", color="red", linestyle="--")
    plt.plot(tr["x"] - d_right * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] - d_right * np.sin(tr["psi"] + np.pi / 2),
             label="Right Boundary", color="green", linestyle="--")
    plt.quiver(tr["x"][::20], tr["y"][::20],
               np.cos(tr["psi"][::20]), np.sin(tr["psi"][::20]),
               angles='xy', scale_units='xy', scale=1, color='blue', label="Direction")
    plt.scatter(tr["x"][0], tr["y"][0], color="yellow", label="Origin (s=0)", zorder=5)
    plt.axis("equal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Track with Boundaries")
    plt.legend()
    plt.grid()
    plt.show()
    
def init_sim_plot(tr, d_left, d_right, x0):

    [x, y, psi] = frenet_to_global(x0[0], x0[1], x0[2], tr)

    
    fig, ax = plt.subplots(figsize=(10, 6))
    vehicle_arrow = ax.arrow(x, y, 1 * np.cos(psi), 1 * np.sin(psi),
                            head_width=0.5, head_length=0.6, fc='black', ec='black', label="Vehicle",zorder=2)
    ax.plot(tr["x"], tr["y"], label="Centerline", color="yellow", zorder=1)
    ax.plot(tr["x"] + d_left * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] + d_left * np.sin(tr["psi"] + np.pi / 2),
             label="Left Boundary", color="red", linestyle="--")
    ax.plot(tr["x"] - d_right * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] - d_right * np.sin(tr["psi"] + np.pi / 2),
             label="Right Boundary", color="green", linestyle="--")
    ax.axis("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Track with Boundaries and Vehicle Position")
    ax.legend()
    ax.grid()
    
    plt.show()
    
    return fig
    
def update_sim_plot(fig, tr, d_left, d_right, x_horizon):
    print("Updating simulation plot...")
    
    fig, ax = plt.subplots(figsize=(10, 6))

    [x, y, psi] = frenet_to_global(x_horizon[:, 0], x_horizon[:, 1], x_horizon[:, 2], tr)

    ax.plot(tr["x"], tr["y"], label="Centerline", color="yellow", zorder=1)
    ax.plot(tr["x"] + d_left * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] + d_left * np.sin(tr["psi"] + np.pi / 2),
             label="Left Boundary", color="red", linestyle="--")
    ax.plot(tr["x"] - d_right * np.cos(tr["psi"] + np.pi / 2),
             tr["y"] - d_right * np.sin(tr["psi"] + np.pi / 2),
             label="Right Boundary", color="green", linestyle="--")
    ax.quiver(x[1], y[1], np.cos(psi[1]), np.sin(psi[1]),
              angles='xy', scale_units='xy', scale=1, color='black', label="Vehicle", zorder=2)
    ax.plot(x, y, 'o-', color='blue', label="Predicted Path", zorder=3)
    ax.axis("equal")
    ax.legend()
    ax.grid()

    plt.show()
    
    return fig
    

def plot_all(x_hist: np.ndarray, u_hist: np.ndarray, dt: float,
             save_prefix: str | None = None, show: bool = True):
    """Create time-series plots for states & inputs.

    Parameters
    ----------
    x_hist : array (T+1, nx)
    u_hist : array (T, nu)
    dt : float, sampling time
    save_prefix : if given, saves figures as f"{save_prefix}_states.png" and
                  f"{save_prefix}_inputs.png" instead of (or in addition to) show.
    show : whether to plt.show() at the end.
    """
    x_hist = _ensure_2d(x_hist)
    u_hist = _ensure_2d(u_hist)
    T = x_hist.shape[0] - 1
    t = np.arange(x_hist.shape[0]) * float(dt)
    tu = np.arange(u_hist.shape[0]) * float(dt)

    # --- States ---
    fig1, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    axes[0].plot(t, x_hist[:, 0])
    axes[0].set_ylabel("s [m]")
    if x_hist.shape[1] > 1:
        axes[1].plot(t, x_hist[:, 1])
        axes[1].set_ylabel("n [m]")
    if x_hist.shape[1] > 2:
        axes[2].plot(t, x_hist[:, 2])
        axes[2].set_ylabel("theta [rad]")
    axes[-1].set_xlabel("t [s]")
    fig1.tight_layout()

    # --- Velocities/Yaw ---
    if x_hist.shape[1] >= 7:
        fig2, axes2 = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
        axes2[0].plot(t, x_hist[:, 3]); axes2[0].set_ylabel("v_x [m/s]")
        axes2[1].plot(t, x_hist[:, 4]); axes2[1].set_ylabel("v_y [m/s]")
        axes2[2].plot(t, x_hist[:, 6]); axes2[2].set_ylabel("yaw_rate [rad/s]")
        axes2[-1].set_xlabel("t [s]")
        fig2.tight_layout()
    else:
        fig2 = None

    # --- Inputs ---
    fig3, axes3 = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes3[0].plot(tu, u_hist[:, 0]); axes3[0].set_ylabel("jerk [m/s^3]")
    if u_hist.shape[1] > 1:
        axes3[1].plot(tu, u_hist[:, 1]); axes3[1].set_ylabel("ddelta [rad/s]")
    axes3[-1].set_xlabel("t [s]")
    fig3.tight_layout()

    # Save/show
    if save_prefix is not None:
        out1 = f"{save_prefix}_states.png"
        fig1.savefig(out1, dpi=150)
        if fig2 is not None:
            out2 = f"{save_prefix}_velyaw.png"
            fig2.savefig(out2, dpi=150)
        out3 = f"{save_prefix}_inputs.png"
        fig3.savefig(out3, dpi=150)
        print(f"Saved: {out1}{', ' + out2 if fig2 is not None else ''}, {out3}")

    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=Path, required=False, help="Load arrays from .npz with x_hist,u_hist,dt")
    ap.add_argument("--save-prefix", type=str, default=None, help="Prefix for saved figures")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    if args.npz is None:
        ap.error("Provide --npz results.npz (with x_hist,u_hist,dt)")

    data = np.load(args.npz)
    x_hist = data["x_hist"]; u_hist = data["u_hist"]; dt = float(data["dt"]) if "dt" in data else 0.05
    plot_all(x_hist, u_hist, dt, save_prefix=args.save_prefix, show=(not args.no_show))


if __name__ == "__main__":
    main()
