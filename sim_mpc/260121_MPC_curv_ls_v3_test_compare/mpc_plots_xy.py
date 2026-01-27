#!/usr/bin/env python3
"""
XY trajectory plotting for the MPC (Frenet -> global).

Quick use from another script:
    from mpc_plots_xy import plot_xy_from_csv
    plot_xy_from_csv(x_hist, "centerline.csv", track_width=None, show=True)

Or, if you already have the loaded track dict {s,x,y,kappa,(nl),(nr)}:
    from mpc_plots_xy import plot_xy_path
    plot_xy_path(x_hist, track)

CLI to plot from saved arrays:
    python mpc_plots_xy.py --npz results.npz --csv centerline.csv [--track-width 1.0]
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# ---------------- basic track I/O ----------------

def load_track_csv(path: Path, track_width_override: float | None = None) -> dict:
    arr = np.loadtxt(path, delimiter=",", ndmin=2)
    if arr.shape[1] < 4:
        raise ValueError(f"CSV needs at least 4 cols [s,x,y,kappa], got {arr.shape}")
    s = arr[:, 0].astype(float)
    x = arr[:, 1].astype(float)
    y = arr[:, 2].astype(float)
    kappa = arr[:, 3].astype(float)
    if arr.shape[1] >= 6:
        nl = arr[:, 4].astype(float)
        nr = arr[:, 5].astype(float)
    else:
        nl = np.ones_like(s)
        nr = np.ones_like(s)
    if track_width_override is not None:
        nl[:] = track_width_override
        nr[:] = track_width_override
    return dict(s=s, x=x, y=y, kappa=kappa, nl=nl, nr=nr)


# --------------- math helpers ----------------

def _ensure_2d(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a)
    if a.ndim == 1:
        a = a[:, None]
    return a


def _periodic_interp(s_base: np.ndarray, y_base: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    s_base = np.asarray(s_base).astype(float)
    y_base = np.asarray(y_base).astype(float)
    s_query = np.asarray(s_query).astype(float)
    L = float(s_base[-1] - s_base[0])
    s0 = float(s_base[0])
    if L <= 0:
        return np.interp(s_query, s_base, y_base)
    s_wrapped = (s_query - s0) % L + s0
    return np.interp(s_wrapped, s_base, y_base)


def compute_track_frames(track: dict) -> dict:
    s = np.asarray(track["s"]).astype(float)
    x = np.asarray(track["x"]).astype(float)
    y = np.asarray(track["y"]).astype(float)
    nl = np.asarray(track.get("nl", np.ones_like(s))).astype(float)
    nr = np.asarray(track.get("nr", np.ones_like(s))).astype(float)
    dxds = np.gradient(x, s)
    dyds = np.gradient(y, s)
    psi = np.unwrap(np.arctan2(dyds, dxds))
    out = dict(s=s, x=x, y=y, psi=psi, nl=nl, nr=nr)
    if "kappa" in track:
        out["kappa"] = np.asarray(track["kappa"]).astype(float)
    return out


ess = np

def s_n_to_xy(s_vals: np.ndarray, n_vals: np.ndarray, frames: dict) -> tuple[np.ndarray, np.ndarray]:
    s_vals = np.asarray(s_vals).astype(float)
    n_vals = np.asarray(n_vals).astype(float)
    xc = _periodic_interp(frames["s"], frames["x"], s_vals)
    yc = _periodic_interp(frames["s"], frames["y"], s_vals)
    psi = _periodic_interp(frames["s"], frames["psi"], s_vals)
    X = xc - np.sin(psi) * n_vals
    Y = yc + np.cos(psi) * n_vals
    return X, Y


# --------------- plotting ----------------

def plot_xy_path(x_hist: np.ndarray, track: dict, save_prefix: str | None = None, show: bool = True,
                 plot_boundaries: bool = True):
    x_hist = _ensure_2d(x_hist)
    s_vals = x_hist[:, 0]
    n_vals = x_hist[:, 1]
    frames = compute_track_frames(track)
    X, Y = s_n_to_xy(s_vals, n_vals, frames)

    s_dense = np.linspace(frames["s"][0], frames["s"][-1], 2000)
    x_c = _periodic_interp(frames["s"], frames["x"], s_dense)
    y_c = _periodic_interp(frames["s"], frames["y"], s_dense)
    psi = _periodic_interp(frames["s"], frames["psi"], s_dense)
    n_hat_x = -np.sin(psi)
    n_hat_y =  np.cos(psi)
    nl = _periodic_interp(frames["s"], frames["nl"], s_dense)
    nr = _periodic_interp(frames["s"], frames["nr"], s_dense)
    x_L = x_c + n_hat_x * nl
    y_L = y_c + n_hat_y * nl
    x_R = x_c - n_hat_x * nr
    y_R = y_c - n_hat_y * nr

    fig, ax = plt.subplots(1, 1, figsize=(7.5, 7.5))
    if plot_boundaries:
        ax.plot(x_L, y_L, linewidth=1.0, label="left boundary")
        ax.plot(x_R, y_R, linewidth=1.0, label="right boundary")
    ax.plot(x_c, y_c, linewidth=1.5, label="centerline")
    ax.plot(X, Y, linewidth=2.0, label="vehicle path")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend(loc="best")
    ax.grid(True, linestyle=":", alpha=0.6)

    if save_prefix is not None:
        out = f"{save_prefix}_xy.png"
        fig.savefig(out, dpi=150)
        print(f"Saved: {out}")

    if show:
        plt.show()


def plot_xy_from_csv(x_hist: np.ndarray, csv_path: str | Path, track_width: float | None = None,
                      save_prefix: str | None = None, show: bool = True):
    track = load_track_csv(Path(csv_path), track_width_override=track_width)
    plot_xy_path(x_hist, track, save_prefix=save_prefix, show=show)


# --------------- CLI ----------------

def main():
    ap = argparse.ArgumentParser(description="Plot XY trajectory from Frenet states and track CSV")
    ap.add_argument("--npz", type=Path, required=True, help="results.npz with x_hist (and dt optional)")
    ap.add_argument("--csv", type=Path, required=True, help="centerline CSV [s,x,y,kappa,(nl),(nr)]")
    ap.add_argument("--track-width", type=float, default=None, help="Override nl,nr with this half-width (m)")
    ap.add_argument("--save-prefix", type=str, default=None, help="Prefix to save figures")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    data = np.load(args.npz)
    x_hist = data["x_hist"]
    track = load_track_csv(args.csv, track_width_override=args.track_width)
    plot_xy_path(x_hist, track, save_prefix=args.save_prefix, show=(not args.no_show))


if __name__ == "__main__":
    main()
