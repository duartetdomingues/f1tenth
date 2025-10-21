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
import matplotlib
import signal, sys
import select
from screeninfo import get_monitors  # pip install screeninfo
from PyQt5.QtCore import Qt   # <-- importa aqui

matplotlib.use("Qt5Agg")
matplotlib.rcParams['figure.raise_window'] = False
import matplotlib.pyplot as plt
from aux_func import frenet_to_global, handler_plots
from matplotlib.collections import LineCollection

def position_window(fig, offset_x=100, offset_y=100):

    w = fig.canvas.manager.window
    backend = matplotlib.get_backend()

    # pega monitores disponíveis
    monitors = get_monitors()
    if len(monitors) < 2:
        print("Só encontrei 1 monitor. Abrindo na tela principal.")
        return
    
    
    secondary_monitor = next((m for m in monitors if not m.is_primary), None)
    if secondary_monitor is None:
        print("Não foi encontrado um monitor secundário. Abrindo na tela principal.")
        return
        
    # tenta posição para o lado (segundo monitor à direita)
    m2 = secondary_monitor
    print(f"Posicionando janela no monitor {m2.name} ({m2.width}x{m2.height} @ {m2.x},{m2.y})")
    target_x = m2.x + offset_x
    target_y = m2.y + offset_y

    if "TkAgg" in backend:
        try:
            print(f"Posicionando janela TkAgg em {target_x},{target_y}")
            w.geometry(f"+{target_x}+{target_y}")
            #w.state("zoomed")
        except Exception:
            print("Não consegui posicionar janela TkAgg.")
            

    elif "Qt5Agg" in backend or "QtAgg" in backend:
       
        print(f"Posicionando janela Qt5Agg em {target_x},{target_y}")
        w.setWindowFlag(Qt.WindowDoesNotAcceptFocus, True)
        w.move(target_x, target_y)
        #w.showMaximized()


   

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
    v_x = 0
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(1, 5))
    cbar = plt.colorbar(lc, ax=ax, pad=0.02)
    cbar.set_label("Velocity $v_x$ [m/s]")
    
    plt.ion()
    position_window(fig)
    plt.show()
    
    return fig
    
def update_sim_plot(fig, tr, d_left, d_right, x_horizon, status, n_step):
    
    """ if not plt.fignum_exists(fig.number):
        print("Figure closed. Exiting...")
        exit() """

    print("Updating simulation plot...")
    
    ax = fig.axes[0]       # Get the single axis of the figure
    ax.cla()               # Clear the content of the plot

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
    # Color the predicted path by velocity (v_x)
    v_x = x_horizon[:, 3]
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='viridis', norm=plt.Normalize(min(v_x), max(v_x)))
    lc.set_array(v_x)
    lc.set_linewidth(3)
    ax.add_collection(lc)
    

    # Add a light indicator for MPC status
    light_color = "green" if status == 0 else "red"
    ax.scatter([], [], color=light_color, label="MPC Status", s=200, edgecolors='black', zorder=4)

    ax.set_title(f"Track with Boundaries and Vehicle Position (Step {n_step})")
    ax.axis("equal")
    ax.legend()
    ax.grid()

    # Refresh the same figure
    fig.canvas.draw()
    fig.canvas.flush_events()
 
    return fig
    

def plot_states(mpc_self,x_hist: np.ndarray, u_hist: np.ndarray, dt: float,
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
    T = np.count_nonzero(x_hist[:, 0])
    t = np.arange(1, T+1) * float(dt)
    tu = np.arange(1, T) * float(dt)
    x_hist = x_hist[:T, :]
    u_hist = u_hist[:T-1, :]

    # --- States ----
    fig1, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(7, 1, sharex=True, figsize=(8, 12))
    ax1.plot(t, x_hist[:, 0], 'o-', label="s [m]")
    ax1.set_ylabel("s [m]")
    ax1.legend()
    ax1.grid(True)
    ax1.yaxis.get_major_formatter().set_useOffset(False)

    ax2.plot(t, x_hist[:, 1], 'o-', label="n [m]")
    ax2.set_ylabel("n [m]")
    ax2.legend()
    ax2.grid(True)
    ax2.yaxis.get_major_formatter().set_useOffset(False)

    ax3.plot(t, x_hist[:, 2], 'o-', label="theta [rad]")
    ax3.set_ylabel("theta [rad]")
    ax3.legend()
    ax3.grid(True)
    ax3.yaxis.get_major_formatter().set_useOffset(False)

    ax4.plot(t, x_hist[:, 3], 'o-', label="v_x [m/s]")
    ax4.set_ylabel("v_x [m/s]")
    ax4.legend()
    ax4.grid(True)
    ax4.yaxis.get_major_formatter().set_useOffset(False)

    ax5.plot(t, x_hist[:, 4], 'o-', label="yaw_rate [rad/s]")
    ax5.set_ylabel("yaw_rate [rad/s]")
    ax5.legend()
    ax5.grid(True)
    ax5.yaxis.get_major_formatter().set_useOffset(False)
    
    ax6.plot(t, x_hist[:, 5], 'o-', label="delta [rad]")
    ax6.set_ylabel("delta [rad]")
    ax6.legend()
    ax6.grid(True)
    ax6.yaxis.get_major_formatter().set_useOffset(False)
   
    ax7.plot(tu, u_hist[:, 0], 'o-', label="cmd_delta [rad])")
    ax7.set_ylabel("cmd_delta [rad]")
    ax7.legend()
    ax7.grid(True)
    ax7.set_xlabel("t [s]")
    ax7.yaxis.get_major_formatter().set_useOffset(False)

    fig1.tight_layout()

    
    
    
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(mpc_self.tr["x"], mpc_self.tr["y"], label="Centerline", color="yellow", zorder=1)
    ax.plot(mpc_self.tr["x"] + mpc_self.d_left * np.cos(mpc_self.tr["psi"] + np.pi / 2),
            mpc_self.tr["y"] + mpc_self.d_left * np.sin(mpc_self.tr["psi"] + np.pi / 2),
            label="Left Boundary", color="blue", linestyle="--")
    ax.plot(mpc_self.tr["x"] - mpc_self.d_right * np.cos(mpc_self.tr["psi"] + np.pi / 2),
            mpc_self.tr["y"] - mpc_self.d_right * np.sin(mpc_self.tr["psi"] + np.pi / 2),
            label="Right Boundary", color="blue", linestyle="--")
    ax.axis("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Track with Boundaries and Vehicle Position")
    ax.legend()
    ax.grid()
    # Color the predicted path by velocity (v_x)
    [x, y, psi] = frenet_to_global(x_hist[:,0], x_hist[:,1], x_hist[:,2], mpc_self.tr)
    v_x = x_hist[:, 3]
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, cmap='turbo', norm=plt.Normalize(min(v_x), max(v_x)))
    lc.set_array(v_x)
    lc.set_linewidth(3)
    ax.add_collection(lc)
    cbar = plt.colorbar(lc, ax=ax, pad=0.02)
    cbar.set_label("Velocity $v_x$ [m/s]")

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

    signal.signal(signal.SIGINT, handler_plots)  # agora Ctrl+C chama handler


    if show:
        plt.show(block=True)
        #wait_for_enter_or_ctrl_c()

        # espera até Enter ou Ctrl+C
        """ try:
            input("Pressione Enter para sair... ")
        except KeyboardInterrupt:
            pass """


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
    plot_states(x_hist, u_hist, dt, save_prefix=args.save_prefix, show=(not args.no_show))


if __name__ == "__main__":
    main()
