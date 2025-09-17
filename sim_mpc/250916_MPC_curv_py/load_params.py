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
from mpc_plots import plot_all
from mpc_plots_xy import plot_xy_from_csv



# ---------------------------- simple YAML-ish parser ----------------------------

def _coerce_scalar(val: str):
    v = val.strip()
    # drop YAML tags like !!float
    v = re.sub(r"^!![a-zA-Z_]+\s*", "", v)
    # strip quotes
    if (len(v) >= 2 and ((v[0] == v[-1] == '"') or (v[0] == v[-1] == "'"))):
        v = v[1:-1]
    # booleans
    if v.lower() in ("true", "yes", "on"): return True
    if v.lower() in ("false", "no", "off"): return False
    # numbers
    try:
        if re.fullmatch(r"[-+]?\d+", v):
            return int(v)
        # allow 1e3, -2.5, etc.
        if re.fullmatch(r"[-+]?\d*(?:\.\d+)?(?:[eE][-+]?\d+)?", v) and v not in ("", "."):
            return float(v)
    except Exception:
        pass
    return v


def load_params_file(path: Path) -> dict:
    """Parse a flat YAML-like file with lines `key: value`. Ignores sections and comments.
    The *last* occurrence of a key wins (like many YAML parsers).
    """
    out: dict[str, object] = {}
    txt = Path(path).read_text(encoding="utf-8")
    for line in txt.splitlines():
        # remove comments
        if "#" in line:
            line = line.split("#", 1)[0]
        if not line.strip():
            continue
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = _coerce_scalar(v)
    return out

# ---------------------------- CONFIG ----------------------------

def load_track_csv(path: Path, track_width_override: float | None = None):
    """Load track arrays from CSV. Returns dict with s, x, y, kappa, nl, nr."""
    arr = np.genfromtxt(path, delimiter=",", skip_header=1)
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    if arr.shape[1] < 4:
        raise ValueError(
            f"CSV needs at least 4 cols [s,x,y,kappa], got shape {arr.shape}")

    s = arr[:, 0].astype(float)
    x = arr[:, 1].astype(float)
    y = arr[:, 2].astype(float)
    kappa = arr[:, 3].astype(float)

    if arr.shape[1] >= 6:
        nl = arr[:, 4].astype(float)
        nr = arr[:, 5].astype(float)
    else:
        nl = np.ones_like(kappa)
        nr = np.ones_like(kappa)

    if track_width_override is not None:
        nl[:] = track_width_override
        nr[:] = track_width_override
        
    # Compute psi by differentiating x,y
    dx = np.gradient(x)
    dy = np.gradient(y)
    psi = np.arctan2(dy, dx)

    return dict(s=s, x=x, y=y, kappa=kappa, nl=nl, nr=nr, psi=psi)

def make_configs(overrides: dict | None = None) -> tuple[SimpleNamespace, SimpleNamespace, SimpleNamespace]:
    """Create config objects matching the attributes used by your code.
    Values come from DEFAULTS updated by `overrides`.
    """
    #cfg = DEFAULTS.copy()
    cfg = dict()
    for override in overrides or []:
        cfg.update(override)

    # == STMPC (solver + weights + constraints) ==
    stmpc = SimpleNamespace()
    stmpc.N = int(cfg["N"])                     # horizon steps
    stmpc.MPC_freq = float(cfg["MPC_freq"])     # Hz (1/Ts)
    stmpc.t_delay = float(cfg.get("t_delay", 0.0))
    stmpc.steps_delay = int(cfg.get("steps_delay", 0))

    stmpc.track_safety_margin = float(cfg["track_safety_margin"])  # m
    stmpc.track_max_width = float(cfg["track_max_width"])          # sentinel upper bound
    stmpc.overtake_d = float(cfg.get("overtake_d", 0.0))

    # slack penalties
    stmpc.Zl = float(cfg["Zl"]) ; stmpc.Zu = float(cfg["Zu"]) ; stmpc.zl = float(cfg["zl"]) ; stmpc.zu = float(cfg["zu"])

    # weights
    stmpc.qadv = float(cfg["qadv"])       # progress maximization
    stmpc.qv = float(cfg["qv"])           # speed tracking (linear/quad depending on your model)
    stmpc.qn = float(cfg["qn"])           # lateral error
    stmpc.qalpha = float(cfg["qalpha"])   # heading error
    stmpc.qjerk = float(cfg["qjerk"])     # smooth accel
    stmpc.qddelta = float(cfg["qddelta"]) # smooth steering rate

    # bounds for states & inputs
    stmpc.v_min = float(cfg["v_min"]) ; stmpc.v_max = float(cfg["v_max"]) ; stmpc.a_min = float(cfg["a_min"]) ; stmpc.a_max = float(cfg["a_max"]) ;
    stmpc.delta_min = float(cfg["delta_min"]) ; stmpc.delta_max = float(cfg["delta_max"]) ;
    stmpc.ddelta_min = float(cfg["ddelta_min"]) ; stmpc.ddelta_max = float(cfg["ddelta_max"]) ;
    stmpc.jerk_min = float(cfg["jerk_min"]) ; stmpc.jerk_max = float(cfg["jerk_max"]) ;

    # nonlinear constraint
    stmpc.alat_max = float(cfg["alat_max"])    # lateral acceleration cap

    # flags / switches
    stmpc.vy_minimization = bool(cfg["vy_minimization"])
    stmpc.adv_maximization = bool(cfg["adv_maximization"])
    stmpc.load_transfer = bool(cfg["load_transfer"])
    stmpc.correct_v_y_dot = bool(cfg["correct_v_y_dot"])

    cc = str(cfg.get("combined_constraints", "None"))
    stmpc.combined_constraints = cc if cc.lower() != "none" else "None"

    # == Vehicle params ==
    car = SimpleNamespace(
        lr=float(cfg["lr"]),
        lf=float(cfg["lf"]),
        m=float(cfg["m"]),
        Iz=float(cfg["Iz"]),
        h_cg=float(cfg["h_cg"]),
        wheelbase=float(cfg.get("wheelbase", float(cfg["lf"]) + float(cfg["lr"]))),
        # pass-through extra params in case your model uses them
        C_0d=float(cfg.get("C_0d", 0.0)),
        C_d=float(cfg.get("C_d", 0.0)),
        C_acc=float(cfg.get("C_acc", 0.0)),
        C_dec=float(cfg.get("C_dec", 0.0)),
        C_R=float(cfg.get("C_R", 0.0)),
        C_0v=float(cfg.get("C_0v", 0.0)),
        C_v=float(cfg.get("C_v", 0.0)),
        tau_steer=float(cfg.get("tau_steer", 0.0)),
        max_steering_angle=float(cfg.get("max_steering_angle", 0.0)),
        max_steering_velocity=float(cfg.get("max_steering_velocity", 0.0)),
        racecar_version=str(cfg.get("racecar_version", "")),
    )

    # == Tire params (keep placeholders; adapt if your bicycle_model uses these)
    tire = SimpleNamespace(
        friction_coeff=1.0,
        Bf=7.6671, Cf=1.2628, Df=1.2307, Ef=0.0,
        Br=7.1036, Cr=1.7356, Dr=1.0252, Er=0.0,
    )

    return stmpc, car, tire



DEFAULTS = {
    # Controller settings
    "N": 40,
    "MPC_freq": 20,
    "t_delay": 0.0025,
    "steps_delay": 3,
    "track_safety_margin": 0.25,
    "track_max_width": 1e3,
    "overtake_d": 1.0,
    # Cost
    "qjerk": 1e-2,
    "qddelta": 5e-1,
    "qadv": 0.0,
    "qn": 40.0,
    "qalpha": 0.1,
    "qv": 10.0,
    "Zl": 1000.0,
    "Zu": 1000.0,
    "zl": 1000.0,
    "zu": 1000.0,
    # Constraints (state)
    "delta_min": -0.4,
    "delta_max": 0.4,
    "v_min": -5.0,  # note: last occurrence wins
    "v_max": 10.0,
    "a_min": -3.0,
    "a_max": 3.0,
    # Constraints (inputs)
    "ddelta_min": -3.2,
    "ddelta_max": 3.2,
    "jerk_min": -50.0,
    "jerk_max": 50.0,
    # Nonlinear
    "alat_max": 10.0,
    # Flags
    "vy_minimization": True,
    "adv_maximization": False,
    "combined_constraints": "None",  # ellipse/diamond/None
    "load_transfer": True,
    "correct_v_y_dot": True,
    # Vehicle
    "m": 3.54,
    "Iz": 0.05797,
    "lf": 0.162,
    "lr": 0.145,
    "wheelbase": 0.307,
    "h_cg": 0.014,
    # Aerodynamic/drivetrain-ish (passed through; use in your model if needed)
    "C_0d": 0.48,
    "C_d": -1.1,
    "C_acc": 8.29,
    "C_dec": 5.77,
    "C_R": 2.03,
    "C_0v": 100.0,
    "C_v": 20.0,
    # Actuator limits/dynamics
    "tau_steer": 0.15779476,
    "max_steering_angle": 0.4189,
    "max_steering_velocity": 3.2,
    "racecar_version": "NUC2",
}

