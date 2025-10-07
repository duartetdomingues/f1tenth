import numpy as np
from typing import Dict, Tuple
import matplotlib.pyplot as plt
import signal
import sys

def _periodic_interp(s_base: np.ndarray, y_base: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    """Periodic linear interpolation over s. Wrap s_query into [s0, s_end)."""
    s_base = np.asarray(s_base, float)
    y_base = np.asarray(y_base, float)
    s_query = np.asarray(s_query, float)
    L = float(s_base[-1] - s_base[0])
    s0 = float(s_base[0])
    
    if L <= 0:
        return np.interp(s_query, s_base, y_base)
    
    s_wrap = (s_query - s0) % L + s0
    return np.interp(s_wrap, s_base, y_base)


def frenet_to_global(s_vals: np.ndarray, n_vals: np.ndarray, u_rel: np.ndarray, frames: Dict[str, np.ndarray]) -> Tuple[np.ndarray, np.ndarray, float]:
    """(s,n,u_rel) -> (X,Y,psi) using the standard Frenet frame definition used in your MATLAB code:
    x = X_c + n * cos(psi + pi/2)
    y = Y_c + n * sin(psi + pi/2)
    psi = psi_c + u_rel
    where (X_c, Y_c, psi_c) are the centerline coordinates and heading interpolated at s.
    """
    s_vals = np.asarray(s_vals, float)
    n_vals = np.asarray(n_vals, float)
    u_rel = np.asarray(u_rel, float)
    X_c = _periodic_interp(frames["s"], frames["x"], s_vals)
    Y_c = _periodic_interp(frames["s"], frames["y"], s_vals)
    psi_c = _periodic_interp(frames["s"], frames["psi"], s_vals)
    X = X_c + n_vals * np.cos(psi_c + np.pi/2.0)
    Y = Y_c + n_vals * np.sin(psi_c + np.pi/2.0)
    psi = psi_c + u_rel
    return X, Y, psi

def handler_plots(sig, frame):
    plt.close("all")
    print("\nInterrompido com Ctrl+C")
    sys.exit(0)
    
    
