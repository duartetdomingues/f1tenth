import numpy as np
from typing import Dict, Tuple
import matplotlib.pyplot as plt
import signal
import sys

def _periodic_interp(s_base: np.ndarray, y_base: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    """Interpolação linear periódica para valores escalares (X, Y, s)."""
    s_base = np.asarray(s_base, float)
    y_base = np.asarray(y_base, float)
    s_query = np.asarray(s_query, float)
    
    L = float(s_base[-1] - s_base[0])
    s0 = float(s_base[0])
    
    if L <= 0:
        return np.interp(s_query, s_base, y_base)
    
    # Wrap s_query para garantir que está dentro do domínio da pista
    s_wrap = (s_query - s0) % L + s0
    return np.interp(s_wrap, s_base, y_base)

def _periodic_angle_interp(s_base: np.ndarray, angle_base: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    """
    Interpolação correta para ângulos.
    Decompõe em sin/cos, interpola, e recompoe com arctan2.
    """
    # 1. Calcular componentes vetoriais da base
    cos_base = np.cos(angle_base)
    sin_base = np.sin(angle_base)
    
    # 2. Interpolar os componentes separadamente
    # Nota: Usamos a _periodic_interp que já definiste para tratar do s_wrap
    cos_i = _periodic_interp(s_base, cos_base, s_query)
    sin_i = _periodic_interp(s_base, sin_base, s_query)
    
    # 3. Reconstruir o ângulo
    return np.arctan2(sin_i, cos_i)

def frenet_to_global(s_vals: np.ndarray, n_vals: np.ndarray, u_rel: np.ndarray, frames: Dict[str, np.ndarray]) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    (s,n,u_rel) -> (X,Y,psi)
    """
    s_vals = np.asarray(s_vals, float)
    n_vals = np.asarray(n_vals, float)
    u_rel = np.asarray(u_rel, float)
    
    # Interpolar posição (Linear é OK para X e Y)
    X_c = _periodic_interp(frames["s"], frames["x"], s_vals)
    Y_c = _periodic_interp(frames["s"], frames["y"], s_vals)
    
    # --- CORREÇÃO AQUI ---
    # Usar a interpolação vetorial para o PSI
    psi_c = _periodic_angle_interp(frames["s"], frames["psi"], s_vals)
    # ---------------------

    # O resto da matemática mantém-se igual
    # Nota: cos(psi + pi/2) é matematicamente igual a -sin(psi)
    #       sin(psi + pi/2) é matematicamente igual a cos(psi)
    # Mas manter explícito ajuda na leitura em relação ao referencial Frenet
    X = X_c + n_vals * np.cos(psi_c + np.pi/2.0)
    Y = Y_c + n_vals * np.sin(psi_c + np.pi/2.0)
    
    psi = psi_c + u_rel
    
    return X, Y, psi

def handler_plots(sig, frame):
    plt.close("all")
    print("\nInterrompido com Ctrl+C")
    sys.exit(0)
    
    
