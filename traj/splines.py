import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pandas as pd

# === 1. Parâmetros da elipse ===
a = 5.0  # eixo semi-maior
b = 3.0  # eixo semi-menor
N = 500  # número de amostras

# === 2. Elipse paramétrica ===
t = np.linspace(0, 2 * np.pi, N)
x = a * np.cos(t)
y = b * np.sin(t)

# === 3. Calcular s (comprimento acumulado do arco) ===
dx = np.gradient(x)
dy = np.gradient(y)
ds = np.sqrt(dx**2 + dy**2)
s = np.cumsum(ds)
s -= s[0]  # começa em zero

# === 4. Criar splines x(s), y(s) ===
spline_x = CubicSpline(s, x)
spline_y = CubicSpline(s, y)

# === 5. Amostrar uniformemente em s ===
s_uniform = np.linspace(s[0], s[-1], N)
x_s = spline_x(s_uniform)
y_s = spline_y(s_uniform)

dx_s = spline_x.derivative()(s_uniform)
dy_s = spline_y.derivative()(s_uniform)
ddx_s = spline_x.derivative(2)(s_uniform)
ddy_s = spline_y.derivative(2)(s_uniform)

# === 6. Calcular curvatura κ(s) ===
kappa = (dx_s * ddy_s - dy_s * ddx_s) / (dx_s**2 + dy_s**2)**1.5

# === 7. Velocidade de referência (simples: inverso da curvatura) ===
def compute_v_ref(kappa, v_max=2.0, a_lat_max=1.5, eps=1e-3):
    return np.clip(np.sqrt(a_lat_max / (np.abs(kappa) + eps)), 0.5, v_max)

v_ref = compute_v_ref(kappa)

# === 8. Guardar como CSV ===
df = pd.DataFrame({
    "s": s_uniform,
    "x": x_s,
    "y": y_s,
    "kappa": kappa,
    "v_ref": v_ref
})
df.to_csv("ellipse_trajectory.csv", index=False)

# === 9. (Opcional) Guardar como .npz ===
np.savez("ellipse_trajectory.npz", s=s_uniform, x=x_s, y=y_s, kappa=kappa, v_ref=v_ref)

# === 10. Visualizar ===
plt.figure(figsize=(10, 4))
plt.subplot(1, 2, 1)
plt.plot(x, y, 'k--', label="Elipse original")
plt.plot(x_s, y_s, 'b', label="Spline amostrado")
plt.axis('equal')
plt.title("Trajetória")
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(s_uniform, kappa, label="Curvatura κ(s)")
plt.plot(s_uniform, v_ref, label="v_ref(s)")
plt.title("Curvatura e Velocidade")
plt.legend()
plt.tight_layout()
plt.show()
