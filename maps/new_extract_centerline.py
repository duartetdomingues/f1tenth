import cv2
import numpy as np
import matplotlib.pyplot as plt

# === 1. Carregar mapa binário (0 = livre, 1 = obstáculo)
map_files = "./maps/test_map/map_output"

map_path = map_files + ".pgm"

map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

if map_img is None:
    raise FileNotFoundError(f"Não foi possível ler: {map_path}")

# Binarizar (255 = livre, 0 = obstáculo)
binary = (map_img > 220).astype(np.uint8) * 255

# === 2. Limpeza morfológica
kernel = np.ones((5, 5), np.uint8)
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

# === 3. Encontrar contornos
contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

# Filtrar apenas os dois maiores contornos (margens interna e externa)
contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

# Garantir que temos exatamente 2 margens
if len(contours) < 2:
    raise RuntimeError("Não foi possível encontrar as duas margens da pista.")

# === 4. Ordenar pontos ao longo do loop
def order_contour_points(contour):
    contour = contour[:, 0, :]  # remover dimensões extra
    # opcional: simplificar para menos pontos
    epsilon = 0.001 * cv2.arcLength(contour, True)
    contour = cv2.approxPolyDP(contour, epsilon, True)[:, 0, :]
    return contour

contour1 = order_contour_points(contours[0])
contour2 = order_contour_points(contours[1])

# === 5. Emparelhar pontos por proximidade
centerline_points = []
for p1 in contour1:
    # encontrar ponto mais próximo na outra margem
    dists = np.linalg.norm(contour2 - p1, axis=1)
    p2 = contour2[np.argmin(dists)]
    mid = (p1 + p2) / 2.0
    centerline_points.append(mid)

centerline_points = np.array(centerline_points)

# === 6. Visualizar
plt.figure(figsize=(10, 6))
plt.imshow(binary, cmap='gray', origin='lower')
plt.plot(contour1[:, 0], contour1[:, 1], 'r-', label='Margem 1')
plt.plot(contour2[:, 0], contour2[:, 1], 'b-', label='Margem 2')
plt.plot(centerline_points[:, 0], centerline_points[:, 1], 'g-', linewidth=1.5, label='Centerline')
plt.legend()
plt.title("Centerline calculada a partir dos contornos")
plt.axis("equal")
plt.show()

print(f"Total de pontos na centerline: {len(centerline_points)}")

occupancy = binary

#Smooth the ordered points
""" from scipy.signal import savgol_filter
import os
import yaml

def smooth_points(points, window_length=51, polyorder=3):
    smoothed_x = savgol_filter(points[:, 0], window_length=window_length, polyorder=polyorder, mode='wrap')
    smoothed_y = savgol_filter(points[:, 1], window_length=window_length, polyorder=polyorder, mode='wrap')
    return np.column_stack([smoothed_x, smoothed_y])

smoothed_points = smooth_points(centerline_points, window_length=5, polyorder=3) """

# Calcular comprimento de arco dos pontos originais
ds = np.sqrt(np.sum(np.diff(centerline_points, axis=0)**2, axis=1))
s = np.insert(np.cumsum(ds), 0, 0)
total_length = s[-1]

# Amostrar pontos a cada 0.5m usando interpolação linear
s_uniform = np.arange(0, total_length, 0.5)
smoothed_points = np.zeros((len(s_uniform), 2))
smoothed_points[:, 0] = np.interp(s_uniform, s, centerline_points[:, 0])
smoothed_points[:, 1] = np.interp(s_uniform, s, centerline_points[:, 1])

print(f"Total de pontos amostrados a cada 0.5m: {len(smoothed_points)}")

# === 7. Plot the smoothed points
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.scatter(smoothed_points[:, 0], smoothed_points[:, 1], c='red', s=5,  label='Smoothed cycle')
plt.scatter(smoothed_points[0, 0], smoothed_points[0, 1], c='blue', s=50, label='Start point')
plt.legend()
plt.title("Generated points from skeleton")
plt.axis("equal")
plt.show()

from scipy.signal import savgol_filter
import os
import yaml

def smooth_points(points, window_length=51, polyorder=3):
    smoothed_x = savgol_filter(points[:, 0], window_length=window_length, polyorder=polyorder, mode='wrap')
    smoothed_y = savgol_filter(points[:, 1], window_length=window_length, polyorder=polyorder, mode='wrap')
    return np.column_stack([smoothed_x, smoothed_y])

smoothed_points = smooth_points(smoothed_points, window_length=200, polyorder=3) 

# === 7. Plot the smoothed points
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.scatter(smoothed_points[:, 0], smoothed_points[:, 1], c='red', s=5,  label='Smoothed cycle')
plt.scatter(smoothed_points[0, 0], smoothed_points[0, 1], c='blue', s=50, label='Start point')
plt.legend()
plt.title("Smoothed cycle extracted from skeleton")
plt.axis("equal")
plt.show()

# === 6. Para coordenadas reais
# Load resolution and origin from a .yaml file
yaml_path = map_files + ".yaml"
with open(yaml_path, 'r') as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

if yaml_data is None:
    raise ValueError(f"Could not read YAML data from path: {yaml_path}")

# Extract resolution and origin
resolution = yaml_data.get('resolution')  # Default to 0.05 if not found
origin = yaml_data.get('origin')  # Default to [-10.0, -10.0] if not found

#origin = [-o for o in origin]  # Invert origin to match the image coordinate system

print(f"Resolution: {resolution}")
print(f"Origin: {origin}")

x_px, y_px = smoothed_points[:, 0], smoothed_points[:, 1]
x = origin[0] + x_px * resolution
y = origin[1] + y_px * resolution

#reorder points to start from the point closest to the origin
# Reorder points to start from the point closest to the origin
distances_to_origin = np.sqrt((x)**2 + (y)**2)
start_index = np.argmin(distances_to_origin)

# Reorder x and y arrays
x = np.concatenate([x[start_index:], x[:start_index]])
y = np.concatenate([y[start_index:], y[:start_index]])

# Inverter sentido da trajetória (reverse direction)
x = x[::-1]
y = y[::-1]

from scipy.interpolate import CubicSpline

# === 7. Arc-length and spline
ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
s = np.insert(np.cumsum(ds), 0, 0)
spline_x = CubicSpline(s, x)
spline_y = CubicSpline(s, y)


# Ensure uniform sampling with proper floating-point precision
s = np.arange(0, s[-1] + 0.01, 0.01).round(2)
s_uniform = np.arange(-s[-1], 2 * s[-1], 0.01)

x_s = spline_x(s_uniform %  s[-1] )
y_s = spline_y(s_uniform %  s[-1] )
dx = np.gradient(x_s, s_uniform %  s[-1] )
dy = np.gradient(y_s, s_uniform %  s[-1] )

ddx = np.gradient(dx, s_uniform %  s[-1] )
ddy = np.gradient(dy, s_uniform %  s[-1] )

kappa_d = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5


# 2) filtro passa-baixo em κ, periódico em s
from scipy.ndimage import gaussian_filter1d

s_step = 0.01  # usa o mesmo passo que o teu MPC por distância (Δs_MPC)
fwhm_m = 0.1  # largura de suavização em metros (ajusta: 0.15–0.40 m p/ F1TENTH)
sigma_samples = (fwhm_m / 2.355) / s_step
kappa = gaussian_filter1d(kappa_d, sigma=sigma_samples, mode='wrap')

s_index = np.searchsorted(s_uniform, s)  # Retorna os índices correspondentes



# === Display básico de κ(s)
print(
    f"κ stats  |  min={kappa.min():.5f}  max={kappa.max():.5f}  "
    f"mean={kappa.mean():.5f}  std={kappa.std():.5f}"
)

plt.figure(figsize=(10,4))
plt.plot(s_uniform, kappa_d, lw=1)
plt.plot(s_uniform, kappa, lw=1)
plt.xlabel('s [m]')
plt.ylabel('κ [1/m]')
plt.title('Curvatura κ ao longo da trajetória')
plt.grid(True)
plt.tight_layout()
plt.show()

kappa=kappa[s_index]
s_uniform=s
x_s = x_s[s_index]
y_s = y_s[s_index]

""" # === 7. Arc-length and spline (versão suavizante, periódica)
# Fecha o loop se necessário
if np.hypot(x[0]-x[-1], y[0]-y[-1]) > 1e-6:
x = np.r_[x, x[0]]
y = np.r_[y, y[0]]

# Parâmetro de suavização (ajusta se precisares mais/menos suavidade)
N = len(x)
s_smooth = 1.0 * N * (resolution**2)  # heurística prática

# Spline suavizante periódica
tck, u = splprep([x, y], s=s_smooth, per=True, k=3)

# Reamostra no parâmetro u
M = max(2000, 4*N)
u_uniform = np.linspace(0, 1, M, endpoint=False)

# Derivadas em u
x_u,  y_u  = splev(u_uniform, tck, der=0)
dx_u, dy_u = splev(u_uniform, tck, der=1)
ddx_u, ddy_u = splev(u_uniform, tck, der=2)

# Curvatura invariante à parametrização
kappa_u = (dx_u*ddy_u - dy_u*ddx_u) / (dx_u**2 + dy_u**2)**1.5

# Reparametriza por comprimento de arco s
speed = np.hypot(dx_u, dy_u)           # ||r'(u)||
du = u_uniform[1] - u_uniform[0]
s_u = np.cumsum((speed[:-1] + speed[1:]) * 0.5 * du)
s_u = np.r_[0.0, s_u]
L = s_u[-1]

# Grelha uniforme em s (passo igual ao teu 0.01 m)
s_step = 0.01
s_uniform = np.arange(0.0, L + 0.5*s_step, s_step)

# Invertemos s(u) -> u(s)
u_of_s = np.interp(s_uniform, s_u, u_uniform)

# Valores finais em s uniforme
x_s = np.array(splev(u_of_s, tck, der=0)[0])
y_s = np.array(splev(u_of_s, tck, der=0)[1])
dx_s, dy_s   = splev(u_of_s, tck, der=1)
ddx_s, ddy_s = splev(u_of_s, tck, der=2)
kappa = (dx_s*ddy_s - dy_s*ddx_s) / (dx_s**2 + dy_s**2)**1.5  # mantém o nome 'kappa'

x_s, y_s    = splev(s_uniform, tck, der=0)
dx,  dy     = splev(s_uniform, tck, der=1)
ddx, ddy    = splev(s_uniform, tck, der=2)
kappa = (dx*ddy - dy*ddx) / (dx**2 + dy**2)**1.5 """


# Get the last folder name from the map path
last_folder_name = os.path.basename(os.path.dirname(map_path))
print(f"Last folder name: {last_folder_name}")

filename = "centerline_v2_" + last_folder_name + ".csv"

import pandas as pd

# Export to CSV
df = pd.DataFrame({"s": s_uniform, "x": x_s, "y": y_s, "kappa": kappa})
df.to_csv(filename, index=False)

# === 9. Visualize
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.plot((x_s - origin[0]) / resolution, (y_s - origin[1]) / resolution, 'r', label='Centerline')
plt.scatter((x_s[0] - origin[0]) / resolution, (y_s[0] - origin[1]) / resolution, c='blue', s=50, label='Start point')
plt.scatter(-origin[0]/ resolution , -origin[1]/ resolution , c='green', s=50, label='Map Origin')
# Add arrows to indicate direction
arrow_spacing = 100  # Adjust spacing between arrows
for i in range(0, len(x_s) - 1, arrow_spacing):
    plt.arrow(
        (x_s[i] - origin[0]) / resolution,
        (y_s[i] - origin[1]) / resolution,
        ((x_s[i + 1] - x_s[i]) / resolution) * 0.5,
        ((y_s[i + 1] - y_s[i]) / resolution) * 0.5,
        head_width=20,
        head_length=40,
        fc='blue',
        ec='blue'
    )
    
plt.legend()
plt.title("Centerline extraída do mapa")
plt.axis("equal")
plt.show()



psi = np.cumsum(kappa[:-1] * 0.01);  # Integração numérica da curvatura ao longo do comprimento da trajetória

psi_init = np.arctan2(y_s[1] - y_s[0], x_s[1] - x_s[0])

print(f"Ângulo inicial (psi_init): {psi_init} rad")

psi= psi  # Ajusta o ângulo inicial para zero

# Plot da trajetória com o ângulo de yaw
plt.figure(figsize=(8, 6))

# Plotando a trajetória (pista)
plt.plot(x_s, y_s, '-o', label="Trajetória")

dx = np.cos(psi)  # Componente x da direção
dy = np.sin(psi)  # Componente y da direção

# Plotando o ângulo de yaw (em função do ângulo de trajetória)
plt.quiver(x_s[:-1], y_s[:-1], dx, dy, scale=10, color='r', label="Direção (Yaw)")

# Títulos e rótulos
plt.title("Trajetória e Ângulo de Yaw")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()

# Mostrar o gráfico
plt.show()


