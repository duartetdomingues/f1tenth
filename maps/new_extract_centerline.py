import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import os
import yaml
import pandas as pd
from shapely.geometry import LineString, Point

# === 1. Carregar mapa binário (0 = livre, 1 = obstáculo)
map_files = "../maps/map_2025-09-09_10-52-29/map_output"

map_path = map_files + ".pgm"

map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

if map_img is None:
    raise FileNotFoundError(f"Não foi possível ler: {map_path}")

# Load resolution and origin from a .yaml file
yaml_path = map_files + ".yaml"
with open(yaml_path, 'r') as yaml_file:
    yaml_data = yaml.safe_load(yaml_file)

if yaml_data is None:
    raise ValueError(f"Could not read YAML data from path: {yaml_path}")

# Extract resolution and origin with defaults
resolution = yaml_data.get('resolution', 0.05)
origin = yaml_data.get('origin', [-10.0, -10.0])

# Flip the map vertically (instead of flipping the origin y)
map_img = np.flipud(map_img)

# Convert origin from meters to pixel coordinates
origin_px = [(-origin[0]) / resolution, (-origin[1]) / resolution]

# Plot the map and origin
plt.figure(figsize=(8, 6))
plt.imshow(map_img, cmap='gray', origin='lower')
plt.scatter(origin_px[0], origin_px[1], c='red', s=80, label='Origin')
plt.title("Mapa binário e origem")
plt.legend()
plt.axis("equal")
plt.show()

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

# Adiciona o primeiro ponto ao final de cada contorno para fechar o loop
contour1 = np.vstack([contour1, contour1[0]])
contour2 = np.vstack([contour2, contour2[0]])

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

# Detectar o loop: encontra o índice onde o ponto está mais próximo do ponto inicial (exceto o próprio)
start_pt = centerline_points[0]
dists_to_start = np.linalg.norm(centerline_points[1:] - start_pt, axis=1)
loop_idx = np.argmin(dists_to_start) + 1  # +1 porque ignoramos o primeiro

# Calcular comprimento de arco apenas até fechar o loop
ds = np.sqrt(np.sum(np.diff(centerline_points[:loop_idx+1], axis=0)**2, axis=1))
s = np.insert(np.cumsum(ds), 0, 0)
total_length_px = s[-1]

print(f"Comprimento total da centerline: {total_length_px*resolution:.2f} metros")


amostragem = 0.01  # em metros

# Amostrar pontos a cada 0.01m usando interpolação linear
s_uniform = np.arange(0, total_length_px * resolution + amostragem, amostragem)
print(f"Total de pontos amostrados: {s_uniform}")
s_uniform_px = s_uniform / resolution
smoothed_points = np.zeros((len(s_uniform_px), 2))
smoothed_points[:, 0] = np.interp(s_uniform_px, s, centerline_points[:, 0])
smoothed_points[:, 1] = np.interp(s_uniform_px, s, centerline_points[:, 1])

print(f"S uniform: {s_uniform}")

# === 7. Plot the smoothed points
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.scatter(smoothed_points[:, 0], smoothed_points[:, 1], c='red', s=5,  label='Smoothed cycle')
plt.scatter(smoothed_points[0, 0], smoothed_points[0, 1], c='blue', s=50, label='Start point')
plt.legend()
plt.title("Generated points from skeleton")
plt.axis("equal")
plt.show()


def smooth_points(points, window_length=51, polyorder=3):
    smoothed_x = savgol_filter(points[:, 0], window_length=window_length, polyorder=polyorder, mode='wrap')
    smoothed_y = savgol_filter(points[:, 1], window_length=window_length, polyorder=polyorder, mode='wrap')
    return np.column_stack([smoothed_x, smoothed_y])

window_length = 0.01 / resolution / amostragem  # janela de 0.1m

print(f"Window length points: {window_length}")

smoothed_points = smooth_points(smoothed_points, window_length=window_length, polyorder=3)

# === 7. Plot the smoothed points
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.scatter(smoothed_points[:, 0], smoothed_points[:, 1], c='red', s=5,  label='Smoothed cycle')
plt.scatter(smoothed_points[0, 0], smoothed_points[0, 1], c='blue', s=50, label='Start point contorns')
plt.legend()
plt.title("Smoothed cycle extracted from skeleton")
plt.axis("equal")
plt.show()

# === 6. Para coordenadas reais
#origin = [-o for o in origin]  # Invert origin to match the image coordinate system

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

print(f"Total length after smoothing: {s[-1]:.2f} meters")


# Ensure uniform sampling with proper floating-point precision
s_uniform = np.arange(0, s[-1], amostragem)
s_3x = np.arange(-s[-1], 2 * s[-1], 0.01)

x_s = spline_x(s_3x)
y_s = spline_y(s_3x)
dx = np.gradient(x_s, s_3x %  s[-1] )
dy = np.gradient(y_s, s_3x %  s[-1] )

ddx = np.gradient(dx, s_3x %  s[-1] )
ddy = np.gradient(dy, s_3x %  s[-1] )

kappa_d = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5


# 2) filtro passa-baixo em κ, periódico em s
from scipy.ndimage import gaussian_filter1d

s_step = 0.01  # usa o mesmo passo que o teu MPC por distância (Δs_MPC)
fwhm_m = 0.1  # largura de suavização em metros (ajusta: 0.15–0.40 m p/ F1TENTH)
sigma_samples = (fwhm_m / 2.355) / s_step
kappa = gaussian_filter1d(kappa_d, sigma=sigma_samples, mode='wrap')

s_index = np.searchsorted(s_3x, s_uniform)  # Retorna os índices correspondentes



# === Display básico de κ(s)
print(
    f"κ stats  |  min={kappa.min():.5f}  max={kappa.max():.5f}  "
    f"mean={kappa.mean():.5f}  std={kappa.std():.5f}"
)

plt.figure(figsize=(10,4))
plt.plot(s_3x, kappa_d, lw=1)
plt.plot(s_3x, kappa, lw=1)
plt.xlabel('s [m]')
plt.ylabel('κ [1/m]')
plt.title('Curvatura κ ao longo da trajetória')
plt.grid(True)
plt.tight_layout()
plt.show()

kappa=kappa[s_index]
x_s = x_s[s_index]
y_s = y_s[s_index]

x_s_pixel = (x_s - origin[0]) / resolution
y_s_pixel = (y_s - origin[1]) / resolution

print(f"len(s_uniform): {len(s_uniform)}")
print(f"len(x_s_pixel): {len(x_s_pixel)}")
print(f"len(y_s_pixel): {len(y_s_pixel)}")


# === 8. Extrair as bordas (margens) reais dos contornos


# Smooth dos contornos reais
def smooth_points(points, window_length=51, polyorder=3):
    # window_length deve ser ímpar e <= número de pontos
    window_length = min(window_length, len(points) // 2 * 2 + 1)
    if window_length < 3: window_length = 3
    smoothed_x = savgol_filter(points[:, 0], window_length=window_length, polyorder=polyorder, mode='wrap')
    smoothed_y = savgol_filter(points[:, 1], window_length=window_length, polyorder=polyorder, mode='wrap')
    return np.column_stack([smoothed_x, smoothed_y])

contour1 = np.vstack([contour1, contour1[0]])
contour2 = np.vstack([contour2, contour2[0]])

# Converta os contornos para coordenadas reais (metros)
contour1_real = np.empty_like(contour1, dtype=np.float64)
contour2_real = np.empty_like(contour2, dtype=np.float64)
# Converta os contornos para coordenadas reais (metros)
contour1_real[:, 0] = origin[0] + contour1[:, 0] * resolution
contour1_real[:, 1] = origin[1] + contour1[:, 1] * resolution
contour2_real[:, 0] = origin[0] + contour2[:, 0] * resolution
contour2_real[:, 1] = origin[1] + contour2[:, 1] * resolution

# Visualize as margens reais dos contornos
plt.figure(figsize=(10, 6))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.plot((x_s - origin[0]) / resolution, (y_s - origin[1]) / resolution, 'g', label='Centerline')
plt.plot(contour1[:, 0], contour1[:, 1], 'r', label='Contour 1 (real margin)')
plt.plot(contour2[:, 0], contour2[:, 1], 'b', label='Contour 2 (real margin)')
plt.legend()
plt.title("Centerline e margens reais dos contornos")
plt.axis("equal")
plt.show()

# === 10. Compute boundaries (right/left) for each centerline point using contours

def find_normal_distances(centerline_points, contour_left, contour_right):
    # Para cada ponto da centerline, projeta perpendicularmente ao vetor tangente local
    # e encontra a interseção com os contornos (margens esquerda e direita).
    # Retorna duas listas: n_l (esquerda), n_r (direita)

    contour_line_left = LineString(contour_left)
    contour_line_right = LineString(contour_right)
    n_l = []
    n_r = []

    for i, pt in enumerate(centerline_points):
        # Vetor tangente: diferença entre pontos vizinhos
        if i == 0:
            tangent = centerline_points[1] - centerline_points[0]
        elif i == len(centerline_points) - 1:
            tangent = centerline_points[-1] - centerline_points[-2]
        else:
            tangent = centerline_points[i + 1] - centerline_points[i - 1]

        tangent = tangent / np.linalg.norm(tangent)
        normal_left = np.array([-tangent[1], tangent[0]])   # 90 graus CCW
        normal_right = np.array([tangent[1], -tangent[0]])  # 90 graus CW

        # Cria linhas longas nas direções normais
        # Semirreta: começa em pt e vai para a esquerda (normal_left)
        line_left = LineString([pt, pt + 100000 * normal_left])
        line_right = LineString([pt, pt + 100000 * normal_right])

        def get_nearest_intersection(intersection, pt, normal):
            if intersection.is_empty:
                return None
            if intersection.geom_type == 'MultiPoint':
                points = np.array([[p.x, p.y] for p in intersection.geoms])
            elif intersection.geom_type == 'Point':
                points = np.array([[intersection.x, intersection.y]])
            else:
                return None
            dists = np.linalg.norm(points - pt, axis=1)
            #print(f"dists: {dists}")
            return dists

        # Margem esquerda
        intersection_left = contour_line_left.intersection(line_left)
        dists_left = get_nearest_intersection(intersection_left, pt, normal_left)
        intersection_right = contour_line_right.intersection(line_left)
        dists_right = get_nearest_intersection(intersection_right, pt, normal_left)

        if dists_left is not None and dists_right is not None:
            dist = min(min(dists_left), min(dists_right))
        elif dists_left is not None:
            dist = min(dists_left)
        elif dists_right is not None:
            dist = min(dists_right)
        else:
            print(f"Warning: No intersection found for left margin at point {i}, using distance from right margin.")

        """if nearest_left is None:
            dist_left = n_l[-1] 
            print(f"Warning: No intersection found for left margin at point {i}, using last valid distance.")
             intersection_right = contour_line_right.intersection(line_left)
            nearest_right, dist_left = get_nearest_intersection(intersection_right, pt, normal_left)
            
            if nearest_right is None:
                # Fallback: ponto mais próximo do contorno direito
                distances = np.linalg.norm(contour_right - pt, axis=1)
                nearest_idx = np.argmin(distances)
                nearest_left = contour_left[nearest_idx]
                dist_left = np.dot(nearest_left - pt, normal_left)
                print(f"Warning: No intersection found for left margin at point {i}, using nearest point from left contour.")
            else:
                print(f"Warning: No intersection found for left margin at point {i}, using nearest point from right contour.") """
        
        #print(f"margem esquerda: {dist}")
        n_l.append(dist)

        # Margem direita
        intersection_right = contour_line_right.intersection(line_right)
        dists_right = get_nearest_intersection(intersection_right, pt, normal_right)
        intersection_left = contour_line_left.intersection(line_right)
        dists_left = get_nearest_intersection(intersection_left, pt, normal_right)
        if dists_left is not None and dists_right is not None:
            dist = min(min(dists_left), min(dists_right))
        elif dists_left is not None:
            dist = min(dists_left)
        elif dists_right is not None:
            dist = min(dists_right)

        """if nearest_right is None:
            dist_right = n_r[-1]
            print(f"Warning: No intersection found for right margin at point {i}, using last valid distance.")
             intersection_left = contour_line_left.intersection(line_right)
            nearest_left, dist_right = get_nearest_intersection(intersection_left, pt, normal_right)
            if nearest_left is None:
                # Fallback: ponto mais próximo do contorno direito
                distances = np.linalg.norm(contour_right - pt, axis=1)
                nearest_idx = np.argmin(distances)
                nearest_right = contour_right[nearest_idx]
                dist_right = np.dot(nearest_right - pt, normal_right)
                print(f"Warning: No intersection found for right margin at point {i}, using nearest point.")
            else:
                print(f"Warning: No intersection found for right margin at point {i}, using nearest point from left contour.") """
                
        #print(f"margem direita: {dist}")
        n_r.append(dist)

    return np.array(n_l), np.array(n_r)

centerline_real = np.column_stack((x_s_pixel, y_s_pixel))

# Calcule as distâncias normais para cada margem
n_l_pixel, n_r_pixel = find_normal_distances(centerline_real, contour1, contour2)
""" n_l_pixel = np.abs(n_l_pixel)
n_r_pixel = np.abs(n_r_pixel) """

n_l = n_l_pixel * resolution
n_r = n_r_pixel * resolution

#plot the normal distances to the boundaries
plt.figure(figsize=(10, 4))
plt.plot( s_uniform, n_l, label='Margem esquerda (n_l)')
plt.plot( s_uniform, n_r, label='Margem direita (n_r)')
plt.xlabel('s [m]')
plt.ylabel('Distância normal [m]')
plt.title('Distâncias normais às margens ao longo de s')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()




# Plot 2D das distâncias normais (n_l e n_r) ao longo da centerline
plt.figure(figsize=(10, 6))
plt.plot(smoothed_points[:, 0], smoothed_points[:, 1], 'k-', label='Centerline')

# Calcular psi (direção local) para cada ponto da centerline
dx = np.gradient(x_s_pixel)
dy = np.gradient(y_s_pixel)
psi = np.arctan2(dy, dx)

# Margem esquerda
x_l = x_s_pixel + n_l_pixel * np.cos(psi + np.pi / 2)
y_l = y_s_pixel + n_l_pixel * np.sin(psi + np.pi / 2)
plt.scatter(x_l, y_l, c='r', s=0.5, label='Margem esquerda (n_l)')

# Margem direita
x_r = x_s_pixel + n_r_pixel * np.cos(psi - np.pi / 2)
y_r = y_s_pixel + n_r_pixel * np.sin(psi - np.pi / 2)
plt.scatter(x_r, y_r, c='b', s=0.5, label='Margem direita (n_r)')

plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Distâncias normais às margens em 2D')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

#plot in pixels 2d
plt.figure(figsize=(10, 4))
plt.imshow(occupancy, cmap='gray', origin='lower')
plt.plot(x_s_pixel, y_s_pixel, 'g-', label='Centerline')
x_l_pixel = x_s_pixel + n_l_pixel * np.cos(psi + np.pi / 2)
y_l_pixel = y_s_pixel + n_l_pixel * np.sin(psi + np.pi / 2)
plt.scatter(x_l_pixel, y_l_pixel, c='red', s=0.5, label='Margem esquerda (n_l)')
x_r_pixel = x_s_pixel + n_r_pixel * np.cos(psi - np.pi / 2)
y_r_pixel = y_s_pixel + n_r_pixel * np.sin(psi - np.pi / 2)
plt.scatter(x_r_pixel, y_r_pixel, c='blue', s=0.5, label='Margem direita (n_r)')
plt.xlabel('X [pixels]')
plt.ylabel('Y [pixels]')
plt.title('Distâncias normais às margens em 2D (pixels)')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()

print(f"len(s_uniform): {len(s_uniform)}")
print(f"len(x_s): {len(x_s_pixel)}")
print(f"len(y_s): {len(y_s_pixel)}")
print(f"len(n_l): {len(n_l)}")
print(f"len(n_r): {len(n_r)}")


# Get the last folder name from the map path
last_folder_name = os.path.basename(os.path.dirname(map_path))
print(f"Last folder name: {last_folder_name}")

filename = "centerline_v2_" + last_folder_name + ".csv"


# Export to CSV
df = pd.DataFrame({"s": np.round(s_uniform, 3), "x": np.round(x_s, 3), "y": np.round(y_s, 3), "kappa": np.round(kappa, 3), "n_l": np.round(n_l, 3), "n_r": np.round(n_r, 3)})
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


