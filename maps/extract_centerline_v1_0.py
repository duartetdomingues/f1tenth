import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree
import pandas as pd
import yaml
import os
from sklearn.neighbors import NearestNeighbors

# 1. Ler e binarizar o mapa
filename_base = '/home/duarte/Desktop/f1tenth/maps/map_2025-07-16_15-28-18/map_output'
pgm_file = filename_base + '.pgm'
yaml_file = filename_base + '.yaml'

# Ler a imagem PGM
map_img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)

# Ler o arquivo YAML para obter as configurações
with open(yaml_file, 'r') as file:
    yaml_text = file.read()

# Identificar as paredes (valores inferiores a 245)
walls = map_img > 200

plt.imshow(walls, cmap='gray')
plt.title("Walls Map")
plt.axis('off')
plt.show()

# 2. Extrair contornos
contours, hierarchy = cv2.findContours(walls.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

# Filtrar os contornos para pegar tanto os internos quanto os externos
external_contours = []
internal_contours = []

for i, contour in enumerate(contours):
    if hierarchy[0][i][3] == -1:  # Contorno externo
        external_contours.append(contour[:, 0, :])
    else:  # Contorno interno
        internal_contours.append(contour[:, 0, :])
        
        
# Plotar os contornos encontrados
plt.imshow(walls, cmap='gray')
for contour in external_contours:
    plt.plot(contour[:, 0], contour[:, 1], linewidth=2, color='blue')
for contour in internal_contours:
    plt.plot(contour[:, 0], contour[:, 1], linewidth=2, color='red')
plt.title("Extracted Contours")
plt.axis('off')
plt.show()

# Suavizar e interpolar os contornos
smoothContours = []
smoothPerimeters = []

for contour in contours:
    # Contorno fechado ou aberto
    n_interp = 80
    contour = contour[:, 0, :]  # Remover a dimensão extra
    t = np.linspace(0, 1, len(contour))
    t_interp = np.linspace(0, 1, n_interp)
    
    x_smooth = np.interp(t_interp, t, contour[:, 0])
    y_smooth = np.interp(t_interp, t, contour[:, 1])
    
    smoothContours.append(np.column_stack([x_smooth, y_smooth]))

    # Calcular perímetro suavizado
    dx = np.diff(x_smooth)
    dy = np.diff(y_smooth)
    smoothPerimeters.append(np.sum(np.sqrt(dx**2 + dy**2)))

# 3. Ordenar os contornos por perímetro (decrescente)
sorted_idx = np.argsort(smoothPerimeters)[::-1]

# Exibir os contornos
plt.imshow(walls, cmap='gray')
for idx in sorted_idx[:4]:
    contour = smoothContours[idx]
    plt.plot(contour[:, 0], contour[:, 1], linewidth=2)
plt.title("Contours map smooth")
plt.show()

# Exibir os contornos da pista
plt.imshow(walls, cmap='gray')
for idx in sorted_idx[1:3]:
    contour = smoothContours[idx]
    plt.plot(contour[:, 0], contour[:, 1], linewidth=2)
plt.title("Contours track")
plt.show()

# 4. Supondo que c1 e c2 são os contornos da pista (bordas)
c1 = smoothContours[sorted_idx[1]]
c2 = smoothContours[sorted_idx[2]]

# Reamostrar os contornos
def resample_contour(c, n_interp):
    d = np.concatenate(([0], np.cumsum(np.sqrt(np.diff(c[:, 0]))**2 + np.diff(c[:, 1])**2)))
    spline_x = CubicSpline(d, c[:, 0])
    spline_y = CubicSpline(d, c[:, 1])
    d_uniform = np.linspace(0, d[-1], n_interp)
    return np.column_stack([spline_x(d_uniform), spline_y(d_uniform)])

c1_resampled = resample_contour(c1, 400)
c2_resampled = resample_contour(c2, 400)

# 5. Calcular a centerline como a média entre c1 e c2
nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(c2_resampled)
distances, indices = nbrs.kneighbors(c1_resampled)

c2_matched = c2_resampled[indices.flatten()]
centerline = (c1_resampled + c2_matched) / 2

# Visualização
plt.imshow(map_img, cmap='gray')
plt.plot(c1[:, 0], c1[:, 1], 'r-', linewidth=1.5, label='Margem C1')
plt.plot(c2[:, 0], c2[:, 1], 'g-', linewidth=1.5, label='Margem C2')
plt.plot(centerline[:, 0], centerline[:, 1], 'b--', linewidth=2, label='Centerline')
plt.legend()
plt.title("Contours and Centerline")
plt.axis('equal')
plt.show()

# 6. Suavização da centerline
window_size = 5  # Janela para suavização

# Estender a curva nas extremidades
centerline_ext = np.vstack([centerline[-window_size:], centerline, centerline[:window_size]])

# Suavização (Gaussian)
from scipy.ndimage import gaussian_filter1d
centerline_smooth = np.column_stack([
    gaussian_filter1d(centerline_ext[:, 0], sigma=window_size),
    gaussian_filter1d(centerline_ext[:, 1], sigma=window_size)
])

# Cortar de volta ao comprimento original
centerline_smooth = centerline_smooth[window_size:-window_size]

# Visualizar a centerline suavizada
plt.imshow(map_img, cmap='gray')
plt.plot(c1[:, 0], c1[:, 1], 'r-', linewidth=1.5, label='Margem C1')
plt.plot(c2[:, 0], c2[:, 1], 'g-', linewidth=1.5, label='Margem C2')
plt.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Centerline')
plt.plot(centerline_smooth[:, 0], centerline_smooth[:, 1], 'm-', linewidth=2, label='Smooth Centerline')
plt.legend()
plt.title("Smoothed Centerline")
plt.axis('equal')
plt.show()

# 7. Extrair resolução e origem do arquivo YAML
import yaml
yaml_data = yaml.safe_load(yaml_text)
resolution = yaml_data['resolution']
origin = np.array(yaml_data['origin'])

# 8. Calcular limites reais
img_height, img_width = map_img.shape
x_real = origin[0] + np.arange(img_width) * resolution
y_real = origin[1] + (img_height - 1 - np.arange(img_height)) * resolution

# Exibir a imagem com eixos reais
plt.imshow(map_img, cmap='gray')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Map with Real Coordinates')
plt.axis('equal')
plt.show()

# Converter para coordenadas reais
centerline_real = np.column_stack([
    centerline_smooth[:, 0] * resolution + origin[0],
    (img_height - 1 - centerline_smooth[:, 1]) * resolution + origin[1]
])

# Visualizar a centerline em coordenadas reais
plt.plot(centerline_real[:, 0], centerline_real[:, 1], 'b-', linewidth=2)
plt.title('Centerline in Real Coordinates')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.show()

# 9. Calcular yaw
dx = np.diff(centerline_real[:, 0])
dy = np.diff(centerline_real[:, 1])
yaw = np.arctan2(dy, dx)

# Plotar yaw
plt.quiver(centerline_real[:-1, 0], centerline_real[:-1, 1], np.cos(yaw), np.sin(yaw), angles='xy', scale_units='xy', scale=0.1)
plt.title('Centerline with Yaw')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.show()

# 10. Salvar os dados em CSV
data = pd.DataFrame({
    'x': centerline_real[:, 0],
    'y': centerline_real[:, 1],
    'yaw': np.concatenate([yaw, [yaw[-1]]]),  # Completar o yaw com o último valor
    'v': np.ones(len(yaw))  # Velocidade constante (pode ser ajustada)
})

csv_filename = '../traj/pts/centerline2_map.csv'
data.to_csv(csv_filename, index=False)
