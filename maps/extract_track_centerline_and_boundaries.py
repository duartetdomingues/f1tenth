import cv2
import yaml
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage.util import invert

# === CONFIGURAÇÕES ===
map_pgm_path = "map_2025_05_07-19_47.pgm"
map_yaml_path = "map_2025_05_07-19_47.yaml"

# === FUNÇÃO: Carrega o mapa e parâmetros ===
def load_map(map_pgm_path, map_yaml_path):
    with open(map_yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']  # [x, y, theta]

    map_img = cv2.imread(map_pgm_path, cv2.IMREAD_GRAYSCALE)
    _, binary = cv2.threshold(map_img, 250, 255, cv2.THRESH_BINARY)

    # === Mostrar imagem com escala em metros ===
    height, width = map_img.shape
    extent = [
        origin[0],                    # min x (esq)
        origin[0] + width * resolution,  # max x (dir)
        origin[1],                    # min y (baixo)
        origin[1] + height * resolution  # max y (cima)
    ]

    plt.figure(figsize=(10, 10))
    plt.imshow(map_img, cmap='gray', origin='lower', extent=extent)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Mapa SLAM com escala real")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    return binary, resolution, origin

# === FUNÇÃO: Extrai skeleton (centerline) ===
def extract_centerline(binary_map):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    eroded = cv2.erode(binary_map, kernel)

    skeleton = skeletonize(invert(eroded // 255))  # Boolean
    return skeleton

# === FUNÇÃO: Extrai contornos (bordas) ===
def extract_boundaries(binary_map):
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    plt.figure(figsize=(10, 10))
    plt.imshow(binary_map, cmap='gray', origin='upper')  # origin 'upper' = pixel (0,0) no topo-esquerdo

    for contour in contours:
        contour = contour.reshape(-1, 2)  # (x, y) em pixels
        xs, ys = contour[:, 0], contour[:, 1]
        plt.plot(xs, ys, color='red', linewidth=1)

    plt.title("Contornos da pista (em pixels)")
    plt.xlabel("Coluna (pixel)")
    plt.ylabel("Linha (pixel)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()
    return contours

# === FUNÇÃO: Converte coordenadas (px → mundo) ===
def pixel_to_world(i, j, resolution, origin):
    x = origin[0] + j * resolution
    y = origin[1] + i * resolution
    return (x, y)

# === MAIN ===
binary_map, resolution, origin = load_map(map_pgm_path, map_yaml_path)


# Boundaries
contours = extract_boundaries(binary_map)  # Invert: obstáculos a branco
boundaries_world = []
for contour in contours:
    contour = contour.reshape(-1, 2)  # Garantir (x, y)
    world_pts = []
    for pt in contour:
        j, i = pt  # (coluna, linha)
        world_pts.append(pixel_to_world(i, j, resolution, origin))
    boundaries_world.append(world_pts)

# === PLOT ===
plt.figure(figsize=(10, 10))
plt.imshow(binary_map, cmap='gray')
#plt.scatter([j for i, j in skeleton_coords_px], [i for i, j in skeleton_coords_px], s=1, c='red', label='Centerline')
for contour in contours:
    contour = contour.squeeze()
    plt.plot(contour[:,0], contour[:,1], color='blue', linewidth=1, label='Boundary')
plt.legend()
plt.title("Centerline and Boundaries")
plt.gca().invert_yaxis()
plt.axis("equal")
plt.show()

# === OUTPUT EXEMPLO ===
print(f"[INFO] Centerline points (world): {len(centerline_world)}")
print(f"[INFO] Boundary points (world): {len(boundary_world)}")
