import cv2
import numpy as np
import matplotlib.pyplot as plt
import os, yaml, pandas as pd
from shapely.geometry import LineString
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.signal import savgol_filter
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

##########################################
# CONFIGURAÇÕES
##########################################
#map_files = "/home/duarte/Desktop/f1tenth/maps/map_2025-10-11_19-15-42/map_output"
map_files = "/home/duarte/Desktop/f1tenth/maps/test_map/map_output"
traj_dir = "traj"
map_path = map_files + ".pgm"
yaml_path =  map_files + ".yaml"
direction = "counter-clockwise"  # ou "clockwise" ou "counter-clockwise"
amostragem = 0.01  # espaçamento em metros
plot = 1 # 0 = none, 1 = some, 2 = all debug plots

PLOT_NONE = 0
PLOT_MAIN = 1
PLOT_DEBUG = 2


def _plot_enabled(level=PLOT_MAIN):
    """Return True when the global plot flag permits plotting for the given level."""
    return plot >= level

##########################################
# FUNÇÕES UTILITÁRIAS
##########################################
def load_map(map_path, yaml_path):
    map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(f"Não foi possível ler: {map_path}")

    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    try:
        resolution = yaml_data['resolution']
        origin = yaml_data['origin'][:2]
    except KeyError as e:
        raise KeyError(f"Chave ausente no arquivo YAML: {e}")

    map_img = np.flipud(map_img)
    binary = (map_img > 150).astype(np.uint8) * 255
    return map_img, binary, resolution, origin

def extract_contours(binary, direction="counter-clockwise"):
    contours, _ = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    for i, contour in enumerate(contours):
        if direction == "counter-clockwise" and cv2.contourArea(contour, oriented=True) < 0:
            contours[i] = contour[::-1]
        elif direction == "clockwise" and cv2.contourArea(contour, oriented=True) > 0:
            contours[i] = contour[::-1]
    return contours[0][:, 0, :], contours[1][:, 0, :]

from scipy.signal import savgol_filter
from scipy.interpolate import splprep, splev, interp1d
from shapely.geometry import LineString, Point
from scipy.spatial import cKDTree
import numpy as np
from matplotlib.collections import LineCollection

def unit_vector(v):
    return v / np.linalg.norm(v)

def build_centerline(contour1, contour2, amostragem, resolution, direction="counter-clockwise", origin=(0,0)):
    """
    Constrói a centerline a partir de dois contornos (em pixels).
    - Usa emparelhamento de pontos entre contornos
    - Funde as duas centerlines
    - Ordena por abscissa curvilínea numa curva-guia
    - Suaviza e ajusta spline periódico
    - Reamostra uniformemente por comprimento de arco (amostragem/resolution)
    - Força orientação CW/CCW
    Retorna: centerline_px (array [N,2] em pixels, equiespaciado, ordenado)
    """

    # Order contours by proximity to origin in the specified direction
    def order_contour_by_origin(contour, origin_px, direction):
        # Calculate distances from each point in the contour to the origin
        distances = np.linalg.norm(contour - origin_px, axis=1)
        # Find the index of the closest point to the origin
        start_idx = np.argmin(distances)
        # Roll the contour so that it starts at the closest point
        ordered_contour = np.roll(contour, -start_idx, axis=0)
        # Reverse the contour if the direction is clockwise
        if direction == "clockwise":
            ordered_contour = ordered_contour[::-1]
        return ordered_contour

    origin_px = np.array([-origin[0]/resolution, -origin[1]/resolution])
    debug_plots = _plot_enabled(PLOT_DEBUG)

    # Order both contours
    contour1 = order_contour_by_origin(contour1, origin_px, direction)
    contour2 = order_contour_by_origin(contour2, origin_px, direction)

    # Visualize the contours and their ordering
    if debug_plots:
        plt.figure()
        plt.imshow(binary, cmap="gray", origin="lower")
        plt.scatter(contour1[:, 0], contour1[:, 1], c=np.arange(len(contour1)), cmap='viridis', s=8)
        plt.scatter(contour2[:, 0], contour2[:, 1], c=np.arange(len(contour2)), cmap='viridis', s=8)
        plt.scatter(origin_px[0], origin_px[1], c="g", label="Origin (px)", zorder=5)
        plt.legend()
        plt.title("Ordered Contours")
        plt.axis("equal")
        plt.show()

    window = int(len(contour2) * 0.1)
    n1 = len(contour1)
    n2 = len(contour2)
    print("Window size for matching:", window, n2)

    # === A) Emparelhar contorno1->contorno2
    centerline_points1 = []
    for i, p1 in enumerate(contour1):
        # Índices locais com wrap-around

        ii = int(i*n2/n1)
        idx_min = (ii - window) % n2
        idx_max = (ii + window) % n2

        if idx_min < idx_max:
            local_contour2 = contour2[idx_min:idx_max]
        else:
            local_contour2 = np.vstack((contour2[idx_min:], contour2[:idx_max]))

        # Emparelha só dentro da janela
        dists = np.linalg.norm(local_contour2 - p1, axis=1)
        p2 = local_contour2[np.argmin(dists)]
        centerline_points1.append((p1 + p2) / 2.0)

    # Garantir que o array seja circular
    centerline_points1 = np.array(centerline_points1)
    centerline_points1 = np.vstack([centerline_points1, centerline_points1[0]])

    # === B) Emparelhar contorno2->contorno1
    centerline_points2 = []
    window = int(len(contour1) * 0.1)

    for i, p2 in enumerate(contour2):
        # Índices locais com wrap-around
        ii = int(i*n1/n2)
        idx_min = (ii - window) % n1
        idx_max = (ii + window) % n1

        if idx_min < idx_max:
            local_contour1 = contour1[idx_min:idx_max]
        else:
            local_contour1 = np.vstack((contour1[idx_min:], contour1[:idx_max]))

        # Emparelha só dentro da janela
        dists = np.linalg.norm(local_contour1 - p2, axis=1)
        p1 = local_contour1[np.argmin(dists)]
        centerline_points2.append((p1 + p2) / 2.0)

    # === C) Fundir e eliminar quase-duplicados
    cl_raw = np.vstack([centerline_points1, centerline_points2])
    cl_raw = np.unique(np.round(cl_raw, 2), axis=0)  # arredonda a 1 casa decimal

    # === D) Construir curva-guia (usando a primeira centerline)
    guide = LineString(np.vstack([centerline_points1, centerline_points1[0]]))
    L_px = guide.length

    # === E) Ordenar por abscissa curvilínea
    s_vals = np.array([guide.project(Point(p[0], p[1])) for p in cl_raw])
    ord_idx = np.argsort(s_vals)
    ordered = cl_raw[ord_idx]
    s_sorted = s_vals[ord_idx]

    # Plot order of points for debugging
    if debug_plots:
        plt.figure()
        plt.imshow(binary, cmap="gray", origin="lower")
        plt.scatter(ordered[:,0], ordered[:,1], c=np.arange(len(ordered)), cmap='viridis', s=8)
        plt.colorbar(label='Order')
        plt.title("Order of centerline points before spline fit")
        plt.axis("equal")
        plt.show()

    # === F) Cortar no maior gap (para fechar corretamente)
    gaps = np.diff(np.r_[s_sorted, s_sorted[0] + L_px])
    cut = np.argmax(gaps)
    ordered = np.roll(ordered, -(cut + 1), axis=0)

    # === G) Deduplicar por raio
    def dedup_by_radius(pts, r=0.5):
        if len(pts) == 0:
            return pts
        tree = cKDTree(pts)
        used = np.zeros(len(pts), dtype=bool)
        keep_idx = []
        for i in range(len(pts)):
            if used[i]:
                continue
            keep_idx.append(i)
            idx = tree.query_ball_point(pts[i], r)
            used[idx] = True
        return pts[keep_idx]

    ordered = dedup_by_radius(ordered, r=0.5)

    # === H) Remover saltos grandes
    if len(ordered) >= 3:
        step = np.hypot(np.diff(np.r_[ordered[:,0], ordered[0,0]]),
                        np.diff(np.r_[ordered[:,1], ordered[0,1]]))
        med = np.median(step)
        if med > 0:
            mask = step < 3.0*med
            keep = np.ones(len(ordered), dtype=bool)
            keep[np.where(~mask)[0]] = False
            ordered = ordered[keep] 

    # === I) Suavização leve antes do fit
    # === I) Suavização (controlada por parâmetro)
    # Defina antes de chamar build_centerline (valor padrão = 1.0):
    # CENTERLINE_SMOOTH = 0    -> sem suavização
    # CENTERLINE_SMOOTH = 0.5  -> muito leve
    # CENTERLINE_SMOOTH = 1.0  -> padrão
    # CENTERLINE_SMOOTH = 2.0  -> mais forte
    smooth_param = 0.5  # ajustar conforme necessário

    if smooth_param <= 0 or len(ordered) < 7:
        # Sem suavização
        x_sg, y_sg = ordered[:, 0], ordered[:, 1]
    else:
        # Calcula janela ímpar proporcional ao tamanho e ao fator
        # Cresce bem devagar com o número de pontos e com smooth_param
        base = int(((len(ordered) / 800.0) * smooth_param))
        win = max(5, base * 2 + 1)              # garante ímpar
        # Limite superior também escalonado (mantém controle)
        max_cap = int(5 + smooth_param * 12)    # 5..17 (para 1.0) / maior p/ >1
        if max_cap % 2 == 0:
            max_cap += 1
        win = min(win, max_cap)
        # Segurança: janela não pode ser >= número de pontos
        if win >= len(ordered):
            win = len(ordered) - 1
            if win % 2 == 0:
                win -= 1
        # Fallback final
        win = max(5, win)
        x_sg = savgol_filter(ordered[:, 0], window_length=win, polyorder=2, mode='wrap')
        y_sg = savgol_filter(ordered[:, 1], window_length=win, polyorder=2, mode='wrap')

    ordered = np.column_stack([x_sg, y_sg])

    # Plot order of points for debugging
    if debug_plots:
        plt.figure()
        plt.imshow(binary, cmap="gray", origin="lower")
        plt.scatter(ordered[:,0], ordered[:,1], c=np.arange(len(ordered)), cmap='viridis', s=8)
        plt.colorbar(label='Order')
        plt.title("Order of centerline points before spline fit")
        plt.axis("equal")
        plt.show()

    # === J) Fit spline periódico
    closed = np.vstack([ordered, ordered[0]])
    s_smooth = (0.2**2) * len(closed)   # suavização ajustada para evitar cortes excessivos

    print("Spline smoothing factor:", s_smooth)
    tck, u = splprep([closed[:,0], closed[:,1]], s=s_smooth, per=True, k=3) # ajuste spline periódico

    # === K) Reamostrar uniformemente
    u_dense = np.linspace(0, 1, 40000) # denso para cálculo de comprimento
    xd, yd = splev(u_dense, tck) # spline denso

    ds_dense = np.hypot(np.diff(xd), np.diff(yd))
    s_dense = np.insert(np.cumsum(ds_dense), 0, 0.0)
    L_px_fit = s_dense[-1]
    u_of_s = interp1d(s_dense / L_px_fit, u_dense, kind="linear", fill_value="extrapolate")

    esp_px = max(1e-6, 0.1 / (resolution * 1.2))
    n_samples = max(64, int(np.floor(L_px_fit / esp_px)))
    print("Length of fitted spline (px):", L_px_fit, "Number of samples:", n_samples)
    s_target_frac = np.linspace(0, 1, n_samples, endpoint=False)
    u_equi = u_of_s(s_target_frac)
    x_eq, y_eq = splev(u_equi, tck)
    centerline_points = np.column_stack([x_eq, y_eq])  # em pixels

    # === L) Forçar orientação
    def signed_area(poly):
        x, y = poly[:,0], poly[:,1]
        return 0.5*np.sum(x*np.roll(y,-1) - y*np.roll(x,-1))

    if direction == "counter-clockwise" and signed_area(centerline_points) < 0:
        centerline_points = centerline_points[::-1]
    elif direction == "clockwise" and signed_area(centerline_points) > 0:
        centerline_points = centerline_points[::-1]

    # === M) Checar auto-intersecção
    if not LineString(np.vstack([centerline_points, centerline_points[0]])).is_simple:
        print("⚠️ Aviso: centerline tem auto-interseção — aumenta s_smooth ou reforça limpeza.")

    return centerline_points


    # spline + reamostragem uniforme
    mids = np.vstack([mids, mids[0]])
    tck, _ = splprep([mids[:,0], mids[:,1]], s=10.0, per=True)
    n_points = int(len(mids) * (amostragem/resolution))
    u_new = np.linspace(0, 1, max(2000, n_points))
    x_new, y_new = splev(u_new, tck)
    return np.column_stack([x_new, y_new])

def smooth_centerline(centerline_px, origin, resolution, amostragem):
    
    centerline_px = gaussian_filter1d(centerline_px, sigma=10, axis=0, mode="wrap")
    
    x = origin[0] + centerline_px[:,0]*resolution
    y = origin[1] + centerline_px[:,1]*resolution

    ds = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    s = np.insert(np.cumsum(ds), 0, 0)
    spline_x = CubicSpline(s, x)
    spline_y = CubicSpline(s, y)

    s_uniform = np.arange(0, s[-1], amostragem)
    x_s = spline_x(s_uniform)
    y_s = spline_y(s_uniform)

    dx = np.gradient(x_s, s_uniform)
    dy = np.gradient(y_s, s_uniform)
    ddx = np.gradient(dx, s_uniform)
    ddy = np.gradient(dy, s_uniform)
    kappa = (dx*ddy - dy*ddx) / (dx**2 + dy**2)**1.5
    
    centerline_px = np.column_stack([(x_s - origin[0])/resolution,
                                      (y_s - origin[1])/resolution])

    return s_uniform, x_s, y_s, kappa, centerline_px

##########################################
# MÉTODO A - INTERSEÇÃO DAS NORMAIS
##########################################
def bounds_by_normals(centerline, contour_left, contour_right, psi):
    contour_line_left = LineString(contour_left)
    contour_line_right = LineString(contour_right)
    # KD-trees para fallback robusto por lado
    tree_left = cKDTree(contour_left)
    tree_right = cKDTree(contour_right)

    # Comprimentos para varrer: base (em função da resolução) e máximo (em função da extensão do mapa)
    all_pts = np.vstack([contour_left, contour_right])
    max_extent = float(max(all_pts[:, 0].ptp(), all_pts[:, 1].ptp()))
    base_len = 10 / resolution
    max_len = max(base_len, 1.5 * max_extent)

    def nearest_dist(intersection, pt):
        if intersection.is_empty:
            return None
        if intersection.geom_type == 'Point':
            return float(np.hypot(intersection.x - pt[0], intersection.y - pt[1]))
        if intersection.geom_type == 'MultiPoint':
            return float(min(np.hypot(p.x - pt[0], p.y - pt[1]) for p in intersection.geoms))
        return None

    def rotate(vec, angle):
        c, s = np.cos(angle), np.sin(angle)
        return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])

    def ray_intersection_distance(contour_line, pt, direction, length):
        line = LineString([pt, pt + length * direction])
        return nearest_dist(contour_line.intersection(line), pt)

    def fan_search_projected(contour_line, pt, base_dir, base_length, fan_rad=np.deg2rad(85), steps=14, max_length=None):
        # Tenta 0°, ±ang até fan_rad com comprimentos crescentes; projeta o resultado no eixo da base (cos(ang))
        lengths = [base_length]
        if max_length is not None and max_length > base_length:
            mid_len = 0.5 * (base_length + max_length)
            lengths += [mid_len, max_length]
        for L in lengths:
            # primeiro a direção base
            d0 = ray_intersection_distance(contour_line, pt, base_dir, L)
            if d0 is not None:
                return d0
            for k in range(1, steps + 1):
                ang = fan_rad * k / steps
                for sign in (1, -1):
                    dir_rot = rotate(base_dir, sign * ang)
                    d = ray_intersection_distance(contour_line, pt, dir_rot, L)
                    if d is not None:
                        # projetar no eixo da normal original
                        return d * np.cos(ang)
        return None

    def kd_projected_distance(tree, contour_arr, pt, base_dir, k=8):
        # Procura o vizinho cuja projeção na base_dir seja positiva e mínima
        k = int(min(k, len(contour_arr)))
        dists, idxs = tree.query(pt, k=k)
        idxs = np.atleast_1d(idxs)
        best = None
        for idx in idxs:
            v = contour_arr[idx] - pt
            proj = float(np.dot(v, base_dir))
            if proj > 0:
                if best is None or proj < best:
                    best = proj
        if best is not None:
            return best
        # Caso extremo: não achou projeção positiva; usa a projeção absoluta do mais próximo
        first_idx = int(idxs[0])
        v = contour_arr[first_idx] - pt
        return abs(float(np.dot(v, base_dir)))

    n_l, n_r = [], []
    left_used_kd, right_used_kd = [], []
    debug_plots = _plot_enabled(PLOT_DEBUG)

    for i, pt in enumerate(centerline):
        pt = np.array(pt)
        normal_left = np.array([np.cos(psi[i] + np.pi / 2), np.sin(psi[i] + np.pi / 2)])
        normal_right = -normal_left

        # 1) tenta intersecção na normal com varredura angular e comprimento adaptativo
        d_left = fan_search_projected(contour_line_left, pt, normal_left, base_len, max_length=max_len)
        d_right = fan_search_projected(contour_line_right, pt, normal_right, base_len, max_length=max_len)

        # 2) fallback KDTree por lado (projeção na normal)
        used_kd_left = False
        used_kd_right = False
        if d_left is None:
            d_left = kd_projected_distance(tree_left, contour_left, pt, normal_left)
            used_kd_left = True
        if d_right is None:
            d_right = kd_projected_distance(tree_right, contour_right, pt, normal_right)
            used_kd_right = True

        if debug_plots and (d_left is None or d_right is None):
            line_len_dbg = base_len
            plt.figure()
            plt.plot(contour_left[:, 0], contour_left[:, 1], "r-", label="Contorno Esquerdo")
            plt.plot(contour_right[:, 0], contour_right[:, 1], "b-", label="Contorno Direito")
            plt.plot([pt[0], pt[0] + line_len_dbg * normal_left[0]],
                     [pt[1], pt[1] + line_len_dbg * normal_left[1]], "g--", label="Normal Esquerda")
            plt.plot([pt[0], pt[0] + line_len_dbg * normal_right[0]],
                     [pt[1], pt[1] + line_len_dbg * normal_right[1]], "y--", label="Normal Direita")
            plt.scatter(pt[0], pt[1], c="k", label="Ponto Centerline")
            plt.legend()
            plt.title("Interseção ausente — usando fallback")
            plt.axis("equal")
            plt.show()

        n_l.append(d_left if d_left is not None else np.nan)
        n_r.append(d_right if d_right is not None else np.nan)
        left_used_kd.append(used_kd_left)
        right_used_kd.append(used_kd_right)

    # Preencher eventuais NaNs por interpolação linear circular (evita o "usar anterior")
    def _fill_circular_linear(values):
        arr = np.asarray(values, dtype=float)
        N = arr.size
        if N == 0:
            return arr
        mask = ~np.isnan(arr)
        if mask.all():
            return arr
        if not np.any(mask):
            return np.zeros_like(arr)
        idx = np.arange(N)
        x = np.r_[idx[mask], idx[mask][0] + N]
        y = np.r_[arr[mask], arr[mask][0]]
        arr[~mask] = np.interp(idx[~mask], x, y)
        return arr

    n_l_raw = np.array(n_l, dtype=float)
    n_r_raw = np.array(n_r, dtype=float)
    n_l = n_l_raw.copy()
    n_r = n_r_raw.copy()
    left_used_kd = np.array(left_used_kd, dtype=bool)
    right_used_kd = np.array(right_used_kd, dtype=bool)

    # ignora segmentos que precisaram do fallback KDTree; interpola a partir de vizinhos válidos
    n_l[left_used_kd] = np.nan
    n_r[right_used_kd] = np.nan

    n_l = _fill_circular_linear(n_l)
    n_r = _fill_circular_linear(n_r)

    # Em casos extremos onde a interpolação falha (tudo NaN), reutiliza o valor bruto calculado
    nan_l = np.isnan(n_l)
    nan_r = np.isnan(n_r)
    if nan_l.any():
        n_l[nan_l] = n_l_raw[nan_l]
    if nan_r.any():
        n_r[nan_r] = n_r_raw[nan_r]

    # suavização leve para garantir curvas mais limpas
    smooth_sigma = 2.0
    if smooth_sigma > 0:
        n_l = gaussian_filter1d(n_l, sigma=smooth_sigma, mode="wrap")
        n_r = gaussian_filter1d(n_r, sigma=smooth_sigma, mode="wrap")

    return n_l, n_r

##########################################
# MÉTODO B - KDTree
##########################################
def bounds_by_kdtree(centerline, contour_left, contour_right, psi):
    tree_left = cKDTree(contour_left)
    tree_right = cKDTree(contour_right)
    n_l, n_r = [], []
    for pt in centerline:
        d_left, _ = tree_left.query(pt)
        d_right, _ = tree_right.query(pt)
        n_l.append(d_left)
        n_r.append(d_right)
    # suavizar
    n_l = gaussian_filter1d(n_l, sigma=10, mode="wrap")
    n_r = gaussian_filter1d(n_r, sigma=10, mode="wrap")
    return np.array(n_l), np.array(n_r)

##########################################
# MAIN
##########################################
map_img, binary, resolution, origin = load_map(map_path, yaml_path)

# Separate gray, black, and white regions of the map
gray_mask = (map_img > 120) & (map_img < 150)  # Gray region
black_mask = map_img <= 120  # Black region
white_mask = map_img >= 150  # White region

# Visualize the separated regions
if _plot_enabled(PLOT_DEBUG):
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 3, 1)
    plt.imshow(gray_mask, cmap="gray", origin="lower")
    plt.title("Gray Region")
    plt.axis("off")

    plt.subplot(1, 3, 2)
    plt.imshow(black_mask, cmap="gray", origin="lower")
    plt.title("Black Region")
    plt.axis("off")

    plt.subplot(1, 3, 3)
    plt.imshow(white_mask, cmap="gray", origin="lower")
    plt.title("White Region")
    plt.axis("off")

    plt.tight_layout()
    plt.show()

# Build an RGB image labeling each region distinctly (plot with meter scale)
h, w = map_img.shape
rgb = np.zeros((h, w, 3), dtype=np.uint8)

# Assign colors
rgb[black_mask] = (0, 0, 0)          # black region
rgb[gray_mask]  = (160, 160, 160)    # gray region
rgb[white_mask] = (255, 255, 255)    # white region

# Extent in world (meters): [xmin, xmax, ymin, ymax]
extent = [origin[0], origin[0] + w * resolution,
          origin[1], origin[1] + h * resolution]

if _plot_enabled(PLOT_MAIN):
    plt.figure(figsize=(8, 6))
    plt.imshow(rgb, origin="lower", extent=extent, aspect='equal')
    plt.plot(0, 0, "ro", label="Origin")
    plt.title("Mapa (escala em metros): regiões preta, cinza e branca")
    plt.grid(alpha=0.3)
    # Make the image fill the axes exactly, keeping ticks visible
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.axis("on")  # ensure axes stay visible
    plt.gca().set_aspect('equal', adjustable='box')  # evita faixas brancas mantendo proporção
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

if _plot_enabled(PLOT_DEBUG):
    plt.figure(figsize=(8, 6))
    plt.imshow(binary, cmap="gray", origin="lower", extent=extent, aspect='equal')
    plt.plot(0, 0, "ro", label="Origem")
    # Make the image fill the axes exactly, keeping ticks visible
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.axis("on")  # ensure axes stay visiblE
    plt.legend()
    plt.title("Mapa carregado (PGM)")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

if _plot_enabled(PLOT_DEBUG):
    plt.figure()
    ax = plt.gca()
    ax.imshow(map_img, cmap="gray", origin="lower", extent=extent)
    ax.plot(0, 0, "ro", label="Origem")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect('equal', adjustable='box')  # evita faixas brancas mantendo proporção
    ax.legend()
    ax.set_title("Mapa carregado (PGM1)")
    plt.tight_layout()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

contour1, contour2 = extract_contours(binary, direction)

contour1_r = contour1 * resolution + origin
contour2_r = contour2 * resolution + origin

if _plot_enabled(PLOT_MAIN):
    plt.figure(figsize=(8, 6))
    plt.imshow(rgb, cmap="gray", origin="lower",extent=extent, aspect='equal')
    plt.scatter(contour1_r[:,0], contour1_r[:,1], s=2, c="r", label="Outside edge")
    plt.scatter(contour2_r[:,0], contour2_r[:,1], s=2, c="b", label="Inside edge")
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.gca().set_aspect('equal', adjustable='box')  # evita faixas brancas mantendo proporção
    plt.legend()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Contornos extraídos")
    plt.show()

centerline_px = build_centerline(contour1, contour2, amostragem, resolution, direction, origin)
#s, x_s, y_s, kappa, centerline_px = smooth_centerline(centerline_px, origin, resolution, amostragem)

# Rebuild centerline so that it starts at the map origin (in pixels)
origin_px = np.array([-origin[0]/resolution, -origin[1]/resolution])
dists_to_origin = np.linalg.norm(centerline_px - origin_px, axis=1)
start_idx = np.argmin(dists_to_origin)
centerline_px = np.roll(centerline_px, -start_idx, axis=0)
s, x_s, y_s, kappa, centerline_px = smooth_centerline(centerline_px, origin, resolution, amostragem)

centerline= centerline_px * resolution + origin

if _plot_enabled(PLOT_MAIN):
    plt.figure(figsize=(8, 6))
    plt.imshow(rgb, cmap="gray", origin="lower", extent=extent, aspect='equal')
    plt.plot(centerline[:,0], centerline[:,1], "g", label="Centerline")
    plt.plot(contour1_r[:,0], contour1_r[:,1], "r", label="Outside edge")
    plt.plot(contour2_r[:,0], contour2_r[:,1], "b", label="Inside edge")
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')  # evita faixas brancas mantendo proporção
    plt.title("Centerline inicial (px)")
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

if _plot_enabled(PLOT_MAIN):
    plt.figure()
    plt.plot(s, kappa)
    plt.xlabel("s [m]"); plt.ylabel("κ [1/m]")
    plt.title("Curvatura ao longo da trajetória")
    plt.grid(True)
    plt.show()

# Curvatura no plano XY (linha colorida por κ)

# Segmentar a linha
pts = np.column_stack((x_s, y_s))
segments = np.stack([pts, np.roll(pts, -1, axis=0)], axis=1)

if _plot_enabled(PLOT_DEBUG):
    fig, ax = plt.subplots()
    norm = plt.Normalize(kappa.min(), kappa.max())
    lc = LineCollection(segments, cmap='coolwarm', norm=norm)
    lc.set_array(kappa)
    lc.set_linewidth(2.0)
    ax.add_collection(lc)
    ax.plot([], [], 'k-', alpha=0.3, label='centerline')  # legenda dummy
    ax.imshow(rgb, cmap="gray", origin="lower", extent=extent, aspect='equal')

    ax.set_aspect('equal', 'box')
    ax.set_xlim(x_s.min()-0.5, x_s.max()+0.5)
    ax.set_ylim(y_s.min()-0.5, y_s.max()+0.5)
    fig.colorbar(lc, ax=ax, label='Curvatura κ [1/m]')
    ax.set_title('Curvatura no plano XY')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.show()

# (Opcional) versão simples com scatter
if _plot_enabled(PLOT_DEBUG):
    plt.figure(figsize=(8, 6))
    sc = plt.scatter(x_s, y_s, c=kappa, cmap='coolwarm', s=8)
    plt.plot(x_s, y_s, 'k-', alpha=0.3, linewidth=0.5)
    plt.imshow(rgb, cmap="gray", origin="lower", extent=extent, aspect='equal')
    plt.gca().set_aspect('equal', 'box')
    plt.colorbar(sc, label='Curvature κ [1/m]')
    plt.title('Curvature (scatter) in XY plane')
    plt.xlabel('x [m]'); plt.ylabel('y [m]')
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.show()


psi = np.arctan2(np.gradient(centerline_px[:,1]), np.gradient(centerline_px[:,0]))


# margens
centerline_real = np.column_stack([x_s, y_s])
contour1_real = np.column_stack([origin[0]+contour1[:,0]*resolution,
                                 origin[1]+contour1[:,1]*resolution])
contour2_real = np.column_stack([origin[0]+contour2[:,0]*resolution,
                                 origin[1]+contour2[:,1]*resolution])

if direction == "counter-clockwise":
    contour1, contour2 = contour2, contour1
    contour1_real, contour2_real = contour2_real, contour1_real
else:
    pass

n_l_A, n_r_A = bounds_by_normals(centerline_px, contour1, contour2, psi)
n_l_B, n_r_B = bounds_by_kdtree(centerline_px, contour1, contour2, psi)

n_l_A_real = n_l_A * resolution
n_r_A_real = n_r_A * resolution
n_l_B_real = n_l_B * resolution
n_r_B_real = n_r_B * resolution

print("shape s:", s.shape, "x_s:", x_s.shape, "y_s:", y_s.shape, "kappa:", kappa.shape)
print("shape centerline_px:", centerline_px.shape)
print("shape n_l_A:", n_l_A.shape, "n_r_A:", n_r_A.shape)
print("shape n_l_B:", n_l_B.shape, "n_r_B:", n_r_B.shape)

if _plot_enabled(PLOT_MAIN):
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(s, n_l_A_real, "r-", label="n_l (normais)")
    axs[0].plot(s, n_r_A_real, "b-", label="n_r (normais)")
    axs[0].set_ylabel("Distância [m]")
    axs[0].set_title("Margens ao longo da trajetória (Normais)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(s, n_l_B_real, "r--", label="n_l (kdtree)")
    axs[1].plot(s, n_r_B_real, "b--", label="n_r (kdtree)")
    axs[1].set_xlabel("s [m]")
    axs[1].set_ylabel("Distância [m]")
    axs[1].set_title("Margens ao longo da trajetória (KDTree)")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

if _plot_enabled(PLOT_DEBUG):
    plt.figure()
    plt.imshow(rgb, cmap="gray", origin="lower")
    plt.plot(centerline_px[:,0], centerline_px[:,1], "k", label="centerline")
    plt.plot(centerline_px[:,0] + n_l_A*np.cos(psi+np.pi/2),
             centerline_px[:,1] + n_l_A*np.sin(psi+np.pi/2), "r.", ms=2, label="left A")
    plt.plot(centerline_px[:,0] + n_r_A*np.cos(psi-np.pi/2),
             centerline_px[:,1] + n_r_A*np.sin(psi-np.pi/2), "b.", ms=2, label="right A")
    plt.legend()
    plt.axis("equal")
    plt.title("Margens reconstruídas px (método A)")
    plt.show()

# reconstruir margens em 2D
x_l_A = x_s + n_l_A_real*np.cos(psi+np.pi/2)
y_l_A = y_s + n_l_A_real*np.sin(psi+np.pi/2)
x_r_A = x_s + n_r_A_real*np.cos(psi-np.pi/2)
y_r_A = y_s + n_r_A_real*np.sin(psi-np.pi/2)

if _plot_enabled(PLOT_MAIN):
    plt.figure()
    plt.imshow(rgb, cmap="gray", origin="lower", extent=extent, aspect='equal')
    #plt.scatter(x_s, y_s, c="k", s=2, label="Centerline")
    plt.scatter(x_l_A, y_l_A, c="r", s=5, label="Left Boundary")
    plt.scatter(x_r_A, y_r_A, c="b", s=5, label="Right Boundary")
    sc = plt.scatter(x_s, y_s, c=kappa, cmap='turbo', s=10, label="Centerline")
    plt.legend()
    plt.colorbar(sc, label='Curvature κ [1/m]')
    plt.gca().set_aspect('equal', 'box')
    plt.title("Margens reconstruídas e Curvatura (método A)")
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.show()

# método B

x_l_B = x_s + n_l_B_real*np.cos(psi+np.pi/2)
y_l_B = y_s + n_l_B_real*np.sin(psi+np.pi/2)
x_r_B = x_s + n_r_B_real*np.cos(psi-np.pi/2)
y_r_B = y_s + n_r_B_real*np.sin(psi-np.pi/2)

if _plot_enabled(PLOT_DEBUG):
    plt.figure()
    plt.plot(x_s, y_s, "k", label="centerline")
    plt.plot(x_l_B, y_l_B, "r.", ms=2, label="left B")
    plt.plot(x_r_B, y_r_B, "b.", ms=2, label="right B")
    plt.legend()
    plt.axis("equal")
    plt.title("Margens reconstruídas (método B)")
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.show()

if _plot_enabled(PLOT_MAIN):
    plt.figure(figsize=(10, 6))
    plt.imshow(rgb, cmap="gray", origin="lower", extent=extent, aspect='equal')
    plt.plot(x_l_B , y_l_B , "r.", ms=2, label="left B")
    plt.plot(x_r_B , y_r_B , "b.", ms=2, label="right B")
    sc = plt.scatter(x_s, y_s, c=kappa, cmap='turbo', s=10, label="Centerline")
    plt.legend()
    plt.colorbar(sc, label='Curvature κ [1/m]')
    plt.gca().set_aspect('equal', 'box')
    plt.title("Centerline + margens (método B)")
    plt.xlim(extent[0], extent[1])
    plt.ylim(extent[2], extent[3])
    plt.show()


# export
df = pd.DataFrame({
    "s": np.round(s,3),
    "x": np.round(x_s,3),
    "y": np.round(y_s,3),
    "kappa": np.round(kappa,3),
    "n_l": np.round(n_l_A_real,3),
    "n_r": np.round(n_r_A_real,3),
})

filename = f"centerline_{amostragem:.2f}_" + os.path.basename(os.path.dirname(map_path)) + ".csv"
csv_file = os.path.join(traj_dir, filename)
df.to_csv(csv_file, index=False)
print(f"CSV exportado: {csv_file}")


df2 = pd.DataFrame({
    "s": np.round(s,3),
    "x": np.round(x_s,3),
    "y": np.round(y_s,3),
    "kappa": np.round(kappa,3),
    "n_l": np.round(n_l_B_real,3),
    "n_r": np.round(n_r_B_real,3),
})

filename2 = f"centerline_{amostragem:.2f}_" + os.path.basename(os.path.dirname(map_path)) + "_v2.csv"
csv_file2 = os.path.join(traj_dir, filename2)
df2.to_csv(csv_file2, index=False)
print(f"CSV exportado: {csv_file2}")
