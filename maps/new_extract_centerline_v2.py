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
map_files = "../maps/test_map/map_output"
map_path = map_files + ".pgm"
yaml_path = map_files + ".yaml"
direction = "clockwise"  # ou "clockwise"
amostragem = 0.1  # espaçamento em metros

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
    binary = (map_img > 220).astype(np.uint8) * 255
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

def build_centerline(contour1, contour2, amostragem, resolution, direction="counter-clockwise"):
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

    # === A) Emparelhar contorno1->contorno2
    centerline_points1 = []
    for p1 in contour1:
        dists = np.linalg.norm(contour2 - p1, axis=1)
        p2 = contour2[np.argmin(dists)]
        centerline_points1.append((p1 + p2) / 2.0)

    # === B) Emparelhar contorno2->contorno1
    centerline_points2 = []
    for p1 in contour2:
        dists = np.linalg.norm(contour1 - p1, axis=1)
        p2 = contour1[np.argmin(dists)]
        centerline_points2.append((p1 + p2) / 2.0)

    # === C) Fundir e eliminar quase-duplicados
    cl_raw = np.vstack([centerline_points1, centerline_points2])
    cl_raw = np.unique(np.round(cl_raw, 1), axis=0)  # arredonda a 1 casa decimal

    # === D) Construir curva-guia (usando a primeira centerline)
    guide = LineString(np.vstack([centerline_points1, centerline_points1[0]]))
    L_px = guide.length

    # === E) Ordenar por abscissa curvilínea
    s_vals = np.array([guide.project(Point(p[0], p[1])) for p in cl_raw])
    ord_idx = np.argsort(s_vals)
    ordered = cl_raw[ord_idx]
    s_sorted = s_vals[ord_idx]

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
    if len(ordered) < 7:
        x_sg, y_sg = ordered[:,0], ordered[:,1]
    else:
        win = max(7, (len(ordered)//180)*2 + 1)   # ímpar
        win = min(win, len(ordered) - (1 - len(ordered)%2))
        x_sg = savgol_filter(ordered[:,0], window_length=win, polyorder=3, mode='wrap')
        y_sg = savgol_filter(ordered[:,1], window_length=win, polyorder=3, mode='wrap')
    ordered = np.column_stack([x_sg, y_sg])

    # === J) Fit spline periódico
    closed = np.vstack([ordered, ordered[0]])
    s_smooth = (0.5**2) * len(closed)   # suavização
    tck, u = splprep([closed[:,0], closed[:,1]], s=s_smooth, per=True, k=3)

    # === K) Reamostrar uniformemente
    u_dense = np.linspace(0, 1, 4000)
    xd, yd = splev(u_dense, tck)
    ds_dense = np.hypot(np.diff(xd), np.diff(yd))
    s_dense = np.insert(np.cumsum(ds_dense), 0, 0.0)
    L_px_fit = s_dense[-1]
    u_of_s = interp1d(s_dense / L_px_fit, u_dense, kind="linear", fill_value="extrapolate")

    esp_px = max(1e-6, amostragem / resolution)
    n_samples = max(32, int(np.floor(L_px_fit / esp_px)))
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
    
    centerline_px = gaussian_filter1d(centerline_px, sigma=15, axis=0, mode="wrap")
    
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
    n_l, n_r = [], []
    for i, pt in enumerate(centerline):
        pt = np.array(pt)
        normal_left = np.array([np.cos(psi[i] + np.pi/2), np.sin(psi[i] + np.pi/2)])
        normal_right = -normal_left

        line_len = 10/resolution
        line_left = LineString([pt, pt + line_len*normal_left])
        line_right = LineString([pt, pt + line_len*normal_right])

        def nearest_dist(intersection, pt):
            if intersection.is_empty: return None
            if intersection.geom_type == 'Point':
                return np.linalg.norm([intersection.x - pt[0], intersection.y - pt[1]])
            elif intersection.geom_type == 'MultiPoint':
                return min(np.linalg.norm([p.x-pt[0], p.y-pt[1]]) for p in intersection.geoms)
            return None

        d_left = nearest_dist(contour_line_left.intersection(line_left), pt)
        d_right = nearest_dist(contour_line_right.intersection(line_right), pt)
        debug = False
        
        if d_left is None or d_right is None:
            if debug:  # debug
                plt.figure()
                plt.plot(contour_left[:, 0], contour_left[:, 1], "r-", label="Contorno Esquerdo")
                plt.plot(contour_right[:, 0], contour_right[:, 1], "b-", label="Contorno Direito")
                plt.plot([pt[0], pt[0] + line_len * normal_left[0]], 
                    [pt[1], pt[1] + line_len * normal_left[1]], "g--", label="Normal Esquerda")
                plt.plot([pt[0], pt[0] + line_len * normal_right[0]], 
                    [pt[1], pt[1] + line_len * normal_right[1]], "y--", label="Normal Direita")
                plt.scatter(pt[0], pt[1], c="k", label="Ponto Centerline")
                plt.legend()
                plt.title("Interseção ausente para d_left ou d_right")
                plt.axis("equal")
                plt.show()
            else:
                print("⚠️ Aviso: interseção ausente para d_left ou d_right — usando último valor válido no ponto", i)
        if i==300:
            plt.figure()
            plt.plot(contour_left[:, 0], contour_left[:, 1], "r-", label="Contorno Esquerdo")
            plt.plot(contour_right[:, 0], contour_right[:, 1], "b-", label="Contorno Direito")
            plt.plot([pt[0], pt[0] + line_len * normal_left[0]], 
                [pt[1], pt[1] + line_len * normal_left[1]], "g--", label="Normal Esquerda")
            plt.plot([pt[0], pt[0] + line_len * normal_right[0]], 
                [pt[1], pt[1] + line_len * normal_right[1]], "y--", label="Normal Direita")
            if d_left is not None:
                plt.scatter(pt[0] + d_left * normal_left[0], pt[1] + d_left * normal_left[1], c="g", label="Interseção Esquerda")
                plt.scatter(pt[0] + d_left * np.cos(psi[i]+np.pi/2), pt[1] + d_left * np.sin(psi[i]+np.pi/2), c="m", label="Interseção Esquerda (usando psi)")
            if d_right is not None:
                plt.scatter(pt[0] + d_right * np.cos(psi[i]-np.pi/2), pt[1] + d_right * np.sin(psi[i]-np.pi/2), c="c", label="Interseção Direita (usando psi)")
            plt.scatter(pt[0], pt[1], c="k", label="Ponto Centerline")
            plt.scatter(pt[0], pt[1], c="k", label="Ponto Centerline")
            plt.legend()
            plt.title("Interseção ausente para d_left ou d_right")
            plt.axis("equal")
            plt.show()
            
        n_l.append(d_left if d_left else n_l[-1] if n_l else 0)
        n_r.append(d_right if d_right else n_r[-1] if n_r else 0)

    return np.array(n_l), np.array(n_r)

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

plt.figure(figsize=(8, 6))
plt.imshow(map_img, cmap="gray", origin="lower")
plt.plot(-origin[0]/resolution, -origin[1]/resolution, "ro", label="Origem")
plt.legend()
plt.title("Mapa carregado (PGM)")
plt.axis("equal")
plt.show()

contour1, contour2 = extract_contours(binary, direction)
plt.figure()
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(contour1[:,0], contour1[:,1], "r", label="Margem 1")
plt.plot(contour2[:,0], contour2[:,1], "b", label="Margem 2")
plt.legend(); plt.axis("equal"); plt.title("Contornos extraídos")
plt.show()

centerline_px = build_centerline(contour1, contour2, amostragem, resolution, direction)
s, x_s, y_s, kappa, centerline_px = smooth_centerline(centerline_px, origin, resolution, amostragem)

plt.figure()
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(centerline_px[:,0], centerline_px[:,1], "g", label="Centerline")
plt.plot(contour1[:,0], contour1[:,1], "r", label="Margem 1")
plt.plot(contour2[:,0], contour2[:,1], "b", label="Margem 2")
plt.legend(); plt.axis("equal"); plt.title("Centerline inicial (px)")
plt.show()

plt.figure()
plt.plot(s, kappa)
plt.xlabel("s [m]"); plt.ylabel("κ [1/m]")
plt.title("Curvatura ao longo da trajetória")
plt.grid(True); plt.show()


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

plt.figure()
plt.plot(s, n_l_A_real, "r-", label="n_l (normais)")
plt.plot(s, n_r_A_real, "b-", label="n_r (normais)")
plt.plot(s, n_l_B_real, "r--", label="n_l (kdtree)")
plt.plot(s, n_r_B_real, "b--", label="n_r (kdtree)")
plt.xlabel("s [m]"); plt.ylabel("Distância [m]")
plt.title("Margens ao longo da trajetória")
plt.legend(); plt.grid(True); plt.show()

plt.figure()
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(centerline_px[:,0], centerline_px[:,1], "k", label="centerline")
plt.plot(centerline_px[:,0] + n_l_A*np.cos(psi+np.pi/2),
         centerline_px[:,1] + n_l_A*np.sin(psi+np.pi/2), "r.", ms=2, label="left A")
plt.plot(centerline_px[:,0] + n_r_A*np.cos(psi-np.pi/2),
         centerline_px[:,1] + n_r_A*np.sin(psi-np.pi/2), "b.", ms=2, label="right A")
plt.legend(); plt.axis("equal"); plt.title("Margens reconstruídas px (método A)")
plt.show()

# reconstruir margens em 2D
x_l_A = x_s + n_l_A_real*np.cos(psi+np.pi/2)
y_l_A = y_s + n_l_A_real*np.sin(psi+np.pi/2)
x_r_A = x_s + n_r_A_real*np.cos(psi-np.pi/2)
y_r_A = y_s + n_r_A_real*np.sin(psi-np.pi/2)

plt.figure()
plt.plot(x_s, y_s, "k", label="centerline")
plt.plot(x_l_A, y_l_A, "r.", ms=2, label="left A")
plt.plot(x_r_A, y_r_A, "b.", ms=2, label="right A")
plt.legend(); plt.axis("equal"); plt.title("Margens reconstruídas (método A)")
plt.show()

# método B

x_l_B = x_s + n_l_B_real*np.cos(psi+np.pi/2)
y_l_B = y_s + n_l_B_real*np.sin(psi+np.pi/2)
x_r_B = x_s + n_r_B_real*np.cos(psi-np.pi/2)
y_r_B = y_s + n_r_B_real*np.sin(psi-np.pi/2)

plt.figure()
plt.plot(x_s, y_s, "k", label="centerline")
plt.plot(x_l_B, y_l_B, "r.", ms=2, label="left B")
plt.plot(x_r_B, y_r_B, "b.", ms=2, label="right B")
plt.legend(); plt.axis("equal"); plt.title("Margens reconstruídas (método B)")
plt.show()

plt.figure(figsize=(10, 6))
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(centerline_px[:,0], centerline_px[:,1], "g", label="Centerline")
plt.plot((x_l_B - origin[0])/resolution, (y_l_B - origin[1])/resolution, "r.", ms=2, label="left B")
plt.plot((x_r_B - origin[0])/resolution, (y_r_B - origin[1])/resolution, "b.", ms=2, label="right B")
plt.legend(); plt.axis("equal"); plt.title("Centerline + margens (método B)")
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

filename = "centerline_" + os.path.basename(os.path.dirname(map_path)) + ".csv"
df.to_csv(filename, index=False)
print(f"CSV exportado: {filename}")


df2 = pd.DataFrame({
    "s": np.round(s,3),
    "x": np.round(x_s,3),
    "y": np.round(y_s,3),
    "kappa": np.round(kappa,3),
    "n_l": np.round(n_l_B_real,3),
    "n_r": np.round(n_r_B_real,3),
})

filename2 = "centerline_" + os.path.basename(os.path.dirname(map_path)) + "_v2.csv"
df2.to_csv(filename2, index=False)
print(f"CSV exportado: {filename2}")
