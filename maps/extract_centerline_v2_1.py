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
# CONFIGURATION
##########################################
map_files = "maps/test_map/map_output"
traj_dir = "traj"
map_path = map_files + ".pgm"
yaml_path =  map_files + ".yaml"
direction = "clockwise"  # or "counter-clockwise"
amostragem = 0.1  # spacing in meters

##########################################
# UTILITIES
##########################################
def load_map(map_path, yaml_path):
    map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(f"Could not read: {map_path}")

    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    try:
        resolution = yaml_data['resolution']
        origin = yaml_data['origin'][:2]
    except KeyError as e:
        raise KeyError(f"Missing key in YAML file: {e}")

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

from scipy.interpolate import splprep, splev, interp1d
from shapely.geometry import Point

def build_centerline(contour1, contour2, amostragem, resolution, direction="counter-clockwise"):
    """
    Builds a smooth centerline from two contours (in pixels).
    """
    # Pair contour1 -> contour2
    centerline_points1 = []
    for p1 in contour1:
        dists = np.linalg.norm(contour2 - p1, axis=1)
        p2 = contour2[np.argmin(dists)]
        centerline_points1.append((p1 + p2) / 2.0)

    # Pair contour2 -> contour1
    centerline_points2 = []
    for p1 in contour2:
        dists = np.linalg.norm(contour1 - p1, axis=1)
        p2 = contour1[np.argmin(dists)]
        centerline_points2.append((p1 + p2) / 2.0)

    # Merge and remove duplicates
    cl_raw = np.vstack([centerline_points1, centerline_points2])
    cl_raw = np.unique(np.round(cl_raw, 1), axis=0)

    # Build guiding curve
    guide = LineString(np.vstack([centerline_points1, centerline_points1[0]]))
    L_px = guide.length

    # Order by curvilinear abscissa
    s_vals = np.array([guide.project(Point(p[0], p[1])) for p in cl_raw])
    ord_idx = np.argsort(s_vals)
    ordered = cl_raw[ord_idx]
    s_sorted = s_vals[ord_idx]

    # Cut at the largest gap
    gaps = np.diff(np.r_[s_sorted, s_sorted[0] + L_px])
    cut = np.argmax(gaps)
    ordered = np.roll(ordered, -(cut + 1), axis=0)

    # Deduplicate by radius
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

    # Remove large jumps
    if len(ordered) >= 3:
        step = np.hypot(np.diff(np.r_[ordered[:,0], ordered[0,0]]),
                        np.diff(np.r_[ordered[:,1], ordered[0,1]]))
        med = np.median(step)
        if med > 0:
            mask = step < 3.0*med
            keep = np.ones(len(ordered), dtype=bool)
            keep[np.where(~mask)[0]] = False
            ordered = ordered[keep]

    # Light smoothing before spline fitting
    if len(ordered) < 7:
        x_sg, y_sg = ordered[:,0], ordered[:,1]
    else:
        win = max(7, (len(ordered)//180)*2 + 1)
        win = min(win, len(ordered) - (1 - len(ordered)%2))
        x_sg = savgol_filter(ordered[:,0], window_length=win, polyorder=3, mode='wrap')
        y_sg = savgol_filter(ordered[:,1], window_length=win, polyorder=3, mode='wrap')
    ordered = np.column_stack([x_sg, y_sg])

    # Periodic spline fit
    closed = np.vstack([ordered, ordered[0]])
    s_smooth = (0.5**2) * len(closed)
    tck, u = splprep([closed[:,0], closed[:,1]], s=s_smooth, per=True, k=3)

    # Uniform resampling
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
    centerline_points = np.column_stack([x_eq, y_eq])  # in pixels

    # Enforce orientation
    def signed_area(poly):
        x, y = poly[:,0], poly[:,1]
        return 0.5*np.sum(x*np.roll(y,-1) - y*np.roll(x,-1))

    if direction == "counter-clockwise" and signed_area(centerline_points) < 0:
        centerline_points = centerline_points[::-1]
    elif direction == "clockwise" and signed_area(centerline_points) > 0:
        centerline_points = centerline_points[::-1]

    # Check for self-intersections
    if not LineString(np.vstack([centerline_points, centerline_points[0]])).is_simple:
        print("⚠️ Warning: centerline has self-intersection — increase smoothing or clean contours.")

    return centerline_points

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
def bounds_by_normals(centerline, contour_left, contour_right, psi, debug=True):
    contour_line_left = LineString(contour_left)
    contour_line_right = LineString(contour_right)
    n_l, n_r = [], []

    if debug:
        plt.figure()
        plt.imshow(map_img, cmap="gray", origin="lower")
        plt.plot(contour_left[:, 0], contour_left[:, 1], "r-", label="Contour Left")
        plt.plot(contour_right[:, 0], contour_right[:, 1], "b-", label="Contour Right")
        plt.plot(centerline[:, 0], centerline[:, 1], "g-", label="Centerline")
    for i, pt in enumerate(centerline):
        pt = np.array(pt)
        normal_left = np.array([np.cos(psi[i] + np.pi/2), np.sin(psi[i] + np.pi/2)])
        normal_right = -normal_left

        line_len = 3/resolution
        line_left = LineString([pt, pt + line_len*normal_left])
        line_right = LineString([pt, pt + line_len*normal_right])

        def nearest_dist(intersection, pt):
            if intersection.is_empty: 
                return None
            if intersection.geom_type == 'Point':
                return np.linalg.norm([intersection.x - pt[0], intersection.y - pt[1]])
            elif intersection.geom_type == 'MultiPoint':
                return min(np.linalg.norm([p.x-pt[0], p.y-pt[1]]) for p in intersection.geoms)
            return None

        d_left = nearest_dist(contour_line_left.intersection(line_left), pt)
        d_right = nearest_dist(contour_line_right.intersection(line_right), pt)

        if d_left is None or d_right is None:
            print(f"⚠️ Warning: missing intersection at point {i}, using last valid value.")

        # store distances (fallback to last valid)
        n_l.append(d_left if d_left else n_l[-1] if n_l else 0)
        n_r.append(d_right if d_right else n_r[-1] if n_r else 0)

        ##########################################
        # DEBUG PLOT every 50 points
        ##########################################

        
        if debug and i % 106 == 0:
           

            # centerline point
            plt.scatter(pt[0], pt[1], c="green", marker="P", s=100, zorder=11)

            # normals
            plt.plot([pt[0], pt[0] + line_len * normal_left[0]],
                     [pt[1], pt[1] + line_len * normal_left[1]], "r--", label="Normal Right" if i == 0 else "")
            plt.plot([pt[0], pt[0] + line_len * normal_right[0]],
                     [pt[1], pt[1] + line_len * normal_right[1]], "b--", label="Normal Right" if i == 0 else "")

           # intersections (left)
            inter_left = contour_line_left.intersection(line_left)
            if not inter_left.is_empty:
                if inter_left.geom_type == "Point":
                    plt.scatter(inter_left.x, inter_left.y, c="darkorange", marker="x", s=100, linewidths=3, zorder=10, label="Intersection Left" if i == 0 else "")
                elif inter_left.geom_type == "MultiPoint":
                    # choose nearest point
                    pts = np.array([[p.x, p.y] for p in inter_left.geoms])
                    dists = np.linalg.norm(pts - pt, axis=1)
                    closest = pts[np.argmin(dists)]
                    plt.scatter(closest[0], closest[1], c="darkorange", marker="x", s=100, linewidths=3, zorder=10)

            # intersections (right)
            inter_right = contour_line_right.intersection(line_right)
            if not inter_right.is_empty:
                if inter_right.geom_type == "Point":
                    plt.scatter(inter_right.x, inter_right.y, c="violet", marker="x", s=100, linewidths=3, zorder=10, label="Intersection Right" if i == 0 else "")
                elif inter_right.geom_type == "MultiPoint":
                    pts = np.array([[p.x, p.y] for p in inter_right.geoms])
                    dists = np.linalg.norm(pts - pt, axis=1)
                    closest = pts[np.argmin(dists)]
                    plt.scatter(closest[0], closest[1], c="violet", marker="x", s=100, linewidths=3, zorder=10)

    if debug:
        plt.axis("equal")
        plt.title(f"Normals and intersections at point {i}")
        plt.legend()
        plt.show()

    return np.array(n_l), np.array(n_r)

##########################################
# MAIN
##########################################
map_img, binary, resolution, origin = load_map(map_path, yaml_path)

plt.figure()
plt.imshow(map_img, cmap="gray", origin="lower")
plt.plot(-origin[0]/resolution, -origin[1]/resolution, "ro", label="Origin")
plt.legend()
plt.title("Loaded occupancy grid (PGM)")
plt.axis("equal")
plt.show()

contour1, contour2 = extract_contours(binary, direction)
plt.figure()
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(contour1[:,0], contour1[:,1], "r", label="Boundary 1")
plt.plot(contour2[:,0], contour2[:,1], "b", label="Boundary 2")
plt.legend(); plt.axis("equal"); plt.title("Extracted track boundaries")
plt.show()

centerline_px = build_centerline(contour1, contour2, amostragem, resolution, direction)
s, x_s, y_s, kappa, centerline_px = smooth_centerline(centerline_px, origin, resolution, amostragem)

plt.figure()
plt.imshow(binary, cmap="gray", origin="lower")
plt.plot(centerline_px[:,0], centerline_px[:,1], "g", label="Centerline")
plt.plot(contour1[:,0], contour1[:,1], "r", label="Boundary 1")
plt.plot(contour2[:,0], contour2[:,1], "b", label="Boundary 2")
plt.legend(); plt.axis("equal"); plt.title("Initial centerline (pixels)")
plt.show()

plt.figure()
plt.plot(s, kappa)
plt.xlabel("s [m]"); plt.ylabel("κ [1/m]")
plt.title("Curvature along the trajectory")
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