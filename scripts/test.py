import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
from acados_template import AcadosOcpSolver
from generate_c import setup_ocp 
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN 

N_HORIZON = 60 
DT = 0.05 
SIM_STEPS = 1000 
INDEX_STEP = 5 
N_OBS_SLOTS = 5   
VEHICLE_RADIUS = 1.5 

LIDAR_NOISE = 0.2        # 点云噪声 (米)
CLUSTER_EPS = 2.5        # DBSCAN聚类半径 (米)
CLUSTER_MIN_SAMPLES = 3  # 聚类最小点数


def get_closest_point_on_segment(p, a, b):
    ap = p - a
    ab = b - a
    t = np.dot(ap, ab) / (np.dot(ab, ab) + 1e-6)
    t = np.clip(t, 0.0, 1.0)
    return a + t * ab

def generate_lidar_points(true_circles, true_walls):
    points = []
    
    for obs in true_circles:
        ox, oy, r = obs[0], obs[1], obs[2]
        # 在圆周上生成点
        num_pts = int(r * 15) # 半径越大点越多
        angles = np.linspace(0, 2*np.pi, num_pts, endpoint=False)
        for ang in angles:
            # 加入随机噪声
            nx = np.random.normal(0, LIDAR_NOISE)
            ny = np.random.normal(0, LIDAR_NOISE)
            px = ox + r * np.cos(ang) + nx
            py = oy + r * np.sin(ang) + ny
            points.append([px, py])

    for w in true_walls:
        start, end = w['start'], w['end']
        length = np.linalg.norm(end - start)
        num_pts = int(length * 5) # 每米5个点
        vec = end - start
        for i in range(num_pts):
            t = i / num_pts
            # 加入随机噪声
            nx = np.random.normal(0, LIDAR_NOISE)
            ny = np.random.normal(0, LIDAR_NOISE)
            p = start + t * vec + np.array([nx, ny])
            points.append(p)
            
    return np.array(points)

def process_point_cloud(points):
    """
    输入: 原始点云 (N, 2)
    输出: 拟合后的障碍物列表 list of {'x', 'y', 'r', 'vx', 'vy'}
    """
    if len(points) == 0:
        return []
        
    clustering = DBSCAN(eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES).fit(points)
    labels = clustering.labels_
    
    detected_objects = []
    unique_labels = set(labels)
    
    for label in unique_labels:
        if label == -1: continue # 噪声点忽略

        cluster_mask = (labels == label)
        cluster_pts = points[cluster_mask]

        center_x = np.mean(cluster_pts[:, 0])
        center_y = np.mean(cluster_pts[:, 1])

        dists = np.linalg.norm(cluster_pts - np.array([center_x, center_y]), axis=1)
        radius = np.max(dists) + 0.2 
        
        vx, vy = 0.0, 0.0
        
        detected_objects.append({
            'x': center_x, 'y': center_y, 'r': radius, 
            'vx': vx, 'vy': vy,
            'type': 'fitted_circle' # 统一视为圆形处理
        })
        
    return detected_objects

def compute_hyperplane(veh_pos, obs_data, dt_time):
    ox, oy, r = obs_data['x'], obs_data['y'], obs_data['r']
    vx, vy = obs_data['vx'], obs_data['vy']

    target_x, target_y = ox + vx * dt_time, oy + vy * dt_time
    target_r = r + VEHICLE_RADIUS
    
    dx = veh_pos[0] - target_x
    dy = veh_pos[1] - target_y
    norm = np.hypot(dx, dy)
    if norm < 0.01: 
        nx, ny = 1.0, 0.0
    else: 
        nx, ny = dx/norm, dy/norm
        
    return target_x, target_y, target_r, nx, ny

def get_plane_segment(tx, ty, tr, nx, ny, length=2.0):
    px = tx + nx * tr
    py = ty + ny * tr
    tan_x, tan_y = -ny, nx
    p1 = (px - tan_x * length, py - tan_y * length)
    p2 = (px + tan_x * length, py + tan_y * length)
    return [p1, p2]

ocp = setup_ocp(N_horizon=N_HORIZON, Tf=N_HORIZON*DT)
solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

# 路径
s = np.linspace(0, 150, 3000) 
ref_x_full = 1.2 * s
ref_y_full = 7.0 * np.sin(0.15 * s) 
waypoints = np.vstack((ref_x_full, ref_y_full)).T
tree = KDTree(waypoints)

true_circle_obs = [
    np.array([30.0, 5.0, 2.0, 0.0, 0.0]),      
    np.array([50.0, -2.0, 1.5, 0.0, 0.0]),    
    np.array([110.0, -3.0, 2.0, -0.5, 0.0]),   
]
true_walls = [
    {'start': np.array([60.0, -10.0]), 'end': np.array([60.0, 5.0])},
    {'start': np.array([65.0, 8.0]),   'end': np.array([65.0, 20.0])},
    {'start': np.array([85.0, 20.0]),  'end': np.array([85.0, 2.0])},
]

plt.ion()
fig, ax = plt.subplots(figsize=(14, 7))

ax.plot(ref_x_full, ref_y_full, color='#bdc3c7', linestyle='--', label='Reference')
line_traj, = ax.plot([], [], 'g-', linewidth=2, label='Driven Path')
line_pred, = ax.plot([], [], 'b-o', markersize=3, alpha=0.6, label='MPC Plan')
scatter_lidar = ax.scatter([], [], s=5, c='k', marker='.', alpha=0.7, label='LiDAR Cloud')
perc_patches = [] 

lc_planes = LineCollection([], linewidths=1, alpha=0.5) 
ax.add_collection(lc_planes)

ax.set_aspect('equal')
ax.grid(True, linestyle=':', alpha=0.5)

from matplotlib.lines import Line2D
custom_lines = [Line2D([0], [0], color='r', lw=2),
                Line2D([0], [0], color='g', lw=2),
                Line2D([0], [0], marker='.', color='k', linestyle='None')]
ax.legend(custom_lines, ['Constraint Active', 'Constraint Ignored', 'Raw Point Cloud'])
ax.set_title("NMPC with LiDAR Perception: Point Cloud -> Clustering -> Convex Constraints")

current_x = np.array([waypoints[0,0], waypoints[0,1], 0.0, 0.0, 0.0])
history_x, history_y = [], []
x_guess = np.zeros((N_HORIZON + 1, 5))
for k in range(N_HORIZON + 1): x_guess[k, :] = current_x

print("Simulation started... Processing Point Clouds...")

for i in range(SIM_STEPS):
    dist, nearest_idx = tree.query([current_x[0], current_x[1]])
    
    raw_points = generate_lidar_points(true_circle_obs, true_walls)
    
    detected_obs = process_point_cloud(raw_points)
    
    active_obstacles = [] 
    curr_pos = current_x[:2]
    
    for idx, obs in enumerate(detected_obs):
        d = np.linalg.norm(curr_pos - np.array([obs['x'], obs['y']])) - obs['r']
        obs['dist'] = d
        active_obstacles.append(obs)
        
    active_obstacles.sort(key=lambda x: x['dist'])

    frame_segments = [] 
    frame_colors = [] 
    
    for j in range(N_HORIZON + 1):
        if j < N_HORIZON:
            ref_idx = min(nearest_idx + j * INDEX_STEP, len(waypoints) - 1)
            pt = waypoints[ref_idx]
            solver.set(j, "yref", np.array([pt[0], pt[1], 5.0, 0, 0, 0, 0]))
        else:
            idx_e = min(nearest_idx + N_HORIZON * INDEX_STEP, len(waypoints) - 1)
            solver.set(j, "yref", np.array([waypoints[idx_e, 0], waypoints[idx_e, 1], 5.0, 0]))

        veh_pred = x_guess[j, :2]
        param_val = []
        
        for k in range(N_OBS_SLOTS):
            if k < len(active_obstacles):
                item = active_obstacles[k]
                tx, ty, tr, nx, ny = compute_hyperplane(veh_pred, item, j*DT)
                param_val.extend([tx, ty, tr, nx, ny])
                
                seg = get_plane_segment(tx, ty, tr, nx, ny)
                frame_segments.append(seg)
                frame_colors.append((1, 0, 0, 0.6)) 
            else:
                param_val.extend([1000.0, 1000.0, 0.1, 1.0, 0.0]) # Dummy
        
        solver.set(j, "p", np.array(param_val))

        for k in range(N_OBS_SLOTS, len(active_obstacles)):
            item = active_obstacles[k]
            if item['dist'] < 15.0:
                tx, ty, tr, nx, ny = compute_hyperplane(veh_pred, item, j*DT)
                seg = get_plane_segment(tx, ty, tr, nx, ny)
                frame_segments.append(seg)
                frame_colors.append((0, 1, 0, 0.4)) 

    solver.set(0, "lbx", current_x)
    solver.set(0, "ubx", current_x)
    
    status = solver.solve()
    solve_time = solver.get_stats("time_tot") 
    print(f"第 {i} 步求解耗时: {solve_time*1000:.2f} ms")
    if status == 0:
        for k in range(N_HORIZON + 1):
            x_guess[k, :] = solver.get(k, "x")
        u = solver.get(0, "u")
    else:
        u = [0, 0]

    current_x[0] += current_x[3] * np.cos(current_x[2]) * DT
    current_x[1] += current_x[3] * np.sin(current_x[2]) * DT
    current_x[2] += current_x[4] * DT
    current_x[3] += u[0] * DT
    current_x[4] += u[1] * DT
    
    for obs in true_circle_obs:
        obs[0] += obs[3] * DT
        obs[1] += obs[4] * DT
        
    history_x.append(current_x[0])
    history_y.append(current_x[1])

    if i % 2 == 0:
        line_traj.set_data(history_x, history_y)
        line_pred.set_data(x_guess[:,0], x_guess[:,1])

        if len(raw_points) > 0:
            scatter_lidar.set_offsets(raw_points)
        
        [p.remove() for p in perc_patches]
        perc_patches = []
        for obs in detected_obs:
            if np.linalg.norm(current_x[:2] - np.array([obs['x'], obs['y']])) < 40:
                c = patches.Circle((obs['x'], obs['y']), obs['r'], fill=False, edgecolor='cyan', linestyle=':', alpha=0.5)
                ax.add_patch(c)
                perc_patches.append(c)
        
        lc_planes.set_segments(frame_segments)
        lc_planes.set_color(frame_colors)
        
        ax.set_xlim(current_x[0]-20, current_x[0]+40)
        ax.set_ylim(current_x[1]-20, current_x[1]+20)
        plt.pause(0.001)

    if np.linalg.norm(current_x[:2] - waypoints[-1]) < 3.0:
        print("Finished!")
        break

plt.ioff()
plt.show()