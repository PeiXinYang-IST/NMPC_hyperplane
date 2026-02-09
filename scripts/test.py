import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle, Polygon, Circle
from matplotlib.collections import LineCollection
from acados_template import AcadosOcpSolver
from scipy.spatial import KDTree
import time

# --- 导入自定义模块 ---
from generate_c import setup_ocp 
from sfc_lib import SFCGenerator, SFCConfig 
from astar_planner import AStarPlanner 

# ==========================================
# 1. 仿真参数配置
# ==========================================
CONF = {
    'T_HORIZON': 3.0,
    'N_HORIZON': 60,
    'DT': 0.05,
    'SIM_STEPS': 1000,
    'ROAD_WIDTH': 7.0,
    'VEH_RADIUS': 0.5,
    'TARGET_SPEED': 6.0,  # 目标速度 m/s
    'ASTAR_RES': 0.4,     # A* 栅格分辨率
    'ASTAR_MARGIN': 0.6,  # A* 避障膨胀
    'REF_WEIGHT': 2.5     # A* 贴合参考线的权重
}

# ==========================================
# 2. 复杂场景生成器
# ==========================================
def create_complex_map():
    """生成包含直道、S弯、狭窄关卡的复杂地图"""
    ds = 0.1
    s_values = np.arange(0, 200, ds)
    
    # --- 1. 生成参考线 (S-Curve + Straight) ---
    ref_x, ref_y = [], []
    for s in s_values:
        if s < 50: # 直道入弯
            ref_x.append(s)
            ref_y.append(0)
        elif s < 100: # 大S弯 1
            ref_x.append(s)
            ref_y.append(8.0 * np.sin((s - 50) * 0.1))
        elif s < 150: # 急弯
            ref_x.append(s)
            ref_y.append(8.0 * np.sin(5.0) + 4.0 * np.cos((s - 100) * 0.15) - 4.0)
        else: # 回正
            ref_x.append(s)
            ref_y.append(ref_y[-1])
            
    path = np.vstack((ref_x, ref_y)).T
    
    # --- 2. 生成道路边界 (墙) ---
    walls = []
    # 计算切线方向
    for i in range(len(path)):
        if i < len(path)-1:
            dx, dy = path[i+1] - path[i]
        else:
            dx, dy = path[i] - path[i-1]
        yaw = np.arctan2(dy, dx)
        nx, ny = -np.sin(yaw), np.cos(yaw)
        
        walls.append(path[i] + np.array([nx, ny]) * CONF['ROAD_WIDTH']/2)
        walls.append(path[i] - np.array([nx, ny]) * CONF['ROAD_WIDTH']/2)
    
    # --- 3. 生成障碍物 (Block list) ---
    # 格式: [center_x, center_y, width, height]
    obstacles_rects = [
        [30.0,  1.5, 2.0, 2.0],  # 障碍1: 逼迫右行
        [70.0, -4.0, 2.0, 2.0],  # 障碍2: 位于波谷，逼迫左行
        [85.0,  4.0, 2.0, 2.0],  # 障碍3: 位于波峰
        # 狭窄关卡 (Gate)
        [110.0, 4.0, 2.0, 8.0],  # 上方挡板
        [110.0, -6.0, 2.0, 8.0], # 下方挡板 (中间留空隙)
        # 终点前的乱石阵
        [160.0, -2.0, 1.5, 1.5],
        [170.0,  2.0, 1.5, 1.5]
    ]
    
    # 将矩形转换为点云用于 A* 和 SFC
    obs_points = []
    for ox, oy, w, h in obstacles_rects:
        # 密集采样表面和内部
        xs = np.linspace(ox - w/2, ox + w/2, int(w/0.2))
        ys = np.linspace(oy - h/2, oy + h/2, int(h/0.2))
        for x in xs:
            for y in ys:
                obs_points.append([x, y])
                
    return path, np.array(walls), np.array(obs_points), obstacles_rects

# ==========================================
# 3. 辅助函数
# ==========================================
def resample_trajectory(path, num_points):
    """基于距离的均匀重采样"""
    if len(path) < 2: return np.tile(path[0], (num_points, 1))
    dists = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))
    dists = np.insert(dists, 0, 0.0)
    total_dist = dists[-1]
    
    # 如果路径太短，通过插值延长
    if total_dist < 0.1: 
        return np.tile(path[0], (num_points, 1))
        
    new_dists = np.linspace(0, total_dist, num_points)
    new_x = np.interp(new_dists, dists, path[:, 0])
    new_y = np.interp(new_dists, dists, path[:, 1])
    return np.vstack((new_x, new_y)).T

# ==========================================
# 4. 可视化类 (Dashboard)
# ==========================================
class SimVisualizer:
    def __init__(self, path, walls, obs_rects):
        plt.ion()
        self.fig = plt.figure(figsize=(16, 9))
        self.gs = gridspec.GridSpec(3, 3, figure=self.fig)
        
        # --- 主地图视角 ---
        self.ax_map = self.fig.add_subplot(self.gs[0:2, :])
        self.ax_map.set_aspect('equal')
        self.ax_map.plot(path[:,0], path[:,1], 'k--', alpha=0.3, label='Global Path')
        self.ax_map.scatter(walls[:,0], walls[:,1], s=1, c='gray', alpha=0.3)
        
        # 绘制实体障碍物矩形
        for ox, oy, w, h in obs_rects:
            rect = Rectangle((ox-w/2, oy-h/2), w, h, color='black', alpha=0.7)
            self.ax_map.add_patch(rect)
            
        # 动态元素
        self.car_patch = Rectangle((0,0), 1.0, 0.5, color='blue', alpha=0.8, zorder=10)
        self.ax_map.add_patch(self.car_patch)
        self.line_pred, = self.ax_map.plot([], [], 'r-', lw=2, label='MPC Pred')
        self.line_astar, = self.ax_map.plot([], [], 'g--', lw=1, alpha=0.6, label='A* Plan')
        self.sfc_patches = [] # 存储 SFC 盒子
        
        self.ax_map.legend(loc='upper right')
        self.ax_map.set_title("NMPC + SFC + A* Obstacle Avoidance")
        
        # --- 子图 1: 速度曲线 ---
        self.ax_vel = self.fig.add_subplot(self.gs[2, 0])
        self.ax_vel.set_title("Velocity (m/s)")
        self.ax_vel.set_ylim(0, 10)
        self.line_v, = self.ax_vel.plot([], [], 'b-')
        self.line_v_ref, = self.ax_vel.plot([], [], 'r--', alpha=0.5)
        self.v_data = []
        
        # --- 子图 2: 控制量 ---
        self.ax_u = self.fig.add_subplot(self.gs[2, 1])
        self.ax_u.set_title("Controls (Acc/Steer)")
        self.ax_u.set_ylim(-3, 3)
        self.line_u0, = self.ax_u.plot([], [], 'm-', label='Acc')
        self.line_u1, = self.ax_u.plot([], [], 'c-', label='Steer')
        self.ax_u.legend(fontsize='small')
        self.u0_data, self.u1_data = [], []
        
        # --- 子图 3: 求解耗时 ---
        self.ax_time = self.fig.add_subplot(self.gs[2, 2])
        self.ax_time.set_title("Solver Time (ms)")
        self.line_time, = self.ax_time.plot([], [], 'k-')
        self.time_data = []

    def update(self, x_curr, pred_traj, astar_traj, sfc_cons, v, u, t_solve):
        # 1. 更新车辆
        cx, cy, cyaw = x_curr[0], x_curr[1], x_curr[2]
        # 旋转车辆矩形
        ts = self.ax_map.transData
        tr =  plt.matplotlib.transforms.Affine2D().rotate_around(cx, cy, cyaw) + ts
        self.car_patch.set_xy((cx - 0.5, cy - 0.25)) # 假设车长1m宽0.5m
        self.car_patch.set_transform(tr)
        
        # 2. 更新路径
        self.line_pred.set_data(pred_traj[:,0], pred_traj[:,1])
        self.line_astar.set_data(astar_traj[:,0], astar_traj[:,1])
        
        # 3. 更新 SFC 盒子 (重绘)
        for p in self.sfc_patches: p.remove()
        self.sfc_patches.clear()
        
        # 只绘制每隔 5 个盒子，减少渲染压力
        for i, cons in enumerate(sfc_cons):
            if i % 5 == 0 or i == len(sfc_cons)-1:
                poly = Polygon(cons.vertices, closed=True, fc='green', ec='none', alpha=0.15)
                self.ax_map.add_patch(poly)
                self.sfc_patches.append(poly)
                
        # 4. 更新视窗
        self.ax_map.set_xlim(cx - 15, cx + 35)
        self.ax_map.set_ylim(cy - 15, cy + 15)
        
        # 5. 更新仪表盘数据
        MAX_HIST = 100
        self.v_data.append(v)
        self.u0_data.append(u[0])
        self.u1_data.append(u[1])
        self.time_data.append(t_solve * 1000) # ms
        
        if len(self.v_data) > MAX_HIST:
            self.v_data.pop(0); self.u0_data.pop(0); self.u1_data.pop(0); self.time_data.pop(0)
            
        x_grid = range(len(self.v_data))
        self.line_v.set_data(x_grid, self.v_data)
        self.ax_vel.set_xlim(0, len(self.v_data))
        
        self.line_u0.set_data(x_grid, self.u0_data)
        self.line_u1.set_data(x_grid, self.u1_data)
        self.ax_u.set_xlim(0, len(self.v_data))
        
        self.line_time.set_data(x_grid, self.time_data)
        self.ax_time.set_xlim(0, len(self.v_data))
        self.ax_time.set_ylim(0, max(max(self.time_data)+1, 10))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

# ==========================================
# 5. 主逻辑
# ==========================================
def main():
    # --- 初始化 ---
    global_path, walls, obs_points, obs_rects = create_complex_map()
    all_obstacles = np.vstack((walls, obs_points)) if len(obs_points) > 0 else walls
    global_tree = KDTree(global_path)
    
    # 模块初始化
    sfc_gen = SFCGenerator(SFCConfig(
        robot_radius=CONF['VEH_RADIUS'],
        search_radius=8.0,
        longitudinal_length=4.0
    ))
    
    astar = AStarPlanner(
        resolution=CONF['ASTAR_RES'], 
        margin=CONF['ASTAR_MARGIN'], 
        ref_weight=CONF['REF_WEIGHT']
    )
    
    ocp = setup_ocp(N_horizon=CONF['N_HORIZON'], Tf=CONF['T_HORIZON'])
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    
    # 初始状态
    x0, y0 = global_path[0]
    yaw0 = np.arctan2(global_path[1,1]-y0, global_path[1,0]-x0)
    current_x = np.array([x0, y0, yaw0, 0.0, 0.0]) # x, y, yaw, v, omega
    
    # 可视化器
    viz = SimVisualizer(global_path, walls, obs_rects)
    
    print("=== Start Complex Scenario Simulation ===")
    
    for step in range(CONF['SIM_STEPS']):
        t_start = time.time()
        
        # 1. 定位与前瞻
        curr_pos = current_x[:2]
        _, idx = global_tree.query(curr_pos)
        
        # 判断终点
        if idx >= len(global_path) - 10:
            print("Reached Goal!")
            break
            
        # 计算前瞻点 (根据速度动态调整，最少前瞻 8m)
        lookahead_dist = max(8.0, current_x[3] * CONF['T_HORIZON'] * 1.2)
        lookahead_idx = min(idx + int(lookahead_dist * 10), len(global_path)-1)
        target_pos = global_path[lookahead_idx]
        
        # 2. A* 路径规划 (从当前点 -> 前瞻点)
        astar_path = astar.plan(curr_pos, target_pos, all_obstacles, global_tree)
        
        if astar_path is None or len(astar_path) < 2:
            # 降级策略: 沿用全局路径 (风险较高，但在简单路段可行)
            fallback_end = min(idx + CONF['N_HORIZON'] + 10, len(global_path))
            astar_path = global_path[idx : fallback_end]
            
        # 3. 生成 SFC 走廊
        # [关键]: 重采样 A* 路径以匹配 MPC 的预测步数
        # 假设 A* 路径就是我们期望的“无碰撞通道中心”
        safe_traj_resampled = resample_trajectory(astar_path, CONF['N_HORIZON'] + 1)
        sfc_constraints = sfc_gen.generate_corridor(safe_traj_resampled, all_obstacles)
        
        # 4. 准备 MPC 参考 (yref) - 使用 GLOBAL PATH 产生势场
        # 我们希望车在安全走廊内，但尽可能贴近全局路径
        global_ref_traj = []
        for k in range(CONF['N_HORIZON'] + 1):
            ridx = min(idx + k, len(global_path) - 1)
            global_ref_traj.append(global_path[ridx])
        global_ref_traj = np.array(global_ref_traj)

        # 5. 设置 Solver
        for k in range(CONF['N_HORIZON'] + 1):
            # 5.1 设置 Cost
            pt = global_ref_traj[k]
            if k < CONF['N_HORIZON']:
                # x, y, v, w, ax, alpha, vx*w
                yref = np.array([pt[0], pt[1], CONF['TARGET_SPEED'], 0, 0, 0, 0])
                solver.set(k, "yref", yref)
            else:
                yref_e = np.array([pt[0], pt[1], CONF['TARGET_SPEED'], 0])
                solver.set(k, "yref", yref_e)
                
            # 5.2 设置 Constraints (SFC)
            if k < CONF['N_HORIZON']:
                # 获取对应的 SFC 盒子 (防止越界)
                sfc_idx = min(k, len(sfc_constraints)-1)
                cons = sfc_constraints[sfc_idx]
                
                # 转换为 Acados 参数: -Ax >= -b
                # param layout: [ox, oy, or, onx, ony] * 4 walls
                p_vec = []
                for w_i in range(4):
                    # Acados constraint: onx*(x-ox) + ony*(y-oy) >= or
                    # SFC: Ax <= b  ->  -Ax >= -b
                    # onx = -A[0], ony = -A[1], or = -b
                    p_vec.extend([0.0, 0.0, -cons.b[w_i], -cons.A[w_i,0], -cons.A[w_i,1]])
                
                solver.set(k, "p", np.array(p_vec))

        # 6. 求解
        solver.set(0, "lbx", current_x)
        solver.set(0, "ubx", current_x)
        status = solver.solve()
        t_solve = time.time() - t_start
        
        if status != 0:
            print(f"Solver Failed: {status}")
            # 简单刹车逻辑
            u = [ -1.0, 0.0 ] 
        else:
            u = solver.get(0, "u")

        # 7. 更新状态
        # 运动学模型更新
        dt = CONF['DT']
        current_x[0] += current_x[3] * np.cos(current_x[2]) * dt
        current_x[1] += current_x[3] * np.sin(current_x[2]) * dt
        current_x[2] += current_x[4] * dt
        current_x[3] += u[0] * dt
        current_x[4] += u[1] * dt
        
        # 8. 可视化更新 (每 2 帧刷新一次以保持流畅)
        if step % 1 == 0:
            # 获取预测轨迹用于绘图
            pred_traj = []
            for k in range(CONF['N_HORIZON']+1):
                pred_traj.append(solver.get(k, "x")[:2])
            pred_traj = np.array(pred_traj)
            
            viz.update(current_x, pred_traj, safe_traj_resampled, sfc_constraints, 
                       current_x[3], u, t_solve)

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()