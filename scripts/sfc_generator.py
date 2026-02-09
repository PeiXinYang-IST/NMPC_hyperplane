import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import KDTree
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class FixedBoxConfig:
    robot_radius: float = 0.5
    search_radius: float = 6.0
    longitudinal_length: float = 4.0  # 沿路径方向的最大前后搜索距离

@dataclass
class FixedBoxConstraint:
    # 这里的约束形式为: lb <= C * x <= ub
    # 或者一般形式: A * x <= b
    # 我们生成 Ax <= b 的形式，矩阵 A 是 4x2，向量 b 是 4x1
    A: np.ndarray 
    b: np.ndarray
    # 另外保存几何信息用于可视化
    vertices: np.ndarray 

class FixedCorridorGenerator:
    def __init__(self, config: FixedBoxConfig):
        self.cfg = config

    def generate_fixed_corridor(self, path_points: np.ndarray, obstacles: np.ndarray) -> List[FixedBoxConstraint]:
        """
        生成固定维度的凸包（定向矩形）
        """
        constraints = []
        obs_tree = KDTree(obstacles)
        
        # 计算路径的切线方向 (Yaw)
        diffs = np.diff(path_points, axis=0)
        # 补全最后一个点的方向
        diffs = np.vstack([diffs, diffs[-1]]) 
        yaws = np.arctan2(diffs[:, 1], diffs[:, 0])

        # 采样步长 (与预测时域 N 对应)
        # 假设 path_points 的密度就是 ACADOS 的 shooting nodes
        for i, (seed, yaw) in enumerate(zip(path_points, yaws)):
            
            # 1. 构建旋转矩阵 (用于将障碍物转换到局部坐标系)
            # Local Frame: X轴指向切线方向，Y轴指向法线方向
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            R_world_to_local = np.array([
                [cos_yaw, sin_yaw],
                [-sin_yaw, cos_yaw]
            ])
            
            # 2. 搜索附近的障碍物
            indices = obs_tree.query_ball_point(seed, self.cfg.search_radius)
            local_points = []
            
            if len(indices) > 0:
                nearby_obs = obstacles[indices]
                # 将障碍物转换到局部坐标系: p_local = R * (p_world - seed)
                # 向量化计算
                diff = nearby_obs - seed
                local_obs = diff @ R_world_to_local.T
            else:
                local_obs = np.empty((0, 2))

            # 3. 初始化边界 (无障碍时的最大范围)
            # local_x (前后), local_y (左右)
            # 前后我们要限制一下，不能太长，否则弯道会切墙
            d_front = self.cfg.longitudinal_length / 2.0
            d_back  = -self.cfg.longitudinal_length / 2.0
            d_left  = self.cfg.search_radius  # +Y
            d_right = -self.cfg.search_radius # -Y
            
            # 4. 根据障碍物收缩边界 (关键步骤)
            # 我们在局部坐标系下找最近的障碍物
            
            margin = self.cfg.robot_radius + 0.1
            
            for p in local_obs:
                px, py = p[0], p[1]
                
                # 简单的轴对齐包围盒收缩 (AABB in Local Frame)
                # 这种方法虽然保守，但极其稳健且计算极快
                
                # 检查这个点是在前、后、左、还是右？
                # 这里我们采用“全象限收缩”策略：
                # 只要点在盒子范围内，就收缩最近的那条边
                
                # 简化逻辑：只考虑纯横向障碍物来收缩左右，纯纵向来收缩前后
                # 更好的逻辑是：Raycasting
                
                # --- Raycasting Logic (更准确) ---
                # 检查点是否在当前的纵向范围内
                if d_back < px < d_front:
                    # 如果在左侧且距离小于当前左边界
                    if 0 < py < d_left:
                        d_left = max(0.0, py - margin)
                    # 如果在右侧
                    elif d_right < py < 0:
                        d_right = min(0.0, py + margin)
                
                # 检查点是否在当前的横向范围内
                if d_right < py < d_left:
                    if 0 < px < d_front:
                        d_front = max(0.0, px - margin)
                    elif d_back < px < 0:
                        d_back = min(0.0, px + margin)

            # 5. 构建约束矩阵 Ax <= b
            # 在局部坐标系下，约束是:
            # x_local <= d_front
            # x_local >= d_back  -> -x_local <= -d_back
            # y_local <= d_left
            # y_local >= d_right -> -y_local <= -d_right
            
            # 转换回世界坐标系: x_local = R * (x_world - seed)
            # A_local * R * (x - seed) <= b_local
            # A_world = A_local * R
            # b_world = b_local + A_world * seed
            
            # 为了 Acados 方便，我们直接给出 A 和 b (4个线性不等式)
            # 定义局部法向量 (前, 后, 左, 右)
            n_front = np.array([1, 0])
            n_back  = np.array([-1, 0])
            n_left  = np.array([0, 1])
            n_right = np.array([0, -1])
            
            A_local = np.array([n_front, n_back, n_left, n_right])
            b_local = np.array([d_front, -d_back, d_left, -d_right])
            
            # 旋转 A 到世界坐标
            # A_world = A_local @ R_world_to_local
            # 注意: R_world_to_local 把世界转局部，所以 A_world = A_local * R
            A_world = A_local @ R_world_to_local
            
            # 计算 b (考虑到 seed 的平移)
            # constraint: n^T * (x - seed) <= d
            # n^T * x <= d + n^T * seed
            b_world = b_local + np.sum(A_world * seed, axis=1)
            
            # 6. 计算顶点用于可视化
            # 局部顶点
            v_local = np.array([
                [d_front, d_left],
                [d_back, d_left],
                [d_back, d_right],
                [d_front, d_right]
            ])
            # 转回世界坐标: v_world = v_local @ R.T + seed
            # R_world_to_local.T 就是 R_local_to_world
            v_world = v_local @ R_world_to_local + seed
            
            constraints.append(FixedBoxConstraint(A=A_world, b=b_world, vertices=v_world))
            
        return constraints

# ==========================================
# 可视化部分 (复用之前的场景)
# ==========================================
def create_scenario():
    road_length = 40
    num_points = 200
    x_range = np.linspace(0, road_length, num_points)
    
    wall_up = np.column_stack((x_range, np.full_like(x_range, 4.0)))
    wall_down = np.column_stack((x_range, np.full_like(x_range, -4.0)))
    
    theta = np.linspace(0, 2*np.pi, 40)
    circle_x = 20.0 + 1.0 * np.cos(theta)
    circle_y = 0.0 + 1.0 * np.sin(theta)
    obs_circle = np.column_stack((circle_x, circle_y))
    
    obstacles = np.vstack((wall_up, wall_down, obs_circle))
    
    path_x = np.linspace(0, road_length, 40) # ACADOS 的节点数，比如 40
    path_y = 2.5 * np.exp(-0.1 * (path_x - 20)**2)
    path = np.column_stack((path_x, path_y))
    
    return path, obstacles

def main():
    path, obstacles = create_scenario()
    
    cfg = FixedBoxConfig()
    cfg.robot_radius = 0.5
    # 纵向长度通常设为 dt * v_ref * 2 左右，保证覆盖
    cfg.longitudinal_length = 3.0 
    
    generator = FixedCorridorGenerator(cfg)
    constraints = generator.generate_fixed_corridor(path, obstacles)
    
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.scatter(obstacles[:,0], obstacles[:,1], c='red', s=5, alpha=0.5)
    ax.plot(path[:,0], path[:,1], 'b--o', markersize=3)
    
    import matplotlib.cm as cm
    colors = cm.viridis(np.linspace(0, 1, len(constraints)))
    
    for i, cons in enumerate(constraints):
        # 绘制矩形
        p = Polygon(cons.vertices, facecolor=colors[i], alpha=0.2, edgecolor='black', linewidth=0.5)
        ax.add_patch(p)

    ax.set_aspect('equal')
    ax.set_title("Fixed-Dimension ACADOS-Ready Corridor (Oriented Boxes)")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()