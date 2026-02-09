import numpy as np
from scipy.spatial import KDTree
from .config import SFCConfig
from .utils import transform_to_local, transform_to_world

class SFCConstraint:
    """存储生成的单个矩形约束"""
    def __init__(self, A, b, vertices):
        self.A = A          # (4, 2) Matrix: A*x <= b
        self.b = b          # (4,) Vector
        self.vertices = vertices # (4, 2) Points: visualize [FR, FL, BL, BR]

class SFCGenerator:
    def __init__(self, config: SFCConfig = None):
        self.cfg = config if config else SFCConfig()

    def generate_corridor(self, path_points: np.ndarray, obstacles: np.ndarray):
        """
        生成安全走廊
        :param path_points: (N, 2) 参考路径点
        :param obstacles: (M, 2) 障碍物点云
        :return: List[SFCConstraint]
        """
        N = len(path_points)
        constraints = []
        
        if N < 1:
            return constraints

        # 1. 预计算每个点的 Yaw
        yaws = self._compute_path_yaws(path_points)

        # 2. 构建 KDTree 加速搜索 (可选，点少时暴力也可)
        obs_tree = None
        if len(obstacles) > 0:
            obs_tree = KDTree(obstacles)

        # 3. 遍历路径点生成约束
        for i in range(N):
            seed = path_points[i]
            yaw = yaws[i]
            
            # 搜索局部障碍物
            local_obs = np.empty((0, 2))
            if obs_tree is not None:
                # 只搜索 search_radius 范围内的点
                idxs = obs_tree.query_ball_point(seed, self.cfg.search_radius)
                if len(idxs) > 0:
                    nearby_obs = obstacles[idxs]
                    local_obs = transform_to_local(nearby_obs, seed, yaw)

            # 执行矩形收缩
            # d_vals: [front, back, left, right]
            d_vals = self._shrink_box(local_obs) 
            
            # 构建局部约束 (Box Constraints)
            # x <= front  =>  1*x <= front
            # x >= back   => -1*x <= -back
            # y <= left   =>  1*y <= left
            # y >= right  => -1*y <= -right
            
            # A_local: (4, 2)
            A_local = np.array([
                [ 1.0,  0.0],
                [-1.0,  0.0],
                [ 0.0,  1.0],
                [ 0.0, -1.0]
            ])
            # b_local: (4,)
            b_local = np.array([d_vals[0], -d_vals[1], d_vals[2], -d_vals[3]])
            
            # 转换回世界坐标系 (用于 ACADOS 输入)
            # Constraint: A_loc * x_loc <= b_loc
            # x_loc = R * (x_world - seed)
            # A_loc * R * x_world <= b_loc + A_loc * R * seed
            
            # R_w2l
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, s], [-s, c]])
            
            A_world = A_local @ R
            b_world = b_local + A_world @ seed
            
            # 计算顶点用于可视化 (Local -> World)
            # Order: FR, FL, BL, BR
            v_local = np.array([
                [d_vals[0], d_vals[2]], # Front-Left (Local Y is Left)
                [d_vals[1], d_vals[2]], # Back-Left
                [d_vals[1], d_vals[3]], # Back-Right
                [d_vals[0], d_vals[3]]  # Front-Right
            ])
            v_world = transform_to_world(v_local, seed, yaw)
            
            constraints.append(SFCConstraint(A_world, b_world, v_world))
            
        return constraints

    def _compute_path_yaws(self, path):
        N = len(path)
        yaws = np.zeros(N)
        for i in range(N):
            if i < N - 1:
                diff = path[i+1] - path[i]
            else:
                diff = path[i] - path[i-1]
            
            if np.linalg.norm(diff) < 1e-3:
                yaws[i] = yaws[i-1] if i > 0 else 0.0
            else:
                yaws[i] = np.arctan2(diff[1], diff[0])
        return yaws

    def _shrink_box(self, local_obs):
        """
        核心收缩算法 (OBB Shrinking)
        :return: [d_front, d_back, d_left, d_right] (Local Frame)
        """
        # 初始最大范围
        d_front = self.cfg.longitudinal_length / 2.0
        d_back  = -self.cfg.longitudinal_length / 2.0
        d_left  = self.cfg.search_radius
        d_right = -self.cfg.search_radius
        
        margin = self.cfg.robot_radius + 0.1 # 安全边距
        
        if len(local_obs) == 0:
            return np.array([d_front, d_back, d_left, d_right])
            
        for p in local_obs:
            px, py = p[0], p[1]
            
            # 1. 左右收缩 (Lateral)
            # 只有当障碍物在当前的 [back, front] 纵向范围内时，才收缩左右
            if d_back < px < d_front:
                if 0 < py < d_left:
                    d_left = max(0.0, py - margin)
                elif d_right < py < 0:
                    d_right = min(0.0, py + margin)
            
            # 2. 前后收缩 (Longitudinal)
            # 只有当障碍物在当前的 [right, left] 横向范围内时，才收缩前后
            if d_right < py < d_left:
                if 0 < px < d_front:
                    d_front = max(0.0, px - margin)
                elif d_back < px < 0:
                    d_back = min(0.0, px + margin)
                    
        return np.array([d_front, d_back, d_left, d_right])