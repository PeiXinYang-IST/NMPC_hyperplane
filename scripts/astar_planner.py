import numpy as np
import heapq
from scipy.spatial import KDTree

class AStarPlanner:
    def __init__(self, resolution=0.4, margin=0.6, ref_weight=2.0):
        """
        :param resolution: 栅格地图分辨率 (m)
        :param margin: 障碍物膨胀半径 (m)
        :param ref_weight: 偏离全局参考路径的惩罚权重
        """
        self.res = resolution
        self.margin = margin
        self.ref_weight = ref_weight
        self.motion = [ # 8 邻域移动
            (1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

    def plan(self, start_pos, goal_pos, obstacles, global_path_tree):
        """
        :param start_pos: [x, y]
        :param goal_pos: [x, y]
        :param obstacles: np.array [[x, y], ...]
        :param global_path_tree: KDTree object of global path (for fast lookup)
        :return: path [[x, y], ...] or None
        """
        # 1. 确定搜索边界 (减少计算量)
        min_x = min(start_pos[0], goal_pos[0]) - 10.0
        max_x = max(start_pos[0], goal_pos[0]) + 10.0
        min_y = min(start_pos[1], goal_pos[1]) - 10.0
        max_y = max(start_pos[1], goal_pos[1]) + 10.0

        # 2. 障碍物栅格化
        obs_set = set()
        for ox, oy in obstacles:
            if min_x <= ox <= max_x and min_y <= oy <= max_y:
                # 简单的膨胀处理：将障碍物周围的栅格都加入 set
                ix = int(round(ox / self.res))
                iy = int(round(oy / self.res))
                margin_steps = int(np.ceil(self.margin / self.res))
                for i in range(-margin_steps, margin_steps + 1):
                    for j in range(-margin_steps, margin_steps + 1):
                        obs_set.add((ix + i, iy + j))

        # 3. A* 初始化
        start_node = (int(round(start_pos[0] / self.res)), int(round(start_pos[1] / self.res)))
        goal_node = (int(round(goal_pos[0] / self.res)), int(round(goal_pos[1] / self.res)))
        
        open_set = []
        heapq.heappush(open_set, (0, start_node))
        
        came_from = {}
        g_score = {start_node: 0}
        
        # 4. 搜索循环
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == goal_node or np.hypot(current[0]-goal_node[0], current[1]-goal_node[1]) < 2.0:
                # 到达目标或足够近
                return self._reconstruct_path(came_from, current, start_node)
            
            for dx, dy, move_cost in self.motion:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 碰撞检测
                if neighbor in obs_set:
                    continue
                
                # 边界检测 (可选，防止无限搜索)
                nx_world = neighbor[0] * self.res
                ny_world = neighbor[1] * self.res
                if not (min_x < nx_world < max_x and min_y < ny_world < max_y):
                    continue

                # --- 代价计算核心 ---
                # G_cost = 移动代价 + 参考线吸引代价
                
                # 计算到全局路径的距离
                dist_to_ref, _ = global_path_tree.query([nx_world, ny_world])
                ref_penalty = self.ref_weight * dist_to_ref
                
                tentative_g = g_score[current] + move_cost + ref_penalty
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # H_cost = 欧几里得距离
                    h_score = np.hypot(neighbor[0] - goal_node[0], neighbor[1] - goal_node[1])
                    f_score = tentative_g + h_score
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return None # 寻路失败

    def _reconstruct_path(self, came_from, current, start):
        path = []
        while current != start:
            path.append([current[0] * self.res, current[1] * self.res])
            current = came_from[current]
        path.append([start[0] * self.res, start[1] * self.res])
        path.reverse()
        
        # 简单的路径平滑 (可选)
        if len(path) > 2:
            path = np.array(path)
            # 简单的均值滤波
            smooth_path = np.copy(path)
            for i in range(1, len(path)-1):
                smooth_path[i] = 0.5 * path[i] + 0.25 * path[i-1] + 0.25 * path[i+1]
            return smooth_path
            
        return np.array(path)