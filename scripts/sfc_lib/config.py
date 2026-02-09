from dataclasses import dataclass

@dataclass
class SFCConfig:
    robot_radius: float = 0.5
    search_radius: float = 6.0
    longitudinal_length: float = 4.0  # 沿路径方向的最大前后搜索距离
    max_obstacles: int = 50           # (可选) 限制考虑的障碍物数量