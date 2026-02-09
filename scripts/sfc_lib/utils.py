import numpy as np

def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def get_rotation_matrix(yaw):
    """获取从世界坐标系到局部坐标系的旋转矩阵 (World -> Local)
    Local X 轴指向 Yaw 方向
    """
    c = np.cos(yaw)
    s = np.sin(yaw)
    # R_w2l = [ c  s]
    #         [-s  c]
    return np.array([[c, s], [-s, c]])

def transform_to_local(points, origin, yaw):
    """将一组点转换到以 origin 为原点，yaw 为 X 轴的局部坐标系"""
    if len(points) == 0:
        return np.empty((0, 2))
    
    diff = points - origin
    R = get_rotation_matrix(yaw)
    # (N, 2) @ (2, 2).T -> (N, 2)
    return diff @ R.T

def transform_to_world(points_local, origin, yaw):
    """将局部点转换回世界坐标系"""
    if len(points_local) == 0:
        return np.empty((0, 2))
    
    # R_l2w = R_w2l.T
    R_w2l = get_rotation_matrix(yaw)
    R_l2w = R_w2l.T 
    
    # p_world = p_local @ R_l2w.T + origin
    return points_local @ R_l2w.T + origin