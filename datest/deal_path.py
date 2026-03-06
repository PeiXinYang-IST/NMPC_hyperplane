import re
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.interpolate import interp1d
from matplotlib.collections import LineCollection

def interpolate_segment(pts, interval=0.05):
    """对路径段进行等间距插值"""
    if pts is None or len(pts) < 2:
        return pts
    # 过滤重复点
    diffs = np.diff(pts, axis=0)
    dist = np.sqrt(np.sum(diffs**2, axis=1))
    keep = np.concatenate([[True], dist > 1e-6])
    pts = pts[keep]
    if len(pts) < 2: return pts

    cumulative = np.concatenate([[0], np.cumsum(np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1)))])
    num_points = int(cumulative[-1] / interval) + 1
    if num_points < 2: return pts
    
    new_s = np.linspace(0, cumulative[-1], num_points)
    kind = 'cubic' if len(pts) > 3 else 'linear'
    
    fx = interp1d(cumulative, pts[:, 0], kind=kind, fill_value='extrapolate')
    fy = interp1d(cumulative, pts[:, 1], kind=kind, fill_value='extrapolate')
    return np.column_stack([fx(new_s), fy(new_s)])

def get_distance(p1, p2_list):
    """计算点 p1 到点集 p2_list 中最近点的距离"""
    if p2_list is None or len(p2_list) == 0:
        return 0
    dists = np.sqrt(np.sum((p2_list - p1)**2, axis=1))
    return np.min(dists)

def extract_pts(text_block):
    pts = re.findall(r'x:\s*([\d\.\-]+)\s*y:\s*([\d\.\-]+)', text_block)
    return np.array(pts, dtype=float) if len(pts) > 0 else None

def solve_with_width_v4(file_path, x_min_limit=450000):
    if not os.path.exists(file_path): return

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    roads = re.findall(r'road\s*\{([\s\S]*?)\n\}', content)
    road_data_list = []

    for r_block in roads:
        # 1. 提取中心线
        cc_match = re.search(r'central_curve\s*\{([\s\S]*?)\n\s{4,6}\}', r_block)
        if not cc_match: continue
        c_pts = extract_pts(cc_match.group(1))
        if c_pts is None or np.mean(c_pts[:, 0]) < x_min_limit: continue

        # 2. 提取边界
        l_pts, r_pts = None, None
        edges = re.findall(r'edge\s*\{([\s\S]*?)\s+type:\s+(\w+)', r_block)
        for e_content, e_type in edges:
            pts = extract_pts(e_content)
            if e_type == "LEFT_BOUNDARY": l_pts = pts
            elif e_type == "RIGHT_BOUNDARY": r_pts = pts

        # 插值平滑
        c_interp = interpolate_segment(c_pts, 0.2) # 采样稍稀疏一点方便计算
        l_interp = interpolate_segment(l_pts, 0.1)
        r_interp = interpolate_segment(r_pts, 0.1)

        # 3. 计算宽度 (中心点到左右边界距离之和)
        widths = []
        for p in c_interp:
            d_left = get_distance(p, l_interp) if l_interp is not None else 1.75 # 缺省半宽
            d_right = get_distance(p, r_interp) if r_interp is not None else 1.75
            widths.append(d_left + d_right)
        
        road_data_list.append({
            'center': c_interp,
            'widths': np.array(widths),
            'left': l_interp,
            'right': r_interp
        })

    if not road_data_list: return

    # 归一化原点
    all_c = np.vstack([d['center'] for d in road_data_list])
    origin = all_c[np.argmin(all_c[:, 0])]

    # --- 绘图 ---
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_facecolor('#1a1a1a')
    
    # 使用 LineCollection 实现宽度着色
    all_segments = []
    all_widths = []
    
    for rd in road_data_list:
        pts = rd['center'] - origin
        w = rd['widths']
        
        # 将点序列转为线段集合
        segments = np.concatenate([pts[:-1, np.newaxis, :], pts[1:, np.newaxis, :]], axis=1)
        all_segments.extend(segments)
        all_widths.extend(w[:-1])
        
        # 绘制边界线（细弱化）
        if rd['left'] is not None:
            lp = rd['left'] - origin
            ax.plot(lp[:,0], lp[:,1], color='white', lw=0.3, alpha=0.4)
        if rd['right'] is not None:
            rp = rd['right'] - origin
            ax.plot(rp[:,0], rp[:,1], color='white', lw=0.3, alpha=0.4)

    # 核心：根据宽度着色
    lc = LineCollection(all_segments, cmap='viridis', array=np.array(all_widths), 
                         linewidths=2.5, capstyle='round')
    line = ax.add_collection(lc)
    
    # 颜色条
    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label('Road Width (meters)', color='white')
    cbar.ax.yaxis.set_tick_params(color='white', labelcolor='white')

    ax.axis('equal')
    ax.set_title("Road Network with Width Heatmap", color='white', fontsize=15)
    plt.grid(True, color='gray', alpha=0.1)
    plt.show()

    # 打印一些统计信息
    avg_w = np.mean(all_widths)
    print(f"分析完成。平均车道宽度: {avg_w:.2f}m, 最大宽度: {np.max(all_widths):.2f}m")

if __name__ == "__main__":
    solve_with_width_v4('release_road.txt')