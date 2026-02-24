import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter # 导入平滑滤波工具

# 文件路径
filename = 'log.txt'
output_filename = 'path.txt' # 直接保存为 ROS 节点读取的文件名

# 1. 解析日志文件
x_coords = []
y_coords = []
# 优化正则表达式：增加 -? 以支持负数坐标
pattern = re.compile(r"x\[(-?[\d\.]+)\].*?y\[(-?[\d\.]+)\]")

try:
    with open(filename, 'r') as file:
        for line in file:
            match = pattern.search(line)
            if match:
                try:
                    x_val = float(match.group(1))
                    y_val = float(match.group(2))
                    
                    # 过滤初始化点，同时允许负数坐标（根据实际需求调整阈值）
                    if abs(x_val) > 0.001 or abs(y_val) > 0.001:
                        x_coords.append(x_val)
                        y_coords.append(y_val)
                except ValueError:
                    continue

    if len(x_coords) < 10: # 点太少无法平滑
        print("Valid coordinates not enough for smoothing.")
    else:
        x = np.array(x_coords)
        y = np.array(y_coords)

        # 2. 插值 (Interpolate to 0.05m)
        dx = np.diff(x)
        dy = np.diff(y)
        distances = np.sqrt(dx**2 + dy**2)
        cum_dist = np.concatenate(([0], np.cumsum(distances)))
        total_length = cum_dist[-1]

        f_x = interp1d(cum_dist, x, kind='linear')
        f_y = interp1d(cum_dist, y, kind='linear')

        step_size = 0.05
        new_distances = np.arange(0, total_length, step_size)
        x_interp = f_x(new_distances)
        y_interp = f_y(new_distances)

        # 3. 平滑处理 (Savitzky-Golay Filter)
        # window_length: 窗口长度（必须是奇数），越大越平滑
        # polyorder: 多项式拟合阶数，通常取 2 或 3
        window_size = 11 if len(x_interp) > 11 else 5 
        x_smooth = savgol_filter(x_interp, window_size, 3)
        y_smooth = savgol_filter(y_interp, window_size, 3)

        # 4. 转换至局部坐标系
        x_origin, y_origin = x_smooth[0], y_smooth[0]
        x_local = x_smooth - x_origin
        y_local = y_smooth - y_origin

        # 5. 绘图对比
        plt.figure(figsize=(10, 6))
        plt.plot(x - x_origin, y - y_origin, 'k.', alpha=0.2, label='Original Raw')
        plt.plot(x_local, y_local, 'b-', label='Smoothed & Interpolated', linewidth=2)
        plt.plot(0, 0, 'ro', label='Start')
        plt.title('Path Smoothing Comparison')
        plt.xlabel('X (Local) [m]')
        plt.ylabel('Y (Local) [m]')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig('path_comparison.png')
        print("Plot saved as path_comparison.png")

        # 6. 保存到文件
        # 注意：为了避免 ROS 节点报错，我们不在第一行写中文或复杂字符
        with open(output_filename, 'w') as f:
            f.write("# x_local, y_local\n") # 使用 # 开头，方便 ROS 节点跳过
            for xi, yi in zip(x_local, y_local):
                f.write(f"{xi:.4f},{yi:.4f}\n")

        print(f"Success! Saved to {output_filename}")
        print(f"Origin: ({x_origin:.4f}, {y_origin:.4f})")

except FileNotFoundError:
    print(f"Error: '{filename}' not found.")