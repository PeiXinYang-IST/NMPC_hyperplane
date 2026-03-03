import re
import matplotlib.pyplot as plt
import numpy as np
import math

def visualize_imu_angular_velocity():
    # 定义文件名列表 (根据你提供的文件名)
    log_files = [
        '0.4log.txt',
        '0.35log.txt',
        '0.3log.txt',
        '0.25log.txt',
        '0.2log.txt',
        '0.15log.txt',
        '0.1log.txt'
    ]

    # 正则表达式匹配 ang[x, y, z]
    # 匹配方括号内的三个浮点数
    pattern = re.compile(r'ang\[\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)\]')

    # 设置绘图布局 (自动计算行数)
    num_files = len(log_files)
    cols = 2
    rows = math.ceil(num_files / cols)
    
    fig, axes = plt.subplots(rows, cols, figsize=(15, 4 * rows))
    axes = axes.flatten() # 展平数组以便遍历

    print("开始解析并绘图...")

    for i, filename in enumerate(log_files):
        ax = axes[i]
        ang_data = []

        try:
            with open(filename, 'r') as f:
                content = f.read()
                matches = pattern.findall(content)
                
                if not matches:
                    ax.text(0.5, 0.5, 'No Data Found', ha='center', va='center')
                    ax.set_title(filename)
                    continue

                # 将字符串转换为浮点数并存储
                for m in matches:
                    ang_data.append([float(m[0]), float(m[1]), float(m[2])])
            
            # 转换为 numpy 数组以便绘图 (Shape: N x 3)
            ang_np = np.array(ang_data)
            
            # 绘制三轴数据
            # X轴: 红色, Y轴: 绿色, Z轴: 蓝色
            ax.plot(ang_np[:, 0], label='Ang X', color='r', alpha=0.7, linewidth=1)
            ax.plot(ang_np[:, 1], label='Ang Y', color='g', alpha=0.7, linewidth=1)
            ax.plot(ang_np[:, 2], label='Ang Z', color='b', alpha=0.7, linewidth=1)
            
            ax.set_title(f'File: {filename} (Samples: {len(ang_np)})')
            ax.set_xlabel('Sample Index')
            ax.set_ylabel('Angular Velocity (rad/s)')
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.legend(loc='upper right', fontsize='small')
            
            # 计算并显示简单的统计信息（可选）
            mean_vals = np.mean(ang_np, axis=0)
            stats_text = f"Mean X: {mean_vals[0]:.3f}\nMean Y: {mean_vals[1]:.3f}\nMean Z: {mean_vals[2]:.3f}"
            # 将统计数据显示在图中
            ax.text(0.02, 0.95, stats_text, transform=ax.transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        except FileNotFoundError:
            ax.text(0.5, 0.5, 'File Not Found', ha='center', va='center')
            ax.set_title(filename)
        except Exception as e:
            print(f"Error parsing {filename}: {e}")

    # 隐藏多余的子图（如果有的话）
    for j in range(i + 1, len(axes)):
        axes[j].axis('off')

    plt.suptitle('IMU 3-Axis Angular Velocity Visualization', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.97]) # 调整布局以适应总标题
    plt.show()

if __name__ == "__main__":
    visualize_imu_angular_velocity()