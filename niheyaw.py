import re
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression

def main():
    # 1. 定义文件名与对应的 cmd_vel 值
    file_map = {
        'yaw0.4log.txt': 0.4,
        'yaw0.5log.txt': 0.5,
        'yaw0.6log.txt': 0.6,
        'yaw0.7log.txt': 0.7,
        'yaw0.8log.txt': 0.8,
        'yaw0.9log.txt': 0.9,
        'yaw1.0log.txt': 1.0
    }

    # 2. 初始化数据
    # x_fit_data: 用于拟合的 x (cmd_vel)
    # y_fit_data: 用于拟合的 y (平均角速度)
    # dist_data:  用于画分布图的原始数据列表 [[v1, v2...], [v1, v2...] ...]
    # dist_positions: 分布图对应的 x 轴位置
    
    x_fit_data = [0.33] # 死区点
    y_fit_data = [0.0]  # 死区点
    
    # 存储每个文件的所有速度数据，用于画分布图
    # 预存死区数据 (模拟一些 0 的点以显示分布，或者仅作为单点)
    dist_data = [[0.0]] 
    dist_positions = [0.33]

    # 正则表达式
    pattern = re.compile(r"IMU: ang\[.*?,.*?,(.*?)\]")

    print("开始解析日志文件...")

    # 3. 循环读取文件
    # 按 cmd_vel 大小排序处理，保证画图顺序
    sorted_files = sorted(file_map.items(), key=lambda item: item[1])

    for filename, cmd_val in sorted_files:
        velocities = []
        try:
            with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
                for line in f:
                    match = pattern.search(line)
                    if match:
                        try:
                            val = float(match.group(1))
                            velocities.append(val)
                        except ValueError:
                            continue
            
            if velocities:
                # 1. 计算平均值用于拟合
                avg_vel = np.mean(velocities)
                std_vel = np.std(velocities)
                
                x_fit_data.append(cmd_val)
                y_fit_data.append(avg_vel)
                
                # 2. 存储所有数据用于画分布图
                dist_data.append(velocities)
                dist_positions.append(cmd_val)

                print(f"文件 {filename}: cmd={cmd_val}, mean={avg_vel:.4f}, std={std_vel:.4f}, points={len(velocities)}")
            else:
                print(f"警告: 文件 {filename} 中未找到有效数据")
                
        except FileNotFoundError:
            print(f"错误: 找不到文件 {filename}")

    # 4. 线性回归拟合 (使用平均值)
    if len(x_fit_data) > 1:
        X = np.array(x_fit_data).reshape(-1, 1)
        y = np.array(y_fit_data)

        model = LinearRegression()
        model.fit(X, y)

        slope = model.coef_[0]
        intercept = model.intercept_
        r_sq = model.score(X, y)

        print("\n" + "="*30)
        print("拟合完成！")
        print(f"线性方程: y = {slope:.4f} * x + {intercept:.4f}")
        print(f"拟合优度 (R^2): {r_sq:.4f}")
        print("="*30)

        # 5. 绘图：分布图 + 拟合线
        plt.figure(figsize=(12, 7))

        # --- A. 绘制箱线图 (分布) ---
        # positions 指定箱子在 X 轴的位置，widths 指定箱子宽度
        # patch_artist=True 允许填充颜色
        box = plt.boxplot(dist_data, positions=dist_positions, widths=0.04, 
                          patch_artist=True, 
                          boxprops=dict(facecolor="lightblue", color="blue"),
                          medianprops=dict(color="red"))
        
        # --- B. 绘制拟合线 ---
        x_range = np.linspace(min(x_fit_data), max(x_fit_data), 100).reshape(-1, 1)
        y_pred = model.predict(x_range)
        plt.plot(x_range, y_pred, color='green', linewidth=2, linestyle='--', 
                 label=f'Fit: y={slope:.2f}x + {intercept:.2f}')

        # --- C. 绘制平均值点 ---
        plt.scatter(x_fit_data, y_fit_data, color='black', zorder=5, label='Mean Value')

        plt.title(f'Cmd_Vel vs Actual Yaw Velocity Distribution\n(Slope={slope:.3f}, Bias={intercept:.3f})')
        plt.xlabel('Command Velocity (cmd_vel)')
        plt.ylabel('Actual Yaw Velocity (rad/s)')
        plt.legend()
        plt.grid(True, linestyle=':', alpha=0.6)
        
        # 调整 X 轴刻度，确保显示所有 cmd_vel
        plt.xticks(dist_positions)
        
        plt.tight_layout()
        plt.show()

    else:
        print("数据点不足，无法拟合。")

if __name__ == "__main__":
    main()