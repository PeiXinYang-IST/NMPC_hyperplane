import re
import matplotlib.pyplot as plt

def plot_rtk_data(file_path):
    v_gprmc_values = []
    v_diff_values = []
    timestamps = []

    # 正则表达式匹配逻辑
    # 匹配 [INFO] 后的时间戳和 v_gprmc, v_diff 的值
    # 示例: [INFO] [1770803857.449816906] ... v_gprmc[0.898], v_diff[1.052]
    info_pattern = re.compile(r"\[INFO\]\s+\[([\d.]+)\]")
    val_pattern = re.compile(r"v_gprmc\[([\d.]+)\]")
    diff_pattern = re.compile(r"v_diff\[([\d.]+)\]")

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                # 只处理包含 [INFO] 的行以避免重复提取
                if "[INFO]" in line:
                    time_match = info_pattern.search(line)
                    v_match = val_pattern.search(line)
                    d_match = diff_pattern.search(line)
                    
                    if v_match and d_match:
                        # 提取数值
                        v_gprmc = float(v_match.group(1))
                        v_diff = float(d_match.group(1))
                        
                        v_gprmc_values.append(v_gprmc)
                        v_diff_values.append(v_diff)
                        
                        # 如果有时间戳则记录，否则记录索引
                        if time_match:
                            timestamps.append(float(time_match.group(1)))
                        else:
                            timestamps.append(len(v_gprmc_values))

        if not v_gprmc_values:
            print("未在文件中找到匹配的数据，请检查格式。")
            return

        # 数据归一化时间轴（从0开始）
        start_time = timestamps[0]
        relative_time = [t - start_time for t in timestamps]

        # 绘图
        plt.figure(figsize=(12, 6))
        plt.plot(relative_time, v_gprmc_values, label='$v_{gprmc}$', marker='.', markersize=4, linestyle='-')
        plt.plot(relative_time, v_diff_values, label='$v_{diff}$', marker='x', markersize=4, linestyle='--')

        plt.title('RTK Driver Velocity Visualization', fontsize=14)
        plt.xlabel('Relative Time (s)', fontsize=12)
        plt.ylabel('Velocity Value', fontsize=12)
        plt.legend()
        plt.grid(True, which='both', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.show()
        # 如果需要保存图片，可以取消下面这一行的注释
        # plt.savefig('velocity_plot.png')

    except FileNotFoundError:
        print(f"错误：找不到文件 '{file_path}'")
    except Exception as e:
        print(f"发生错误：{e}")

if __name__ == "__main__":
    # 在此处替换你的文件名
    plot_rtk_data('log1.5.txt')