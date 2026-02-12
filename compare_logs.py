#!/usr/bin/env python3
"""
对比 orinlog.txt 和 mylog.txt 中的 v step 曲率 总耗时 plan耗时 数据
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def parse_log_file(filepath):
    """解析日志文件，提取 V, Step, 曲率, 总耗时(Tot), plan耗时(Plan)"""
    data = {
        'time': [],
        'v': [],
        'step': [],
        'curvature': [],
        'time_total': [],
        'time_plan': []
    }

    # 匹配: TIME[Tot:9.4 Plan:0.7] ... V:0.00 ... Crv:0.0 Step:0.25
    pattern = r'TIME\[Tot:([0-9.]+) Plan:([0-9.]+)\].*V:([0-9.]+).*Crv:([0-9.-]+).*Step:([0-9.]+)'

    with open(filepath, 'r') as f:
        for line in f:
            match = re.search(pattern, line)
            if match:
                data['time_total'].append(float(match.group(1)))
                data['time_plan'].append(float(match.group(2)))
                data['v'].append(float(match.group(3)))
                data['curvature'].append(float(match.group(4)))
                data['step'].append(float(match.group(5)))
                # 使用索引作为时间
                data['time'].append(len(data['time']))

    return data

def plot_comparison(data_orin, data_mine, output_path=None):
    """绘制对比图"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Orin vs MyLog 对比分析', fontsize=14, fontweight='bold')

    # 统一时间轴（取较短的长度）
    min_len = min(len(data_orin['v']), len(data_mine['v']))
    time_orin = np.arange(min_len)
    time_mine = np.arange(min_len)

    # 1. V对比
    ax1 = axes[0, 0]
    ax1.plot(time_orin, data_orin['v'][:min_len], 'b-', label='Orin', alpha=0.8)
    ax1.plot(time_mine, data_mine['v'][:min_len], 'r--', label='MyLog', alpha=0.8)
    ax1.set_xlabel('Frame')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('速度 V 对比')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. 曲率对比
    ax2 = axes[0, 1]
    ax2.plot(time_orin, data_orin['curvature'][:min_len], 'b-', label='Orin', alpha=0.8)
    ax2.plot(time_mine, data_mine['curvature'][:min_len], 'r--', label='MyLog', alpha=0.8)
    ax2.set_xlabel('Frame')
    ax2.set_ylabel('Curvature')
    ax2.set_title('曲率 Curvature 对比')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Step对比
    ax3 = axes[0, 2]
    ax3.plot(time_orin, data_orin['step'][:min_len], 'b-', label='Orin', alpha=0.8)
    ax3.plot(time_mine, data_mine['step'][:min_len], 'r--', label='MyLog', alpha=0.8)
    ax3.set_xlabel('Frame')
    ax3.set_ylabel('Step')
    ax3.set_title('步长 Step 对比')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. 总耗时对比
    ax4 = axes[1, 0]
    ax4.plot(time_orin, data_orin['time_total'][:min_len], 'b-', label='Orin', alpha=0.8)
    ax4.plot(time_mine, data_mine['time_total'][:min_len], 'r--', label='MyLog', alpha=0.8)
    ax4.set_xlabel('Frame')
    ax4.set_ylabel('Total Time (ms)')
    ax4.set_title('总耗时 TIME[Tot] 对比')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. Plan耗时对比
    ax5 = axes[1, 1]
    ax5.plot(time_orin, data_orin['time_plan'][:min_len], 'b-', label='Orin', alpha=0.8)
    ax5.plot(time_mine, data_mine['time_plan'][:min_len], 'r--', label='MyLog', alpha=0.8)
    ax5.set_xlabel('Frame')
    ax5.set_ylabel('Plan Time (ms)')
    ax5.set_title('Plan耗时 TIME[Plan] 对比')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 6. Plan耗时占比对比
    ax6 = axes[1, 2]
    ratio_orin = np.array(data_orin['time_plan'][:min_len]) / np.array(data_orin['time_total'][:min_len]) * 100
    ratio_mine = np.array(data_mine['time_plan'][:min_len]) / np.array(data_mine['time_total'][:min_len]) * 100
    ax6.plot(time_orin, ratio_orin, 'b-', label='Orin', alpha=0.8)
    ax6.plot(time_mine, ratio_mine, 'r--', label='MyLog', alpha=0.8)
    ax6.set_xlabel('Frame')
    ax6.set_ylabel('Plan Ratio (%)')
    ax6.set_title('Plan耗时占比对比')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"图片已保存到: {output_path}")

    return fig

def print_statistics(data_orin, data_mine):
    """打印统计信息"""
    print("=" * 60)
    print("数据统计对比")
    print("=" * 60)

    min_len = min(len(data_orin['v']), len(data_mine['v']))

    metrics = [
        ('V (速度)', 'v'),
        ('曲率', 'curvature'),
        ('Step (步长)', 'step'),
        ('总耗时 Tot', 'time_total'),
        ('Plan耗时', 'time_plan')
    ]

    for name, key in metrics:
        orin_data = np.array(data_orin[key][:min_len])
        mine_data = np.array(data_mine[key][:min_len])

        print(f"\n{name}:")
        print(f"  Orin   - Mean: {np.mean(orin_data):.4f}, Std: {np.std(orin_data):.4f}, Max: {np.max(orin_data):.4f}, Min: {np.min(orin_data):.4f}")
        print(f"  MyLog  - Mean: {np.mean(mine_data):.4f}, Std: {np.std(mine_data):.4f}, Max: {np.max(mine_data):.4f}, Min: {np.min(mine_data):.4f}")
        print(f"  差异   - Mean Diff: {np.mean(np.abs(orin_data - mine_data)):.4f}")

    # Plan占比统计
    print(f"\nPlan耗时占比 (%):")
    ratio_orin = np.array(data_orin['time_plan'][:min_len]) / np.array(data_orin['time_total'][:min_len]) * 100
    ratio_mine = np.array(data_mine['time_plan'][:min_len]) / np.array(data_mine['time_total'][:min_len]) * 100
    print(f"  Orin   - Mean: {np.mean(ratio_orin):.2f}%")
    print(f"  MyLog  - Mean: {np.mean(ratio_mine):.2f}%")

def main():
    # 文件路径
    base_path = Path(__file__).parent
    orin_log = base_path / 'orinlog_opt52.txt'
    my_log = base_path / 'mylog.txt'
    output_fig = base_path / 'log_comparison.png'

    print(f"读取文件: {orin_log}")
    print(f"读取文件: {my_log}")

    # 解析日志
    data_orin = parse_log_file(orin_log)
    data_mine = parse_log_file(my_log)

    print(f"\nOrin数据点数: {len(data_orin['v'])}")
    print(f"MyLog数据点数: {len(data_mine['v'])}")

    # 打印统计
    print_statistics(data_orin, data_mine)

    # 绘图
    plot_comparison(data_orin, data_mine, str(output_fig))

    print(f"\n对比完成！")

if __name__ == '__main__':
    main()
