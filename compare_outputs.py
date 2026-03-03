#!/usr/bin/env python3
"""
对比两个NMPC日志输出文件的数据
"""

import re
import sys
from pathlib import Path


def parse_log_file(filepath):
    """解析日志文件，提取NMPC节点的数据"""
    data = {
        'time_tot': [],
        'time_plan': [],
        'iter': [],
        'velocity': [],
        'angular_velocity': [],
        'acc_raw': [],
        'acc_comp': [],
        'eso': [],
        'cte': []
    }

    # 正则表达式匹配NMPC节点的日志行
    pattern = r'TIME\[Tot:([\d.]+) Plan:([\d.]+)\].*Iter:(\d+).*V:([\d.]+).*W:([\d.]+).*Acc_Raw:([\d.]+).*Acc_Comp:([\d.]+).*ESO:([-\d.]+).*CTE:([\d.]+)'

    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            match = re.search(pattern, line)
            if match:
                data['time_tot'].append(float(match.group(1)))
                data['time_plan'].append(float(match.group(2)))
                data['iter'].append(int(match.group(3)))
                data['velocity'].append(float(match.group(4)))
                data['angular_velocity'].append(float(match.group(5)))
                data['acc_raw'].append(float(match.group(6)))
                data['acc_comp'].append(float(match.group(7)))
                data['eso'].append(float(match.group(8)))
                data['cte'].append(float(match.group(9)))

    return data


def calculate_stats(values):
    """计算统计信息"""
    if not values:
        return {'count': 0, 'mean': 0, 'min': 0, 'max': 0, 'std': 0}

    import statistics
    return {
        'count': len(values),
        'mean': statistics.mean(values),
        'min': min(values),
        'max': max(values),
        'std': statistics.stdev(values) if len(values) > 1 else 0
    }


def compare_data(data1, data2, name1='File 1', name2='File 2'):
    """对比两组数据并打印结果"""
    print("=" * 80)
    print(f"数据对比: {name1} vs {name2}")
    print("=" * 80)

    metrics = [
        ('time_tot', '总时间 (ms)'),
        ('time_plan', '规划时间 (ms)'),
        ('iter', '迭代次数'),
        ('velocity', '速度 (m/s)'),
        ('angular_velocity', '角速度 (rad/s)'),
        ('acc_raw', '原始加速度'),
        ('acc_comp', '补偿加速度'),
        ('eso', 'ESO'),
        ('cte', '横向跟踪误差')
    ]

    results = []

    for key, label in metrics:
        stats1 = calculate_stats(data1[key])
        stats2 = calculate_stats(data2[key])

        diff_mean = stats2['mean'] - stats1['mean']
        diff_pct = (diff_mean / stats1['mean'] * 100) if stats1['mean'] != 0 else 0

        results.append({
            'label': label,
            'mean1': stats1['mean'],
            'mean2': stats2['mean'],
            'diff': diff_mean,
            'diff_pct': diff_pct,
            'max1': stats1['max'],
            'max2': stats2['max']
        })

    # 打印表格
    print(f"\n{'指标':<20} {name1:<20} {name2:<20} {'差值':<15} {'变化率':<10}")
    print("-" * 85)

    for r in results:
        print(f"{r['label']:<20} {r['mean1']:<20.4f} {r['mean2']:<20.4f} "
              f"{r['diff']:<15.4f} {r['diff_pct']:>+.2f}%")

    print("\n" + "=" * 80)
    print("最大值对比:")
    print("-" * 85)
    for r in results:
        print(f"{r['label']:<20} {r['max1']:<20.4f} {r['max2']:<20.4f}")

    # 数据点数
    print("\n" + "=" * 80)
    print(f"数据点数: {len(data1['time_tot'])} vs {len(data2['time_tot'])}")

    return results


def plot_comparison(data1, data2, name1='File 1', name2='File 2'):
    """绘制对比图表"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        fig, axes = plt.subplots(3, 3, figsize=(15, 12))
        fig.suptitle(f'数据对比: {name1} vs {name2}', fontsize=14)

        metrics = [
            ('time_tot', '总时间 (ms)'),
            ('time_plan', '规划时间 (ms)'),
            ('iter', '迭代次数'),
            ('velocity', '速度 (m/s)'),
            ('acc_raw', '原始加速度'),
            ('acc_comp', '补偿加速度'),
            ('eso', 'ESO'),
            ('cte', '横向跟踪误差')
        ]

        for idx, (key, label) in enumerate(metrics):
            ax = axes[idx // 3, idx % 3]
            x1 = range(len(data1[key]))
            x2 = range(len(data2[key]))

            ax.plot(x1, data1[key], label=name1, alpha=0.7)
            ax.plot(x2, data2[key], label=name2, alpha=0.7)
            ax.set_title(label)
            ax.legend()
            ax.grid(True, alpha=0.3)

        # 隐藏最后一个空的子图
        axes[2, 2].axis('off')

        plt.tight_layout()
        plt.savefig('comparison_plot.png', dpi=150)
        print("\n图表已保存到 comparison_plot.png")
        plt.close()

    except ImportError:
        print("\n[提示] 如需生成图表，请安装 matplotlib: pip install matplotlib")


def main():
    # 文件路径
    file1 = 'output.txt'
    file2 = 'outputeso.txt'

    # 尝试多个可能的位置
    paths = [
        Path(file1),
        Path(__file__).parent / file1,
        Path.cwd() / file1,
    ]

    for p in paths:
        if p.exists():
            file1 = str(p)
            break

    paths2 = [
        Path(file2),
        Path(__file__).parent / file2,
        Path.cwd() / file2,
    ]

    for p in paths2:
        if p.exists():
            file2 = str(p)
            break

    print(f"读取文件 1: {file1}")
    print(f"读取文件 2: {file2}")

    # 解析数据
    print("\n正在解析文件...")
    data1 = parse_log_file(file1)
    data2 = parse_log_file(file2)

    print(f"文件1 解析到 {len(data1['time_tot'])} 条记录")
    print(f"文件2 解析到 {len(data2['time_tot'])} 条条记录")

    # 对比数据
    results = compare_data(data1, data2, Path(file1).name, Path(file2).name)

    # 绘制图表
    plot_comparison(data1, data2, Path(file1).name, Path(file2).name)


if __name__ == '__main__':
    main()
