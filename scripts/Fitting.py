import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

measured_cmds = np.array([
    -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0
])
measured_vels = np.array([
    -1.8, -1.4, -0.95, -0.48, 0.0, 0.48, 0.95, 1.4, 1.8
])

def model_poly2(x, a, b, c):
    return a * x**2 + b * x + c

def model_odd_cubic(x, a, b):
    return a * x**3 + b * x

def get_piecewise_gain(cmd, vel):
    # 正向增益
    pos_mask = cmd > 0.01
    k_pos = np.mean(vel[pos_mask] / cmd[pos_mask]) if np.any(pos_mask) else 1.0
    # 反向增益
    neg_mask = cmd < -0.01
    k_neg = np.mean(vel[neg_mask] / cmd[neg_mask]) if np.any(neg_mask) else 1.0
    return k_pos, k_neg


# 拟合 A
popt2, _ = curve_fit(model_poly2, measured_cmds, measured_vels)
# 拟合 B
popt_odd, _ = curve_fit(model_odd_cubic, measured_cmds, measured_vels)
# 拟合 C
k_pos, k_neg = get_piecewise_gain(measured_cmds, measured_vels)

print(">>> 模型对比分析 <<<")

print(f"[1] 二阶模型 (ax^2+bx+c):")
print(f"    公式: y = {popt2[0]:.3f}x² + {popt2[1]:.3f}x + {popt2[2]:.3f}")
print(f"    问题: x=0时速度为{popt2[2]:.3f} (漂移); x²项导致正负不对称")

print(f"\n[2] 物理模型 (ax^3+bx) [推荐]:")
print(f"    公式: y = {popt_odd[0]:.4f}x³ + {popt_odd[1]:.3f}x")
print(f"    物理含义: 线性增益 {popt_odd[1]:.3f}, 高速阻力/饱和系数 {popt_odd[0]:.4f}")

print(f"\n[3] 分段线性:")
print(f"    正向增益: {k_pos:.3f}, 反向增益: {k_neg:.3f}")

x_line = np.linspace(min(measured_cmds), max(measured_cmds), 100)

plt.figure(figsize=(10, 6))
plt.scatter(measured_cmds, measured_vels, c='k', label='Measured Data', zorder=5)

plt.plot(x_line, model_poly2(x_line, *popt2), 'g--', label='Poly2 (ax^2+bx+c)')
plt.plot(x_line, model_odd_cubic(x_line, *popt_odd), 'r-', linewidth=2, label='Odd Cubic (ax^3+bx)')

plt.axhline(0, color='gray', alpha=0.3)
plt.axvline(0, color='gray', alpha=0.3)
plt.title("Why x^2 is usually wrong for velocity mapping")
plt.xlabel("Command")
plt.ylabel("Velocity")
plt.legend()
plt.grid(True)
plt.show()