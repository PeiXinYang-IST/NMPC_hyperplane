import os
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def export_race_model(n_obstacles=5):
    model_name = 'racing_control_hyperplane'
    
    # --- 状态变量 (States) ---
    x     = ca.SX.sym('x')      # 车辆在世界坐标系下的 X 位置 (m)
    y     = ca.SX.sym('y')      # 车辆在世界坐标系下的 Y 位置 (m)
    theta = ca.SX.sym('theta')  # 车辆的航向角 (rad)
    vx    = ca.SX.sym('vx')     # 车辆的纵向线速度 (m/s)
    omega = ca.SX.sym('omega')  # 车辆的角速度 (rad/s)
    state = ca.vertcat(x, y, theta, vx, omega)
    
    # --- 控制变量 (Controls) ---
    ax    = ca.SX.sym('ax')     # 纵向加速度 (m/s^2) - 对应油门/刹车
    alpha = ca.SX.sym('alpha')  # 角加速度 (rad/s^2) - 对应转向速率
    control = ca.vertcat(ax, alpha)

    # --- 运行时参数 (Runtime Parameters) ---
    # 这些参数在每一帧求解前可以动态更新，无需重新编译
    p = []
    obs_params = [] 
    
    for i in range(n_obstacles):
        ox  = ca.SX.sym(f'obs_x_{i}')   # 第 i 个障碍物的中心点 X 坐标
        oy  = ca.SX.sym(f'obs_y_{i}')   # 第 i 个障碍物的中心点 Y 坐标
        or_ = ca.SX.sym(f'obs_r_{i}')   # 第 i 个障碍物的膨胀安全半径 (包含车身半径)
        
        # 超平面参数：用于定义一条直线，将车辆与障碍物隔开
        # onx, ony 构成该平面的单位法向量 (Normal Vector)
        onx = ca.SX.sym(f'obs_nx_{i}')  
        ony = ca.SX.sym(f'obs_ny_{i}')  
        
        p.extend([ox, oy, or_, onx, ony])
        obs_params.append((ox, oy, or_, onx, ony))
    
    p_vert = ca.vertcat(*p)
    
    # --- 动力学方程 (Dynamics - ODE) ---
    # 定义状态的变化率 x_dot = f(x, u)
    rhs = ca.vertcat(
        vx * ca.cos(theta), # dx/dt: X轴位移速度
        vx * ca.sin(theta), # dy/dt: Y轴位移速度
        omega,              # dtheta/dt: 偏航角变化率
        ax,                 # dvx/dt: 加速度
        alpha               # domega/dt: 角加速度
    )
    
    model = AcadosModel()
    model.f_expl_expr = rhs # 显式常微分方程
    model.x = state
    model.u = control
    model.p = p_vert        # 将定义的参数传入模型
    model.name = model_name
    
    return model, n_obstacles

def setup_ocp(N_horizon=60, Tf=3.0):
    ocp = AcadosOcp()
    
    N_OBS = 4 # 设定最大支持 5 个障碍物
    model, _ = export_race_model(n_obstacles=N_OBS)
    ocp.model = model
    
    # --- 求解器时域配置 ---
    ocp.solver_options.N_horizon = N_horizon # 预测步数 (预测未来多远)
    ocp.solver_options.tf = Tf               # 预测总时长 (秒)
    nx = model.x.size1()
    
    # --- 代价函数配置 (Cost Function) ---
    # 使用非线性最小二乘 (Nonlinear Least Squares)
    ocp.cost.cost_type   = 'NONLINEAR_LS' # 运行步代价值
    ocp.cost.cost_type_0 = 'NONLINEAR_LS' # 初始步代价值
    ocp.cost.cost_type_e = 'NONLINEAR_LS' # 终点步代价值
    
    vx = model.x[3]
    omega = model.x[4]
    
    # 跟踪误差项: [x, y, vx, omega, ax, alpha, 乘向速度项]
    # vx * omega 项可以惩罚高速急转弯，增加平顺性
    cost_y_expr = ca.vertcat(model.x[0], model.x[1], vx, omega, model.u[0], model.u[1], vx * omega)
    ocp.model.cost_y_expr = cost_y_expr
    ocp.model.cost_y_expr_0 = cost_y_expr
    ocp.model.cost_y_expr_e = ca.vertcat(model.x[0], model.x[1], vx, omega)
    
    # 权重矩阵 W: 对应 cost_y_expr 中每一项的惩罚力度
    W = np.diag([10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    ocp.cost.W = W
    ocp.cost.W_0 = W
    ocp.cost.W_e = np.diag([10.0, 10.0, 0.1, 0.5]) # 终端权重
    
    # 期望参考值 (Reference): 实际运行时会从外部（如全局路径）更新
    ocp.cost.yref = np.zeros(7)
    ocp.cost.yref_0 = np.zeros(7)
    ocp.cost.yref_e = np.zeros(4)

    # --- 状态与控制约束 (Bounds) ---
    ocp.constraints.idxbx = np.array([3, 4])   # 针对 vx, omega 设置状态约束
    ocp.constraints.lbx = np.array([0.0, -2.5]) # 速度下限 0, 最小角速度 -2.5
    ocp.constraints.ubx = np.array([7.0, 2.5])  # 速度上限 7, 最大角速度 2.5
    
    ocp.constraints.idxbu = np.array([0, 1])   # 针对 ax, alpha 设置控制约束
    ocp.constraints.lbu = np.array([-1.0, -1.0]) # 最大刹车 -1.5, 最大转角速率 -0.75
    ocp.constraints.ubu = np.array([1.0, 1.0])  # 最大加速 1.5, 最大转角速率 0.75
    
    ocp.constraints.x0 = np.zeros(nx) # 初始状态占位

    # --- 超平面避障约束逻辑 (Hyperplane Obstacle Avoidance) ---
    # 原理：车辆必须位于由法向量 (nx, ny) 和障碍物中心 (ox, oy) 确定的直线的一侧
    p_sym = ocp.model.p
    x_sym = ocp.model.x[0]
    y_sym = ocp.model.x[1]
    h_list = []
    
    for i in range(N_OBS):
        ox  = p_sym[i*5 + 0]
        oy  = p_sym[i*5 + 1]
        or_ = p_sym[i*5 + 2] # 安全距离阈值
        onx = p_sym[i*5 + 3] # 超平面法向量 X
        ony = p_sym[i*5 + 4] # 超平面法向量 Y
        
        # 约束公式: onx * (车辆x - 障碍x) + ony * (车辆y - 障碍y) - 安全半径 >= 0
        # 几何意义：车辆在法向量方向上距离障碍物中心的投影距离必须大于半径
        h_list.append( onx * (x_sym - ox) + ony * (y_sym - oy) - or_ )
    
    ocp.model.con_h_expr = ca.vertcat(*h_list)
    
    # 避障约束边界：h >= 0
    ocp.constraints.lh = np.zeros(N_OBS)      # 下界为 0
    ocp.constraints.uh = np.ones(N_OBS) * 1e9 # 上界为无穷大
    
    # --- 软约束 (Soft Constraints) ---
    # 目的：防止避障约束过于严苛导致优化问题无解 (Infeasible)
    # 当车辆被迫违反约束时，允许微小违反但会加上极大的惩罚代价 (Zl, Zu)
    ocp.constraints.idxsh = np.arange(N_OBS) # 哪些不等式约束需要软化
    Z_val, z_val = 500.0, 500.0
    ocp.cost.zl = np.ones(N_OBS) * z_val # 线性惩罚 (下界)
    ocp.cost.zu = np.ones(N_OBS) * z_val # 线性惩罚 (上界)
    ocp.cost.Zl = np.ones(N_OBS) * Z_val # 二次惩罚 (下界)
    ocp.cost.Zu = np.ones(N_OBS) * Z_val # 二次惩罚 (上界)

    # --- 参数初始默认值 ---
    # 将障碍物默认放在很远的地方 (1000, 1000)，防止初始时刻误触发避障
    ocp.parameter_values = np.tile(np.array([1000.0, 1000.0, 1.0, 1.0, 0.0]), N_OBS)

    # --- 求解器算法配置 ---
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # 常用高性能 QP 求解器
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'        # 高斯-牛顿近似 (适用于最小二乘)
    ocp.solver_options.integrator_type = 'ERK'                 # 显式龙格-库塔积分器
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'             # 实时迭代 (Real-Time Iteration) 算法
    tol = 1e-6
    ocp.solver_options.qp_solver_tol_stat = tol
    ocp.solver_options.qp_solver_tol_eq   = tol
    ocp.solver_options.qp_solver_tol_ineq = tol
    ocp.solver_options.qp_solver_tol_comp = tol

    # 限制 QP 最大迭代次数 (简单场景5次够了，复杂场景10次强制截断)
    ocp.solver_options.qp_solver_iter_max = 10 

    ocp.solver_options.print_level = 0
    ocp.solver_options.num_threads = 4 
    return ocp

if __name__ == '__main__':
    # 设置 acados 源代码路径环境变量
    if 'ACADOS_SOURCE_DIR' not in os.environ:
        os.environ['ACADOS_SOURCE_DIR'] = '/mnt/c/Users/yang/Downloads/acados'

    print("=== 开始生成 Acados C 代码 (Hyperplane Version) ===")
    ocp = setup_ocp()
    # 实例化求解器，生成 C 代码到 'c_generated_code' 文件夹
    solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("\n[Success] 代码生成成功，参数维度已更新为 N_OBS * 5")