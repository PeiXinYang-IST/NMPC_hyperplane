import os
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

def export_kinematic_model():
    """
    导出 3-DOF 运动学单车模型 (Kinematic Bicycle Model)
    状态: [x, y, theta]
    控制: [v_cmd, w_cmd]
    """
    model_name = 'racing_control_kinematic'

    # --- 1. 状态变量 (States) ---
    x     = ca.SX.sym('x')      # 世界坐标系 X (m)
    y     = ca.SX.sym('y')      # 世界坐标系 Y (m)
    theta = ca.SX.sym('theta')  # 航向角 (rad)
    state = ca.vertcat(x, y, theta)

    # --- 2. 控制变量 (Controls) ---
    # 在运动学模型中，速度和角速度直接作为控制量输入给底层驱动
    v_cmd = ca.SX.sym('v_cmd')  # 线速度指令 (m/s)
    w_cmd = ca.SX.sym('w_cmd')  # 角速度指令 (rad/s)
    control = ca.vertcat(v_cmd, w_cmd)

    # --- 3. 动力学方程 (Kinematics) ---
    # x_dot = v * cos(theta)
    # y_dot = v * sin(theta)
    # theta_dot = w
    rhs = ca.vertcat(
        v_cmd * ca.cos(theta),
        v_cmd * ca.sin(theta),
        w_cmd
    )

    # --- 4. 构建 Acados 模型 ---
    model = AcadosModel()
    model.f_expl_expr = rhs
    model.x = state
    model.u = control
    model.name = model_name
    
    # 运动学 Tracker 不需要额外的障碍物参数，保持参数为空即可
    # 如果需要在线动态调参(如权重)，可以在这里定义 p
    model.p = [] 
    
    return model

def setup_tracker_ocp(N_horizon=40, Tf=2.0):
    """
    配置 NMPC 求解器
    默认频率 50Hz (dt = 0.4 / 20 = 0.02s)
    """
    ocp = AcadosOcp()
    model = export_kinematic_model()
    ocp.model = model

    # --- 1. 时域配置 ---
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = Tf
    
    # --- 2. 代价函数 (Cost Function) ---
    # 目标：最小化与参考轨迹的误差 (x, y, theta) 以及控制量的平滑度
    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # 定义残差向量 y = [x, y, theta, v_cmd, w_cmd]
    # 我们希望 x->x_ref, y->y_ref, theta->theta_ref, v->v_ref, w->w_ref
    cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr = cost_y_expr
    ocp.model.cost_y_expr_0 = cost_y_expr
    
    # 终端只关心状态
    ocp.model.cost_y_expr_e = model.x

    # --- 3. 权重矩阵 (Weights) ---
    # Tracker 的核心任务是位置跟踪，因此 Position 权重最大
    # Inputs (v, w) 的权重用于正则化，防止指令剧烈跳变
    
    #               x     y     th    v    w
    W_diag = np.array([50.0, 50.0, 10.0, 1.0, 1.0])
    
    ocp.cost.W = np.diag(W_diag)
    ocp.cost.W_0 = np.diag(W_diag)
    ocp.cost.W_e = np.diag([50.0, 50.0, 10.0]) # 终端权重

    # 参考值 (Reference) 初始化为 0，实际运行时会由 C++ 节点更新
    nx = 3
    nu = 2
    ny = nx + nu
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_0 = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # --- 4. 约束条件 (Constraints) ---
    # 限制物理执行机构的能力
    
    # 速度限制 v_cmd
    ocp.constraints.idxbx = np.array([]) # 状态无硬约束（让代价函数去拉回）
    
    # 控制限制 [v_cmd, w_cmd]
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([0.0, -2.5]) # 最小速度 0, 最大右转 -2.5
    ocp.constraints.ubu = np.array([7.3,  2.5]) # 最大速度 6, 最大左转 2.5
    
    ocp.constraints.x0 = np.zeros(nx)

    # --- 5. 求解器选项 ---
    # 追求极致速度，使用 SQP_RTI (Real Time Iteration)
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    
    # 容差设置 (Tracker 不需要像 Planner 那么高的精度，速度优先)
    tol = 1e-4
    ocp.solver_options.qp_solver_tol_stat = tol
    ocp.solver_options.qp_solver_tol_eq   = tol
    ocp.solver_options.qp_solver_tol_ineq = tol
    ocp.solver_options.qp_solver_tol_comp = tol
    
    # 预测步数较少，HPIPM 迭代次数可以设低一点
    ocp.solver_options.qp_solver_iter_max = 5
    
    ocp.solver_options.print_level = 0
    
    return ocp

if __name__ == '__main__':
    # 自动设置路径 (如果需要)
    # if 'ACADOS_SOURCE_DIR' not in os.environ:
    #     os.environ['ACADOS_SOURCE_DIR'] = '/path/to/acados'

    print("=== 开始生成 3-DOF Kinematic Tracker 代码 ===")
    
    # 建议配置：N=20, Tf=0.4 (dt=0.02s / 50Hz)
    ocp = setup_tracker_ocp(N_horizon=40, Tf=2.0)
    
    # 生成文件名区分
    json_file = 'acados_tracker_kinematic.json'
    
    solver = AcadosOcpSolver(ocp, json_file=json_file)
    
    print(f"\n[Success] 生成成功！")
    print(f"  - 模型名称: {ocp.model.name}")
    print(f"  - 预测时域: Tf={ocp.solver_options.tf}s, N={ocp.solver_options.N_horizon}")
    print(f"  - 状态维度: 3 (x, y, theta)")
    print(f"  - 控制维度: 2 (v, w)")
    print(f"请在 C++ 中加载生成的库，并确保 dt={ocp.solver_options.tf/ocp.solver_options.N_horizon} 与控制周期匹配。")