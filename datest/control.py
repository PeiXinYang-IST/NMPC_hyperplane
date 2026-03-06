import graphviz

def draw_control_loop():
    dot = graphviz.Digraph('NMPC_ESO_Loop', comment='Focus on Control Logic')
    dot.attr(rankdir='LR', size='10,5')
    dot.attr('node', shape='box', style='filled', fillcolor='white', fontname='Microsoft YaHei')

    # 1. NMPC 控制器 (产生标称控制量)
    dot.node('NMPC', 'NMPC Solver\n(求解标称控制量 u_nom)', fillcolor='#E3F2FD')

    # 2. 补偿节点
    dot.node('Sum', 'Σ', shape='circle', fillcolor='#FFF9C4')

    # 3. 机器人实体 (受扰动影响)
    dot.node('Robot', 'Robot / Plant\n(x_dot = f(x, u) + d)', fillcolor='#E8F5E9')

    # 4. ESO 观测器
    dot.node('ESO', 'LESO (Linear ESO)\n估计总扰动 d_hat', fillcolor='#FCE4EC')

    # 5. 延迟单元 (用于 ESO 的控制输入反馈)
    dot.node('Delay', 'Unit Delay\n(z⁻¹)', shape='plaintext')

    # 连线关系
    # NMPC 输出到求和点
    dot.edge('NMPC', 'Sum', 'u_nom (ax, w)')
    
    # 补偿后的指令输入机器人
    dot.edge('Sum', 'Robot', 'Final u_cmd\n(cmd_vel)')
    
    # 闭环反馈：状态反馈给 ESO
    dot.edge('Robot', 'ESO', 'Measured y\n(vx, omega)', constraint='true')
    
    # 闭环反馈：控制指令反馈给 ESO
    dot.edge('Sum', 'Delay')
    dot.edge('Delay', 'ESO', 'u_{k-1}')
    
    # ESO 输出扰动项到求和点 (通常是减去扰动或加补偿)
    dot.edge('ESO', 'Sum', 'Compensation\n(-d_hat / b0)', color='red', fontcolor='red')

    return dot

# 渲染
diagram = draw_control_loop()
diagram.render('control_compensation_loop', format='png', cleanup=True)