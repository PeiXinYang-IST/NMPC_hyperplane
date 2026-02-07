import numpy as np
import bisect

class CubicSpline1D:
    """ 一维三次样条插值 """
    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted")
        
        self.a = y
        self.x = x
        self.nx = len(x)
        
        # 求解三对角矩阵构建样条系数 b, c, d
        A = np.zeros((self.nx, self.nx))
        B = np.zeros(self.nx)
        A[0, 0] = 1.0
        A[self.nx - 1, self.nx - 1] = 1.0
        
        for i in range(1, self.nx - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            B[i] = 3.0 * (y[i + 1] - y[i]) / h[i] - 3.0 * (y[i] - y[i - 1]) / h[i - 1]
            
        self.c = np.linalg.solve(A, B)
        self.b = np.zeros(self.nx - 1)
        self.d = np.zeros(self.nx - 1)
        
        for i in range(self.nx - 1):
            self.d[i] = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            self.b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * self.c[i] + self.c[i + 1]) / 3.0

    def calc_position(self, t):
        if t < self.x[0]: return None
        if t > self.x[-1]: return None
        
        i = bisect.bisect(self.x, t) - 1
        i = min(max(i, 0), self.nx - 2)
        dx = t - self.x[i]
        position = self.a[i] + self.b[i] * dx + self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return position

    def calc_first_derivative(self, t):
        i = bisect.bisect(self.x, t) - 1
        i = min(max(i, 0), self.nx - 2)
        dx = t - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, t):
        i = bisect.bisect(self.x, t) - 1
        i = min(max(i, 0), self.nx - 2)
        dx = t - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

class CubicSpline2D:
    """ 二维三次样条 (处理 x, y 坐标) """
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)
        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** 1.5)
        return k

    def calc_yaw(self, s):
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = np.arctan2(dy, dx)
        return yaw