import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 初始化参数
L_c = 5.0  # 卡车长度
L_t = 10.0  # 拖车长度
v_c = 5.0  # 卡车速度 (m/s)
v_t = 5.0  # 拖车速度 (m/s)
delta_c = 0.1  # 卡车转角 (rad)

# 初始位置和方向
x_c, y_c, theta_c = 0.0, 0.0, np.pi / 4  # 卡车位置与方向
x_t, y_t, theta_t = x_c - L_c * np.cos(theta_c), y_c - L_c * np.sin(theta_c), theta_c - delta_c  # 拖车初始位置与方向

# 计算运动学方程
def kinematics(x_c, y_c, theta_c, x_t, y_t, theta_t, v_c, v_t, delta_c, dt=0.1):
    # 卡车更新
    x_c_new = x_c + v_c * np.cos(theta_c) * dt
    y_c_new = y_c + v_c * np.sin(theta_c) * dt
    theta_c_new = theta_c + v_c * np.tan(delta_c) / L_c * dt
    
    # 拖车更新
    theta_t_new = theta_c_new - delta_c
    x_t_new = x_c_new - L_t * np.cos(theta_t_new)
    y_t_new = y_c_new - L_t * np.sin(theta_t_new)
    
    return x_c_new, y_c_new, theta_c_new, x_t_new, y_t_new, theta_t_new

# 动画更新函数
fig, ax = plt.subplots()
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)

truck_line, = ax.plot([], [], 'b-', label="卡车")
trailer_line, = ax.plot([], [], 'r-', label="拖车")
ax.legend()

def update(frame):
    global x_c, y_c, theta_c, x_t, y_t, theta_t
    x_c, y_c, theta_c, x_t, y_t, theta_t = kinematics(x_c, y_c, theta_c, x_t, y_t, theta_t, v_c, v_t, delta_c)
    
    # 更新图像
    truck_line.set_data([x_c, x_t], [y_c, y_t])
    trailer_line.set_data([x_t, x_t + L_t * np.cos(theta_t)], [y_t, y_t + L_t * np.sin(theta_t)])
    return truck_line, trailer_line

ani = FuncAnimation(fig, update, frames=200, interval=100, blit=True)
plt.show()
