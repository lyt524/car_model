from drawtrailer import VehicleInfo, draw_trailer
from build import KiCarModule

import math
import numpy as np
import matplotlib.pyplot as plt


kicar = KiCarModule.KiCar(0.05, 3.0, 0.0, 0.0, 0, 0, 2)

x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01 = [], [], [], [], [], []

amplitude = 0.1
frequency = 0.05
sampleRate = 20
duration = 20
numSamples = duration * sampleRate

def LogState(x, y, yaw, vx, delta_f, t):
        x_list_01.append(x)
        y_list_01.append(y)
        phi_list_01.append(yaw)
        vx_list_01.append(vx)
        delta_f_list_01.append(delta_f)
        t_list_01.append(t)

def main():
    plt.figure(1)

    for i in range(numSamples):
        t = i / sampleRate
        steer_value = amplitude * math.sin(2 * math.pi * frequency * t)
        kicar.UpdateState_RK4(steer_value)
        LogState(kicar.GetX, kicar.GetY, kicar.GetYaw, kicar.GetVx, kicar.GetDeltaF, t)
        plt.cla()
        draw_trailer(kicar.GetX, kicar.GetY, kicar.GetYaw, kicar.GetDeltaF, plt)  # 画车辆轮廓及四个车轮轮廓
        plt.plot(x_list_01, y_list_01, "-r", label="trajectory")
        plt.axis("equal")
        plt.pause(0.001)

if __name__ == "__main__":
    main()

