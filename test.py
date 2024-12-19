from build import KiCarModule
import math
# import numpy as np
import matplotlib.pyplot as plt


kicar = KiCarModule.KiCar(0.05, 2.8, 0.0, 0.0, 0, 0, 2)

x_list, y_list, yaw_list, vx_list, delta_f_list, t_list = [], [], [], [], [], []

amplitude = 0.1
frequency = 0.05
sampleRate = 20
duration = 90
numSamples = duration * sampleRate

def LogState(x, y, yaw, vx, delta_f, t):
    x_list.append(x)
    y_list.append(y)
    yaw_list.append(yaw)
    vx_list.append(vx)
    delta_f_list.append(delta_f)
    t_list.append(t)

def ShowFigure(x_list, y_list, yaw_list, delta_f_list, t_list):
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))

    # First subplot
    axs[0, 0].plot(x_list, y_list)
    axs[0, 0].set_title('Position')

    axs[0, 1].plot(t_list, vx_list)
    axs[0, 1].set_title('Vx')

    axs[1, 0].plot(t_list, yaw_list)
    axs[1, 0].set_title('Yaw')

    axs[1, 1].plot(t_list, delta_f_list)
    axs[1, 1].set_title('DeltaF')

    # Automatically adjust the spacing between subplots
    plt.tight_layout()

    # Display all the plots
    plt.show()

def main():
    for i in range(numSamples):
        t = i / sampleRate
        value = amplitude * math.sin(2 * math.pi * frequency * t)
        kicar.UpdateState_RK4(value)
        LogState(kicar.GetX, kicar.GetY, kicar.GetYaw, kicar.GetVx, kicar.GetDeltaF, t)

    ShowFigure(x_list, y_list, yaw_list, delta_f_list, t_list)


if __name__ == "__main__":
    main()




