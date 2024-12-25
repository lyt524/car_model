# from build import KiCarModule
# from build import HelloTest
# from build import PathModule
from build import CppModule

import math
import numpy as np
import matplotlib.pyplot as plt


# kicar = KiCarModule.KiCar(0.05, 2.8, 0.0, 0.0, 0, 0, 2)

x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01 = [], [], [], [], [], []
x_list_02, y_list_02, phi_list_02, vx_list_02, delta_f_list_02, t_list_02 = [], [], [], [], [], []

amplitude = 0.1
frequency = 0.05
sampleRate = 20
duration = 90
numSamples = duration * sampleRate

def main_test_path():
    sine_info = CppModule.SineInfo(3.5, 0.01)
    path = CppModule.RefPath(2000, 4)
    CppModule.GenerateSinewavePath(100, path, sine_info)
    path.ShowPath()

def LogState(x, y, yaw, vx, delta_f, t):
    x_list_01.append(x)
    y_list_01.append(y)
    phi_list_01.append(yaw)
    vx_list_01.append(vx)
    delta_f_list_01.append(delta_f)
    t_list_01.append(t)

# def LogState(x, y, yaw, vx, delta_f, t, index):
#     if index == "01":
#         x_list_01.append(x)
#         y_list_01.append(y)
#         phi_list_01.append(yaw)
#         vx_list_01.append(vx)
#         delta_f_list_01.append(delta_f)
#         t_list_01.append(t)
#     elif index == "02":
#         x_list_02.append(x)
#         y_list_02.append(y)
#         phi_list_02.append(yaw)
#         vx_list_02.append(vx)
#         delta_f_list_02.append(delta_f)
#         t_list_02.append(t)

def ShowFigure_contrast(x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01, 
                        x_list_02, y_list_02, phi_list_02, vx_list_02, delta_f_list_02, t_list_02,
                        discrete_method_01, discrete_method_02):
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))

    axs[0, 0].plot(x_list_01, y_list_01, label=discrete_method_01, color='blue', linestyle='-', linewidth=2)
    axs[0, 0].plot(x_list_02, y_list_02, label=discrete_method_02, color='green', linestyle='--', linewidth=3)
    axs[0, 0].set_title('Position')
    axs[0, 0].legend() 

    axs[0, 1].plot(t_list_01, vx_list_01, label=discrete_method_01, color='blue', linestyle='-', linewidth=2)
    axs[0, 1].plot(t_list_02, vx_list_02, label=discrete_method_02, color='green', linestyle='--', linewidth=3)
    axs[0, 1].set_title('V')
    axs[0, 0].legend() 

    axs[1, 0].plot(t_list_01, phi_list_01, label=discrete_method_01, color='blue', linestyle='-', linewidth=2)
    axs[1, 0].plot(t_list_02, phi_list_02, label=discrete_method_02, color='green', linestyle='--', linewidth=3)
    axs[1, 0].set_title('Phi')
    axs[0, 0].legend() 

    axs[1, 1].plot(t_list_01, delta_f_list_01, label=discrete_method_01, color='blue', linestyle='-', linewidth=2)
    axs[1, 1].plot(t_list_02, delta_f_list_02, label=discrete_method_02, color='green', linestyle='--', linewidth=3)
    axs[1, 1].set_title('DeltaF')
    axs[0, 0].legend() 

    # Automatically adjust the spacing between subplots
    plt.tight_layout()

    # Display all the plots
    plt.show()

def ShowFigure(x_list, y_list, yaw_list, vx_list, delta_f_list, t_list):
    fig, axs = plt.subplots(2, 2, figsize=(10, 8))

    # First subplot
    axs[0, 0].plot(x_list, y_list)
    axs[0, 0].set_title('Position')

    axs[0, 1].plot(t_list, vx_list)
    axs[0, 1].set_title('V')

    axs[1, 0].plot(t_list, yaw_list)
    axs[1, 0].set_title('Yaw')

    axs[1, 1].plot(t_list, delta_f_list)
    axs[1, 1].set_title('DeltaF')

    # Automatically adjust the spacing between subplots
    plt.tight_layout()

    # Display all the plots
    plt.show()


# def main_test_module_HelloTest():
#     HelloTest.PrintHello()

def main():
    kicar01 = CppModule.KiCar(0.05, 2.8, 0.0, 0.0, 0, 0, 2)
    for i in range(numSamples):
        t = i / sampleRate
        value = amplitude * math.sin(2 * math.pi * frequency * t)
        value = 0.1
        kicar01.UpdateState_RK4(value)
        LogState(kicar01.GetX, kicar01.GetY, kicar01.GetYaw, kicar01.GetV, kicar01.GetDeltaF, t, "02")

    kicar = CppModule.KiCar(0.05, 2.8, 0.0, 0.0, 0, 0, 2)
    for i in range(numSamples):
        t = i / sampleRate
        value = amplitude * math.sin(2 * math.pi * frequency * t)
        value = 0.1
        kicar.UpdateState_ForwardEuler(value)
        LogState(kicar.GetX, kicar.GetY, kicar.GetYaw, kicar.GetV, kicar.GetDeltaF, t, "01")

    ShowFigure_contrast(x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01, 
                        x_list_02, y_list_02, phi_list_02, vx_list_02, delta_f_list_02, t_list_02,
                        "ForwardEuler", "RK4")
    
    # ShowFigure(x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01)

def main_stanley():
    car = CppModule.KiCar(0.05, 3.0, 0.0, 0.0, 0, 0, 2)
    sine_info = CppModule.SineInfo(3.5, 0.01)
    path = CppModule.RefPath(2000, 4)
    CppModule.GenerateSinewavePath(100, path, sine_info)
    path.ShowPath()
    stanley_controller = CppModule.Stanley()

    MAX_SIM_TIME = 60.0
    total_t = 0.0

    while(MAX_SIM_TIME > total_t and path.point_num - 10 > path.lastNearestPointIndex):
        total_t += car.GetTs
        delta_f = stanley_controller.StanleyControl(car, path)
        car.UpdateState_RK4(delta_f, 0.0)
        LogState(car.GetX, car.GetY, car.GetYaw, car.GetV, car.GetDeltaF, total_t)
        car.PrintState()

    ShowFigure(x_list_01, y_list_01, phi_list_01, vx_list_01, delta_f_list_01, t_list_01)


if __name__ == "__main__":
    # main_test_module_HelloTest()
    # main_test_path()
    # main()
    main_stanley()