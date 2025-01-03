from build import CppModule
from drawtrailer import draw_trailer

import math
import numpy as np
import matplotlib.pyplot as plt

def LogState(car_x_list, car_y_list, car_phi_list,
             car_v_list, car_delta_f_list, car_t_list,
             x, y, yaw, v, delta_f, t):
    car_x_list.append(x)
    car_y_list.append(y)
    car_phi_list.append(yaw)
    car_v_list.append(v)
    car_delta_f_list.append(delta_f)
    car_t_list.append(t)

def main_stanley():
    ref_path_list_x = []
    ref_path_list_y = []
    ref_path_list_phi = []

    car_x_list = []
    car_y_list = []
    car_phi_list = []
    car_v_list = []
    car_delta_f_list = []
    car_t_list = []

    car_lat_err_list = []
    car_heading_err_list = []
    
    plt.figure(1)
    car = CppModule.KiCar(0.05, 3.0, 0.0, -3.0, 0, 0, 10)
    sine_info = CppModule.SineInfo(3.5, 0.01)
    path = CppModule.RefPath(2000, 4)
    CppModule.GenerateSinewavePath(100, path, sine_info)
    
    for path_index in range(path.point_num):
        ref_path_list_x.append(path.GetPointX(path_index))
        ref_path_list_y.append(path.GetPointY(path_index))
        ref_path_list_phi.append(path.GetPointPhi(path_index))

    stanley_controller = CppModule.Stanley()

    MAX_SIM_TIME = 60.0
    total_t = 0.0

    plt.plot(ref_path_list_x, ref_path_list_y, '-.b', linewidth=1.0)

    while(MAX_SIM_TIME > total_t and path.point_num - 4 > path.lastNearestPointIndex):
        total_t += car.GetTs
        delta_f = stanley_controller.StanleyControl(car, path)
        car.UpdateState_RK4(delta_f, 0.0)

        LogState(car_x_list, car_y_list, car_phi_list,
                 car_v_list, car_delta_f_list, car_t_list,
                 car.GetX, car.GetY, car.GetYaw,
                 car.GetV, car.GetDeltaF, total_t)
        
        car_lat_err_list.append(stanley_controller.GetLateralError)
        car_heading_err_list.append(stanley_controller.GetHeadingError)
        
        plt.cla()
        plt.plot(ref_path_list_x, ref_path_list_y, '-.b', linewidth=1.0, label="reference path")
        draw_trailer(car.GetX, car.GetY, car.GetYaw, car.GetDeltaF, plt)
        plt.plot(car_x_list, car_y_list, "-r", label="trajectory")

        plt.plot(ref_path_list_x[stanley_controller.FindNearestIndex(car, path)], 
                 ref_path_list_y[stanley_controller.FindNearestIndex(car, path)], 
                 "go", label="target")
        plt.axis("equal")
        plt.legend(loc='upper right')
        plt.pause(0.001)
    
    print("len(ref_path_list_x) = ", len(ref_path_list_x))
    print("len(car_t_list) = ", len(car_t_list))
    print("len(car_x_list) = ", len(car_x_list))
    print("len(car_heading_err_list) = ", len(car_heading_err_list))
    print("total_t = ", total_t)
    print("path.lastNearestPointIndex = ", path.lastNearestPointIndex)

    plt.figure(2)
    
    plt.subplot(2, 3, 1)
    plt.plot(ref_path_list_x, ref_path_list_y, '-.b', linewidth=1.0, label="reference path")
    plt.plot(car_x_list, car_y_list, 'r', label="trajectory")
    plt.title("actual tracking effect")
    plt.legend(loc='upper right')

    plt.subplot(2, 3, 2)
    plt.plot(car_x_list, car_heading_err_list)
    plt.title("heading error")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 3)
    plt.plot(car_x_list, car_lat_err_list)
    plt.title("lateral error")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 4)
    plt.plot(car_x_list, car_delta_f_list)
    plt.title("delta_f")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 5)
    plt.plot(ref_path_list_x, ref_path_list_phi, '-.b', linewidth=1.0, label="reference heading")
    plt.plot(car_x_list, car_phi_list, 'r', label="actual heading")
    plt.title("heading contrast")
    plt.legend(loc='upper right')

    plt.subplot(2, 3, 6)
    plt.plot(car_x_list, car_v_list)
    plt.title("vehicle velocity")
    # plt.legend(loc='upper right')

    plt.show()

    
if __name__ == "__main__":
    main_stanley()