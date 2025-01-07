import numpy as np
import matplotlib.pyplot as plt

ref_path_data = np.loadtxt('/home/plusai/car_model/build/reference_path.txt')
car_state_data = np.loadtxt('/home/plusai/car_model/build/car_state.txt')
# control_result_data = np.loadtxt('/home/plusai/car_model/build/control_result.txt')

def showResult_mpc(ref_path_data, car_state_data):
    ref_path_x = ref_path_data[:, 0]
    ref_path_y = ref_path_data[:, 1]
    ref_path_phi = ref_path_data[:, 2]

    car_pose_x = car_state_data[:, 0]
    car_pose_y = car_state_data[:, 1]
    car_pose_yaw = car_state_data[:, 2]
    car_pose_v = car_state_data[:, 3]

    plt.subplot(2, 3, 1)
    plt.plot(ref_path_x, ref_path_y, '-.b', linewidth=1.0, label="reference path")
    plt.plot(car_pose_x, car_pose_y, 'r', label="trajectory")
    plt.title("actual tracking effect")
    plt.legend(loc='upper right')

    plt.show()


def showResult(ref_path_data, car_state_data, control_result_data):
    ref_path_x = ref_path_data[:, 0]
    ref_path_y = ref_path_data[:, 1]
    ref_path_phi = ref_path_data[:, 2]

    car_pose_x = car_state_data[:, 0]
    car_pose_y = car_state_data[:, 1]
    car_pose_yaw = car_state_data[:, 2]
    car_pose_v = car_state_data[:, 3]

    control_value_deltaf = control_result_data[:, 0]
    control_value_heading_err = control_result_data[:, 1]
    control_value_lateral_err = control_result_data[:, 2]

    plt.subplot(2, 3, 1)
    plt.plot(ref_path_x, ref_path_y, '-.b', linewidth=1.0, label="reference path")
    plt.plot(car_pose_x, car_pose_y, 'r', label="trajectory")
    plt.title("actual tracking effect")
    plt.legend(loc='upper right')

    plt.subplot(2, 3, 2)
    plt.plot(car_pose_x, control_value_heading_err)
    plt.title("heading error")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 3)
    plt.plot(car_pose_x, control_value_lateral_err)
    plt.title("lateral error")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 4)
    plt.plot(car_pose_x, control_value_deltaf)
    plt.title("delta_f")
    # plt.legend(loc='upper right')

    plt.subplot(2, 3, 5)
    plt.plot(ref_path_x, ref_path_phi, '-.b', linewidth=1.0, label="reference heading")
    plt.plot(car_pose_x, car_pose_yaw, 'r', label="actual heading")
    plt.title("heading contrast")
    plt.legend(loc='upper right')

    plt.subplot(2, 3, 6)
    plt.plot(car_pose_x, car_pose_v)
    plt.title("vehicle velocity")
    # plt.legend(loc='upper right')

    plt.show()

if __name__ == "__main__":
    # showResult(ref_path_data, car_state_data, control_result_data)
    showResult_mpc(ref_path_data, car_state_data)

