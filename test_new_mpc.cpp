#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "control/mpc_new.h"
#include "models/kinematics_model.h"
#include "referencepath/reference_path.h"

int main(){
// Files
    std::ofstream outFileRefPath("reference_path.txt");
    std::ofstream outFileControlResult("control_result.txt");
    std::ofstream outFileCarState("car_state.txt");

    // Reference Path
    SineInfo sine_info(3.5, 0.01);
    RefPath ref_path(2000, 5);
    GenerateSinewavePath(100, ref_path, sine_info);
    ref_path.ShowPath();
    ref_path.WritePath(outFileRefPath);

    // Simulation Time
    double MAX_SIM_TIME = 60.0;
    double total_t = 0.0;

    KiCar ego_car(0.01, 3.0, 0.0, 0.0, 0.0, 0.03, 2.0);

    MPC mpc_controller(ego_car, ref_path);
    
    
    
    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        // add simulator total t
        total_t += ego_car.GetTs();

        mpc_controller.calMPC();
        // update ego car state
        ego_car.UpdateState_RK4((ego_car.GetDeltaF() + mpc_controller.d_delta_f), mpc_controller.d_v);
        ego_car.WriteCarState(outFileCarState);
        ego_car.PrintState();
    }

    outFileRefPath.close();
    outFileControlResult.close();
    outFileCarState.close();
}