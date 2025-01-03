#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "models/kinematics_model.h"
#include "referencepath/reference_path.h"
#include "control/mpc_coupling.h"


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

    KiCar pre_car(0.05, 3.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    KiCar ego_car(0.05, 3.0, 0.0, 0.0, 0.0, 0.01, 2.0);
    Bicycle mpc_bicycle_model(3.0, 0.05);
    QPSolver qpsolver;
    MpcController mpc_controller(mpc_bicycle_model,  // mpc model
                                pre_car,             // predict model
                                ego_car,             // ego car
                                qpsolver,            // solver
                                ref_path,            // ref_path
                                20,                  // predict step
                                5,                   // v max
                                1,                   // a max
                                0.5,                 // delta_f max
                                0.2);                // ddelta_f max

    mpc_controller.SetConstrains();

    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        // add simulator total t
        total_t += ego_car.GetTs();

        // get mpc control command
        mpc_controller.SolveQP();

        mpc_controller.WriteControlResult(outFileControlResult);

        // update ego car state
        ego_car.UpdateState_RK4(mpc_controller.command_delta_f, mpc_controller.command_a_);
        ego_car.WriteCarState(outFileCarState);
        ego_car.PrintState();
    }

    outFileRefPath.close();
    outFileControlResult.close();
    outFileCarState.close();
    return 0;
}

