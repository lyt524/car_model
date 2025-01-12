#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "../controller/mpc_new.h"
#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"

int main(){
    // Files
    std::ofstream outFileRefPath("reference_path.txt");
    std::ofstream outFileControlResult("control_result.txt");
    std::ofstream outFileCarState("car_state.txt");

    // Reference Path
    SineInfo sine_info(3.5,  // amplitude
                       0.01  // frequency
                       );

    RefPath ref_path(4000,   // total point number
                     5       // rows: 4-x,y,h,v  5-x,y,h,v,kappa
                     );
                     
    generateSinewavePath(200,  // traj length(meter)
                         ref_path, 
                         sine_info
                         );

    ref_path.showPath();
    ref_path.writePath(outFileRefPath);

    // Simulation Time
    double MAX_SIM_TIME = 60.0;
    double total_t = 0.0;

    KiCar ego_car(0.01,       // simulator time step
                  3.0,        // car length
                  0.0,        // x init
                  0.0,        // y init
                  0.2165,     // phi init
                  0.0,        // delta_f init
                  2.0          // v init
                  );

    MPC mpc_controller(ego_car, ref_path);

    // mpc_controller.calMPC();
    
    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        // add simulator total t
        total_t += ego_car.getTs();

        mpc_controller.calMPC();
        mpc_controller.writeControlResult(outFileControlResult);

        // update ego car state
        ego_car.updateState_RK4((ego_car.getDeltaF() + mpc_controller.d_delta_f), mpc_controller.d_v);
        ego_car.writeCarState(outFileCarState);
        ego_car.printState();
    }

    outFileRefPath.close();
    outFileControlResult.close();
    outFileCarState.close();
}

