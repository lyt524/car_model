#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include <fstream>
#include "models/kinematics_model.h"
#include "referencepath/reference_path.h"
#include "control/stanley.h"


int main(){
    // Files
    std::ofstream outFileRefPath("reference_path.txt");
    std::ofstream outFileControlResult("control_result.txt");
    std::ofstream outFileCarState("car_state.txt");

    // Reference Path
    SineInfo sine_info(3.5, 0.01);
    RefPath ref_path(2000, 4);
    GenerateSinewavePath(100, ref_path, sine_info);
    ref_path.ShowPath();
    ref_path.WritePath(outFileRefPath);

    // Simulator Time
    double MAX_SIM_TIME = 60.0;
    double total_t = 0.0;

    KiCar car(0.05, 3.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    Stanley stanley_controller;

    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        total_t += car.GetTs();
        double delta_f = stanley_controller.StanleyControl(car, ref_path);
        stanley_controller.WriteControlResult(outFileControlResult);
        car.UpdateState_RK4(delta_f, 0.0);
        car.WriteCarState(outFileCarState);
        car.PrintState();
    }

    outFileRefPath.close();
    outFileControlResult.close();
    outFileCarState.close();
    return 0;
}
