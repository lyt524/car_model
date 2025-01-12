#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include <fstream>

#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"
#include "../controller/stanley.h"


int main(){
    // Files
    std::ofstream outFileRefPath("reference_path.txt");
    std::ofstream outFileControlResult("control_result.txt");
    std::ofstream outFileCarState("car_state.txt");

    // Reference Path
    SineInfo sine_info(3.5, 0.01);
    RefPath ref_path(2000, 4);
    generateSinewavePath(100, ref_path, sine_info);
    ref_path.showPath();
    ref_path.writePath(outFileRefPath);

    // Simulation Time
    double MAX_SIM_TIME = 60.0;
    double total_t = 0.0;

    KiCar car(0.05, 3.0, 0.0, 0.0, 0.0, 0.0, 2.0);
    Stanley stanley_controller;

    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        total_t += car.getTs();
        double delta_f = stanley_controller.stanleyControl(car, ref_path);
        stanley_controller.writeControlResult(outFileControlResult);
        car.updateState_RK4(delta_f, 0.0);
        car.writeCarState(outFileCarState);
        car.printState();
    }

    outFileRefPath.close();
    outFileControlResult.close();
    outFileCarState.close();
    return 0;
}
