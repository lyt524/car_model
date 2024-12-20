#include <iostream>
#include <cmath>
#include "tools.h"



// Normalize radians angle to (-pi, pi]
void NormalizeAngle(double& angle){
    while(angle > M_PI){
        angle -= 2. * M_PI;
    }
    while(angle <= -M_PI){
        angle += 2. * M_PI;
    }
}