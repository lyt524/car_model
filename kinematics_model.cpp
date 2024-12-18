#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include "kinematics_model.h"


KiCar::KiCar(float dt, float l): TS(dt), L(l){}
KiCar::KiCar(float dt, 
             float l, 
             double x, 
             double y, 
             double yaw, 
             double delta_f, 
             double vx): 
             TS(dt), 
             L(l), 
             x(x), 
             y(y), 
             yaw(yaw), 
             delta_f(delta_f), 
             vx(vx){}

float KiCar::GetTs(){
    return this->TS;
}

float KiCar::GetL(){
    return this->L;
}

void KiCar::GetPosition(){

}

void KiCar::PrintState(){
    // std::cout << "x = "   << this->x   << endl;
    // std::cout << "y = "   << this->y   << endl;
    // std::cout << "yaw = " << this->yaw << endl;
    // std::cout << "vx = "  << this->vx  << endl;

    std::cout << "x = "  << this->x << 
     "              y = "<< this->y <<
     "              delta_f = "<< this->delta_f << endl;
}

void KiCar::UpdateState(double delta_f){
    this->delta_f = delta_f;
    this->x += this->vx * cos(this->yaw) * this->TS;
    this->y += this->vx * sin(this->yaw) * this->TS;
    this->yaw += this->vx * tan(this->delta_f)  * this->TS / this->L;
}

void KiCar::UpdateState(double delta_f, double ax){
    this->delta_f = delta_f;
    this->ax = ax;
    this->vx += this->ax * this->TS;
    this->x += this->vx * cos(this->yaw) * this->TS;
    this->y += this->vx * sin(this->yaw) * this->TS;
    this->yaw += this->vx * tan(this->delta_f) * this->TS / this->L;
}


