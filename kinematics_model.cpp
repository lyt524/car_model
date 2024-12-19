#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include "kinematics_model.h"

KiCar::~KiCar() = default;
KiCar::KiCar(float dt, float l): TS(dt), L(l){}
KiCar::KiCar(float dt, 
             float l, 
             double x, 
             double y, 
             double phi, 
             double delta_f, 
             double v): 
             TS(dt), 
             L(l), 
             x(x), 
             y(y), 
             phi(phi), 
             delta_f(delta_f), 
             v(v){}

float KiCar::GetTs() { return this->TS; }
float KiCar::GetL() { return this->L; }
double KiCar::GetX() { return this->x; }
double KiCar::GetY() { return this->y; }
double KiCar::GetYaw() { return this->phi; }
double KiCar::GetDeltaF() { return this->delta_f; }
double KiCar::GetVx() { return this->v; }

void KiCar::GetPosition(){

}

void KiCar::PrintState(){
    // std::cout << "x = "   << this->x   << endl;
    // std::cout << "y = "   << this->y   << endl;
    // std::cout << "phi = " << this->phi << endl;
    // std::cout << "v = "  << this->v  << endl;

    std::cout << "x = "  << this->x << 
     "              y = "<< this->y <<
     "              delta_f = "<< this->delta_f << endl;
}

void KiCar::UpdateState_ForwardEuler(double delta_f){
    this->x += this->v * cos(this->phi) * this->TS;
    this->y += this->v * sin(this->phi) * this->TS;
    this->phi += this->v * tan(this->delta_f)  * this->TS / this->L;
    this->delta_f = delta_f;
}


void KiCar::UpdateState_ForwardEuler(double delta_f, double a){
    this->x += this->v * cos(this->phi) * this->TS;
    this->y += this->v * sin(this->phi) * this->TS;
    this->phi += this->v * tan(this->delta_f) * this->TS / this->L;
    this->a = a;
    this->v += this->a * this->TS;
    this->delta_f = delta_f;
}


void KiCar::UpdateState_BackwardEuler(double delta_f){
    this->phi += this->v * tan(this->delta_f) * this->TS / this->L;
    this->x += this->v * cos(this->phi) * this->TS;
    this->y += this->v * sin(this->phi) * this->TS;
    this->delta_f = delta_f;
}

void KiCar::UpdateState_BackwardEuler(double delta_f, double a){
    this->a = a;
    this->v += this->a * this->TS;
    this->phi += this->v * tan(this->delta_f) * this->TS / this->L;
    this->x += this->v * cos(this->phi) * this->TS;
    this->y += this->v * sin(this->phi) * this->TS;
    this->delta_f = delta_f;
}

void KiCar::UpdateState_RK4(double delta_f){
    double K1_x = this->v * cos(this->phi);
    double K1_y = this->v * sin(this->phi);
    double K1_phi = this->v * tan(this->delta_f)/ this->L;

    double K2_x = this->v * cos(this->phi + 0.5 * this->TS * K1_phi);
    double K2_y = this->v * sin(this->phi + 0.5 * this->TS * K1_phi);
    double K2_phi = this->v * tan(this->delta_f)/ this->L;

    double K3_x = this->v * cos(this->phi + 0.5 * this->TS * K2_phi);
    double K3_y = this->v * sin(this->phi + 0.5 * this->TS * K2_phi);
    double K3_phi = this->v * tan(this->delta_f)/ this->L;

    double K4_x = this->v * cos(this->phi + this->TS * K3_phi);
    double K4_y = this->v * sin(this->phi + this->TS * K3_phi);
    double K4_phi = this->v * tan(this->delta_f)/ this->L;

    this->x += (this->TS / 6) * (K1_x + 2 * K2_x + 2 * K3_x + K4_x);
    this->y += (this->TS / 6) * (K1_y + 2 * K2_y + 2 * K3_y + K4_y);
    this->phi += (this->TS / 6) * (K1_phi + 2 * K2_phi + 2 * K3_phi + K4_phi);

    this->delta_f = delta_f;
}

void KiCar::UpdateState_RK4(double delta_f, double a){
}
