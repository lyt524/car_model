#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include "kinematics_model.h"
#include <tuple>

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
    std::cout << "x = "   << this->x   << endl;
    std::cout << "y = "   << this->y   << endl;
    std::cout << "phi = " << this->phi << endl;
    std::cout << "v = "  << this->v  << endl;
    std::cout << "---------------" << endl;

    // std::cout << "x = "  << this->x << 
    //  "              y = "<< this->y <<
    //  "              delta_f = "<< this->delta_f << endl;
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
    auto f = [this](double delta_f, double x, double y, double phi){
        double dx = this->v * cos(phi);
        double dy = this->v * sin(phi);
        double dphi = this->v * tan(delta_f) / this->L;

        return std::tuple<double, double, double>{dx, dy, dphi};
    };

    double x0 = this->x;
    double y0 = this->y;
    double phi0 = this->phi;

    auto [dx1, dy1, dphi1] = f(delta_f, x0, y0, phi0);
    double K1_x = dx1;
    double K1_y = dy1;
    double K1_phi = dphi1;

    auto [dx2, dy2, dphi2] = f(delta_f, 
                              x0 + 0.5 * this->TS * K1_x, 
                              y0 + 0.5 * this->TS * K1_y, 
                              phi0 + 0.5 * this->TS * K1_phi);
    double K2_x = dx2;
    double K2_y = dy2;
    double K2_phi = dphi2;

    auto [dx3, dy3, dphi3] = f(delta_f, 
                              x0 + 0.5 * this->TS * K2_x, 
                              y0 + 0.5 * this->TS * K2_y, 
                              phi0 + 0.5 * this->TS * K2_phi);
    double K3_x = dx3;
    double K3_y = dy3;
    double K3_phi = dphi3;

    auto [dx4, dy4, dphi4] = f(delta_f, 
                              x0 + this->TS * K3_x, 
                              y0 + this->TS * K3_y, 
                              phi0 + this->TS * K3_phi);
    double K4_x = dx4;
    double K4_y = dy4;
    double K4_phi = dphi4;

    this->x += (this->TS / 6) * (K1_x + 2 * K2_x + 2 * K3_x + K4_x);
    this->y += (this->TS / 6) * (K1_y + 2 * K2_y + 2 * K3_y + K4_y);
    this->phi += (this->TS / 6) * (K1_phi + 2 * K2_phi + 2 * K3_phi + K4_phi);

    this->delta_f = delta_f;
}

void KiCar::UpdateState_RK4(double delta_f, double a){
    auto f = [this](double delta_f, double a, double x, double y, double phi, double v){
        double dx = v * cos(phi);
        double dy = v * sin(phi);
        double dphi = v * tan(delta_f) / this->L;
        double dv = a;

        return std::tuple<double, double, double, double>{dx, dy, dphi, dv};
    };

    double x0 = this->x;
    double y0 = this->y;
    double phi0 = this->phi;
    double v0 = this->v;

    auto [dx1, dy1, dphi1, dv1] = f(delta_f, a, x0, y0, phi0, v0);
    double K1_x = dx1;
    double K1_y = dy1;
    double K1_phi = dphi1;
    double K1_v = dv1;

    auto [dx2, dy2, dphi2, dv2] = f(delta_f, a,
                                    x0 + 0.5 * this->TS * K1_x,
                                    y0 + 0.5 * this->TS * K1_y,
                                    phi0 + 0.5 * this->TS * K1_phi,
                                    v0 + 0.5 * this->TS * K1_v);
    double K2_x = dx2;
    double K2_y = dy2;
    double K2_phi = dphi2;
    double K2_v = dv2;

    auto [dx3, dy3, dphi3, dv3] = f(delta_f, a,
                                    x0 + 0.5 * this->TS * K2_x,
                                    y0 + 0.5 * this->TS * K2_y,
                                    phi0 + 0.5 * this->TS * K2_phi,
                                    v0 + 0.5 * this->TS * K2_v);
    double K3_x = dx3;
    double K3_y = dy3;
    double K3_phi = dphi3;
    double K3_v = dv3;

    auto [dx4, dy4, dphi4, dv4] = f(delta_f, a,
                                    x0 + this->TS * K3_x,
                                    y0 + this->TS * K3_y,
                                    phi0 + this->TS * K3_phi,
                                    v0 + this->TS * K3_v);
    double K4_x = dx4;
    double K4_y = dy4;
    double K4_phi = dphi4;
    double K4_v = dv4;

    this->x += (this->TS / 6) * (K1_x + 2 * K2_x + 2 * K3_x + K4_x);
    this->y += (this->TS / 6) * (K1_y + 2 * K2_y + 2 * K3_y + K4_y);
    this->phi += (this->TS / 6) * (K1_phi + 2 * K2_phi + 2 * K3_phi + K4_phi);
    this->v += (this->TS / 6) * (K1_v + 2 * K2_v + 2 * K3_v + K4_v);

    this->delta_f = delta_f;
}