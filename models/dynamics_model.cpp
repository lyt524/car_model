#include <iostream>
#include <tuple>
#include "dynamics_model.h"
#include "../tools/mathtools.h"


DyCar::DyCar(float TS, float L, float MASS, float CF, float CR, float IZ, float LF, float LR,
          double x, double y, double phi, double d_vy, double d_phi, double d_d_phi): 
          TS(TS), L(L), MASS(MASS), CF(CF), CR(CR), IZ(IZ), LF(LF), LR(LR),
          x(x), y(y), phi(phi), d_vy(d_vy), d_phi(d_phi), d_d_phi(d_d_phi){}



void DyCar::UpdateState_ForwardEuler(double delta_f, double a){
    auto f = [this](double delta_f, double x, double y, double phi){
        double d_vy = (this->CF + this->CR) / (this->MASS * this->vx) * this->d_vy +
                      ((this->LF * this->CF - this->LR * this->CR) / (this->MASS * this->vx) - this->vx) * this->d_phi +
                      (-this->CF / this->MASS)*delta_f;
        

        double dx = this->v * cos(phi);
        double dy = this->v * sin(phi);
        double dphi = this->v * tan(delta_f) / this->L;

        return std::tuple<double, double, double>{dx, dy, dphi};
    };
}