#include "bicycle_model_mpc.h"


Bicycle::Bicycle(double L, double DT){
    this->ll_ = L;
    this->dt_ = DT;
    std::cout << "Bicycle model: " << std::endl;
    std::cout << "car ll_ = " << this->ll_ << std::endl;
    std::cout << "simulator dt_ = " << this->dt_ << std::endl;
}

void Bicycle::linearization(const double& phi, const double& v, const double& delta_f) {
    // X_k+1 = Ad_*X_k + Bd_*U_k + gd_
    this->Ad_ <<  0, 0, -v*sin(phi), cos(phi),
                  0, 0,  v*cos(phi), sin(phi),
                  0, 0, 0, tan(delta_f) / this->ll_,
                  0, 0, 0, 0;

    this->Bd_ <<  0, 0,
                  0, 0,
                  0, v/(this->ll_*pow(cos(delta_f),2)),
                  1, 0;

    this->gd_ <<  v*phi*sin(phi), -v*phi*cos(phi), -v*delta_f/(this->ll_*pow(cos(delta_f), 2)), 0;
 
    this->Ad_ = MatrixA::Identity() + this->dt_ * this->Ad_;
    this->Bd_ = this->dt_ * this->Bd_;
    this->gd_ = this->dt_ * this->gd_;
}

