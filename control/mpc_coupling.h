#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <deque>
#include "../models/bicycle_model_mpc.h"
#include "../models/kinematics_model.h"

class QPSolver {
public:
    QPSolver() = default;
    ~QPSolver() = default;

    void setMats(Eigen::SparseMatrix<double> P_, 
                Eigen::VectorXd q_d,
                Eigen::SparseMatrix<double> A_,
                Eigen::VectorXd l_d,
                Eigen::VectorXd u_d);

    bool getRes();

    OsqpEigen::Solver qpSolver_;
};


/* 
*               /  x1  \
*               |  x2  |
*  lx_ <=  Cx_  |  x3  |  <= ux_
*               | ...  |
*               \  xN  /
*
*               /  u0  \
*               |  u1  |
*  lu_ <=  Cu_  |  u2  |  <= uu_
*               | ...  |
*               \ uN-1 /
* 
*/

// v a delta_f ddelta_f Constrained state variables and control variables
static constexpr int n_cons = 4;

class MpcController{
public:
    MpcController(Bicycle& car_model,
                  KiCar& car,
                  QPSolver& qpSolver,
                  int pre_step,
                  double v_max,
                  double a_max,
                  double delta_f_max,
                  double ddelta_f_max);
    ~MpcController() = default;

    void SetConstrains();
    void SolveQP(const VectorX& x0_observe);
    void GetCarState();
    void qpInit();
    Eigen::Vector2d FindRefPos();

    int N_;  // prediction step size
    double v_max_, a_max_, delta_f_max_, ddelta_f_max_;
    Bicycle& car_model_;
    KiCar& car_;
    Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // v constrains 
    Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // a delta_f ddelta_f constrains
    Eigen::SparseMatrix<double> Qx_;
    Eigen::SparseMatrix<double> A_, l_, u_;
    Eigen::SparseMatrix<double> P_, q_;

    // TODO: ensure these values
    double ref_phi_;
    double ref_v_;
    double rho_ = 2.0;
    double rhoN_ = 2.0;

    double ego_x_, ego_y_, ego_phi_, ego_v_, ego_delta_f_;

    QPSolver& qpSolver_;
};
