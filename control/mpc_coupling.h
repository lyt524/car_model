#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <deque>
#include <fstream>

#include "../models/bicycle_model_mpc.h"
#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"

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

    Eigen::VectorXd solution_;
    OsqpEigen::Solver qpSolver_;
};

// v a delta_f ddelta_f Constrained state variables and control variables
static constexpr int n_cons = 3;
// static constexpr int n_cons = 4;

class MpcController{
public:
    MpcController(Bicycle& car_model,
                  KiCar& pre_car,
                  KiCar& ego_car,
                  QPSolver& qpSolver,
                  RefPath& ref_path,
                  int pre_step,
                  double v_max,
                  double a_max,
                  double delta_f_max,
                  double ddelta_f_max);
    ~MpcController() = default;

    void SetConstrains();
    void SolveQP();
    void GetCarState();
    void qpInit();
    void FindRefPos();
    void WriteControlResult(std::ofstream& outFile);

    int N_;  // prediction step size
    double v_max_, a_max_, delta_f_max_, ddelta_f_max_;

    Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // v constrains 
    Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // a delta_f ddelta_f constrains
    Eigen::SparseMatrix<double> Qx_;
    Eigen::SparseMatrix<double> A_, l_, u_;
    Eigen::SparseMatrix<double> P_, q_;

    Eigen::VectorXd solution_;

    // TODO: ensure these values
    double rho_ = 2.0;
    double rhoN_ = 2.0;

    double ref_x_, ref_y_, ref_phi_, ref_v_;
    double ego_x_, ego_y_, ego_phi_, ego_v_, ego_delta_f_, ego_a_;
    double pre_x_, pre_y_, pre_phi_, pre_v_, pre_delta_f_, pre_a_;

    Bicycle& car_model_;
    KiCar& pre_car_;
    KiCar& ego_car_;
    QPSolver& qpSolver_;
    RefPath& ref_path_;

    int ref_path_match_index_;

    double command_a_;
    double command_delta_f;

    const int n = 4;  // state num: x y phi v
    const int m = 2;  // input num: a delta_f
};
