#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <deque>
#include <fstream>
#include <unsupported/Eigen/KroneckerProduct>

#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"

class MPC{
public:
    MPC(KiCar& car, RefPath& ref_path);
    ~MPC() = default;

    void setBicycleModel();
    void FindRefPos();
    void setKesi();
    void setQR();
    void setABC();
    void setPHI_THETA();
    void setH_F();
    void setConstrains();
    void calMPC();

    // osqp
    void setMats(Eigen::SparseMatrix<double> P_, 
                Eigen::VectorXd q_d,
                Eigen::SparseMatrix<double> A_,
                Eigen::VectorXd l_d,
                Eigen::VectorXd u_d);

    bool getRes();
    
    // model construct
    Eigen::Matrix<double, 3, 3> A1;
    Eigen::Matrix<double, 3, 2> B1;
    Eigen::Matrix<double, 5, 5> A;
    Eigen::Matrix<double, 5, 2> B;
    Eigen::Matrix<double, 3, 5> C;
    // weight matrix
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    // state vriable and control vriable (bias)
    Eigen::VectorXd kesi;
    // state vriable dimension expansion
    Eigen::MatrixXd PHI;
    Eigen::MatrixXd THETA;
    // osqp matrix construct
    Eigen::MatrixXd H;
    Eigen::VectorXd F;
    Eigen::MatrixXd Ad;
    Eigen::VectorXd ld;
    Eigen::VectorXd ud;
    Eigen::SparseMatrix<double> Ad_sparse;
    Eigen::SparseMatrix<double> H_sparse;

    const double dt = 0.01;
    const int Np_ = 10; // 预测步长
    const int Nx_ = 3;   // 状态量个数
    const int Nu_ = 2;   // 控制量个数

    double d_delta_f = 0.0;
    double d_v = 0.0;

    KiCar& car_;
    RefPath& ref_path_;
    Eigen::VectorXd solution_;
    OsqpEigen::Solver qpSolver_;
};

Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int n);