#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <deque>
#include <fstream>
#include <unsupported/Eigen/KroneckerProduct>
#include <algorithm>

#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"

#define Np_ 20
#define Nx_ 3
#define Nu_ 2
#define dt_ 0.01

class MPC{
public:
    MPC(KiCar& ego_car, RefPath& ref_path);
    ~MPC() = default;

    void setKinematicsMatrices();
    void setInequalityConstraints();
    void setWeightMatrices();

    // set Hessian Matrix
    void castMPCToQPHessian();
    // set Gradient Matrix
    void castMPCToQPGradient();
    // set Linear Matrix (Constraint Matrix)
    void castMPCToQPConstraintMatrix();
    // set Lower Bound and Upper Bound
    void castMPCToQPConstraintVectors();
    // update Lower Bound and Upper Bound based on new x0_
    void updateConstraintVectors();

    void findRefPos();
    void setX0();
    void setXRef();
    void qpInit();

    bool setSolver();
    bool resetSolver();
    bool getQPResult();

    void writeControlResult(std::ofstream& outFile);
    void calLateralError();

    Eigen::DiagonalMatrix<double, Nx_> Q_;         // Q
    Eigen::DiagonalMatrix<double, Nu_> R_;         // R
    Eigen::Matrix<double, Nx_, Nx_> A_;            // A
    Eigen::Matrix<double, Nx_, Nu_> B_;            // B
    Eigen::Matrix<double, Nx_, 1> xMax_;           // x_max
    Eigen::Matrix<double, Nx_, 1> xMin_;           // x_min
    Eigen::Matrix<double, Nu_, 1> uMax_;           // u_max
    Eigen::Matrix<double, Nu_, 1> uMin_;           // u_min
    Eigen::Matrix<double, Nx_, 1> x0_;             // x_0
    Eigen::Matrix<double, Nx_, 1> xRef_;           // x_ref

    // osqp form Matrix
    Eigen::SparseMatrix<double> hessian_;          // P
    Eigen::VectorXd gradient_;                     // q
    Eigen::SparseMatrix<double> linearMatrix_;     // A_c
    Eigen::VectorXd lowerBound_;                   // l
    Eigen::VectorXd upperBound_;                   // u

    int mpcWindow_ = Np_;

    KiCar& ego_car_;
    RefPath& ref_path_;

    // Initialize solver instance
    OsqpEigen::Solver solver_;

    // controller result
    Eigen::Vector2d ctr_;
    Eigen::VectorXd QPSolution_;
    double d_delta_f;

    // error
    double headingError;
    double lateralError;
};

