#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

// extern const int n = 4;  // state num: x y phi v
// extern const int m = 2;  // input num: a delta_f
typedef Eigen::Matrix<double, 4, 4> MatrixA;
typedef Eigen::Matrix<double, 4, 2> MatrixB;
typedef Eigen::Vector4d VectorG;
typedef Eigen::Vector4d VectorX;
typedef Eigen::Vector2d VectorU;

class Bicycle{
public:
    Bicycle(double L, double DT);
    ~Bicycle() = default;

    MatrixA Ad_;
    MatrixB Bd_;
    VectorG gd_;

    double ll_;
    double dt_;

    // Do both linearization and discretization (Matrix)
    void linearization(const double &phi, const double &v, const double &delta);
};

