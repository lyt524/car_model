#pragma once

#include <iostream>
#include <cstddef>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"
#include "../tools/mathtools.h"

class Stanley{
public:
    ~Stanley() = default;
    Stanley() = default;

    int findNearestIndex(KiCar& ki_car, RefPath& ref_path);
    double calHeadingError(KiCar& ki_car, RefPath& ref_path);
    double calLateralError(KiCar& ki_car, RefPath& ref_path);
    double stanleyControl(KiCar& ki_car, RefPath& ref_path);
    double getLateralError();
    double getHeadingError();
    void writeControlResult(std::ofstream& outFile);

    double stanley_K_ = 0.5;
    double delta_f_;
    double heading_error_;
    double lateral_error_;
};

