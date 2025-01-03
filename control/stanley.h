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
#include "../referencepath/point.h"
#include "../tools/mathtools.h"

class Stanley{
public:
    ~Stanley() = default;
    Stanley() = default;

    int FindNearestIndex(KiCar& ki_car, RefPath& ref_path);
    double CalHeadingError(KiCar& ki_car, RefPath& ref_path);
    double CalLateralError(KiCar& ki_car, RefPath& ref_path);
    double StanleyControl(KiCar& ki_car, RefPath& ref_path);
    double GetLateralError();
    double GetHeadingError();
    void WriteControlResult(std::ofstream& outFile);

    double stanleyK = 0.5;
    double deltaF;
    double headingError;
    double lateralError;
};

