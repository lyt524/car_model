#pragma once

#include <iostream>
#include <cstddef>
#include <cmath>

class Stanley{
public:
    ~Stanley() = default;
    Stanley() = default;
    size_t FindNearestIndex();
    double StanleyControl();

};

