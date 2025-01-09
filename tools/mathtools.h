#pragma once

#include <iostream>
#include <cmath>
#include <vector>

#include "../referencepath/reference_path.h"

// Normalize radians value to (-pi, pi]
void NormalizeAngle(double& angle);

void CalKappa(RefPath& path, int interval);

// transform a radians value to degree value
double radToDeg(const double& angle);
