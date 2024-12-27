#pragma once

#include <iostream>
#include <cmath>
#include <vector>

#include "../referencepath/reference_path.h"

// Normalize radians angle to (-pi, pi]
void NormalizeAngle(double& angle);

void CalKappa(RefPath& path, int interval);
