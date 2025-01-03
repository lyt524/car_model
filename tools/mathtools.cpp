#include <iostream>
#include <cmath>
#include <vector>

#include "mathtools.h"

// Normalize radians angle to (-pi, pi]
void NormalizeAngle(double& angle){
    while(angle > M_PI){
        angle -= 2. * M_PI;
    }
    while(angle <= -M_PI){
        angle += 2. * M_PI;
    }
}


// FIXME: check interval != 1
void CalKappa(RefPath& path, int interval){
    int path_point_num = path.point_num;
    if(interval < 1 || interval >= 5){
        std::cerr << "interval out of range" << std::endl;
        interval = 2;
    }
    // auto kappa_data = path.ref_path[4];
    for(int i = interval; i < path.point_num - interval; i += interval){
        int index_pre = i - interval;
        int index_cur = i;
        int index_nxt = i + interval;

        double x1 = path.ref_path[1][index_pre];
        double x2 = path.ref_path[1][index_cur];
        double x3 = path.ref_path[1][index_nxt];

        double y1 = path.ref_path[2][index_pre];
        double y2 = path.ref_path[2][index_cur];
        double y3 = path.ref_path[2][index_nxt];

        double denominator = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

        double x_center = ((pow(x1, 2) + pow(y1, 2)) * (y2 - y3) + 
                           (pow(x2, 2) + pow(y2, 2)) * (y3 - y1) + 
                           (pow(x3, 2) + pow(y3, 2)) * (y1 - y2)) / denominator;

        double y_center = ((pow(x1, 2) + pow(y1, 2)) * (x3 - x2) + 
                           (pow(x2, 2) + pow(y2, 2)) * (x1 - x3) + 
                           (pow(x3, 2) + pow(y3, 2)) * (x2 - x1)) / denominator;
        
        double r = sqrt(pow((x1 - x_center), 2) + pow((y1 - y_center), 2));
        double kappa = 1 / r;

        std::vector<double> v1(2, 0.0);
        std::vector<double> v2(2, 0.0);
        v1[0] = x2 - x1;
        v1[1] = y2 - y1;
        v2[0] = x3 - x2;
        v2[1] = y3 - y2;

        double cross_product = v1[0] * v2[1] - v1[1] * v2[0];
        if(cross_product <= 0){
            kappa *= -1;
        }
        path.ref_path[5][i] = kappa;
    }
}