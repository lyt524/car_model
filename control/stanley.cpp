#include <iostream>
#include <cstddef>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include "stanley.h"
#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"
#include "../referencepath/point.h"
#include "../tools/mathtools.h"


int Stanley::FindNearestIndex(KiCar& ki_car, RefPath& ref_path){
    // egoPose base on front axle
    Eigen::Vector2d egoPose(ki_car.GetX() + ki_car.GetL() * cos(ki_car.GetYaw()), 
                            ki_car.GetY() + ki_car.GetL() * sin(ki_car.GetYaw()));
    double minDis = std::numeric_limits<double>::infinity();

    int startIndex = ref_path.lastNearestPointIndex;
    int foundIndex = startIndex;

    // continueFindCntNum 的值可能需要动态调整
    int continueFindCntNum = 20;
    if(ref_path.point_num - startIndex < continueFindCntNum){
        continueFindCntNum = ref_path.point_num - startIndex;
    }

    int continueFindCnt = continueFindCntNum;

    for(int i = startIndex; i < ref_path.point_num; i++){
        Eigen::Vector2d pathPose(ref_path.GetPointX(i), ref_path.GetPointY(i));
        double curDis = (pathPose - egoPose).norm();
        if(curDis < minDis){
            foundIndex = i;
            minDis = curDis;
            continueFindCnt = continueFindCntNum;
        }
        else{
            continueFindCnt --;
        }
        if(continueFindCnt <= 0) break;
    }

    ref_path.lastNearestPointIndex = foundIndex;
    std::cout << "matchedIndex = " << foundIndex <<std::endl;
    return foundIndex;
}

double Stanley::CalHeadingError(KiCar& ki_car, RefPath& ref_path){
    double phiError = ki_car.GetYaw() - ref_path.GetPointPhi(ref_path.lastNearestPointIndex);
    NormalizeAngle(phiError);
    this->headingError = phiError;
    std::cout << "phiError = " << phiError << std::endl;
    return phiError;
}

double Stanley::CalLateralError(KiCar& ki_car, RefPath& ref_path){
    Eigen::Vector2d egoPose(ki_car.GetX() + ki_car.GetL() * cos(ki_car.GetYaw()), 
                            ki_car.GetY() + ki_car.GetL() * sin(ki_car.GetYaw()));

    Eigen::Vector2d matchedPathPose(ref_path.GetPointX(ref_path.lastNearestPointIndex),
                                    ref_path.GetPointY(ref_path.lastNearestPointIndex));
    
    Eigen::Vector2d matchedPathMinusEgo = matchedPathPose - egoPose;

    Eigen::Vector2d egoYawRote90ClockWise(cos(ki_car.GetYaw() - M_PI_2),
                                          sin(ki_car.GetYaw() - M_PI_2));

    double lateralError = matchedPathMinusEgo.dot(egoYawRote90ClockWise);
    this->lateralError = lateralError;
    std::cout << "lateralError = " << lateralError << std::endl;
    return lateralError;
}

double Stanley::StanleyControl(KiCar& ki_car, RefPath& ref_path){
    int matchedIndex = FindNearestIndex(ki_car, ref_path);
    double alpha = atan2(this->stanleyK * CalLateralError(ki_car, ref_path), ki_car.GetV());
    double phiError = CalHeadingError(ki_car, ref_path);

    double deltaF = -(alpha + phiError);
    NormalizeAngle(deltaF);
    this->deltaF = deltaF;
    std::cout << "deltaF = " << deltaF << std::endl;
    return deltaF;
}


double Stanley::GetLateralError(){
    return this->lateralError;
}

double Stanley::GetHeadingError(){
    return this->headingError;
}

void Stanley::WriteControlResult(std::ofstream& outFile){
    if (outFile.is_open()) {
        outFile << this->deltaF << " "
        << this->headingError << " "
        << this->lateralError << " "
        << std::endl;
        std::cout << "File written successfully." << std::endl;
    } else {
        std::cout << "Error opening the stanley control record file." << std::endl;
    }
}
