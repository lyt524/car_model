#include "stanley.h"


int Stanley::findNearestIndex(KiCar& ki_car, RefPath& ref_path){
    // egoPose base on front axle
    Eigen::Vector2d egoPose(ki_car.getX() + ki_car.getL() * cos(ki_car.getYaw()), 
                            ki_car.getY() + ki_car.getL() * sin(ki_car.getYaw()));
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
        Eigen::Vector2d pathPose(ref_path.getPointX(i), ref_path.getPointY(i));
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

double Stanley::calHeadingError(KiCar& ki_car, RefPath& ref_path){
    double phiError = ki_car.getYaw() - ref_path.getPointPhi(ref_path.lastNearestPointIndex);
    NormalizeAngle(phiError);
    this->heading_error_ = phiError;
    std::cout << "phiError = " << phiError << std::endl;
    return phiError;
}

double Stanley::calLateralError(KiCar& ki_car, RefPath& ref_path){
    Eigen::Vector2d egoPose(ki_car.getX() + ki_car.getL() * cos(ki_car.getYaw()), 
                            ki_car.getY() + ki_car.getL() * sin(ki_car.getYaw()));

    Eigen::Vector2d matchedPathPose(ref_path.getPointX(ref_path.lastNearestPointIndex),
                                    ref_path.getPointY(ref_path.lastNearestPointIndex));
    
    Eigen::Vector2d matchedPathMinusEgo = matchedPathPose - egoPose;

    Eigen::Vector2d egoYawRote90ClockWise(cos(ki_car.getYaw() - M_PI_2),
                                          sin(ki_car.getYaw() - M_PI_2));

    double lateral_error_ = matchedPathMinusEgo.dot(egoYawRote90ClockWise);
    this->lateral_error_ = lateral_error_;
    std::cout << "lateral_error_ = " << lateral_error_ << std::endl;
    return lateral_error_;
}

double Stanley::stanleyControl(KiCar& ki_car, RefPath& ref_path){
    int matchedIndex = findNearestIndex(ki_car, ref_path);
    double alpha = atan2(this->stanley_K_ * calLateralError(ki_car, ref_path), ki_car.getV());
    double phiError = calHeadingError(ki_car, ref_path);

    double delta_f_ = -(alpha + phiError);
    NormalizeAngle(delta_f_);
    this->delta_f_ = delta_f_;
    std::cout << "delta_f_ = " << delta_f_ << std::endl;
    return delta_f_;
}

double Stanley::getLateralError(){ return this->lateral_error_; }
double Stanley::getHeadingError(){ return this->heading_error_; }

void Stanley::writeControlResult(std::ofstream& outFile){
    if (outFile.is_open()) {
        outFile << this->delta_f_ << " "
        << this->heading_error_ << " "
        << this->lateral_error_ << " "
        << std::endl;
        std::cout << "File written successfully." << std::endl;
    } else {
        std::cout << "Error opening the stanley control record file." << std::endl;
    }
}
