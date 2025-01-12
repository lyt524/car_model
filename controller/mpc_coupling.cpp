#include "mpc_coupling.h"

MPC::MPC(KiCar& ego_car, RefPath& ref_path):
         ego_car_(ego_car), ref_path_(ref_path){
    
}

void MPC::setKinematicsMatrices() {
    // this->A_ << 1, 0, -ego_car_.getV() * dt_ * sin(ego_car_.getYaw()),
    //             0, 1, ego_car_.getV() * dt_ * cos(ego_car_.getYaw()),
    //             0, 0, 1;
 
    // this->B_ << dt_ * cos(ego_car_.getYaw()), 0,
    //             dt_ * sin(ego_car_.getYaw()), 0,
    //             tan(ego_car_.getDeltaF()) * dt_ / ego_car_.getL(), ego_car_.getV() * dt_ / ego_car_.getL() / pow(cos(ego_car_.getDeltaF()), 2);

    this->A_ << 1, 0, -ego_car_.getV() * dt_ * sin(ref_path_.getPointPhi(ref_path_.lastNearestPointIndex)),
                0, 1, ego_car_.getV() * dt_ * cos(ref_path_.getPointPhi(ref_path_.lastNearestPointIndex)),
                0, 0, 1;
 
    this->B_ << dt_ * cos(ref_path_.getPointPhi(ref_path_.lastNearestPointIndex)), 0,
                dt_ * sin(ref_path_.getPointPhi(ref_path_.lastNearestPointIndex)), 0,
                tan(0.) * dt_ / ego_car_.getL(), ego_car_.getV() * dt_ / ego_car_.getL() / pow(cos(0.), 2);
}

void MPC::setInequalityConstraints() {
    this->uMin_ << -0.1, -0.4;
    this->uMax_ << 0.1, 0.4;
 
    this->xMin_ << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    this->xMax_ << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
}

void MPC::setWeightMatrices() {
    this->Q_.diagonal() << 10., 10., 100.;
    this->R_.diagonal() << 10., 1000.;
}

void MPC::castMPCToQPHessian() {
    this->hessian_.resize(Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_);
 
    for (int i = 0; i < Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_; i++) {
        if (i < Nx_ * (mpcWindow_ + 1)) {
            int posQ = i % Nx_;
            float value = this->Q_.diagonal()[posQ];
            if (value != 0) this->hessian_.insert(i, i) = value;
        }
        else {
            int posR = i % Nu_;
            float value = this->R_.diagonal()[posR];
            if (value != 0) this->hessian_.insert(i, i) = value;
        }
    }
}

void MPC::castMPCToQPGradient() {
    Eigen::Matrix<double, Nx_, 1> Qx_ref;
    Qx_ref = this->Q_ * (-this->xRef_);

    this->gradient_ = Eigen::VectorXd::Zero(Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, 1);
    for (int i = 0; i < Nx_ * (mpcWindow_ + 1); i++) {
        int posQ = i % Nx_;
        float value = Qx_ref(posQ, 0);
        this->gradient_(i, 0) = value;
    }
}

void MPC::castMPCToQPConstraintMatrix() {
    this->linearMatrix_.resize(Nx_ * (mpcWindow_ + 1) + Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_,
                               Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_);
 
    for (int i = 0; i < Nx_ * (mpcWindow_ + 1); i++) {
        this->linearMatrix_.insert(i, i) = -1;
    }

    for (int i = 0; i < mpcWindow_; i++) {
        for (int j = 0; j < Nx_; j++) {
            for (int k = 0; k < Nx_; k++) {
                float value = this->A_(j, k);
                if (value != 0) this->linearMatrix_.insert(Nx_ * (i + 1) + j, Nx_ * i + k) = value;
            }
        }
    }

    for (int i = 0; i < mpcWindow_; i++) {
        for (int j = 0; j < Nx_; j++) {
            for (int k = 0; k < Nu_; k++) {
                float value = this->B_(j, k);
                if (value != 0) {
                    this->linearMatrix_.insert(Nx_ * (i + 1) + j, Nu_ * i + k + Nx_ * (mpcWindow_ + 1)) = value;
                }
            }
        }
    }
 
    for (int i = 0; i < Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_; i++) {
        this->linearMatrix_.insert(i + (mpcWindow_ + 1) * Nx_, i) = 1;
    }
}

void MPC::castMPCToQPConstraintVectors() {
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, 1);
    for (int i = 0; i <= mpcWindow_; i++) {
        lowerInequality.block(Nx_ * i, 0, Nx_, 1) = this->xMin_;
        upperInequality.block(Nx_ * i, 0, Nx_, 1) = this->xMax_;
    }
    for (int i = 0; i < mpcWindow_; i++) {
        lowerInequality.block(Nu_ * i + Nx_ * (mpcWindow_ + 1), 0, Nu_, 1) = this->uMin_;
        upperInequality.block(Nu_ * i + Nx_ * (mpcWindow_ + 1), 0, Nu_, 1) = this->uMax_;
    }
 
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow_ + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, Nx_, 1) = - this->x0_;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;
 
    this->lowerBound_ = Eigen::MatrixXd::Zero(2 * Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, 1);
    this->lowerBound_ << lowerEquality, lowerInequality;
    this->upperBound_ = Eigen::MatrixXd::Zero(2 * Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_, 1);
    this->upperBound_ << upperEquality, upperInequality;
}

void MPC::updateConstraintVectors(){
    this->lowerBound_.block(0, 0, Nx_, 1) = -this->x0_;
    this->upperBound_.block(0, 0, Nx_, 1) = -this->x0_;
}

void MPC::findRefPos(){
    // egoPose based on rear axle
    Eigen::Vector2d egoPose(ego_car_.getX(), ego_car_.getY());
    double minDis = std::numeric_limits<double>::infinity();

    int startIndex = ref_path_.lastNearestPointIndex;
    int foundIndex = startIndex;

    int continueFindCntNum = 20;
    if(ref_path_.point_num - startIndex < continueFindCntNum){
        continueFindCntNum = ref_path_.point_num - startIndex;
    }

    int continueFindCnt = continueFindCntNum;

    for(int i = startIndex; i < ref_path_.point_num; i++){
        Eigen::Vector2d pathPose(ref_path_.getPointX(i), ref_path_.getPointY(i));
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

    ref_path_.lastNearestPointIndex = foundIndex;

    std::cout << "_________________" << std::endl;
    std::cout << "foundIndex = " <<  foundIndex << std::endl;
    std::cout << "_________________" << std::endl;
}

void MPC::setX0(){
    this->x0_ << ego_car_.getX(), ego_car_.getY(), ego_car_.getYaw();
}

void MPC::setXRef(){
    findRefPos();
    this->xRef_ << ref_path_.getPointX(ref_path_.lastNearestPointIndex), 
                   ref_path_.getPointY(ref_path_.lastNearestPointIndex),
                   ref_path_.getPointPhi(ref_path_.lastNearestPointIndex);
}

bool MPC::setSolver(){
    this->solver_.settings()->setVerbosity(false);
    this->solver_.settings()->setWarmStart(true);

    this->solver_.data()->setNumberOfVariables(Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_);
    this->solver_.data()->setNumberOfConstraints(2 * Nx_ * (mpcWindow_ + 1) + Nu_ * mpcWindow_);

    if (!this->solver_.data()->setHessianMatrix(this->hessian_)) return false;
    if (!this->solver_.data()->setGradient(this->gradient_)) return false;
    if (!this->solver_.data()->setLinearConstraintsMatrix(this->linearMatrix_)) return false;
    if (!this->solver_.data()->setLowerBound(this->lowerBound_)) return false;
    if (!this->solver_.data()->setUpperBound(this->upperBound_)) return false;
    if (!this->solver_.initSolver()) return false;
    return true;
}

bool MPC::resetSolver(){
    if (!this->solver_.updateBounds(this->lowerBound_, this->upperBound_)) return false;
    if (!this->solver_.updateGradient(this->gradient_)) return false;
    if (!this->solver_.updateLinearConstraintsMatrix(this->linearMatrix_)) return false;
    return true;
}

bool MPC::getQPResult(){
    if (this->solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    this->QPSolution_ = this->solver_.getSolution();
    this->ctr_ = this->QPSolution_.block(Nx_ * (mpcWindow_ + 1), 0, Nu_, 1);
    return true;
}

void MPC::qpInit(){
    setX0();
    setXRef();

    setWeightMatrices();
    setKinematicsMatrices();
    setInequalityConstraints();

    castMPCToQPHessian();
    castMPCToQPGradient();
    castMPCToQPConstraintMatrix();
    castMPCToQPConstraintVectors();

    setSolver();
 }

void MPC::writeControlResult(std::ofstream& outFile){
    if (outFile.is_open()) {
        this->headingError = ego_car_.getYaw() - ref_path_.getPointPhi(ref_path_.lastNearestPointIndex);
        calLateralError();
        this->d_delta_f = this->ctr_[1];

        outFile << this->d_delta_f << " "
        // outFile << ego_car_.getDeltaF() + this->d_delta_f << " "
        << this->headingError << " "
        << this->lateralError << " "
        << std::endl;
        std::cout << "File written successfully." << std::endl;
    } else {
        std::cout << "Error opening the mpc control record file." << std::endl;
    }
}

void MPC::calLateralError(){
    Eigen::Vector2d egoPose(ego_car_.getX() + ego_car_.getL() * cos(ego_car_.getYaw()), 
                            ego_car_.getY() + ego_car_.getL() * sin(ego_car_.getYaw()));

    Eigen::Vector2d matchedPathPose(ref_path_.getPointX(ref_path_.lastNearestPointIndex),
                                    ref_path_.getPointY(ref_path_.lastNearestPointIndex));
    
    Eigen::Vector2d matchedPathMinusEgo = matchedPathPose - egoPose;

    Eigen::Vector2d egoYawRote90ClockWise(cos(ego_car_.getYaw() - M_PI_2),
                                          sin(ego_car_.getYaw() - M_PI_2));

    double lateralError = matchedPathMinusEgo.dot(egoYawRote90ClockWise);
    this->lateralError = lateralError;
}
