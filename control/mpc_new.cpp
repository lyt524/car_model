#include "mpc_new.h"


Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int n) {
    int size = A.rows();
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(size, size);  // 初始化为单位矩阵
    Eigen::MatrixXd base = A;

    while (n > 0) {
        if (n % 2 == 1) {
            result *= base;
        }
        base *= base;
        n /= 2;
    }
    return result;
}

MPC::MPC(KiCar& car,
         RefPath& ref_path
        ):car_(car), ref_path_(ref_path){
    this->Q.resize(this->Nx_ * this->Np_, this->Nx_ * this->Np_);
    this->R.resize(this->Nu_ * this->Np_, this->Nu_ * this->Np_);
    this->kesi.resize(this->Nx_ + this->Nu_, 1);
    this->PHI.resize(this->Nx_ * this->Np_, this->Nx_ + this->Nu_);
    this->THETA.resize(this->Nx_ * this->Np_, this->Nu_ * this->Np_);
    this->H.resize(this->Nu_ * this->Np_, this->Nu_ * this->Np_);
    this->F.resize(this->Nu_ * this->Np_);
    // this->Ad.resize(this->Nu_ * this->Np_, this->Nu_ * this->Np_);
    // this->ld.resize(this->Nu_ * this->Np_);
    // this->ud.resize(this->Nu_ * this->Np_);
    
    // this->Ad.setIdentity();
    // this->ld.setConstant(-1);
    // this->ud.setConstant(1);

    // this->Ad_sparse = this->Ad.sparseView();

    // std::cout << "________" << std::endl;
    // std::cout << "Ad = " << std::endl;
    // std::cout << Ad << std::endl;
    // std::cout << "ld = " << std::endl;
    // std::cout << ld << std::endl;
    // std::cout << "ud = " << std::endl;
    // std::cout << ud << std::endl;
}

// Do both linearization and discretization at ego pos
void MPC::setBicycleModel(){
    this->A1 << 1, 0, -car_.GetV() * this->dt * sin(car_.GetYaw()),
                0, 1, car_.GetV() * this->dt * cos(car_.GetYaw()),
                0, 0, 1;

    this->B1 << this->dt * cos(car_.GetYaw()), 0,
                this->dt * sin(car_.GetYaw()), 0,
                tan(car_.GetDeltaF()) * this->dt / car_.GetL(), 
                car_.GetV()* this->dt / car_.GetL() / pow(cos(car_.GetYaw()), 2);
    
    // std::cout << "________" << std::endl;
    // std::cout << "A1 = " << std::endl;
    // std::cout << A1 << std::endl;
    // std::cout << "B1 = " << std::endl;
    // std::cout << B1 << std::endl;
}

void MPC::FindRefPos(){
    // egoPose based on rear axle
    Eigen::Vector2d egoPose(car_.GetX(), car_.GetY());
    double minDis = std::numeric_limits<double>::infinity();

    int startIndex = ref_path_.lastNearestPointIndex;
    int foundIndex = startIndex;

    int continueFindCntNum = 20;
    if(ref_path_.point_num - startIndex < continueFindCntNum){
        continueFindCntNum = ref_path_.point_num - startIndex;
    }

    int continueFindCnt = continueFindCntNum;

    for(int i = startIndex; i < ref_path_.point_num; i++){
        Eigen::Vector2d pathPose(ref_path_.GetPointX(i), ref_path_.GetPointY(i));
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

    std::cout << "________" << std::endl;
    std::cout << "foundIndex = " << std::endl;
    std::cout << foundIndex << std::endl;
}

void MPC::setKesi(){
    double x0 = car_.GetX();
    double y0 = car_.GetY();
    double yaw0 = car_.GetYaw();
    double v0 = car_.GetV();
    double delta_f0 = car_.GetDeltaF();

    double x_ref = ref_path_.GetPointX(ref_path_.lastNearestPointIndex);
    double y_ref = ref_path_.GetPointY(ref_path_.lastNearestPointIndex);
    double yaw_ref = ref_path_.GetPointPhi(ref_path_.lastNearestPointIndex);
    double v_ref = 2;
    double delta_f_ref = 0;

    this->kesi << x0 - x_ref,
                  y0 - y_ref,
                  yaw0 - yaw_ref,
                  v0 - v_ref,
                  delta_f0 - delta_f_ref;

    std::cout << "________" << std::endl;
    std::cout << "kesi = " << std::endl;
    std::cout << kesi << std::endl;
}

void MPC::setQR(){
    this->Q = 10 * Eigen::MatrixXd::Identity(this->Nx_ * this->Np_, this->Nx_ * this->Np_);
    this->R = 10 * Eigen::MatrixXd::Identity(this->Nu_ * this->Np_, this->Nu_ * this->Np_);
    // std::cout << "________" << std::endl;
    // std::cout << "Q = " << std::endl;
    // std::cout << Q << std::endl;
    // std::cout << "R = " << std::endl;
    // std::cout << R << std::endl;
}

void MPC::setABC(){
    this->A.block(0, 0, 3, 3) = this->A1;
    this->A.block(0, 3, 3, 2) = this->B1;
    this->A.block(3, 0, 2, 3) = Eigen::MatrixXd::Zero(2, 3);
    this->A.block(3, 3, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    this->B.block(0, 0, 3, 2) = this->B1;
    this->B.block(3, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

    this->C.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    this->C.block(0, 3, 3, 2) = Eigen::MatrixXd::Zero(3, 2);

    // std::cout << "________" << std::endl;
    // std::cout << "A = " << std::endl;
    // std::cout << A << std::endl;
    // std::cout << "B = " << std::endl;
    // std::cout << B << std::endl;
    // std::cout << "C = " << std::endl;
    // std::cout << C << std::endl;
}

void MPC::calMPC(){
    FindRefPos();
    setBicycleModel();
    setKesi();
    setQR();
    setABC();
    setPHI_THETA();
    setH_F();
    setConstrains();

    // std::cout << "Ad_sparse rows: " << Ad_sparse.rows() << std::endl;
    // std::cout << "Ad_sparse cols: " << Ad_sparse.cols() << std::endl;

    // ld = Eigen::VectorXd::Constant(Nu_ * Np_, -1); // -1 对应下限
    // ud = Eigen::VectorXd::Constant(Nu_ * Np_, 1);  // 1 对应上限


    // setMats(H_sparse, F, Ad_sparse, ld, ud);
    // if(getRes()){
    //     std::cout << "________" << std::endl;
    //     std::cout << "this->solution_ = " << std::endl;
    //     std::cout << this->solution_ << std::endl;
    // }
}

void MPC::setPHI_THETA(){
    for(int i = 0; i < this->Np_; ++i){
        if(i == 0){
            this->PHI.block(0, 0, 3, 5) = C * A;
            this->THETA.block(0, 0, 3, 2) = C * B;
        }else{
            for(int j = 0; j < this->Np_; ++j){
                if(i >= j){
                    this->THETA.block(i * 3, j * 2, 3, 2) = C * matrixPower(A, i-j) * B;
                }else{
                    this->THETA.block(i * 3, j * 2, 3, 2) = Eigen::MatrixXd::Zero(3, 2);
                }
            }
            this->PHI.block(i * 3, 0, 3, 5) = this->PHI.block((i-1) * 3, 0, 3, 5) * A;
        }
    }

    // std::cout << "________" << std::endl;
    // std::cout << "THETA = " << std::endl;
    // std::cout << THETA << std::endl;
    // std::cout << "PHI = " << std::endl;
    // std::cout << PHI << std::endl;
}

void MPC::setH_F(){
    this->H = (this->THETA).transpose() * this->Q * this->THETA + this->R;
    this->F = 2 * (this->PHI * this->kesi).transpose() * this->Q * this->THETA;
    this->H_sparse = this->H.sparseView();

    // std::cout << "________" << std::endl;
    // std::cout << "H = " << std::endl;
    // std::cout << H << std::endl;
    // std::cout << "F = " << std::endl;
    // std::cout << F << std::endl;
}

void MPC::setConstrains(){
    Eigen::MatrixXd A_t = Eigen::MatrixXd::Zero(this->Np_, this->Np_);
    for (int i = 0; i < this->Np_; ++i) {
        for (int j = 0; j < this->Np_; ++j) {
            if (i >= j) {
                A_t(i, j) = 1;
            } else {
                A_t(i, j) = 0;
            }
        }
    }
    Eigen::MatrixXd A_I = Eigen::kroneckerProduct(A_t, Eigen::MatrixXd::Identity(this->Nu_, this->Nu_));
    // std::cout << "________" << std::endl;
    // std::cout << "A_I = " << std::endl;
    // std::cout << A_I << std::endl;

    Eigen::VectorXd U(this->Nu_);
    // U.resize(this->Nu_);
    double v_ref = 2;
    double delta_f_ref = 0;
    double dv = car_.GetV() - v_ref;
    double d_deltaf = car_.GetDeltaF() - delta_f_ref;
    U << dv, d_deltaf;
    Eigen::VectorXd Ut = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(this->Np_), U);

    Eigen::VectorXd umax(2), umin(2), umax_dt(2), umin_dt(2);
    umax << 0.2, 0.44;
    umin << 0.0, -0.44;
    umax_dt << 0.05, 0.005;
    umin_dt << -0.05, -0.005;

    Eigen::VectorXd Umax = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(this->Np_), umax);
    Eigen::VectorXd Umin = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(this->Np_), umin);
    Eigen::VectorXd Umax_dt = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(this->Np_), umax_dt);
    Eigen::VectorXd Umin_dt = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(this->Np_), umin_dt);
    // std::cout << "________" << std::endl;
    // std::cout << "Umax = " << std::endl;
    // std::cout << Umax << std::endl;
    // std::cout << "________" << std::endl;
    // std::cout << "Umin = " << std::endl;
    // std::cout << Umin << std::endl;
    // std::cout << "________" << std::endl;
    // std::cout << "Umax_dt = " << std::endl;
    // std::cout << Umax_dt << std::endl;
    // std::cout << "________" << std::endl;
    // std::cout << "Umin_dt = " << std::endl;
    // std::cout << Umin_dt << std::endl;

    Eigen::MatrixXd A_cons(this->Nu_ * this->Np_ * 4, this->Nu_ * this->Np_);
    A_cons << A_I, -A_I, -1*Eigen::MatrixXd::Identity(20, 20), Eigen::MatrixXd::Identity(20, 20);
    Eigen::SparseMatrix<double> A_ = A_cons.sparseView();
    
    // std::cout << "________" << std::endl;
    // std::cout << "A_cons = " << std::endl;
    // std::cout << A_cons << std::endl;

    Eigen::VectorXd lb(this->Nu_ * this->Np_), ub(this->Nu_ * this->Np_);
    lb << Umin_dt;
    ub << Umax_dt;

    Eigen::VectorXd B_cons(4 * this->Nu_ * this->Np_);
    B_cons << Umax - Ut, -(Umin - Ut), -1*lb, ub;

    // std::cout << "________" << std::endl;
    // std::cout << "B_cons = " << std::endl;
    // std::cout << B_cons << std::endl;

    // std::cout << "________" << std::endl;
    // std::cout << "lb = " << std::endl;
    // std::cout << lb << std::endl;

    // std::cout << "________" << std::endl;
    // std::cout << "ub = " << std::endl;
    // std::cout << ub << std::endl;

    // std::cout << "________" << std::endl;
    // std::cout << "H_sparse.rows = " << std::endl;
    // std::cout << H_sparse.rows() << std::endl;
    // std::cout << "H_sparse.cols = " << std::endl;
    // std::cout << H_sparse.cols() << std::endl;
    
    // std::cout << "F.rows = " << std::endl;
    // std::cout << F.rows() << std::endl;
    // std::cout << "F.cols = " << std::endl;
    // std::cout << F.cols() << std::endl;

    // std::cout << "A_.rows = " << std::endl;
    // std::cout << A_.rows() << std::endl;
    // std::cout << "A_.cols = " << std::endl;
    // std::cout << A_.cols() << std::endl;

    // std::cout << "lb.rows = " << std::endl;
    // std::cout << lb.rows() << std::endl;
    // std::cout << "lb.cols = " << std::endl;
    // std::cout << lb.cols() << std::endl;

    // std::cout << "B_cons.rows = " << std::endl;
    // std::cout << B_cons.rows() << std::endl;
    // std::cout << "B_cons.cols = " << std::endl;
    // std::cout << B_cons.cols() << std::endl;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H);
    if (es.eigenvalues().minCoeff() < 0) {
        std::cerr << "Hessian matrix is not positive definite!" << std::endl;
    }

    setMats(H_sparse, F, A_, lb, B_cons);
    getRes();
    this->d_v = this->solution_[0];
    this->d_delta_f = this->solution_[1];
}

void MPC::setMats(Eigen::SparseMatrix<double> P_, 
            Eigen::VectorXd q_d,
            Eigen::SparseMatrix<double> A_,
            Eigen::VectorXd l_d,
            Eigen::VectorXd u_d){

    qpSolver_.data()->clearHessianMatrix();
    qpSolver_.data()->clearLinearConstraintsMatrix();
    qpSolver_.clearSolver();

    qpSolver_.data()->setNumberOfVariables(this->Nu_ * this->Np_);
    qpSolver_.data()->setNumberOfConstraints(this->Nu_ * this->Np_ * 4);
    qpSolver_.data()->setNumberOfConstraints(0);
    qpSolver_.data()->setHessianMatrix(P_);
    qpSolver_.data()->setGradient(q_d);
    qpSolver_.data()->setLinearConstraintsMatrix(A_);
    // qpSolver_.data()->setLowerBound(l_d);
    qpSolver_.data()->setUpperBound(u_d);
}

bool MPC::getRes(){
    qpSolver_.settings()->setVerbosity(false);
    qpSolver_.settings()->setWarmStart(false);

    if (!qpSolver_.initSolver()) {
        std::cerr << "Solver initialization failed!" << std::endl;
        return false;
    }

    if (qpSolver_.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        this->solution_ = qpSolver_.getSolution();

        std::cout << "Solution: " << this->solution_ << std::endl;
        return true;
    } else {
        std::cerr << "QP solving failed!" << std::endl;
        return false;
    }
}