#include "mpc_coupling.h"


MpcController::MpcController(Bicycle& car_model,  // mpc model
                  KiCar& pre_car,                 // predict model
                  KiCar& ego_car,
                  QPSolver& qpSolver,
                  RefPath& ref_path,
                  int pre_step,
                  double v_max,
                  double a_max,
                  double delta_f_max,
                  double ddelta_f_max)
    : car_model_(car_model), pre_car_(pre_car), ego_car_(ego_car), qpSolver_(qpSolver), ref_path_(ref_path){
    this->N_ = pre_step;
    this->v_max_ = v_max;
    this->a_max_ = a_max;
    this->delta_f_max_ = delta_f_max;
    this->ddelta_f_max_ = ddelta_f_max;

    std::cout << "MpcController: " << endl;
    std::cout << "state variables size: " << this->n << endl
              << "control variables size: " << this->m << endl
              << "predect step: " << this->N_ << endl
              << endl;
}

void MpcController::SetConstrains(){
    /* 
    *               /  x1  \
    *               |  x2  |
    *  lx_ <=  Cx_  |  x3  |  <= ux_
    *               | ...  |
    *               \  xN  /
    *
    *               /  u0  \
    *               |  u1  |
    *  lu_ <=  Cu_  |  u2  |  <= uu_
    *               | ...  |
    *               \ uN-1 /
    */
    
    // v constrains
    this->Cx_.resize(1 * this->N_, n * this->N_);
    this->lx_.resize(1 * this->N_, 1);
    this->ux_.resize(1 * this->N_, 1);

    // a delta_f ddelta_f constrains
    this->Cu_.resize(2 * this->N_, m * this->N_);
    this->lu_.resize(2 * this->N_, 1);
    this->uu_.resize(2 * this->N_, 1);

    // this->Cu_.resize(3 * this->N_, m * this->N_);
    // this->lu_.resize(3 * this->N_, 1);
    // this->uu_.resize(3 * this->N_, 1);

    for (int i = 0; i < this->N_; ++i) {
        this->Cu_.coeffRef(i * 2 + 0, i * m + 0) = 1;
        this->lu_.coeffRef(i * 2 + 0, 0) = -this->a_max_;
        this->uu_.coeffRef(i * 2 + 0, 0) = this->a_max_;

        this->Cu_.coeffRef(i * 2 + 1, i * m + 1) = 1;
        this->lu_.coeffRef(i * 2 + 1, 0) = -this->delta_f_max_;
        this->uu_.coeffRef(i * 2 + 1, 0) = this->delta_f_max_;

        // // -a_max <= a <= a_max for instance:
        // this->Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
        // this->lu_.coeffRef(i * 3 + 0, 0) = -this->a_max_;
        // this->uu_.coeffRef(i * 3 + 0, 0) = this->a_max_;

        // this->Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
        // this->lu_.coeffRef(i * 3 + 1, 0) = -this->delta_f_max_;
        // this->uu_.coeffRef(i * 3 + 1, 0) = this->delta_f_max_;
 
        // this->Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
        // if (i > 0){
        //     this->Cu_.coeffRef(i * 3 + 2, (i - 1) * m + 1) = -1;
        // }
        // this->lu_.coeffRef(i * 3 + 2, 0) = -this->ddelta_f_max_ * this->car_model_.dt_;
        // this->uu_.coeffRef(i * 3 + 2, 0) = this->ddelta_f_max_ * this->car_model_.dt_;
 
        // -v_max <= v <= v_max
        Cx_.coeffRef(i, i * n + 3) = 1;
        lx_.coeffRef(i, 0) = -v_max_;
        ux_.coeffRef(i, 0) = v_max_;
    }

    // Prepare other matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    Qx_.resize(n * N_, n * N_);

    Qx_.setIdentity();
    for (int i = 1; i < N_; ++i) {
      Qx_.coeffRef(i * n - 2, i * n - 2) = rho_;
      Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
    }
    Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
    Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
    Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;

    A_.resize(n_cons * N_, m * N_);
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
}


void MpcController::FindRefPos(){
    // egoPose based on rear axle
    Eigen::Vector2d egoPose(pre_car_.GetX(), pre_car_.GetY());
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
    std::cout << "matchedIndex = " << foundIndex <<std::endl;
    this->ref_path_match_index_ = foundIndex;
    this->ref_x_ = ref_path_.GetPointX(foundIndex);
    this->ref_y_ = ref_path_.GetPointY(foundIndex);
    this->ref_phi_ = ref_path_.GetPointPhi(foundIndex);
    this->ref_v_ = ref_path_.GetPointV(foundIndex);

    std::cout << "_______________________" << std::endl;
    std::cout << "ref_x_ = " << ref_x_ << std::endl;
    std::cout << "ref_y_ = " << ref_y_ << std::endl;
    std::cout << "ref_phi_ = " << ref_phi_ << std::endl;
    std::cout << "ref_v_ = " << ref_v_ << std::endl;
}

void MpcController::SolveQP(){
    // set BB, AA, gg size
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);

    // set qx size
    Eigen::SparseMatrix<double> qx;
    qx.resize(n * N_, 1);

    // TODO: VectorX x0 = compensateDelay(x0_observe_);

    // initial state variables
    GetCarState();
    VectorX x0;
    x0 << this->ego_x_, this->ego_y_, this->ego_phi_, this->ego_v_;

    this->pre_car_ = this->ego_car_;  // copy car

    for(int i = 0; i < N_; ++i){
        // set car_model_.Bd_, Ad_, gd_
        if(i == 0){
            this->car_model_.linearization(this->ego_phi_, this->ego_v_, this->ego_delta_f_);
        }else{
            this->car_model_.linearization(this->pre_car_.GetYaw(), this->pre_car_.GetV(), this->pre_car_.GetDeltaF());
        }
        std::cout << "_______________________" << endl;
        std::cout << "car_model_.Bd_" << endl;
        std::cout << this->car_model_.Bd_ << endl;

        std::cout << "_______________________" << endl;
        std::cout << "car_model_.Ad_" << endl;
        std::cout << this->car_model_.Ad_ << endl;

        std::cout << "_______________________" << endl;
        std::cout << "car_model_.gd_" << endl;
        std::cout << this->car_model_.gd_ << endl;

        // calculate big state-space matrices
        /*
        *                BB                  AA
        * x1    /       B    0  ... 0 \    /   A \
        * x2    |      AB    B  ... 0 |    |  A2 |
        * x3  = |    A^2B   AB  ... 0 |U + | ... |x0 + gg
        * ...   |     ...  ...  ... 0 |    | ... |
        * xN    \A^(n-1)B  ...  ... B /    \ A^N /
        *
        *     X = BB * U + AA * x0 + gg
        */
        if(i == 0){
            BB.block(0, 0, n, m) = this->car_model_.Bd_;
            AA.block(0, 0, n, n) = this->car_model_.Ad_;
            gg.block(0, 0, n, 1) = this->car_model_.gd_;
        }else{
            BB.block(i * n, i * m, n, m) = this->car_model_.Bd_;
            for(int j = i - 1; j >= 0; --j){
                BB.block(i * n, j * m, n, m) = this->car_model_.Ad_ * BB.block((i - 1) * n, j * m, n, m);
            }
            AA.block(i * n, 0, n, n) = this->car_model_.Ad_ * AA.block((i - 1) * n, 0, n, n);
            gg.block(i * n, 0, n, 1) = this->car_model_.Ad_ * gg.block((i - 1) * n, 0, n, 1) + this->car_model_.gd_;
        }

        // cost function should be represented as follows:
        /*
        *           /  x1  \T       /  x1  \         /  x1  \
        *           |  x2  |        |  x2  |         |  x2  |
        *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
        *           | ...  |        | ...  |         | ...  |
        *           \  xN  /        \  xN  /         \  xN  / 
        */
        // qx = -Qx_^T * x_ref

        FindRefPos();
        qx.coeffRef(i * n + 0, 0) = -Qx_.coeffRef(i * n + 0, i * n + 0) * this->ref_x_;
        qx.coeffRef(i * n + 1, 0) = -Qx_.coeffRef(i * n + 1, i * n + 1) * this->ref_y_;
        qx.coeffRef(i * n + 2, 0) = -Qx_.coeffRef(i * n + 2, i * n + 2) * this->ref_phi_;
        qx.coeffRef(i * n + 3, 0) = -Qx_.coeffRef(i * n + 3, i * n + 3) * this->ref_v_;

        // predition model move a step
        this->pre_car_.UpdateState_RK4(this->ego_delta_f_, this->ego_a_);
    }
    std::cout << "_______________________" << std::endl;
    std::cout << "BB" << endl;
    std::cout << BB << endl;

    std::cout << "_______________________" << std::endl;
    std::cout << "AA" << endl;
    std::cout << AA << endl;

    std::cout << "_______________________" << std::endl;
    std::cout << "gg" << endl;
    std::cout << gg << endl;

    std::cout << "_______________________" << std::endl;
    Eigen::MatrixXd qx_dense = Eigen::MatrixXd(qx);
    std::cout << "qx" << endl;
    std::cout << qx_dense << endl;

    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints 
    // using "X = BB * U + AA * x0 + gg"
    /*
    *               /  x1  \                                /  u0  \
    *               |  x2  |                                |  u1  |
    *  lx_ <=  Cx_  |  x3  |  <= ux_    ====>    lx <=  Cx  |  u2  |  <= ux
    *               | ...  |                                | ...  |
    *               \  xN  /                                \ uN-1 /
    */
    Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse;
    Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
    Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;

    /*
    *        / Cx  \       / lx  \       / ux  \
    *   A_ = \ Cu_ /, l_ = \ lu_ /, u_ = \ uu_ /
    */
    // set A_

    Eigen::SparseMatrix<double> A_T = A_.transpose();
    A_T.middleCols(0, Cx.rows()) = Cx.transpose();
    A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
    A_ = A_T.transpose();
    // set l_ u_
    for (int i = 0; i < lx.rows(); ++i) {
        l_.coeffRef(i, 0) = lx.coeff(i, 0);
        u_.coeffRef(i, 0) = ux.coeff(i, 0);
    }
    for (int i = 0; i < lu_.rows(); ++i) {
        l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
        u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
    }

    Eigen::MatrixXd l_dense = Eigen::MatrixXd(l_);
    Eigen::MatrixXd u_dense = Eigen::MatrixXd(u_);
    Eigen::MatrixXd A_dense = Eigen::MatrixXd(A_);

    std::cout << "_______________________" << std::endl;
    std::cout << "A_ :" << endl;
    std::cout << A_dense << std::endl;

    std::cout << "_______________________" << std::endl;
    std::cout << "l_ :" << endl;
    std::cout << l_dense << std::endl;

    std::cout << "_______________________" << std::endl;
    std::cout << "u_ :" << endl;
    std::cout << u_dense << std::endl;

    // set P_ q_
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse;
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;

    // osqp
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();

    std::cout << "_______________________" << std::endl;
    std::cout << "P_ :" << endl;
    std::cout << P_ << std::endl;

    std::cout << "_______________________" << std::endl;
    std::cout << "q_ :" << endl;
    std::cout << q_d << std::endl;

    // qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.setMats(P_, q_d, A_, l_dense, u_dense);
    qpSolver_.getRes();

    // set control command
    this->command_delta_f = qpSolver_.solution_(0);
    this->command_a_ = qpSolver_.solution_(1);
}

void MpcController::GetCarState(){
    this->ego_x_ = ego_car_.GetX();
    this->ego_y_ = ego_car_.GetY();
    this->ego_phi_ = ego_car_.GetYaw();
    this->ego_v_ = ego_car_.GetV();
    this->ego_delta_f_ = ego_car_.GetDeltaF();
}

void MpcController::WriteControlResult(std::ofstream& outFile){
    if (outFile.is_open()) {
        outFile << this->command_delta_f << " "
        << this->command_a_ << " "
        // << this->headingError << " "
        // << this->lateralError << " "
        << std::endl;
        std::cout << "File written successfully." << std::endl;
    } else {
        std::cout << "Error opening the stanley control record file." << std::endl;
    }
}

void QPSolver::setMats(Eigen::SparseMatrix<double> P_, 
            Eigen::VectorXd q_d,
            Eigen::SparseMatrix<double> A_,
            Eigen::VectorXd l_d,
            Eigen::VectorXd u_d){
    qpSolver_.data()->setNumberOfVariables(2 * 20);
    qpSolver_.data()->setNumberOfConstraints(3 * 20);
    qpSolver_.data()->setHessianMatrix(P_);
    qpSolver_.data()->setGradient(q_d);
    qpSolver_.data()->setLinearConstraintsMatrix(A_);
    qpSolver_.data()->setLowerBound(l_d);
    qpSolver_.data()->setUpperBound(u_d);
}

bool QPSolver::getRes(){
    qpSolver_.settings()->setVerbosity(false);
    qpSolver_.settings()->setWarmStart(true);

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
