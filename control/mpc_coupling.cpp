#include "mpc_coupling.h"


MpcController::MpcController(Bicycle& car_model,
                  KiCar& car,
                  QPSolver& qpSolver,
                  int pre_step,
                  double v_max,
                  double a_max,
                  double delta_f_max,
                  double ddelta_f_max)
    : car_model_(car_model), car_(car), qpSolver_(qpSolver){
    this->N_ = pre_step;
    this->v_max_ = v_max;
    this->a_max_ = a_max;
    this->delta_f_max_ = delta_f_max;
    this->ddelta_f_max_ = ddelta_f_max;
}

void MpcController::SetConstrains(){
    // v constrains
    this->Cx_.resize(1 * this->N_, n * this->N_);
    this->lx_.resize(1 * this->N_, 1);
    this->ux_.resize(1 * this->N_, 1);

    // a delta_f ddelta_f constrains
    this->Cu_.resize(3 * this->N_, m * this->N_);
    this->lu_.resize(3 * this->N_, 1);
    this->uu_.resize(3 * this->N_, 1);

    for (int i = 0; i < this->N_; ++i) {
        // -a_max <= a <= a_max for instance:
        this->Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
        this->lu_.coeffRef(i * 3 + 0, 0) = -this->a_max_;
        this->uu_.coeffRef(i * 3 + 0, 0) = this->a_max_;

        this->Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
        this->lu_.coeffRef(i * 3 + 1, 0) = -this->delta_f_max_;
        this->uu_.coeffRef(i * 3 + 1, 0) = this->delta_f_max_;
 
        this->Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
        if (i > 0){
            this->Cu_.coeffRef(i * 3 + 2, (i - 1) * m + 1) = -1;
        }
        this->lu_.coeffRef(i * 3 + 2, 0) = -this->ddelta_f_max_ * this->car_model_.dt_;
        this->uu_.coeffRef(i * 3 + 2, 0) = this->ddelta_f_max_ * this->car_model_.dt_;
 
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

// TODO: 
Eigen::Vector2d MpcController::FindRefPos(){
    ;
}

void MpcController::SolveQP(const VectorX& x0_observe){
    // set BB, AA, gg
    Eigen::MatrixXd BB, AA, gg;
    BB.setZero(n * N_, m * N_);
    AA.setZero(n * N_, n);
    gg.setZero(n * N_, 1);

    // set qx
    Eigen::SparseMatrix<double> qx;
    qx.resize(n * N_, 1);

    // TODO: VectorX x0 = compensateDelay(x0_observe_);
    // initial state variables
    VectorX x0 = x0_observe;

    for(int i = 0; i < N_; ++i){
        GetCarState();
        // set car_model_.Bd_, Ad_, gd_
        // TODO: 这里不应该是自车状态量，应该是对预测模型的状态量进行线性化和离散化?
        this->car_model_.linearization(this->ego_phi_, this->ego_v_, this->ego_delta_f_);
        // calculate big state-space matrices
        /*
        *                BB                  AA
        * x1    /       B    0  ... 0 \    /   A \
        * x2    |      AB    B  ... 0 |    |  A2 |
        * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
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

        // TODO:
        Eigen::Vector2d ref_position = FindRefPos();

        qx.coeffRef(i * n + 0, 0) = -Qx_.coeffRef(i * n + 0, i * n + 0) * ref_position(0);
        qx.coeffRef(i * n + 1, 0) = -Qx_.coeffRef(i * n + 1, i * n + 1) * ref_position(1);
        // TODO: get ref_phi, ref_v (should be a function?)
        qx.coeffRef(i * n + 2, 0) = -Qx_.coeffRef(i * n + 2, i * n + 2) * ref_phi_;
        qx.coeffRef(i * n + 3, 0) = -Qx_.coeffRef(i * n + 3, i * n + 3) * ref_v_;

        //TODO:
        //predition model move a step  !!!!!!!!!!!!!
    }

    Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
    Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
    Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
    Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

    // state constrants propogate to input constraints using "X = BB * U + AA * x0 + gg"
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

    /*       / Cx  \       / lx  \       / ux  \
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

    // set P_ q_
    Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
    P_ = BBT_sparse * Qx_ * BB_sparse;
    q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;

    // osqp
    Eigen::VectorXd q_d = q_.toDense();
    Eigen::VectorXd l_d = l_.toDense();
    Eigen::VectorXd u_d = u_.toDense();
    qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
    qpSolver_.getRes();
}

void MpcController::GetCarState(){
    this->ego_x_ = car_.GetX();
    this->ego_y_ = car_.GetY();
    this->ego_phi_ = car_.GetYaw();
    this->ego_v_ = car_.GetV();
    this->ego_delta_f_ = car_.GetDeltaF();
}

void QPSolver::setMats(Eigen::SparseMatrix<double> P_, 
            Eigen::VectorXd q_d,
            Eigen::SparseMatrix<double> A_,
            Eigen::VectorXd l_d,
            Eigen::VectorXd u_d){
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

    if (qpSolver_.solve()) {
        Eigen::VectorXd solution = qpSolver_.getSolution();

        std::cout << "Solution: " << solution.transpose() << std::endl;
        return true;
    } else {
        std::cerr << "QP solving failed!" << std::endl;
        return false;
    }
}

