#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <deque>
#include <fstream>
#include <unsupported/Eigen/KroneckerProduct>
#include <algorithm>

#include "models/kinematics_model.h"
#include "referencepath/reference_path.h"

#define Np_ 20
#define Nx_ 3
#define Nu_ 2
#define dt_ 0.01

// 函数作用：对矩阵 A / B 赋值
// 注意：这个函数根据自己实际的需要进行赋值
void setDynamicsMatrices(Eigen::Matrix<double, Nx_, Nx_>& a, Eigen::Matrix<double, Nx_, Nu_>& b, KiCar& car_) {
    a << 1, 0, -car_.getV() * dt_ * sin(car_.getYaw()),
         0, 1, car_.getV() * dt_ * cos(car_.getYaw()),
         0, 0, 1;
 
    b << dt_ * cos(car_.getYaw()), 0,
         dt_ * sin(car_.getYaw()), 0,
         tan(car_.getDeltaF()) * dt_ / car_.getL(), 
         car_.getV() * dt_ / car_.getL() / pow(cos(car_.getDeltaF()), 2);
}

// 函数作用：对向量 x_min / x_max / u_min / u_max 赋值
// 注意：这个函数根据自己实际的需要进行赋值
void setInequalityConstraints(Eigen::Matrix<double, Nx_, 1>& xMax,
                              Eigen::Matrix<double, Nx_, 1>& xMin,
                              Eigen::Matrix<double, Nu_, 1>& uMax,
                              Eigen::Matrix<double, Nu_, 1>& uMin) {
    // 注意：因为 MPC 输出的当前时刻控制量 uk 是基于初始时刻控制量 u0 的增量
    // 解释：当前时刻为 0，初始时刻为 -1，所以这个 u0 代表的是 -1 时刻的控制量大小
    //TODO:
    double u0 = 0.;
    uMin << -0.1, -0.4;
    uMax << 0.1, 0.4;
 
    xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
}
 

// 函数作用：对矩阵 Q / R 赋值
// 注意：这个函数根据自己实际的需要进行赋值
void setWeightMatrices(Eigen::DiagonalMatrix<double, Nx_>& Q, Eigen::DiagonalMatrix<double, Nu_>& R) {
    Q.diagonal() << 10., 10., 10.;
    R.diagonal() << 0.1, 0.1;
}
 
 
// 函数作用：对稀疏矩阵 P 赋值
void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, Nx_>& Q,
                        const Eigen::DiagonalMatrix<double, Nu_>& R,
                        int mpcWindow,
                        Eigen::SparseMatrix<double>& hessianMatrix) {
    hessianMatrix.resize(Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow);
 
    // 使用 Q / R 填充稀疏矩阵 P
    for (int i = 0; i < Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow; i++) {
        if (i < Nx_ * (mpcWindow + 1)) {
            int posQ = i % Nx_;
            float value = Q.diagonal()[posQ];
            if (value != 0) hessianMatrix.insert(i, i) = value;
        }
        else {
            int posR = i % Nu_;
            float value = R.diagonal()[posR];
            if (value != 0) hessianMatrix.insert(i, i) = value;
        }
    }
}
 
 
// 函数作用：赋值向量 q
void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, Nx_>& Q,
                         const Eigen::Matrix<double, Nx_, 1>& xRef,
                         int mpcWindow,
                         Eigen::VectorXd& gradient) {
    Eigen::Matrix<double, Nx_, 1> Qx_ref;
    Qx_ref = Q * (-xRef);
 
    // 填充向量 q
    gradient = Eigen::VectorXd::Zero(Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, 1);
    for (int i = 0; i < Nx_ * (mpcWindow + 1); i++) {
        int posQ = i % Nx_;
        float value = Qx_ref(posQ, 0);
        gradient(i, 0) = value;
    }
}
 
 
// 函数作用：赋值稀疏矩阵 A_c
void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, Nx_, Nx_>& dynamicMatrix,     // dynamicMatrix a
                                 const Eigen::Matrix<double, Nx_, Nu_>& controlMatrix,     // controlMatrix b
                                 int mpcWindow,
                                 Eigen::SparseMatrix<double>& constraintMatrix) {
    constraintMatrix.resize(Nx_ * (mpcWindow + 1) + Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow);
 
    // 等式约束 -- 填充
    for (int i = 0; i < Nx_ * (mpcWindow + 1); i++) {
        constraintMatrix.insert(i, i) = -1;
    }
    // 填充 A_c 矩阵中的 A
    for (int i = 0; i < mpcWindow; i++) {
        for (int j = 0; j < Nx_; j++) {
            for (int k = 0; k < Nx_; k++) {
                float value = dynamicMatrix(j, k);
                if (value != 0) constraintMatrix.insert(Nx_ * (i + 1) + j, Nx_ * i + k) = value;
            }
        }
    }
    // 填充 A_c 矩阵中的 B
    for (int i = 0; i < mpcWindow; i++) {
        for (int j = 0; j < Nx_; j++) {
            for (int k = 0; k < Nu_; k++) {
                float value = controlMatrix(j, k);
                if (value != 0) {
                    constraintMatrix.insert(Nx_ * (i + 1) + j, Nu_ * i + k + Nx_ * (mpcWindow + 1)) = value;
                }
            }
        }
    }
 
    // 不等式约束 -- 填充
    for (int i = 0; i < Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow; i++) {
        constraintMatrix.insert(i + (mpcWindow + 1) * Nx_, i) = 1;
    }
}
 
 
// 函数作用：赋值左右约束 l / u
void castMPCToQPConstraintVectors(const Eigen::Matrix<double, Nx_, 1>& xMax,
                                  const Eigen::Matrix<double, Nx_, 1>& xMin,
                                  const Eigen::Matrix<double, Nu_, 1>& uMax,
                                  const Eigen::Matrix<double, Nu_, 1>& uMin,
                                  const Eigen::Matrix<double, Nx_, 1>& x0,
                                  int mpcWindow,
                                  Eigen::VectorXd& lowerBound,
                                  Eigen::VectorXd& upperBound) {
    // 不等式约束的左右边界数组：[xmin , xmin , ... , xmin | umin , umin , ... umin ]
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, 1);
    for (int i = 0; i <= mpcWindow; i++) {
        lowerInequality.block(Nx_ * i, 0, Nx_, 1) = xMin;
        upperInequality.block(Nx_ * i, 0, Nx_, 1) = xMax;
    }
    for (int i = 0; i < mpcWindow; i++) {
        lowerInequality.block(Nu_ * i + Nx_ * (mpcWindow + 1), 0, Nu_, 1) = uMin;
        upperInequality.block(Nu_ * i + Nx_ * (mpcWindow + 1), 0, Nu_, 1) = uMax;
    }
 
    // 不全数组 l / u 的上半部分：[ -x0 , 0 , 0 , ... , 0 ]
    Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(Nx_ * (mpcWindow + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, Nx_, 1) = -x0;
    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;
 
    // 将数组融合，得到真正的上下边界数组 l / u
    lowerBound = Eigen::MatrixXd::Zero(2 * Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, 1);    // 分配内存空间
    lowerBound << lowerEquality, lowerInequality;
    upperBound = Eigen::MatrixXd::Zero(2 * Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow, 1);    // 分配内存空间
    upperBound << upperEquality, upperInequality;
}
 
 
// 函数作用：更新约束边界 l 和 u
void updateConstraintVectors(const Eigen::Matrix<double, Nx_, 1>& x0,
                             Eigen::VectorXd& lowerBound,
                             Eigen::VectorXd& upperBound) {
    lowerBound.block(0, 0, Nx_, 1) = -x0;
    upperBound.block(0, 0, Nx_, 1) = -x0;
}
 
 
double getErrorNorm(const Eigen::Matrix<double, Nx_, 1>& x, const Eigen::Matrix<double, Nx_, 1>& xRef) {
    Eigen::Matrix<double, Nx_, 1> error = x - xRef;  // 计算误差（对比目标状态 xref 和当前状态 x0 之间的差距）
    return error.norm();                             // 返回误差的二范数
}

void findRefPos(KiCar& car_, RefPath& ref_path_){
    // egoPose based on rear axle
    Eigen::Vector2d egoPose(car_.getX(), car_.getY());
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

    std::cout << "________" << std::endl;
    std::cout << "foundIndex = " <<  foundIndex << std::endl;
}
 
// 主函数：
int main() {
    SineInfo sine_info(3.5,  // amplitude
                       0.01  // frequency
                       );

    RefPath ref_path(2000,   // total point number
                     5       // rows: 4-x,y,h,v  5-x,y,h,v,kappa
                     );
                     
    generateSinewavePath(100,  // traj length(meter)
                         ref_path, 
                         sine_info
                         );

    ref_path.showPath();

    KiCar ego_car(0.01,      // simulator time step
                  3.,        // car length
                  0.,        // x init
                  0.,        // y init
                  0.2165,    // phi init
                  0.,        // delta_f init
                  5.         // v init
                  );
    
    // 设定有限时域长度：
    int mpcWindow = Np_;
 
    // 初始化原始问题涉及到的矩阵：
    Eigen::DiagonalMatrix<double, Nx_> Q;         // Q    （对角阵）
    Eigen::DiagonalMatrix<double, Nu_> R;         // R    （对角阵）
    Eigen::Matrix<double, Nx_, Nx_> a;            // A    （状态量 x 的维数为 Nx_）
    Eigen::Matrix<double, Nx_, Nu_> b;            // B    （控制量 u 的维数为 Nu_）
    Eigen::Matrix<double, Nx_, 1> xMax;           // x_max
    Eigen::Matrix<double, Nx_, 1> xMin;           // x_min
    Eigen::Matrix<double, Nu_, 1> uMax;           // u_max
    Eigen::Matrix<double, Nu_, 1> uMin;           // u_min
    Eigen::Matrix<double, Nx_, 1> x0;             // x_0
    Eigen::Matrix<double, Nx_, 1> xRef;           // x_ref
 
    // 指定初始值和参考值：
    x0 << ego_car.getX(), ego_car.getY(), ego_car.getYaw();
    findRefPos(ego_car, ref_path);
    xRef << ref_path.getPointX(ref_path.lastNearestPointIndex), 
            ref_path.getPointY(ref_path.lastNearestPointIndex),
            ref_path.getPointPhi(ref_path.lastNearestPointIndex);
 
    // 初始化 QP 问题涉及到的矩阵：
    // 注意：这里只是初始化了名字，并未指定分配的内存空间的大小
    Eigen::SparseMatrix<double> hessian;        // P    （稀疏矩阵）
    Eigen::VectorXd gradient;                   // q
    Eigen::SparseMatrix<double> linearMatrix;   // A_c  （稀疏矩阵）
    Eigen::VectorXd lowerBound;                 // l
    Eigen::VectorXd upperBound;                 // u
 
    // 对原始问题中的矩阵进行赋值：
    // 注意：这些函数根据自己实际的需要进行赋值，不同的系统这些矩阵的值肯定是不一样的
    setWeightMatrices(Q, R);                            // 赋值矩阵 Q / R
    setDynamicsMatrices(a, b, ego_car);                 // 赋值矩阵 A / B
    setInequalityConstraints(xMax, xMin, uMax, uMin);   // 赋值约束条件
 
    // 对 QP 问题中的矩阵进行赋值：
    // 注意：这些函数不需要调整，固定的函数格式，因为都是通过上面原始问题的矩阵进行赋值的，只要传入的原始矩阵被修改了就行了
    castMPCToQPHessian(Q, R, mpcWindow, hessian);                                                   // 赋值稀疏矩阵 P
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);                                              // 赋值向量 q
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);                                     // 赋值稀疏矩阵 A_c
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);    // 赋值左右约束 l / u
 
    // 创建求解器：
    // 注意：这句话只是实例化了一个求解器对象，但并未对其进行任何的配置
    OsqpEigen::Solver solver;
 
    // 配置求解器的设置：
    solver.settings()->setVerbosity(false);
    // 解释：这行代码会设置求解器的输出冗长程度，setVerbosity(false) 会关闭求解器的详细输出，使其在求解过程中不输出额外的信息，这在需要安静地运行求解器时非常有用；
    solver.settings()->setWarmStart(true);
    // 解释：启用 WarmStart 功能，加快求解速度（启用 WarmStart 意味着求解器在求解问题时，可以利用之前求解的结果作为初始猜测来加速收敛，这在处理连续求解类似问题时特别有用，可以显著减少计算时间）；
 
    // 配置求解器的数据：
    // 包括：变量数目 + 约束数目 + 矩阵P + 梯度向量q + 线性约束矩阵A_c + 变量下界l + 上界u
    // 解释：如果传入失败，各个函数返回值为 false
    solver.data()->setNumberOfVariables(Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow);          // 设置优化问题的变量数目（ x_0 ~ x_N  +  u_0 ~ u_N-1 ）
    solver.data()->setNumberOfConstraints(2 * Nx_ * (mpcWindow + 1) + Nu_ * mpcWindow);    // 设置优化问题的约束数目
    if (!solver.data()->setHessianMatrix(hessian)) return 1;                               // 传入 P 矩阵
    if (!solver.data()->setGradient(gradient)) return 1;                                   // 传入 q 向量
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;                // 传入 A_c 矩阵
    if (!solver.data()->setLowerBound(lowerBound)) return 1;                               // 传入 l
    if (!solver.data()->setUpperBound(upperBound)) return 1;                               // 传入 u
 
    // 初始化求解器：
    // 解释：使用上面已经传入 settings() 和 data() 的配置参数，来初始化求解器
    if (!solver.initSolver()) return 1;
 
    // 定义矩阵存放：控制输入 / 求解器输出的解
    Eigen::Vector2d ctr;            // 存储 MPC 控制器输出的控制量
    Eigen::VectorXd QPSolution;     // 存储求解器的解
 
    // 定义求解器最大迭代次数
    int numberOfSteps = 50;
    double MAX_SIM_TIME = 60.0;
    double total_t = 0.0;

    // 使用求解器 solver 求解 QP 问题
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;
 
    // 将 QP 问题的解存储在向量 QPSolution 中
    QPSolution = solver.getSolution();
 
    // // 取解的第一个时间步的控制量 u0 放入向量 ctr 中
    // ctr = QPSolution.block(Nx_ * (mpcWindow + 1), 0, Nu_, 1);
    // std::cout << ctr << std::endl;
 
    // 开始求解：
    while(MAX_SIM_TIME > total_t && ref_path.ref_path[0].size() - 10 > ref_path.lastNearestPointIndex){
        // 仿真时间更新
        total_t += ego_car.getTs();

        // 使用求解器 solver 求解 QP 问题
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;
 
        // 将 QP 问题的解存储在向量 QPSolution 中
        QPSolution = solver.getSolution();
 
        // 取解的第一个时间步的控制量 u0 放入向量 ctr 中
        ctr = QPSolution.block(Nx_ * (mpcWindow + 1), 0, Nu_, 1);
        std::cout << ctr << std::endl;

        ego_car.updateState_RK4(ctr[1], 0.0);
        ego_car.printState();

        // 找匹配点
        findRefPos(ego_car, ref_path);

        // 重新赋值ab矩阵
        setDynamicsMatrices(a, b, ego_car);

        // 重新指定初始值和参考值：
        x0 << ego_car.getX(), ego_car.getY(), ego_car.getYaw();
        findRefPos(ego_car, ref_path);
        xRef << ref_path.getPointX(ref_path.lastNearestPointIndex), 
                ref_path.getPointY(ref_path.lastNearestPointIndex),
                ref_path.getPointPhi(ref_path.lastNearestPointIndex);
        
        // 更新约束 l 和 u
        updateConstraintVectors(x0, lowerBound, upperBound);

        // 重新赋值向量q 和稀疏矩阵A_c
        castMPCToQPGradient(Q, xRef, mpcWindow, gradient);                     // 赋值向量 q
        castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);            // 赋值稀疏矩阵 A_c

        // 将更新后的约束边界应用于求解器
        if (!solver.updateBounds(lowerBound, upperBound)) return 1;
        if (!solver.updateGradient(gradient)) return 1;                                   // 更新 q 向量
        if (!solver.updateLinearConstraintsMatrix(linearMatrix)) return 1;                // 更新 A_c 矩阵
    }
 
    return 0;
}

