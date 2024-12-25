#pragma once

using namespace std;
#include <vector>


class KiCar{
private:
    float TS, L;
    double x, y, phi, vx, vy, v;
    double ax, ay, a, yaw_rate;
    double kappa;
    double delta_f;
    
public:
    KiCar(float TS, float L);
    KiCar(float TS, float L, double x, double y, double phi, double delta_f, double v);
    ~KiCar();
    float GetTs();
    float GetL();
    double GetX();
    double GetY();
    double GetYaw();
    double GetDeltaF();
    double GetV();
    void GetPosition();
    void UpdateState_ForwardEuler(double delta_f);
    void UpdateState_ForwardEuler(double delta_f, double a);
    void UpdateState_BackwardEuler(double delta_f);
    void UpdateState_BackwardEuler(double delta_f, double a);
    void UpdateState_RK4(double delta_f);
    void UpdateState_RK4(double delta_f, double a);
    void PrintState();
    void WriteCarState(std::ofstream& outFile);
};


