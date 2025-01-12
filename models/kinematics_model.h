#pragma once

#include <vector>
#include <algorithm>

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
    float getTs();
    float getL();
    double getX();
    double getY();
    double getYaw();
    double getDeltaF();
    double getV();
    void updateState_ForwardEuler(double delta_f);
    void updateState_ForwardEuler(double delta_f, double a);
    void updateState_BackwardEuler(double delta_f);
    void updateState_BackwardEuler(double delta_f, double a);
    void updateState_RK4(double delta_f);
    void updateState_RK4(double delta_f, double a);
    void printState();
    void writeCarState(std::ofstream& outFile);
};


