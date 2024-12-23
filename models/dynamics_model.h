#pragma once


class DyCar{
private:
    float TS, L, MASS, CF, CR, IZ, LF, LR;
    double x, y, phi, vx, vy, v;
    double ax, ay, a;
    double d_vy, d_phi, d_d_phi;
    double kappa;
    double delta_f;

public:
    ~DyCar() = default;
    DyCar(float TS, float L, float MASS, float CF, float CR, float IZ, float LF, float LR,
          double x, double y, double phi, double d_vy, double d_phi, double d_d_phi);
    void UpdateState_ForwardEuler(double delta_f, double a);
    void UpdateState_BackwardEuler(double delta_f, double a);
    void UpdateState_RK4(double delta_f, double a);

};