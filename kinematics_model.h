using namespace std;
#include <vector>


class KiCar{
private:
    float TS, L;
    double x, y, yaw, vx, vy, v;
    double ax, ay, a, yaw_rate;
    double kappa;
    double delta_f;
    

public:
    KiCar(float TS, float L);
    KiCar(float TS, float L, double x, double y, double yaw, double delta_f, double vx);
    ~KiCar(){}
    float GetTs();
    float GetL();
    double GetX();
    double GetY();
    double GetYaw();
    double GetDeltaF();
    double GetVx();
    void GetPosition();
    void UpdateState(double delta_f);
    void UpdateState(double delta_f, double ax);
    void PrintState();
};


