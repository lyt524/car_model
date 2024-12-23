#include <iostream>
using namespace std;
#include <vector>
#include <cmath>
#include "models/kinematics_model.h"
#include "referencepath/reference_path.h"

double amplitude = 1.0;       // 幅度
double frequency = 0.05;       // 频率（Hz，1Hz = 1次/秒）
double sampleRate = 20;       // 采样率（每秒采样点数）
double duration = 90;         // 持续时间（秒）
int numSamples = static_cast<int>(duration * sampleRate);

KiCar car(0.05, 2.8, 0.0, 0.0, 0, 0, 2);

double total_t = 20.0;
size_t total_index = total_t / 0.05;



int main(){
    SineInfo sine_info(3.5, 0.01);
    RefPath ref_path(2000);
    GenerateSinewavePath(100, ref_path, sine_info);
    ref_path.ShowPath();

    // for(int i = 0; i < numSamples; ++i){
    //     double t = i / sampleRate;                                 // 时间 t
    //     double value = amplitude * sin(2 * M_PI * frequency * t);  // 计算正弦值
    //     // std::cout << "t = " << t << "s, Sine Value = " << value << std::endl;
    //     car.UpdateState_ForwardEuler(value);
    //     car.PrintState();
    // }
    return 0;
}
