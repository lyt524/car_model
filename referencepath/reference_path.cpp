#include <vector>
#include <cmath>
#include "reference_path.h"

RefPath::RefPath(int point_num, int row_num){
    this->point_num = point_num;
    ref_path.resize(row_num, std::vector<double>(this->point_num, 0.0));
}

void RefPath::GetPoint(int index, double& x, double& y, double& phi){
    if (index >= 0 && index < ref_path[0].size()) {
        x = ref_path[1][index];
        y = ref_path[2][index];
        phi = ref_path[3][index];
    }
    else{
        std::cout << "out of range" << std::endl;
    }
}

double RefPath::GetPointX(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[1][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

double RefPath::GetPointY(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[2][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

double RefPath::GetPointPhi(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[3][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

void RefPath::ShowPath(){
    for (int i = 0; i < this->point_num; i++) {
        std::cout << "Point " << ref_path[0][i] 
                << ": x = " << ref_path[1][i]
                << ", y = " << ref_path[2][i]
                << ", phi = " << ref_path[3][i] 
                << std::endl;
    }
}

SineInfo::SineInfo(double amp, double freq){
    this->amplitude = amp;
    this->frequency = freq;
}

void GenerateSinewavePath(int path_length, RefPath& _ref_path, SineInfo& _sine_info){
    int point_num = _ref_path.ref_path[0].size();
    for(int i = 0; i < point_num; i++){
        // fill index
        _ref_path.ref_path[0][i] =  i;

        // fill x
        _ref_path.ref_path[1][i] =  i * (static_cast<double>(path_length) / point_num);
        // cout << " x = " << ref_path[1][i] << endl;

        // fill y
        _ref_path.ref_path[2][i] = _sine_info.amplitude * sin(2 * M_PI * _sine_info.frequency * _ref_path.ref_path[1][i]);
        // cout << " y = " << ref_path[2][i] << endl;

        // fill phi
        if(i != 0 && i != point_num - 1){
            _ref_path.ref_path[3][i] = atan2(_ref_path.ref_path[2][i] - _ref_path.ref_path[2][i - 1], _ref_path.ref_path[1][i] - _ref_path.ref_path[1][i - 1]);
        }
    }
    _ref_path.ref_path[3][0] = _ref_path.ref_path[3][1];
    _ref_path.ref_path[3][point_num - 1] = _ref_path.ref_path[3][point_num - 2];
}

