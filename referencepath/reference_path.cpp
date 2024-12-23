#include <vector>
#include <cmath>
#include "reference_path.h"

RefPath::RefPath(int point_num){
    this->point_num = point_num;
    ref_path.resize(3, std::vector<double>(this->point_num, 0.0));
}

void RefPath::GetPoint(int index, double& x, double& y, double& phi){
    if (index >= 0 && index < ref_path[0].size()) {
        x = ref_path[0][index];
        y = ref_path[1][index];
        phi = ref_path[2][index];
    }
}

void RefPath::ShowPath(){
    for (int i = 0; i < this->point_num; i++) {
        std::cout << "Point " << i << ": x = " << ref_path[0][i] 
                << ", y = " << ref_path[1][i] 
                << ", phi = " << ref_path[2][i] << std::endl;
    }
}

SineInfo::SineInfo(double amp, double freq){
    this->amplitude = amp;
    this->frequency = freq;
}

void GenerateSinewavePath(int path_length, RefPath& _ref_path, SineInfo& _sine_info){
    int point_num = _ref_path.ref_path[0].size();
    for(int i = 0; i < point_num; i++){
        _ref_path.ref_path[0][i] =  i * (static_cast<double>(path_length) / point_num);
        // cout << " x = " << ref_path[0][i] << endl;
        _ref_path.ref_path[1][i] = _sine_info.amplitude * sin(2 * M_PI * _sine_info.frequency * _ref_path.ref_path[0][i]);
        // ref_path[1][i] = 3.5 * sin(2 * M_PI * 0.01 * ref_path[0][i]);
        // cout << " y = " << ref_path[1][i] << endl;
        if(i != 0 && i != point_num - 1){
            _ref_path.ref_path[2][i] = atan2(_ref_path.ref_path[1][i] - _ref_path.ref_path[1][i - 1], _ref_path.ref_path[0][i] - _ref_path.ref_path[0][i - 1]);
        }
    }
    _ref_path.ref_path[2][0] = _ref_path.ref_path[2][1];
    _ref_path.ref_path[2][point_num - 1] = _ref_path.ref_path[2][point_num - 2];
}