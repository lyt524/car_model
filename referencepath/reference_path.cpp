#include "reference_path.h"

RefPath::RefPath(int point_num, int row_num){
    this->point_num = point_num;
    ref_path.resize(row_num, std::vector<double>(this->point_num, 0.0));
}

void RefPath::GetPoint(int index, double& x, double& y, double& phi){
    if (index >= 0 && index < ref_path[0].size()) {
        x = ref_path[PATH_X][index];
        y = ref_path[PATH_Y][index];
        phi = ref_path[PATH_PHI][index];
    }
    else{
        std::cout << "out of range" << std::endl;
    }
}

double RefPath::GetPointX(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[PATH_X][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

double RefPath::GetPointY(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[PATH_Y][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

double RefPath::GetPointPhi(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[PATH_PHI][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

double RefPath::GetPointV(int index){
    if (index >= 0 && index < ref_path[0].size()) {
        return ref_path[PATH_V][index];
    }
    else{
        std::cout << "out of range" << std::endl;
        return 0.0;
    }
}

void RefPath::ShowPath(){
    for (int i = 0; i < this->point_num; i++) {
        std::cout << "Point " << ref_path[PATH_INDEX][i] 
                << ": x = " << ref_path[PATH_X][i]
                << ", y = " << ref_path[PATH_Y][i]
                << ", phi = " << ref_path[PATH_PHI][i]
                << ", v = " << ref_path[PATH_V][i]
                << std::endl;
    }
}

void RefPath::WritePath(std::ofstream& outFile){
    if (outFile.is_open()) {
        for(int i = 0; i < this->ref_path[0].size(); i++){
            outFile << ref_path[PATH_X][i] << " "  // x
            << ref_path[PATH_Y][i] << " "          // y
            << ref_path[PATH_PHI][i] << " "          // phi
            << ref_path[PATH_V][i] << " "          // v
            << std::endl;
        }
        std::cout << "Trajectory record written successfully." << std::endl;
    } else {
        std::cout << "Error opening the trajectory record file." << std::endl;
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
        _ref_path.ref_path[PATH_INDEX][i] =  i;

        // fill x
        _ref_path.ref_path[PATH_X][i] =  i * (static_cast<double>(path_length) / point_num);
        // cout << " x = " << ref_path[1][i] << endl;

        // fill y
        _ref_path.ref_path[PATH_Y][i] = _sine_info.amplitude * sin(2 * M_PI * _sine_info.frequency * _ref_path.ref_path[1][i]);
        // cout << " y = " << ref_path[2][i] << endl;

        // fill phi
        if(i != 0 && i != point_num - 1){
            _ref_path.ref_path[PATH_PHI][i] = atan2(_ref_path.ref_path[PATH_Y][i] - _ref_path.ref_path[PATH_Y][i - 1], 
                                                    _ref_path.ref_path[PATH_X][i] - _ref_path.ref_path[PATH_X][i - 1]);
        }

        // fill v
        _ref_path.ref_path[PATH_V][i] = 2.0;
    }
    _ref_path.ref_path[PATH_PHI][0] = _ref_path.ref_path[PATH_PHI][1];
    _ref_path.ref_path[PATH_PHI][point_num - 1] = _ref_path.ref_path[PATH_PHI][point_num - 2];
}

