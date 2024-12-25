#pragma once

#include <vector>
#include <iostream>

class RefPath{
public:
    RefPath(int point_num, int row_num);
    ~RefPath() = default;
    
    void GetPoint(int index, double& x, double& y, double& phi);
    double GetPointX(int index);
    double GetPointY(int index);
    double GetPointPhi(int index);
    void ShowPath();

public:
    std::vector<std::vector<double>> ref_path;
    int point_num;
    int lastNearestPointIndex = 0;
};

class SineInfo{
public:
    SineInfo(double amp, double freq);
    ~SineInfo() = default;

public:
    double amplitude;
    double frequency;
};

void GenerateSinewavePath(int path_length, RefPath& _ref_path, SineInfo& _sine_info);
