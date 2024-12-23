#pragma once

#include <vector>
#include <iostream>

class RefPath{
public:
    RefPath(int point_num);
    ~RefPath() = default;
    
    void GetPoint(int index, double& x, double& y, double& phi);
    void ShowPath();

public:
    std::vector<std::vector<double>> ref_path;
    int point_num;
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
