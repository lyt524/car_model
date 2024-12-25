#pragma once

class Point{
private:
    double _x;
    double _y;
public:
    double Get_X();
    double Get_Y();
    void ShowPoint();
    Point(double x, double y);
    ~Point() = default;
};