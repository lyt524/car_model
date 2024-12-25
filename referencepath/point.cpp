#include <iostream>
#include "point.h"


Point::Point(double x, double y){
    this->_x = x;
    this->_y = y;
}

double Point::Get_X(){
    return this->_x;
}

double Point::Get_Y(){
    return this->_y;
}

void Point::ShowPoint(){
    std::cout << "_x = " << this->_x << ", _y = " << this->_y << std::endl;
}

