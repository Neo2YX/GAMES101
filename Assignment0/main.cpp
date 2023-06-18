#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main(){
    float angle = 45.0/180 * acosf(-1);
    //Example of composite
    Eigen::Vector3f p(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f result;
    Eigen::Matrix3f rotation, translation;
    rotation << std::cos(angle), -std::sin(angle), 0,
                std::sin(angle), std::cos(angle), 0,
                0, 0, 1;
    translation << 1, 0, 1,
                   0, 1, 2,
                   0, 0, 1;
    result = translation * rotation * p ;
    std::cout << "translate * rotate * p = (" << result(0) << ", " << result(1) << ")"<< std::endl;
    Eigen::Matrix3f transform;
    transform << std::cos(angle), -std::sin(angle), 1,
                 std::sin(angle), std::cos(angle), 2,
                 0, 0, 1;
    result = transform * p;
    std::cout << "transform * p = (" << result(0) << ", " << result(1) << ")" << std::endl;

    std::cout << "press any key to exit";
    
    std::cin.get();
    return 0;
}