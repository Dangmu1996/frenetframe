#pragma once
#include <cmath>
#include <Eigen/Dense>

class QuinticPolynomial
{
public:
    QuinticPolynomial() = default;
    
    void getParam(float xs, float vxs, float axs, float xe, float vxe, float axe, float time);
    
    float calPoint(float current_time);
    float calFirstDerivative(float current_time);
    float calSecondDerivative(float current_time);
    float calThirdDerivative(float current_time);

private:
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;
    float a5 = 0;

    Eigen::Matrix3f t;
    Eigen::Vector3f left_hand;
    Eigen::Vector3f a;

};

class QuarticPolynomial
{
public:
    QuarticPolynomial() = default;

    void getParam(float xs, float vxs, float axs, float vxe, float axe, float time);

    float calPoint(float current_time);
    float calFirstDerivative(float current_time);
    float calSecondDerivative(float current_time);
    float calThirdDerivative(float current_time);

private:
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    float a4 = 0;

    Eigen::Matrix2f t;
    Eigen::Vector2f left_hand;
    Eigen::Vector2f a;
};