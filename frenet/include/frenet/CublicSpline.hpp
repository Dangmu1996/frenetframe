#pragma once

#include <Eigen/Dense>
#include <vector>

class CublicSpline1D //a=dt^3+ct^2+b*t+a
{
private:
    size_t dimension;
    std::vector<float> a_, b_, c_, d_;
    std::vector<float> x_,y_;

    Eigen::MatrixXf calA(std::vector<float> h);
    Eigen::VectorXf calB(std::vector<float> h);
    int searchIndex(float x);

public:
    CublicSpline1D() = default;
    void getSXY(std::vector<float> x, std::vector<float> y);

    float calcPosition(float x);
    float clacFirstDerivative(float x);
    float calcSecondDerivative(float x);
};
