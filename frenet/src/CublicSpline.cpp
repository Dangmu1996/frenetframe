#include "frenet/CublicSpline.hpp"

Eigen::MatrixXf CublicSpline1D::calA(std::vector<float> h)
{
    Eigen::MatrixXf A=Eigen::MatrixXf::Zero(dimension, dimension);
    A(0, 0) = 1.0;
    for(size_t i=0; i<dimension-1; i++)
    {
        if(i != dimension -2)
        {
            A(i+1, i+1) = 2*(h[i]+h[i+1]);
        }
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(dimension -1, dimension -2) = 0.0;
    A(dimension -1, dimension -1) = 1.0;
    return A;
}

Eigen::VectorXf CublicSpline1D::calB(std::vector<float> h)
{
    Eigen::VectorXf B=Eigen::VectorXf::Zero(dimension);
    for(size_t i=0; i<dimension -2; i++)
    {
        B[i+1] = 3*(a_[i+2]- a_[i+1])/h[i+1] - 3*(a_[i+1]- a_[i])/h[i]; 
    }
    return B;
}

int CublicSpline1D::searchIndex(float x)
{
    int i=0;
    if(x==0)
    {
        return 0;
    }
    while(i < dimension -2)
    {
        if(x_[i] < x && x <= x_[i+1])
        {
            break;
        }
        i++;
    }
    return i;
}

void CublicSpline1D::getSXY(std::vector<float> x, std::vector<float> y)
{
    x_.clear();
    y_.clear();
    a_.clear();
    b_.clear();
    c_.clear();
    d_.clear();
    x_=x;
    y_=y;
    std::vector<float> h;
    for(int i=0; i<x.size()-1; i++)
    {
        h.push_back(x[i+1]-x[i]);
    }
    dimension = x.size();
    for(auto &i : y)
    {
        a_.push_back(i);
    }
    Eigen::MatrixXf A = this->calA(h);
    Eigen::VectorXf B = this->calB(h);
    Eigen::VectorXf C = A.inverse() * B;
    for(int i=0; i<C.rows(); i++)
    {
        c_.push_back(C[i]);
    }
    for(size_t i=0; i<dimension-1; i++)
    {
        float d_temp = (c_[i+1] - c_[i]) / (3*h[i]);
        float b_temp = 1/h[i] *(a_[i+1]-a_[i]) - h[i]/3* (2*c_[i]+c_[i+1]);
        d_.push_back(d_temp);
        b_.push_back(b_temp);
    }
}

float CublicSpline1D::calcPosition(float x)
{
    int i = this->searchIndex(x);
    float dx = x - x_[i];
    float position = a_[i] + b_[i] * dx + c_[i]*dx*dx + d_[i] * dx * dx * dx;
    return position;
}

float CublicSpline1D::clacFirstDerivative(float x)
{
    int i = this->searchIndex(x);
    float dx = x - x_[i];
    float dy = b_[i] + 2 * c_[i] * dx + 3 * d_[i] * dx * dx;
    return dy;
}

float CublicSpline1D::calcSecondDerivative(float x)
{
    int i = this->searchIndex(x);
    float dx = x - x_[i];
    float ddy = 2 * c_[i] + 6*d_[i]*dx;
    return ddy;
}