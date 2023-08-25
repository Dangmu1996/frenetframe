#pragma once

#include <vector>

struct FrenetPath
{
    std::vector<float> t;
    std::vector<float> d;
    std::vector<float> d_d;
    std::vector<float> d_dd;
    std::vector<float> d_ddd;
    std::vector<float> s;
    std::vector<float> s_d;
    std::vector<float> s_dd;
    std::vector<float> s_ddd;
    float cd;
    float cv;
    float cf;

    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> yaw;
    std::vector<float> vyaw;
    std::vector<float> v;
    std::vector<float> ds;
    std::vector<float> c;
    bool defalt = false;
};