#include "frenet/QuinticPolynomial.hpp"

/******************************************QuinticPolynomial********************************************************************************************/

void QuinticPolynomial::getParam(float xs, float vxs, float axs, float xe, float vxe, float axe, float time)
{
    a0=xs;
    a1=vxs;
    a2=axs/2;
    t << pow(time, 3), pow(time, 4), pow(time,5),
         pow(time,2)*3, pow(time, 3)*4, pow(time, 4)*5,
         6*time, 12*pow(time, 2), 20*pow(time, 3);
    left_hand << xe - a0 - a1*time - a2*pow(time, 2), vxe - a1 - 2*a2*time, axe-2*a2;
    a=t.inverse()*left_hand;
    a3=a[0];
    a4=a[1];
    a5=a[2];
}

float QuinticPolynomial::calPoint(float current_time)
{
        float pos;
        pos = a0 + a1*current_time + a2*pow(current_time,2) + a3*pow(current_time,3) + a4*pow(current_time, 4) + a5*pow(current_time, 5);
        return pos;
}

float QuinticPolynomial::calFirstDerivative(float current_time)
{
    float vel;
    vel = a1 + 2*a2*current_time + 3*a3*pow(current_time,2) + 4*a4*pow(current_time, 3) + 5*a5*pow(current_time, 4);
    return vel;
}

float QuinticPolynomial::calSecondDerivative(float current_time)
{
    float accel;
    accel =  2*a2 + 6*a3*current_time + 12*a4*pow(current_time, 2) + 20*a5*pow(current_time, 3);
    return accel;
}

float QuinticPolynomial::calThirdDerivative(float current_time)
{
    float jerk;
    jerk =  6*a3 + 24*a4*current_time + 60*a5*pow(current_time, 2);
    return jerk;
}


/*******************************************QuarticPolynomial*******************************************************************************************/

void QuarticPolynomial::getParam(float xs, float vxs, float axs, float vxe, float axe, float time)
{
    a0 = xs;
    a1 = vxs;
    a2 = axs / 2.0;
    t << pow(time,2)*3, pow(time, 3)*4,
         6*time, 12*pow(time, 2);
    left_hand << vxe - a1 - 2*a2*time, axe-2*a2;
    a=t.inverse()*left_hand;
    a3 = a[0];
    a4 = a[1];
}

float QuarticPolynomial::calPoint(float current_time)
{
    float pos;
    pos = a0 + a1*current_time + a2*pow(current_time,2) + a3*pow(current_time,3) + a4*pow(current_time, 4);
    return pos;
}

float QuarticPolynomial::calFirstDerivative(float current_time)
{
    float vel;
    vel = a1 + 2*a2*current_time + 3*a3*pow(current_time,2) + 4*a4*pow(current_time, 3);
    return vel;
}

float QuarticPolynomial::calSecondDerivative(float current_time)
{
    float accel;
    accel =  2*a2 + 6*a3*current_time + 12*a4*pow(current_time, 2);
    return accel;
}

float QuarticPolynomial::calThirdDerivative(float current_time)
{
    float jerk;
    jerk =  6*a3 + 24*a4*current_time;
    return jerk;
}