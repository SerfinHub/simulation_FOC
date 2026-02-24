/*
 * Controllers.cpp
 * SBACH
 * 03.04.2022
 */

#include <math.h>

#include "Controllers.h"
#include "../global/type.h"

#define myMATH_PI 3.1415926535897932384626433832795
#define myMATH_SQRT3 1.7320508075688772935274463415059
#define myMATH_SQRT2 1.4142135623730950488016887242097

constexpr float MATH_PI = myMATH_PI;
constexpr float MATH_2PI = myMATH_PI * 2.0;
constexpr float MATH_2PI_3 = myMATH_PI * 2.0 / 3.0;
constexpr float MATH_4PI_3 = myMATH_PI * 4.0 / 3.0;
constexpr float MATH_5PI_3 = myMATH_PI * 5.0 / 3.0;
constexpr float MATH_PI_3 = myMATH_PI / 3.0;
constexpr float MATH_1_2PI = 1.0 / (myMATH_PI * 2.0);
constexpr float MATH_1_PI = 1.0 / myMATH_PI;
constexpr float MATH_1_3 = 1.0 / 3.0;
constexpr float MATH_1_SQRT3 = 1.0 / myMATH_SQRT3;
constexpr float MATH_1_SQRT2 = 1.0 / myMATH_SQRT2;
constexpr float MATH_SQRT2_3 = myMATH_SQRT2 / 3.0;
constexpr float MATH_SQRT3_2 = myMATH_SQRT3 / 2.0;
constexpr float MATH_SQRT2 = myMATH_SQRT2;
constexpr float MATH_SQRT3 = myMATH_SQRT3;
constexpr float MATH_2_3 = 2.0 / 3.0;
constexpr float MATH_1_325 = 1.0 / 325.0;
constexpr float MATH_N2_325 = -2.0 / 325.0;

MyReg::MyReg(float mKp, float mTi, float mLh, float mLl) : Kp(mKp),
                                                           Ts_Ti(mTi),
                                                           integrator(0.0f),
                                                           proportional(0.0f),
                                                           lim_H(mLh),
                                                           lim_L(mLl),
                                                           mode(0)
{
}

MyReg::MyReg()
{
}

float MyReg::Calculate(input_t input)
{
    float out;

    proportional = Kp * input.error;
    integrator += proportional * Ts_Ti;
    out = integrator + proportional;
    if (out > lim_H)
    {
        out = lim_H;
        integrator = lim_H - proportional;
    }
    else if (out < lim_L)
    {
        out = lim_L;
        integrator = lim_L - proportional;
    }

    return out;
}

float MyReg::Ramp(float target, float slew, float Ts)
{
    static float y = 0.0f;
    float step = slew * Ts;              // max zmiana na próbkę
    float e = target - y;
    if (e > step) e = step;
    if (e < -step) e = -step;
    y += e;
    return y;
}

/* Select the proper regulator
 * Power control not applied */
float MyReg::selectController(float power, float voltage, float current)
{
    float out;

    // TODO: wybor regulatora

    return out;
}

ref_frame_t MyReg::abc_ab(ref_frame_t &in)
{
    ref_frame_t out;
    out.alpha = (2/3)*(a - 0.5*b - 0.5*c)
    out.beta  = (2/3)*( (sqrt(3)/2)*b - (sqrt(3)/2)*c )
    return out;
}

ref_frame_t MyReg::ab_abc(ref_frame_t &in)
{
    ref_frame_t out;
    float out_temp, t_struct_bet_temp;
    in.a = out.alfa;
    out_temp = -0.5f * in.alfa;
    t_struct_bet_temp = in.beta * MATH_SQRT3_2;
    out.b = out_temp + t_struct_bet_temp;
    out.c = out_temp - t_struct_bet_temp;
    return out;
}

ref_frame_t MyReg::ab_dq(ref_frame_t &in, float &angle)
{
    ref_frame_t out;
    float sine, cosine;
    sine = std::sin(angle);
    cosine = std::cos(angle);
    out.d = cosine * in.alfa + sine * in.beta;
    out.q = -sine * in.alfa + cosine * in.beta;
    return out;
}

ref_frame_t MyReg::dq_ab(ref_frame_t &in, float &angle)
{
    ref_frame_t out;
    float sine, cosine;
    sine = std::sin(angle);
    cosine = std::cos(angle);
    out.alfa = cosine * in.d - sine * in.q;
    out.beta = sine * in.d + cosine * in.q;
    return out;
}