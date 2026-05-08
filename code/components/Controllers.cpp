/*
 * Controllers.cpp
 * SBACH
 * 03.04.2022
 */

#include <cmath>

#include "Controllers.h"
#include "../global/type.h"

MyReg::MyReg()
    : frame{},
      mode(0),
      out(0.0f),
      Kp(0.0f),
      Ts_Ti(0.0f),
      integrator(0.0f),
      proportional(0.0f),
      lim_H(0.0f),
      lim_L(0.0f),
      ramp_y(0.0f)
{
}

MyReg::MyReg(float mKp, float mTi, float mLh, float mLl)
    : frame{},
      mode(0),
      out(0.0f),
      Kp(mKp),
      Ts_Ti(mTi),
      integrator(0.0f),
      proportional(0.0f),
      lim_H(mLh),
      lim_L(mLl),
      ramp_y(0.0f)
{
}

float MyReg::Calculate(input_t input)
{
    proportional = Kp * input.error;
    integrator += proportional * Ts_Ti;

    float regulator_output = integrator + proportional;

    if (regulator_output > lim_H)
    {
        regulator_output = lim_H;
        integrator = lim_H - proportional;
    }
    else if (regulator_output < lim_L)
    {
        regulator_output = lim_L;
        integrator = lim_L - proportional;
    }

    return regulator_output;
}

void MyReg::Reset(float outputInitial)
{
    integrator = outputInitial;
    proportional = 0.0f;
    out = outputInitial;
    ramp_y = outputInitial;
}

float MyReg::Ramp(float target, float slew, float Ts)
{
    const float step = slew * Ts;
    float error = target - ramp_y;

    if (error > step)
    {
        error = step;
    }
    if (error < -step)
    {
        error = -step;
    }

    ramp_y += error;
    return ramp_y;
}

/* Select the proper regulator
 * Power control not applied */
float MyReg::selectController(float power, float voltage, float current)
{
    // TODO: implement controller selection.
    (void)power;
    (void)voltage;
    (void)current;

    return 0.0f;
}

ref_frame_t MyReg::abc_ab(ref_frame_t &in)
{
    ref_frame_t result{};
    result.alfa = MATH_2_3 * (in.a - 0.5f * in.b - 0.5f * in.c);
    result.beta = MATH_2_3 * MATH_SQRT3_2 * (in.b - in.c);
    return result;
}

ref_frame_t MyReg::ab_abc(ref_frame_t &in)
{
    ref_frame_t result{};
    result.a = in.alfa;

    const float common = -0.5f * in.alfa;
    const float beta_part = in.beta * MATH_SQRT3_2;

    result.b = common + beta_part;
    result.c = common - beta_part;

    return result;
}

ref_frame_t MyReg::ab_dq(ref_frame_t &in, float &angle)
{
    ref_frame_t result{};
    const float sine = std::sin(angle);
    const float cosine = std::cos(angle);

    result.d = cosine * in.alfa + sine * in.beta;
    result.q = -sine * in.alfa + cosine * in.beta;

    return result;
}

ref_frame_t MyReg::dq_ab(ref_frame_t &in, float &angle)
{
    ref_frame_t result{};
    const float sine = std::sin(angle);
    const float cosine = std::cos(angle);

    result.alfa = cosine * in.d - sine * in.q;
    result.beta = sine * in.d + cosine * in.q;

    return result;
}
