/*
 * Controllers.h
 * SBACH
 * 24.12.2022
 */
#pragma once

struct input_t
{
    float error;
};

struct ref_frame_t
{
    float a;
    float b;
    float c;

    float alfa;
    float beta;

    float d;
    float q;
};

class MyReg
{
public:
    MyReg();
    MyReg(float mKp, float mTi, float mLh, float mLl);

    ref_frame_t frame;

    float Calculate(input_t);
    float Ramp(float input, float delay, float Ts);
    float selectController(float power, float voltage, float current);

    ref_frame_t abc_ab(ref_frame_t &in);
    ref_frame_t ab_abc(ref_frame_t &in);
    ref_frame_t ab_dq(ref_frame_t &in, float &angle);
    ref_frame_t dq_ab(ref_frame_t &in, float &angle);

    int mode;
    float out;

private:
    float Kp;
    float Ts_Ti;
    float integrator;
    float proportional;
    float lim_H;
    float lim_L;
};

#define myMATH_PI 3.1415926535897932384626433832795
#define myMATH_SQRT3 1.7320508075688772935274463415059
#define myMATH_SQRT2 1.4142135623730950488016887242097

constexpr float MATH_PI = myMATH_PI;
constexpr float MATH_2PI = myMATH_PI * 2.0f;
constexpr float MATH_2PI_3 = myMATH_PI * 2.0f / 3.0f;
constexpr float MATH_4PI_3 = myMATH_PI * 4.0f / 3.0f;
constexpr float MATH_5PI_3 = myMATH_PI * 5.0f / 3.0f;
constexpr float MATH_PI_3 = myMATH_PI / 3.0f;
constexpr float MATH_1_2PI = 1.0f / (myMATH_PI * 2.0f);
constexpr float MATH_1_PI = 1.0f / myMATH_PI;
constexpr float MATH_1_3 = 1.0f / 3.0f;
constexpr float MATH_1_SQRT3 = 1.0f / myMATH_SQRT3;
constexpr float MATH_1_SQRT2 = 1.0f / myMATH_SQRT2;
constexpr float MATH_SQRT2_3 = myMATH_SQRT2 / 3.0f;
constexpr float MATH_SQRT3_2 = myMATH_SQRT3 / 2.0f;
constexpr float MATH_SQRT2 = myMATH_SQRT2;
constexpr float MATH_SQRT3 = myMATH_SQRT3;
constexpr float MATH_2_3 = 2.0f / 3.0f;
constexpr float MATH_1_325 = 1.0f / 325.0f;
constexpr float MATH_N2_325 = -2.0f / 325.0f;