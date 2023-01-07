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

private:
    float Kp;
    float Ts_Ti;
    float integrator;
    float proportional;
    float lim_H;
    float lim_L;
};

extern const double MATH_PI;
extern const double MATH_2PI;
extern const double MATH_2PI_3;
extern const double MATH_4PI_3;
extern const double MATH_5PI_3;
extern const double MATH_PI_3;
extern const double MATH_1_2PI;
extern const double MATH_1_PI;
extern const double MATH_1_3;
extern const double MATH_1_SQRT3;
extern const double MATH_1_SQRT2;
extern const double MATH_SQRT2_3;
extern const double MATH_SQRT3_2;
extern const double MATH_SQRT2;
extern const double MATH_SQRT3;
extern const double MATH_2_3;
extern const double MATH_1_325;
extern const double MATH_N2_325;
