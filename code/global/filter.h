/*
 * filter.h
 * SBACH
 * 24.05.2022
 */

#pragma once

struct FilterParameters_t
{
    float a0 = 1.0f;
    float a1 = -1.8255;
    float a2 = 0.8425f;
    float b0 = 0.0959f;
    float b1 = -0.1749f;
    float b2 = 0.0959f;
};

class Filter
{
public:
    Filter();
    Filter(float a0, float a1, float a2, float b0, float b1, float b2);

    FilterParameters_t fParam;

    float xn0;
    float xn1;
    float xn2;

    float yn;
    float yn1;
    float yn2;

    float filterValue();
    void reset(float x0);
    float step(float x);
};
