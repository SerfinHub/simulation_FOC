/*
 * filter.cpp
 * SBACH
 * 24.05.2022
 */

#include "filter.h"

Filter::Filter() : fParam(),
    xn0(0.0f), xn1(0.0f), xn2(0.0f),
    yn(0.0f), yn1(0.0f), yn2(0.0f)
{}

// TODO: let's make other high order filters
float Filter::filterValue()
{
    yn = ((fParam.b0 * xn0) + (fParam.b1 * xn1) + (fParam.b2 * xn2) + (-1.0f * fParam.a1 * yn1) + (-1.0f * fParam.a2 * yn2));

    xn2 = xn1;
    xn1 = xn0;
    yn2 = yn1;
    yn1 = yn;

    return yn;
}

void Filter::reset(float x0)
{
    xn0 = xn1 = xn2 = x0;
    yn  = yn1 = yn2 = x0;
}

float Filter::step(float x)
{
    xn0 = x;
    return filterValue();
}