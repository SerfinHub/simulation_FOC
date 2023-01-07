/*
 * filter.h
 * SBACH
 * 24.05.2022
 */
#include "filter.h"

Filter::Filter()
    : fParam()
{
}

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