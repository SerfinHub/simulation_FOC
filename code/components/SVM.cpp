/*
 * SVM.cpp
 * SBACH
 * 24.12.2022
 */

#include "SVM.h"

SVM_machine::SVM_machine() : Mstate(SVM_000),
                             Mstate_last(SVM_000),
                             SVM()

{
}

float SVM_machine::calc_modulation_index(float alfa, float beta)
{
    SVM.U_motor = sqrt(alfa * alfa + beta * beta);

    SVM.M_index = (sqrt(3.3f) * SVM.U_motor) / Meas_filter.U_dc1;

    /* Flux limitation */
    if (SVM.M_index > 1.0f)
        SVM.M_index = 1.0f;

    if (SVM.M_index < 0.0f)
        SVM.M_index = 0.0f;

    return SVM.M_index;
}

void SVM_machine::select_state(float theta)
{
    if ((0 <= theta) & (theta < MATH_PI_3))
        Mstate = SVM_001;
    if ((MATH_PI_3 <= theta) & (theta < MATH_2PI_3))
        Mstate = SVM_011;
    if ((MATH_2PI_3 <= theta) & (theta < MATH_PI))
        Mstate = SVM_010;
    if ((MATH_PI <= theta) & (theta < MATH_4PI_3))
        Mstate = SVM_110;
    if ((MATH_4PI_3 <= theta) & (theta < MATH_5PI_3))
        Mstate = SVM_100;
    if ((MATH_5PI_3 <= theta) & (theta < 2.0f * MATH_PI))
        Mstate = SVM_101;
}

void SVM_machine::iteration(float theta)
{
    select_state(theta);

    switch (Mstate)
    {
    case SVM_001: /* 0 <= Theta < 60 */
        handle_001(theta);
        break;
    case SVM_011: /* 60 <= Theta < 120 */
        handle_011(theta);
        break;
    case SVM_010: /* 120 <= Theta < 180 */
        handle_010(theta);
        break;
    case SVM_110: /* 180 <= Theta < 240 */
        handle_110(theta);
        break;
    case SVM_100: /* 240 <= Theta < 300 */
        handle_100(theta);
        break;
    case SVM_101: /* 300 <= Theta < 360 */
        handle_101(theta);
        break;
    default:
        SVM.dutyA = 0.0f;
        SVM.dutyB = 0.0f;
        SVM.dutyC = 0.0f;
        break;
    }
}

void SVM_machine::handle_001(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_011(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta - MATH_PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta - MATH_PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_010(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta - MATH_2PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta - MATH_2PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_110(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta - MATH_PI);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta - MATH_PI) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_100(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta - MATH_4PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta - MATH_4PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_101(float theta)
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(theta - MATH_5PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(theta - MATH_5PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_1;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::set_outputs(float dutyA, float dutyB, float dutyC, float deadtime)
{
    if (dutyA > 1.0f)
        dutyA = 1.0f;
    if (dutyA < 0.0f)
        dutyA = 0.0f;

    if (dutyB > 1.0f)
        dutyB = 1.0f;
    if (dutyB < 0.0f)
        dutyB = 0.0f;

    if (dutyC > 1.0f)
        dutyC = 1.0f;
    if (dutyC < 0.0f)
        dutyC = 0.0f;

    aState_global->outputs[0] = (dutyA);
    aState_global->outputs[1] = (1.0f - dutyA);
    aState_global->outputs[2] = (dutyB);
    aState_global->outputs[3] = (1.0f - dutyB);
    aState_global->outputs[4] = (dutyC);
    aState_global->outputs[5] = (1.0f - dutyC);
}