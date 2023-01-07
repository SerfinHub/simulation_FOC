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

void SVM_machine::calc_modulation_index()
{
    SVM.M_index = sqrt(3.3f) * SVM.U_motor / Meas_filter.U_dc1;

    /* Flux limitation */
    if (SVM.M_index > 1.0f)
        SVM.M_index = 1.0f;

    if (SVM.M_index < 0.0f)
        SVM.M_index = 0.0f;
}

void SVM_machine::select_state()
{
    if ((0 <= Meas.theta) & (Meas.theta < MATH_PI_3))
        Mstate = SVM_100;
    if ((MATH_PI_3 <= Meas.theta) & (Meas.theta < MATH_2PI_3))
        Mstate = SVM_110;
    if ((MATH_2PI_3 <= Meas.theta) & (Meas.theta < MATH_PI))
        Mstate = SVM_010;
    if ((MATH_PI <= Meas.theta) & (Meas.theta < MATH_4PI_3))
        Mstate = SVM_011;
    if ((MATH_4PI_3 <= Meas.theta) & (Meas.theta < MATH_5PI_3))
        Mstate = SVM_001;
    if ((MATH_5PI_3 <= Meas.theta) & (Meas.theta < 2.0f * MATH_PI))
        Mstate = SVM_101;
}

void SVM_machine::iteration()
{
    calc_modulation_index();

    select_state();

    switch (Mstate)
    {
    case SVM_100: /* 0 <= Theta < 60 */
        handle_100();
        break;
    case SVM_110: /* 60 <= Theta < 120 */
        handle_110();
        break;
    case SVM_010: /* 120 <= Theta < 180 */
        handle_010();
        break;
    case SVM_011: /* 180 <= Theta < 240 */
        handle_011();
        break;
    case SVM_001: /* 240 <= Theta < 300 */
        handle_001();
        break;
    case SVM_101: /* 300 <= Theta < 360 */
        handle_101();
        break;
    default:
        SVM.dutyA = 0.0f;
        SVM.dutyB = 0.0f;
        SVM.dutyC = 0.0f;
        break;
    }
}

void SVM_machine::handle_100()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_110()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta - MATH_PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta - MATH_PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_010()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta - MATH_2PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta - MATH_2PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_011()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta - MATH_PI);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta - MATH_PI) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0;
    SVM.dutyB = 0.5f * SVM.T_0 + SVM.T_1;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_2 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_001()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta - MATH_4PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta - MATH_4PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::handle_101()
{
    /* Time index */
    SVM.T_2 = SVM.M_index * std::sin(Meas.theta - MATH_5PI_3);
    SVM.T_1 = SVM.M_index * std::sqrt(3.0f) * std::cos(Meas.theta - MATH_5PI_3) * 0.5f - SVM.T_2 * 0.5f;
    SVM.T_0 = 1.0f - (SVM.T_1 + SVM.T_2);

    /* Calculated duty */
    SVM.dutyA = 0.5f * SVM.T_0 + SVM.T_1 + SVM.T_2;
    SVM.dutyB = 0.5f * SVM.T_0;
    SVM.dutyC = 0.5f * SVM.T_0 + SVM.T_1;

    set_outputs(SVM.dutyA, SVM.dutyB, SVM.dutyC, 0.02f);
}

void SVM_machine::set_outputs(float dutyA, float dutyB, float dutyC, float deadtime)
{
    aState_global->outputs[0] = (dutyA - 0.5f * deadtime);
    aState_global->outputs[1] = (1.0f - dutyA - 0.5f * deadtime);

    aState_global->outputs[2] = (dutyB - 0.5f * deadtime);
    aState_global->outputs[3] = (1.0f - dutyB - 0.5f * deadtime);

    aState_global->outputs[4] = (dutyC - 0.5f * deadtime);
    aState_global->outputs[5] = (1.0f - dutyC - 0.5f * deadtime);
}