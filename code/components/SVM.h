/*
 * SVM.h
 * SBACH
 * 24.12.2022
 */

#pragma once

#include <math.h>
#include "../global/type.h"
#include "Controllers.h"

enum SVM_enum
{
    SVM_000,
    SVM_111,
    SVM_001,
    SVM_011,
    SVM_010,
    SVM_110,
    SVM_100,
    SVM_101,
};

struct SVM_t
{
    float M_index;
    float T_0;
    float T_1;
    float T_2;
    float U_motor;
    float check;

    float dutyA;
    float dutyB;
    float dutyC;
};

class SVM_machine
{
public:
    SVM_enum Mstate;
    SVM_machine();

    void iteration(float theta);
    float calc_modulation_index(float alfa, float beta);

private:
    SVM_enum Mstate_last;
    SVM_t SVM;

    void select_state(float theta);

    void handle_100(float theta);
    void handle_110(float theta);
    void handle_010(float theta);
    void handle_011(float theta);
    void handle_001(float theta);
    void handle_101(float theta);

    void set_outputs(float dutyA, float dutyB, float dutyC, float deadtime);
};