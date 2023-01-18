/*
 * type.h
 * Sbach
 * 03.04.2022
 */

#pragma once

#include <stdint.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "../ports/dllheader.h"

/* IO DEFINES */
#define DLL_INPUTS_NUMBER 10
#define DLL_OUTPUTS_NUMBER 10
#define DLL_PARAMETERS_NUMBER 2

typedef UINT16 uint16_t;
typedef UINT32 uint32_t;
typedef UINT64 uint64_t;
typedef INT64 int64_t;
typedef INT32 int32_t;
typedef INT16 int16_t;

#define PI_antiwindup_fast_asm PI_antiwindup_fast

#define GPIO_SET(x) aState_global->outputs[x] = 1
#define GPIO_CLEAR(x) aState_global->outputs[x] = 0
#define GPIO_TOGGLE(x) aState_global->outputs[x] ^= 1

struct Measurements_t
{
    float U_dc1;
    float I_m1;
    float I_m2;
    float I_m3;
    float theta;
    float speed;
    float position;
};

struct Settings_t
{
    float Ts;
    float Cdc;
    float Llk;

    float current_ref;
    float torque_ref;
    float speed_ref;
    float position_ref;

    float I_old;
    float S_old;
    float P_old;

    float state;
};

struct ALARM_BITS_t
{
    uint32_t U_dc1_H : 1;
    uint32_t U_dc1_L : 1;
    uint32_t I_m1_H : 1;
    uint32_t I_m1_L : 1;
    uint32_t I_m2_H : 1;
    uint32_t I_m2_L : 1;
    uint32_t I_m3_H : 1;
    uint32_t I_m3_L : 1;
    uint32_t TZ_supply : 1;
    uint32_t PLL_ERR : 1;
    uint32_t RECT_ERR : 1;
    uint32_t FOC_ERR : 1;
    uint32_t rsvd1 : 20;
};

union ALARM
{
    uint32_t all;
    struct ALARM_BITS_t bit;
};

void portInit(struct SimulationState *aState);
void measRead(struct SimulationState *aState);
void setOutput(struct SimulationState *aState);
void Oscilloscope(float ch1, float ch2, float ch3, float ch4);

float IIR_filter(float x0, float number, float a0, float a1, float a2, float b0, float b1, float b2);

extern union ALARM alarm;

extern struct Settings_t Param;
extern struct Settings_t Set;

extern struct Measurements_t Meas;
extern struct Measurements_t Meas_filter;
extern struct Measurements_t Meas_alarm_H;
extern struct Measurements_t Meas_alarm_L;

extern struct SimulationState *aState_global;