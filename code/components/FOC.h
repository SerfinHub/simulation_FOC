/*
 * FOC.h
 * Sbach
 * 24.12.2022
 */
#pragma once

#include <cmath>
#include "../global/type.h"
#include "Controllers.h"
#include "SVM.h"
#include "AngleUnwrapper.h"
#include "TrajectoryPlanner.h"

enum FOC_enum
{
    FOC_Init,
    FOC_SS,
    FOC_Active
};

enum ControlMode_t
{
    CONTROL_CURRENT = 0,
    CONTROL_TORQUE = 1,
    CONTROL_SPEED = 2,
    CONTROL_POSITION = 3
};

// Temporary motor constants and limits. Move these to PLECS parameters later.
#define MOTOR_KT              1.0f    // [Nm/A], temporary torque constant
#define MOTOR_MAX_IQ          20.0f   // [A]
#define MOTOR_MAX_TORQUE      20.0f   // [Nm]
#define MOTOR_MAX_SPEED       100.0f  // [rad/s]
#define MOTOR_POLE_PAIRS      1.0f    // mechanical-to-electrical angle ratio
#define MOTOR_THETA_OFFSET    0.0f    // [rad]

struct Reg_settings_t
{
    // Position loop output is speed_ref [rad/s].
    float pKp = 10.00f;
    float pTi = 0.001f;
    float pLh = MOTOR_MAX_SPEED;
    float pLl = -MOTOR_MAX_SPEED;

    // Speed loop output is torque_ref [Nm].
    float sKp = 1.10f;
    float sTi = 0.01f;
    float sLh = MOTOR_MAX_TORQUE;
    float sLl = -MOTOR_MAX_TORQUE;

    float idKp = 0.100f;
    float idTi = 0.00011f;
    float idLh = 300.0f;
    float idLl = -300.0f;

    float iqKp = 25.000f;
    float iqTi = 0.00011f;
    float iqLh = 300.0f;
    float iqLl = -300.0f;
};

struct FOC_t
{
    float duty{};
    float deadtime{};
    float FrameAngle{};
    float theta{};
};

struct RAMP_t
{
    float Iramp{};
    float Sramp{};
    float Pramp{};
    float Iold{};
    float Sold{};
    float Pold{};
};

class stateMachine
{
public:
    FOC_enum Mstate;
    stateMachine();

    void iteration();

private:
    float counter_ss;

    FOC_enum Mstate_last;
    ControlMode_t controlMode_last;

    FOC_t FOC;
    RAMP_t ramp;
    Reg_settings_t reg;

    MyReg piId;
    MyReg piIq;
    MyReg piS;
    MyReg piP;

    MyReg TransMatrix;

    ref_frame_t Idq;
    ref_frame_t Vab;
    ref_frame_t Vdq;

    SVM_machine SVM;

    AngleUnwrapper angleUnwrapper;
    SCurvePlanner positionPlanner;
    TrajectoryState_t trajState;

    float I_d;
    float I_q;

    float Mindex;

    friend void operator++(FOC_enum &val, int);

    void next_state();

    void handle_sInit(FOC_t &foc);
    void handle_sSS(FOC_t &foc);
    void handle_sActive(FOC_t &foc);
};
