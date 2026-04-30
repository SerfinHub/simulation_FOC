/*
 * stdafx.cpp
 * SBACH
 * 03.04.2022
 */
 
#include "type.h"
#include "filter.h"

union ALARM alarm{};

struct Settings_t Param{};
struct Settings_t Set{};

struct Measurements_t Meas{};
struct Measurements_t Meas_filter{};
struct Measurements_t Meas_alarm_H{};
struct Measurements_t Meas_alarm_L{};

struct SimulationState *aState_global = nullptr;

static Filter U_dc1_IIR;
static Filter I_m1_IIR;
static Filter I_m2_IIR;
static Filter I_m3_IIR;

static void readInputsAndReferences(struct SimulationState *aState)
{
    Meas.U_dc1 = aState->inputs[0];
    Meas.I_m1 = aState->inputs[1];
    Meas.I_m2 = aState->inputs[2];
    Meas.I_m3 = aState->inputs[3];
    Meas.theta = aState->inputs[4];
    Meas.speed = aState->inputs[5];

    Set.current_ref = aState->inputs[6];
    Set.torque_ref = aState->inputs[7];
    Set.speed_ref = aState->inputs[8];
    Set.position_ref = aState->inputs[9];
}

/* Called once before the simulation */
DLLEXPORT void plecsSetSizes(struct SimulationSizes *aSizes)
{
    aSizes->numInputs = DLL_INPUTS_NUMBER;
    aSizes->numOutputs = DLL_OUTPUTS_NUMBER;
    aSizes->numStates = 0;
    aSizes->numParameters = DLL_PARAMETERS_NUMBER;
}

/* Only one measurement of simulation parameters */
void portInit(struct SimulationState *aState)
{
    Param.Ts = aState->parameters[0];
    Param.Cdc = aState->parameters[1];
}

/* Initialize measurements and filters at the start of each simulation. */
void measInit(struct SimulationState *aState)
{
    readInputsAndReferences(aState);

    Meas.position = 0.0f;

    U_dc1_IIR.reset(Meas.U_dc1);
    I_m1_IIR.reset(Meas.I_m1);
    I_m2_IIR.reset(Meas.I_m2);
    I_m3_IIR.reset(Meas.I_m3);

    Meas_filter.U_dc1 = Meas.U_dc1;
    Meas_filter.I_m1 = Meas.I_m1;
    Meas_filter.I_m2 = Meas.I_m2;
    Meas_filter.I_m3 = Meas.I_m3;
    Meas_filter.theta = Meas.theta;
    Meas_filter.speed = Meas.speed;
    Meas_filter.position = Meas.position;
}

/* Reading of cycling parameters */
void measRead(struct SimulationState *aState)
{
    readInputsAndReferences(aState);

    // Filtration
    Meas_filter.U_dc1 = U_dc1_IIR.step(Meas.U_dc1);
    Meas_filter.I_m1 = I_m1_IIR.step(Meas.I_m1);
    Meas_filter.I_m2 = I_m2_IIR.step(Meas.I_m2);
    Meas_filter.I_m3 = I_m3_IIR.step(Meas.I_m3);
    Meas_filter.theta = Meas.theta;
    Meas_filter.speed = Meas.speed;
    Meas_filter.position = Meas.position;
}

/* Present state data for different channels in PLECS*/
void Oscilloscope(float ch1, float ch2, float ch3, float ch4)
{
    aState_global->outputs[6] = ch1;
    aState_global->outputs[7] = ch2;
    aState_global->outputs[8] = ch3;
    aState_global->outputs[9] = ch4;
}
