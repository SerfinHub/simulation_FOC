/*
 * FOC motor control
 * main.cpp
 * SBACH
 * 24.12.2022
 */
#include "stdafx.h"

static stateMachine machine;

/* Called once during the initialization of a new simulation.                */
DLLEXPORT void plecsStart(struct SimulationState *aState)
{
    aState_global = aState;

    alarm.all = 0;

    Param = {};
    Set = {};
    Meas = {};
    Meas_filter = {};
    Meas_alarm_H = {};
    Meas_alarm_L = {};

    portInit(aState);
    measInit(aState);

    machine = stateMachine();

    for (int i = 0; i < DLL_OUTPUTS_NUMBER; ++i)
    {
        aState->outputs[i] = 0.0f;
    }
}

/* Called whenever the simulation time reaches a multiple of the sample      */
/* time.                                                                     */
DLLEXPORT void plecsOutput(struct SimulationState *aState)
{
    aState_global = aState;

    measRead(aState);
    machine.iteration();
    
    aState->outputs[6] = machine.Mstate;
    aState->outputs[7] = machine.Mstate;
}

/* Called when simulation is terminated */
DLLEXPORT void plecsTerminate(struct SimulationState* aState)
{
    (void)aState;
    aState_global = nullptr;
}
