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
	portInit(aState);
}

/* Called whenever the simulation time reaches a multiple of the sample      */
/* time.                                                                     */
DLLEXPORT void plecsOutput(struct SimulationState *aState)
{
	measRead(aState);

	machine.iteration();
}