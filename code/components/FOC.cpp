/*
 * DAB.cpp
 * SBACH
 * 24.12.2022
 */
#include "FOC.h"

stateMachine::stateMachine() : counter_ss(0.0f),
							   Mstate(FOC_Init),
							   Mstate_last(FOC_Init),
							   FOC(),
							   reg(),
							   piId(reg.idKp, reg.idTi, reg.idLh, reg.idLl),
							   piIq(reg.iqKp, reg.iqTi, reg.iqLh, reg.iqLl),
							   piS(reg.sKp, reg.sTi, reg.sLh, reg.sLl),
							   piP(reg.pKp, reg.pTi, reg.pLh, reg.pLl)
{
}

static ref_frame_t Iabc;
static ref_frame_t Iab;

void operator++(FOC_enum &val, int)
{
	switch (val)
	{
	case FOC_Init:
		val = FOC_SS;
		break;
	case FOC_SS:
		val = FOC_Active;
		break;
	default:
		val = FOC_Init;
		break;
	}
}

void stateMachine::iteration()
{
	switch (Mstate)
	{
	case FOC_Init:
		handle_sInit(FOC);
		break;

	case FOC_SS:
		handle_sSS(FOC);
		break;

	case FOC_Active:
		handle_sActive(FOC);
		break;
	default:
		alarm.bit.FOC_ERR = 1;
		break;
	}
}

void stateMachine::next_state()
{
	Mstate++;
}

void stateMachine::handle_sInit(FOC_t &FOC)
{

	if (Mstate_last != Mstate)
	{
		Mstate_last = Mstate;
	}
	next_state();
}

void stateMachine::handle_sSS(FOC_t &FOC)
{
	if (Mstate_last != Mstate)
	{
		counter_ss = 0.0f;
		Mstate_last = Mstate;

		// High keys set to 1 to magnetize stator
		FOC.duty = 1.0f;
	}

	counter_ss += Param.Ts;
	if (counter_ss > 30.0f)
		alarm.bit.FOC_ERR = 1;
	if ((Meas_filter.U_dc1 > 299.0f))
		next_state();
}

void stateMachine::handle_sActive(FOC_t &FOC)
{
	input_t inS;
	input_t inP;
	input_t inId;
	input_t inIq;

	float outS;
	float outP;

	if (Mstate_last != Mstate)
	{
		counter_ss = 0.0f;
		Mstate_last = Mstate;
	}

	/* Theta recalculation*/
	FOC.FrameAngle = Meas.theta;

	/* Position control loop */
	Meas.position += 10.1f * Meas.speed * Param.Ts;

	inP.error = Set.position_ref - Meas.position;
	piP.out = piP.Calculate(inP);

	/* Speed control loop */
	// inS.error = Set.speed_ref - Meas.speed;
	inS.error = piP.out - Meas.speed;
	piS.out = piS.Calculate(inS);

	/* Current -> dq */
	Iabc.a = Meas_filter.I_m1;
	Iabc.b = Meas_filter.I_m2;
	Iabc.c = Meas_filter.I_m3;

	Iab = TransMatrix.abc_ab(Iabc);
	Idq = TransMatrix.ab_dq(Iab, FOC.FrameAngle);

	/* Iq pi torque */
	// ramp.Iramp += Param.Ts * 400.0f * (Set.current_ref - ramp.Iold);
	// ramp.Iold = ramp.Iramp;
	inIq.error = piS.out - Idq.q;
	Vdq.q = piIq.Calculate(inIq);

	/* Id pi flux*/
	inId.error = 0.0f - Idq.d;
	Vdq.d = piId.Calculate(inId);

	/* Vdq -> Vab*/
	Vab = TransMatrix.dq_ab(Vdq, FOC.FrameAngle);

	/* Stator angle calculation */
	FOC.theta = atan2(Vab.beta, Vab.alfa);
	if (FOC.theta < 0.0f)
		FOC.theta += (2.0f * MATH_PI);

	/* Modulator */
	Mindex = SVM.calc_modulation_index(Vab.alfa, Vab.beta);
	SVM.iteration(FOC.theta);

	Oscilloscope(Set.position_ref, Meas.position, inP.error, piP.out);
}
