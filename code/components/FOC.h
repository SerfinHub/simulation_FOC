/*
 * FOC.h
 * Sbach
 * 24.12.2022
 */
#pragma once

#include <math.h>
#include "../global/type.h"
#include "Controllers.h"
#include "SVM.h"

enum FOC_enum
{
	FOC_Init,
	FOC_SS,
	FOC_Active
};
struct Reg_settings_t
{
	float pKp = 10.00f;
	float pTi = 0.001f;
	float pLh = 600.0f;
	float pLl = -600.0f;

	float sKp = 1.10f;
	float sTi = 0.01f;
	float sLh = 300.0f;
	float sLl = -300.0f;

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
	float duty;
	float deadtime;
	float FrameAngle;
	float theta;
};

struct RAMP_t
{
	float Iramp, Sramp, Pramp;
	float Iold, Sold, Pold;
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

	float I_d;
	float I_q;

	float Mindex;

	friend void operator++(FOC_enum &val, int);

	void next_state();

	void handle_sInit(FOC_t &);
	void handle_sSS(FOC_t &);
	void handle_sActive(FOC_t &);
};
