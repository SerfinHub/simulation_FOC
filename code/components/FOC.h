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
	float sKp = 1.0f;
	float sTi = 0.01f;
	float sLh = 60.0f;
	float sLl = -60.0f;

	float idKp = 4.0f;
	float idTi = 0.002f;
	float idLh = 60.0f;
	float idLl = -60.0f;

	float iqKp = 4.0f;
	float iqTi = 0.002f;
	float iqLh = 60.0f;
	float iqLl = -60.0f;

	float pKp = 0.0050f;
	float pTi = 0.2f;
	float pLh = 60.0f;
	float pLl = -60.0f;
};

struct FOC_t
{
	float duty;
	float deadtime;
	float angle;
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

	ref_frame_t Iabc;
	ref_frame_t Iab;
	ref_frame_t Idq;

	SVM_machine SVM;

	float I_d;
	float I_q;

	friend void operator++(FOC_enum &val, int);

	void next_state();

	void handle_sInit(FOC_t &);
	void handle_sSS(FOC_t &);
	void handle_sActive(FOC_t &);
};
