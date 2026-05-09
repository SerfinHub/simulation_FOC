/*
 * DAB.cpp
 * SBACH
 * 24.12.2022
 */
#include "FOC.h"

static ref_frame_t Iabc{};
static ref_frame_t Iab{};

static float clampFloat(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }

    if (value < low)
    {
        return low;
    }

    return value;
}

static float wrapTo2Pi(float angle)
{
    while (angle >= MATH_2PI)
    {
        angle -= MATH_2PI;
    }

    while (angle < 0.0f)
    {
        angle += MATH_2PI;
    }

    return angle;
}

static ControlMode_t decodeControlMode(float modeInput)
{
    const int mode = static_cast<int>(modeInput + 0.5f);

    switch (mode)
    {
    case CONTROL_CURRENT:
        return CONTROL_CURRENT;

    case CONTROL_TORQUE:
        return CONTROL_TORQUE;

    case CONTROL_SPEED:
        return CONTROL_SPEED;

    case CONTROL_POSITION:
        return CONTROL_POSITION;

    default:
        return CONTROL_CURRENT;
    }
}

static float torqueToIq(float torqueRef)
{
    if (MOTOR_KT > -1.0e-6f && MOTOR_KT < 1.0e-6f)
    {
        return 0.0f;
    }

    return clampFloat(
        MOTOR_TORQUE_DIRECTION * torqueRef / MOTOR_KT,
        -MOTOR_MAX_IQ,
        MOTOR_MAX_IQ
    );
}

stateMachine::stateMachine()
    : Mstate(FOC_Init),
      counter_ss(0.0f),
      Mstate_last(FOC_Init),
      controlMode_last(CONTROL_CURRENT),
      FOC{},
      ramp{},
      reg{},
      piId(reg.idKp, reg.idTi, reg.idLh, reg.idLl),
      piIq(reg.iqKp, reg.iqTi, reg.iqLh, reg.iqLl),
      piS(reg.sKp, reg.sTi, reg.sLh, reg.sLl),
      piP(reg.pKp, reg.pTi, reg.pLh, reg.pLl),
      TransMatrix{},
      Idq{},
      Vab{},
      Vdq{},
      SVM{},
      angleUnwrapper{},
      positionPlanner{},
      trajState{},
      I_d(0.0f),
      I_q(0.0f),
      Mindex(0.0f)
{
}

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

void stateMachine::handle_sInit(FOC_t &foc)
{
    (void)foc;

    if (Mstate_last != Mstate)
    {
        Mstate_last = Mstate;
    }

    angleUnwrapper.Reset(Meas.theta);
    positionPlanner.Reset(Meas.theta);
    trajState = positionPlanner.GetState();

    next_state();
}

void stateMachine::handle_sSS(FOC_t &foc)
{
    if (Mstate_last != Mstate)
    {
        counter_ss = 0.0f;
        Mstate_last = Mstate;

        // High keys set to 1 to magnetize stator.
        foc.duty = 1.0f;
    }

    counter_ss += Param.Ts;
    if (counter_ss > 30.0f)
    {
        alarm.bit.FOC_ERR = 1;
    }
    if (Meas_filter.U_dc1 > 299.0f)
    {
        next_state();
    }
}

void stateMachine::handle_sActive(FOC_t &foc)
{
    input_t inS{};
    input_t inP{};
    input_t inId{};
    input_t inIq{};

    if (Mstate_last != Mstate)
    {
        counter_ss = 0.0f;
        Mstate_last = Mstate;

        angleUnwrapper.Reset(Meas.theta);
        Meas.position = MOTOR_POSITION_DIRECTION * angleUnwrapper.GetPosition();
        positionPlanner.Reset(Meas.position);
        trajState = positionPlanner.GetState();

        piP.Reset(0.0f);
        piS.Reset(0.0f);
    }

    // Angle sensor gives wrapped mechanical angle [-pi, pi].
    // Meas.position is the unwrapped multi-turn mechanical position [rad].
    // Raw angle from sensor [-pi, pi]
    const float raw_angle = Meas.theta;

    // Raw multi-turn mechanical position
    const float raw_unwrapped_position = angleUnwrapper.Update(raw_angle);

    // Servo/control position convention
    Meas.position = MOTOR_POSITION_DIRECTION * raw_unwrapped_position;

    // Electrical frame angle for FOC.
    foc.FrameAngle = wrapTo2Pi(raw_angle * MOTOR_POLE_PAIRS + MOTOR_THETA_OFFSET);

    /* Current -> dq */
    Iabc.a = Meas_filter.I_m1;
    Iabc.b = Meas_filter.I_m2;
    Iabc.c = Meas_filter.I_m3;

    Iab = TransMatrix.abc_ab(Iabc);
    Idq = TransMatrix.ab_dq(Iab, foc.FrameAngle);

    I_d = Idq.d;
    I_q = Idq.q;

    const ControlMode_t controlMode = decodeControlMode(Set.control_mode);

    if (controlMode != controlMode_last)
    {
        // External loops must not keep old integral state after mode changes.
        piP.Reset(0.0f);
        piS.Reset(0.0f);

        if (controlMode == CONTROL_POSITION)
        {
            positionPlanner.Reset(Meas.position);
            trajState = positionPlanner.GetState();
        }

        controlMode_last = controlMode;
    }

    float id_ref_cmd = 0.0f;
    float iq_ref_cmd = 0.0f;
    float torque_ref_cmd = 0.0f;
    float speed_ref_cmd = 0.0f;

    switch (controlMode)
    {
    case CONTROL_CURRENT:
        // Direct q-axis current control [A].
        iq_ref_cmd = clampFloat(Set.current_ref, -MOTOR_MAX_IQ, MOTOR_MAX_IQ);
        break;

    case CONTROL_TORQUE:
        // Direct torque control [Nm]. Kt is temporarily 1.0 Nm/A.
        torque_ref_cmd = clampFloat(Set.torque_ref, -MOTOR_MAX_TORQUE, MOTOR_MAX_TORQUE);
        iq_ref_cmd = torqueToIq(torque_ref_cmd);
        break;

    case CONTROL_SPEED:
        // Speed loop output is torque_ref_cmd [Nm].
        speed_ref_cmd = clampFloat(Set.speed_ref, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
        inS.error = speed_ref_cmd - Meas.speed;
        torque_ref_cmd = clampFloat(piS.Calculate(inS), -MOTOR_MAX_TORQUE, MOTOR_MAX_TORQUE);
        iq_ref_cmd = torqueToIq(torque_ref_cmd);
        break;

    case CONTROL_POSITION:
        // Full-state S-curve planning: planner starts from current trajectory
        // position, velocity and acceleration when the target changes.
        trajState = positionPlanner.Step(Set.position_ref, Param.Ts);

        inP.error = trajState.position - Meas.position;
        speed_ref_cmd = piP.Calculate(inP) + trajState.velocity;
        speed_ref_cmd = clampFloat(speed_ref_cmd, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

        inS.error = speed_ref_cmd - Meas.speed;
        torque_ref_cmd = clampFloat(piS.Calculate(inS), -MOTOR_MAX_TORQUE, MOTOR_MAX_TORQUE);
        iq_ref_cmd = torqueToIq(torque_ref_cmd);
        break;

    default:
        iq_ref_cmd = 0.0f;
        break;
    }

    /* Iq PI torque/current loop */
    inIq.error = iq_ref_cmd - I_q;
    Vdq.q = piIq.Calculate(inIq);

    /* Id PI flux loop */
    inId.error = id_ref_cmd - I_d;
    Vdq.d = piId.Calculate(inId);

    /* Vdq -> Vab */
    Vab = TransMatrix.dq_ab(Vdq, foc.FrameAngle);

    /* Stator angle calculation */
    foc.theta = std::atan2(Vab.beta, Vab.alfa);
    if (foc.theta < 0.0f)
    {
        foc.theta += MATH_2PI;
    }

    /* Modulator */
    Mindex = SVM.calc_modulation_index(Vab.alfa, Vab.beta);
    SVM.iteration(foc.theta);

    // Debug channels
    Oscilloscope(
        Meas.position,                       // outputs[6]
        trajState.position - Meas.position,  // outputs[7]
        speed_ref_cmd,                       // outputs[8]
        iq_ref_cmd                           // outputs[9]
    );
}
