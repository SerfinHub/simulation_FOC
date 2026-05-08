#include "TrajectoryPlanner.h"

#include <cmath>

void SCurvePlanner::Reset(float position)
{
    ClearPlan();
    state.position = position;
    state.velocity = 0.0f;
    state.acceleration = 0.0f;
    state.jerk = 0.0f;
    state.done = true;
    lastTarget = position;
    initialized = true;
    active = false;
}

TrajectoryState_t SCurvePlanner::Step(float targetPosition, float Ts)
{
    if (!initialized)
    {
        Reset(targetPosition);
        return state;
    }

    const bool targetChanged = Abs(targetPosition - lastTarget) > TRAJ_TARGET_EPS;

    if (targetChanged)
    {
        PlanFromCurrentState(targetPosition);
        elapsedTime = 0.0f;
        lastTarget = targetPosition;
        active = segmentCount > 0;
    }
    else if (!active && Abs(state.position - targetPosition) > TRAJ_TARGET_EPS)
    {
        PlanFromCurrentState(targetPosition);
        elapsedTime = 0.0f;
        lastTarget = targetPosition;
        active = segmentCount > 0;
    }

    if (!active)
    {
        state.jerk = 0.0f;
        state.done = true;
        return state;
    }

    elapsedTime += Ts;
    state = Sample(elapsedTime);

    if (elapsedTime >= totalTime)
    {
        state = EndState();
        state.velocity = 0.0f;
        state.acceleration = 0.0f;
        state.jerk = 0.0f;
        state.done = true;
        active = false;
    }

    return state;
}

TrajectoryState_t SCurvePlanner::GetState() const
{
    return state;
}

void SCurvePlanner::PlanFromCurrentState(float targetPosition)
{
    const TrajectoryState_t start = state;
    ClearPlan();

    const bool moving =
        Abs(start.velocity) > TRAJ_VELOCITY_EPS ||
        Abs(start.acceleration) > TRAJ_ACCEL_EPS;

    if (moving)
    {
        (void)AppendStopToRest(start);
        const TrajectoryState_t stopState = EndState();
        (void)AppendRestToRest(stopState.position, targetPosition);
    }
    else
    {
        (void)AppendRestToRest(start.position, targetPosition);
    }

    if (segmentCount == 0)
    {
        state.position = targetPosition;
        state.velocity = 0.0f;
        state.acceleration = 0.0f;
        state.jerk = 0.0f;
        state.done = true;
        active = false;
    }
}

void SCurvePlanner::ClearPlan()
{
    for (int i = 0; i < MAX_SEGMENTS; ++i)
    {
        segment[i] = TrajectorySegment_t{};
    }

    segmentCount = 0;
    elapsedTime = 0.0f;
    totalTime = 0.0f;
}

bool SCurvePlanner::AppendSegment(float duration, float jerk)
{
    if (duration <= 0.0f)
    {
        return true;
    }

    if (segmentCount >= MAX_SEGMENTS)
    {
        return false;
    }

    TrajectoryState_t start{};
    if (segmentCount == 0)
    {
        start = state;
    }
    else
    {
        start = EndState();
    }

    segment[segmentCount].duration = duration;
    segment[segmentCount].jerk = jerk;
    segment[segmentCount].x0 = start.position;
    segment[segmentCount].v0 = start.velocity;
    segment[segmentCount].a0 = start.acceleration;

    ++segmentCount;
    totalTime += duration;

    return true;
}

bool SCurvePlanner::AppendRestToRest(float startPosition, float targetPosition)
{
    const float distanceSigned = targetPosition - startPosition;
    const float direction = Sign(distanceSigned);
    const float distance = Abs(distanceSigned);

    if (distance <= TRAJ_TARGET_EPS)
    {
        return true;
    }

    // Force the next appended profile to start exactly at rest at startPosition.
    if (segmentCount == 0)
    {
        state.position = startPosition;
        state.velocity = 0.0f;
        state.acceleration = 0.0f;
        state.jerk = 0.0f;
    }

    const float vmax = TRAJ_MAX_SPEED;
    const float amax = TRAJ_MAX_ACCEL;
    const float jmax = TRAJ_MAX_JERK;

    float tj = amax / jmax;
    float ta = (vmax / amax) - tj;
    float tv = 0.0f;

    if (ta < 0.0f)
    {
        // Velocity limit is reached before maximum acceleration.
        tj = std::sqrt(vmax / jmax);
        ta = 0.0f;
    }

    const float dAccDec = 2.0f * amax *
        ((tj * tj) + (1.5f * tj * ta) + (0.5f * ta * ta));

    if (distance > dAccDec)
    {
        tv = (distance - dAccDec) / vmax;
    }
    else
    {
        tv = 0.0f;

        const float taShort = -1.5f * tj +
            0.5f * std::sqrt((tj * tj) + (4.0f * distance / amax));

        if (taShort >= 0.0f)
        {
            ta = taShort;
        }
        else
        {
            ta = 0.0f;
            tj = Cbrt(distance / (2.0f * jmax));
        }
    }

    const float j = jmax * direction;

    bool ok = true;
    ok = ok && AppendSegment(tj, +j);
    ok = ok && AppendSegment(ta, 0.0f);
    ok = ok && AppendSegment(tj, -j);
    ok = ok && AppendSegment(tv, 0.0f);
    ok = ok && AppendSegment(tj, -j);
    ok = ok && AppendSegment(ta, 0.0f);
    ok = ok && AppendSegment(tj, +j);

    return ok;
}

bool SCurvePlanner::AppendStopToRest(const TrajectoryState_t &startState)
{
    const float vAbs = Abs(startState.velocity);
    const float aAbs = Abs(startState.acceleration);

    if (vAbs <= TRAJ_VELOCITY_EPS && aAbs <= TRAJ_ACCEL_EPS)
    {
        return true;
    }

    // Braking direction follows current velocity. If velocity is almost zero,
    // it follows acceleration. This produces a jerk-limited stop profile that
    // preserves continuity of position, velocity and acceleration.
    const float direction =
        (vAbs > TRAJ_VELOCITY_EPS) ? Sign(startState.velocity) : Sign(startState.acceleration);

    const float v0 = direction * startState.velocity;
    const float a0 = direction * startState.acceleration;
    const float jmax = TRAJ_MAX_JERK;
    const float amax = TRAJ_MAX_ACCEL;

    if (segmentCount == 0)
    {
        state = startState;
        state.done = false;
    }

    float aPeakLow = Abs(a0);
    if (aPeakLow < TRAJ_ACCEL_EPS)
    {
        aPeakLow = TRAJ_ACCEL_EPS;
    }


    auto velocityAfterNoPlateau = [v0, a0, jmax](float aPeak) -> float
    {
        const float t1 = (a0 + aPeak) / jmax;
        const float t3 = aPeak / jmax;

        float v = v0;
        v += a0 * t1 - 0.5f * jmax * t1 * t1;
        v += -aPeak * t3 + 0.5f * jmax * t3 * t3;
        return v;
    };

    float aPeak = amax;
    float t2 = 0.0f;
    float vAfterNoPlateau = velocityAfterNoPlateau(aPeak);

    if (vAfterNoPlateau < 0.0f)
    {
        // Triangular deceleration. Find a smaller peak deceleration that makes
        // v_end equal zero without a constant-acceleration segment.
        float low = aPeakLow;
        float high = amax;

        for (int i = 0; i < 32; ++i)
        {
            const float mid = 0.5f * (low + high);
            const float vMid = velocityAfterNoPlateau(mid);

            if (vMid > 0.0f)
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }

        aPeak = 0.5f * (low + high);
        t2 = 0.0f;
    }
    else
    {
        t2 = vAfterNoPlateau / aPeak;
    }

    const float t1 = (a0 + aPeak) / jmax;
    const float t3 = aPeak / jmax;
    const float j = jmax * direction;

    bool ok = true;
    ok = ok && AppendSegment(t1, -j);
    ok = ok && AppendSegment(t2, 0.0f);
    ok = ok && AppendSegment(t3, +j);

    return ok;
}

TrajectoryState_t SCurvePlanner::Sample(float time) const
{
    float t = time;

    for (int i = 0; i < segmentCount; ++i)
    {
        const TrajectorySegment_t &seg = segment[i];

        if (t <= seg.duration)
        {
            const float dt = t;
            const float j = seg.jerk;

            TrajectoryState_t out{};
            out.position = seg.x0 + seg.v0 * dt + 0.5f * seg.a0 * dt * dt +
                (1.0f / 6.0f) * j * dt * dt * dt;
            out.velocity = seg.v0 + seg.a0 * dt + 0.5f * j * dt * dt;
            out.acceleration = seg.a0 + j * dt;
            out.jerk = j;
            out.done = false;
            return out;
        }

        t -= seg.duration;
    }

    return EndState();
}

TrajectoryState_t SCurvePlanner::EndState() const
{
    TrajectoryState_t out = state;

    for (int i = 0; i < segmentCount; ++i)
    {
        const TrajectorySegment_t &seg = segment[i];
        const float dt = seg.duration;
        const float j = seg.jerk;

        out.position = seg.x0 + seg.v0 * dt + 0.5f * seg.a0 * dt * dt +
            (1.0f / 6.0f) * j * dt * dt * dt;
        out.velocity = seg.v0 + seg.a0 * dt + 0.5f * j * dt * dt;
        out.acceleration = seg.a0 + j * dt;
        out.jerk = j;
        out.done = false;
    }

    return out;
}

float SCurvePlanner::Abs(float value)
{
    return value < 0.0f ? -value : value;
}

float SCurvePlanner::Sign(float value)
{
    return value < 0.0f ? -1.0f : 1.0f;
}

float SCurvePlanner::Cbrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }

    return std::pow(value, 1.0f / 3.0f);
}
