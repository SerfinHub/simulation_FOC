#pragma once

#include <cstddef>

// S-curve trajectory limits for the position servo.
// Units are mechanical radians, seconds.
#define TRAJ_MAX_SPEED       50.0f     // [rad/s]
#define TRAJ_MAX_ACCEL       500.0f    // [rad/s^2]
#define TRAJ_MAX_JERK        5000.0f   // [rad/s^3]
#define TRAJ_TARGET_EPS      0.0001f   // [rad]
#define TRAJ_VELOCITY_EPS    0.0001f   // [rad/s]
#define TRAJ_ACCEL_EPS       0.001f    // [rad/s^2]

struct TrajectoryState_t
{
    float position{};
    float velocity{};
    float acceleration{};
    float jerk{};
    bool done{true};
};

struct TrajectorySegment_t
{
    float duration{};
    float jerk{};

    float x0{};
    float v0{};
    float a0{};
};

class SCurvePlanner
{
public:
    void Reset(float position);

    // Generates a continuous trajectory from the current trajectory state.
    // If a new target is received while moving, the plan starts from the
    // current state: position, velocity and acceleration.
    TrajectoryState_t Step(float targetPosition, float Ts);

    TrajectoryState_t GetState() const;

private:
    static constexpr int MAX_SEGMENTS = 16;

    TrajectoryState_t state{};
    TrajectorySegment_t segment[MAX_SEGMENTS]{};

    int segmentCount{0};
    float elapsedTime{};
    float totalTime{};
    float lastTarget{};
    bool initialized{false};
    bool active{false};

    void PlanFromCurrentState(float targetPosition);
    void ClearPlan();

    bool AppendSegment(float duration, float jerk);
    bool AppendRestToRest(float startPosition, float targetPosition);
    bool AppendStopToRest(const TrajectoryState_t &startState);

    TrajectoryState_t Sample(float time) const;
    TrajectoryState_t EndState() const;

    static float Abs(float value);
    static float Sign(float value);
    static float Cbrt(float value);
};
