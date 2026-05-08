#pragma once

#include "Controllers.h"

class AngleUnwrapper
{
public:
    void Reset(float wrappedAngle)
    {
        lastAngle = wrappedAngle;
        position = wrappedAngle;
        initialized = true;
    }

    float Update(float wrappedAngle)
    {
        if (!initialized)
        {
            Reset(wrappedAngle);
            return position;
        }

        float delta = wrappedAngle - lastAngle;

        if (delta > MATH_PI)
        {
            delta -= MATH_2PI;
        }
        else if (delta < -MATH_PI)
        {
            delta += MATH_2PI;
        }

        position += delta;
        lastAngle = wrappedAngle;

        return position;
    }

    float GetPosition() const
    {
        return position;
    }

private:
    float lastAngle{};
    float position{};
    bool initialized{false};
};
