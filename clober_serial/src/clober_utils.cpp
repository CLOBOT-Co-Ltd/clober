#include <clober_serial/clober_utils.hpp>


float CloberUtils::toRPM(float w){
    float rpm = (w * 60.0) / (2 * PI);
    return rpm;
}

float CloberUtils::toVelocity(float rpm)
{
    float w = (rpm * 2.0 * PI) / 60.0;
    return w;
}

float CloberUtils::toRad(float enc, int ppr)
{
    float rad = (enc * 2.0 * PI) / ppr;
    return rad;
}
