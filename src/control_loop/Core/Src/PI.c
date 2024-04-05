#include "PI.h"
#include "stdint.h"

void PIController_Init(PIController *pid, float _Kp, float _Ki)
{
	// clear variables
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->out = 0.0f;

	// set gains
	pid->Kp = _Kp;
	pid->Ki = _Ki;

}

float PIController_Update(PIController *pid, uint16_t setpoint, uint16_t measurement) {

	// error
	uint16_t error = setpoint - measurement;

	// proportional
    float proportional = pid->Kp * error;

    // integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	// integrator clamping
    if (pid->integrator > pid->limMaxInt)
    {
        pid->integrator = pid->limMaxInt;
    }
    else if (pid->integrator < pid->limMinInt)
    {
        pid->integrator = pid->limMinInt;
    }

    pid->out = proportional + pid->integrator;

    if (pid->out > pid->limMax)
    {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin)
    {
    	pid->out = pid->limMin;
    }

    pid->prevError = error;


    return pid->out;
}
