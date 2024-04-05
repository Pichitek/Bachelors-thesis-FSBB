#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct
{
	// gain
	float Kp;
	float Ki;

	// sample time [s]
	float T;

	// "memory"
	float integrator;
	float prevError;

	// clamping variables
	float limMin;
	float limMax;

	float limMinInt;
	float limMaxInt;

	//output
	float out;

}PIController;

void PI_Init(PIController *pi, float _Kp, float _Ki);
float PI_update(PIController *pi, float set, float meas);

#endif
