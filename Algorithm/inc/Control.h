/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f4xx.h"
#include "arm_math.h"

// physical constants
typedef struct CONTROLLERPARAMS_T
{
	float32_t g; //gravity acceleration
  float32_t m; //mass of the quadrotor
	float32_t L; //center of quadrotor to center of propeller distance
	float32_t k;
	float32_t b; //thrust factor
	float32_t I[9]; //body inertia matrix
	float32_t kd; //drag factor
	
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	
	float32_t integral[3];
	float32_t integral2rd[3];
	//if theta is too small to handle use below parameters
	float32_t KpZ;
	float32_t KiZ;
	float32_t KdZ;
	float32_t integralZ[3];
}ControllerParams;

void InitControllerParams(ControllerParams* pparams);
void thrust(float32_t *T, float32_t *inputs, float32_t k);
void torques(float32_t *tau, float32_t *inputs, float32_t L, float32_t b, float32_t k);
void acceleration(float32_t *a, float32_t *inputs, float32_t *angles, float32_t *vels, float32_t m, float32_t g, float32_t k, float32_t kd);
void angular_acceleration(float32_t *omegad , float32_t *inputs, float32_t* omega, float32_t* I, float32_t L, float32_t b, float32_t k);
void thetadot2omega(float32_t *omega, float32_t *thetadot, float32_t *rpy);
void omega2thetadot(float32_t *thetadot, float32_t *omega, float32_t *rpy);
void err2inputs(float32_t *inputs, ControllerParams params, float32_t *err, float32_t total);
void controller(float32_t *input, ControllerParams params, float32_t *thetadot, float32_t *error, float32_t dt);

#endif
