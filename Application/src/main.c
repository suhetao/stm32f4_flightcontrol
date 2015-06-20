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

#include "stm32f4_delay.h"
#include "INS_EKF.h"
#include "Control.h"
#include "Vector.h"
#include "AdditionalMatrix.h"

int main(void)
{
	unsigned long ulNowTime = 0;
	unsigned long ulLastTime = 0;

	float dt = 1.0f;
	u32 u32KFState = 0;
	//
	float32_t pos[3], vel[3], accel[3], gyro[3], mag[3];
	//
	//system state.
	//position velocity  in the inertial frame
	//roll pitch yaw and the time derivative of yaw, pitch, roll
	float32_t x[3], xdot[3] = {0.0f, 0.0f, 0.0f};
	float32_t theta[3] = {0.0f, 0.0f, 0.0f}, thetadot[3];
	//angular velocitie and angular acceleration in the body fram
	float32_t omega[3], omegadot[3];
	
	//other stuff
	float32_t a[3]; // acceleration
	float32_t input[4]; //motor input
	//
	float32_t dirx[3], error[3];

	INS_EKF_Filter ins_ekf;
	ControllerParams params;
	
	Delay_Init();
	INS_EKF_New(&ins_ekf);
	InitControllerParams(&params);
	
	for(;;){
		//to do
		//get 3d pos and vel from gps 
		
		//to do
		//get 3-axies  accel gyro mag from ahrs
		
		//to do
		//get target location from remote controller
		//assignment dirx
		
		Get_Ms(&ulNowTime);
		
		if(!u32KFState){
			//initialise ins ekf filter
			INS_EKF_Init(&ins_ekf, pos, vel, accel, mag);
			ulLastTime = ulNowTime;
			u32KFState = 1;
		}
		else{
			//update ins ekf filter
			dt = 0.001f * (float)(ulNowTime - ulLastTime);
			INS_EFK_Update(&ins_ekf, mag, pos, vel, gyro, accel, dt);
			INS_EKF_GetPos(&ins_ekf, x);
			
			Vector_Subtract(error, dirx, x);
			//transmit position error to theta and thetadot
			//to do
			
			controller(input, params, thetadot, error, dt);
			
			// Compute forces, torques, and accelerations.
			thetadot2omega(omega, thetadot, theta);
			acceleration(a, input, theta, xdot, params.m, params.g, params.k, params.kd);
			angular_acceleration(omegadot, input, omega, params.I, params.L, params.b, params.k);
			
			//output input to motor
			//to do
			
			// Advance system state.
			Vector_Integral(omega, omegadot, dt);
			omega2thetadot(thetadot, omega, theta); 
			Vector_Integral(theta, thetadot, dt);
			Vector_Integral(xdot, a, dt);
			Vector_Integral(x, xdot, dt);
			//to do 
		}
		ulLastTime = ulNowTime; 
	}
}
