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

#include "Control.h"
#include "AdditionalMatrix.h"
#include "Vector.h"
#include "Rotation.h"
#include "FastMath.h"

void InitControllerParams(ControllerParams* pparams)
{
	//initialise physical constants parameters
	//all parameters must be consistent with the actual physical values
	pparams->g = 9.81f;
	pparams->m = 2.5f;
	pparams->L = 0.45f;
	pparams->k = 0.0003f;
	pparams->b = 0.0000002f;
	Matrix_Diag_3x3(pparams->I, 0.004f, 0.004f, 0.007f);
	pparams->kd = 0.15f;
	//pid controller, parameters need to be tune
	pparams->Kp = 1.0f;
	pparams->Ki = 0.05f;
	pparams->Kd = 0.01f;
	
	pparams->integral[0] = pparams->integral[1] = pparams->integral[2] = 0.0f;
	pparams->integral2rd[0] = pparams->integral2rd[1] = pparams->integral2rd[2] = 0.0f;
}

// Compute thrust given current inputs and thrust coefficient.
void thrust(float32_t *T, float32_t *inputs, float32_t k)
{
	T[0] = 0; T[1] = 0; T[2] = k * (inputs[0] + inputs[1] + inputs[2] + inputs[3]);
}

// Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
void torques(float32_t *tau, float32_t *inputs, float32_t L, float32_t b, float32_t k)
{
	tau[0] = L * k * (inputs[0] - inputs[2]);
	tau[1] = L * k * (inputs[1] - inputs[3]);
	tau[2] = b * (inputs[0] - inputs[1] + inputs[2] - inputs[3]);
}

// Compute acceleration in inertial reference frame
// Parameters:
//   g: gravity acceleration
//   m: mass of quadcopter
//   k: thrust coefficient
//   kd: global drag coefficient
void acceleration(float32_t *a, float32_t *inputs, float32_t *angles, float32_t *vels, float32_t m, float32_t g, float32_t k, float32_t kd)
{
	float32_t gravity[3];
	float32_t R[9];
	float32_t tmp_3x1[3];
	float32_t T[3];
	float32_t Fd[3];

	gravity[0] = 0; gravity[1] = 0; gravity[2] = -g;
	Rotation(R, angles);
	thrust(tmp_3x1, inputs, k);
	Matrix_3x3_Multiply_Vector_3x1(T, R, tmp_3x1);
	
	Fd[0] = -kd * vels[0];
	Fd[1] = -kd * vels[1];
	Fd[2] = -kd * vels[2];
	
	a[0] = gravity[0] + 1.0f / m * T[0] + Fd[0];
	a[1] = gravity[1] + 1.0f / m * T[1] + Fd[1];
	a[2] = gravity[2] + 1.0f / m * T[2] + Fd[2];
}

// Compute angular acceleration in body frame
// Parameters:
//   I: inertia matrix
void angular_acceleration(float32_t *omegad , float32_t *inputs, float32_t* omega, float32_t* I, float32_t L, float32_t b, float32_t k)
{
	float32_t tau[3];
	float32_t tmp_3x1[3], cross[3];
	float32_t II[9];
	
	torques(tau, inputs, L, b, k);
	Matrix_Inv_3x3(II, I);
	Matrix_3x3_Multiply_Vector_3x1(tmp_3x1, I, omega);
	Vector_Cross(cross, omega, tmp_3x1);
	Vector_Subtract(tmp_3x1, tau, cross);
	Matrix_3x3_Multiply_Vector_3x1(omegad, II, tmp_3x1);
}

// Convert derivatives of roll, pitch, yaw to omega.
void thetadot2omega(float32_t *omega, float32_t *thetadot, float32_t *rpy)
{
	float32_t phi = rpy[0];
	float32_t theta = rpy[1];
	float32_t W[9];
	float32_t stheta, ctheta;
	float32_t sphi, cphi;
	
	FastSinCos(theta, &stheta, &ctheta);
	FastSinCos(phi, &sphi, &cphi);
	
	W[0] = 1; W[1] = 0; W[2] = -stheta;
	W[3] = 0; W[4] = cphi; W[5] = ctheta * sphi;
	W[6] = 0; W[7] = -sphi; W[8] = ctheta * cphi;

	Matrix_3x3_Multiply_Vector_3x1(omega, W, thetadot);
}

// Convert omega to roll, pitch, yaw derivatives
void omega2thetadot(float32_t *thetadot, float32_t *omega, float32_t *rpy)
{
	float32_t phi = rpy[0];
	float32_t theta = rpy[1];
	float32_t W[9], WI[9];

	float32_t stheta, ctheta;
	float32_t sphi, cphi;
	FastSinCos(theta, &stheta, &ctheta);
	FastSinCos(phi, &sphi, &cphi);

	W[0] = 1; W[1] = 0; W[2] = -stheta;
	W[3] = 0; W[4] = cphi; W[5] = ctheta * sphi;
	W[6] = 0; W[7] = -sphi; W[8] = ctheta * cphi;

	Matrix_Inv_3x3(WI, W);
	Matrix_3x3_Multiply_Vector_3x1(thetadot, WI, omega);
}

// Given desired torques, desired total thrust, and physical parameters,
// solve for required system inputs.
void err2inputs(float32_t *inputs, ControllerParams params, float32_t *err, float32_t total)
{
    float32_t e0 = err[0], e1 = err[1], e2 = err[2];
    float32_t Ix = params.I[0], Iy = params.I[4], Iz = params.I[8];
    float32_t k = params.k;
    float32_t L = params.L;
    float32_t b = params.b;

    inputs[0] = total/4 -(2 * b * e0 * Ix + e2 * Iz * k * L)/(4 * b * k * L);
    inputs[1] = total/4 + e2 * Iz/(4 * b) - (e1 * Iy)/(2 * k * L);
    inputs[2] = total/4 -(-2 * b * e0 * Ix + e2 * Iz * k * L)/(4 * b * k * L);
    inputs[3] = total/4 + e2 * Iz/(4 * b) + (e1 * Iy)/(2 * k * L);
}

void controller(float32_t *input, ControllerParams params, float32_t *thetadot, float32_t *error, float32_t dt)
{
	float32_t ctheta, cphi;
	float32_t total;
	
	float32_t tmp_1[3], tmp_2[3], tmp_3[3];
	float32_t err[3];
	
	//Compute total thrust
	cphi = FastCos(params.integral[0]);
	ctheta = FastCos(params.integral[1]);
	total = params.m * params.g / params.k / (cphi * ctheta);
	
	//Compute error and inputs.
	Vector_Multiply_By_Scale(tmp_1, thetadot, params.Kd);
	Vector_Multiply_By_Scale(tmp_2, params.integral, params.Kp);
	Vector_Multiply_By_Scale(tmp_3, params.integral2rd, params.Ki);
	Vector_Add(err, tmp_1, tmp_2);
	Vector_Subtract(err, err, tmp_3);
	err2inputs(input, params, err, total);
	
	// Update controller state.
	Vector_Integral(params.integral, thetadot, dt);
	Vector_Integral(params.integral2rd, params.integral, dt);
	
   if ((thetadot[0] < 0.2f) && (thetadot[1] < 0.2f) && (thetadot[2] < 0.2f)){
		 //recalculate
		 cphi = FastCos(params.integral[0]);
		 ctheta = FastCos(params.integral[1]);
		 total = params.m * params.g / params.k / (cphi * ctheta);
		 //to do
	 }
}

