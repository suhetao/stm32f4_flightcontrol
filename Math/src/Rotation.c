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

#include "Rotation.h"
#include "FastMath.h"

void Rotation(float32_t *R, float32_t *angles)
{
	float32_t phi = angles[2];
	float32_t theta = angles[1];
	float32_t psi = angles[0];

	float32_t sphi, cphi;
	float32_t stheta, ctheta;
	float32_t spsi, cpsi;

	FastSinCos(phi, &sphi, &cphi);
	FastSinCos(theta, &stheta, &ctheta);
	FastSinCos(psi, &spsi, &cpsi);

	R[0] = cphi * ctheta;
	R[3] = ctheta * sphi;
	R[6] = -stheta;

	R[1] = cphi * stheta * spsi - cpsi * sphi;
	R[4] = cphi * cpsi + sphi * stheta * spsi;
	R[7] = ctheta * spsi;

	R[2] = sphi * spsi + cphi * cpsi * stheta;
	R[5] = cpsi * sphi * stheta - cphi * spsi;
	R[8] = ctheta * cpsi;
}
