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

#include "Vector.h"

void Vector_Cross(float32_t *c, float32_t *a, float32_t *b)
{
	c[0] = (a[1] * b[2]) - (a[2] * b[1]);
	c[1] = (a[2] * b[0]) - (a[0] * b[2]);
	c[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

void Vector_Subtract(float32_t *c, float32_t *a, float32_t *b)
{
	c[0] = a[0] - b[0];
	c[1] = a[1] - b[1];
	c[2] = a[2] - b[2];
}

void Vector_Add(float32_t *c, float32_t *a, float32_t *b)
{
	c[0] = a[0] + b[0];
	c[1] = a[1] + b[1];
	c[2] = a[2] + b[2];
}

void Vector_Integral(float32_t *result, float32_t *differential, float32_t dt)
{
	result[0] = result[0] + dt * differential[0];
	result[1] = result[1] + dt * differential[1];
	result[2] = result[2] + dt * differential[2];
}

void Vector_Multiply_By_Scale(float32_t *a, float32_t *b, float32_t scale)
{
	a[0] = b[0] * scale;
	a[1] = b[1] * scale;
	a[2] = b[2] * scale;
}
