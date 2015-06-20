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

#include "AdditionalMatrix.h"

s32 Matrix_Inv_3x3(float32_t *M, float32_t* A)
{
	// det = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)
	// det = a00(a22a11-a21a12)-a10(a22a01-a21a02)+a20(a12a01-a11a02)
	float32_t det;
	// Invert the matrix
	/*
	| a11 a12 a13 |-1 | a33a22-a32a23 -(a33a12-a32a13) a23a12-a22a13 |
	| a21 a22 a23 | = 1/DET * | -(a33a21-a31a23) a33a11-a31a13 -(a23a11-a21a13) |
	| a31 a32 a33 | | a32a21-a31a22 -(a32a11-a31a12) a22a11-a21a12 |
	| a00 a01 a02 |-1 | a22a11-a21a12 -(a22a01-a21a02) a12a01-a11a02 |
	| a10 a11 a12 | = 1/DET * | -(a22a10-a20a12) a22a00-a20a02 -(a12a00-a10a02) |
	| a20 a21 a22 | | a21a10-a20a11 -(a21a00-a20a01) a11a00-a10a01 |
	*/
	det  = A[0] * (A[8] * A[4] - A[7] * A[5]) -
		A[3] * (A[8] * A[1] - A[7] * A[2]) +
		A[6] * (A[5] * A[1] - A[4] * A[2]);
	// Row 1
	// M[0] = (a22a11-a21a12)/det;
	M[0] = (A[8] * A[4] - A[7] * A[5]) / det;
	// M[1] = -(a22a01-a21a02)/det;
	M[1] = -(A[8] * A[1] - A[7] * A[2]) / det;
	// M[2] = (a12a01-a11a02)/det;
	M[2] = (A[5] * A[1] - A[4] * A[2]) / det;
	// Row 2
	// M[3] = -(a22a10-a20a12)/det;
	M[3] = -(A[8] * A[3] - A[6] * A[5]) / det;
	// M[4] = (a22a00-a20a02)/det;
	M[4] = (A[8] * A[0] - A[6] * A[2]) / det;
	// M[5] = -(a12a00-a10a02)/det;
	M[5] = -(A[5] * A[0] - A[3] * A[2]) / det;
	// Row 3
	// M[6] = (a21a10-a20a11)/det;
	M[6] = (A[7] * A[3] - A[6] * A[4]) / det;
	// M[7] = -(a21a00-a20a01)/det;
	M[7] = -(A[7] * A[0] - A[6] * A[1]) / det;
	// M[8] = (a11a00-a10a01)/det;
	M[8] = (A[4] * A[0] - A[3] * A[1]) / det;
	
	return 1;
}

void Matrix_Multiply_3x3(float32_t *C, float32_t *A, float32_t *B)
{
	C[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	C[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	C[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	C[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	C[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	C[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	C[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	C[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	C[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

void Matrix_Subtract_3x3(float32_t *C, float32_t *A, float32_t *B)
{
	C[0] = A[0] - B[0]; C[1] = A[1] - B[1]; C[2] = A[2] - B[2];
	C[3] = A[3] - B[3]; C[4] = A[4] - B[4]; C[5] = A[5] - B[5];
	C[6] = A[6] - B[6]; C[7] = A[7] - B[7]; C[8] = A[8] - B[8];
}

void Matrix_3x3_Multiply_Vector_3x1(float32_t *VM, float32_t *M, float32_t *V)
{
	VM[0] = M[0] * V[0] + M[1] * V[1] + M[2] * V[2];
	VM[1] = M[3] * V[0] + M[4] * V[1] + M[5] * V[2];
	VM[2] = M[6] * V[0] + M[7] * V[1] + M[8] * V[2];
}

void Matrix_Diag_3x3(float32_t *M, float32_t a0, float32_t a4, float32_t a8)
{
	M[1] = M[2] = M[3] = M[5] = M[6] = M[7] = 0.0f;
	M[0] = a0; M[4] = a4; M[8] = a8;
}
