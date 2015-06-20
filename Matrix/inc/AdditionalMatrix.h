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

#ifndef _ADDITIONALMATRIX_H_
#define _ADDITIONALMATRIX_H_

#include "stm32f4xx.h"
#include "arm_math.h"

s32 Matrix_Inv_3x3(float32_t *M, float32_t* A);
void Matrix_Multiply_3x3(float32_t *C, float32_t *A, float32_t *B);
void Matrix_Subtract_3x3(float32_t *C, float32_t *A, float32_t *B);
void Matrix_3x3_Multiply_Vector_3x1(float32_t *VM, float32_t *M, float32_t *V);
void Matrix_Diag_3x3(float32_t *M, float32_t a0, float32_t a4, float32_t a8);

#endif
