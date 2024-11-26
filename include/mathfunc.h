#ifndef MATHFUNC_H
#define MATHFUNC_H

#include<math.h>
#define pi M_PI

extern "C"{//添加extern "C"
float vec_dot(float v1[3],float v2[3]);
float vec_length(float v[3]);
void vec_rescale(float v[3],float l,float nv[3]);
int fx2solv(float a,float b,float c,float *p1,float *p2);
float pointsdistance(float P1[3],float P2[3]);
void vec_subtraction(float v1[3],float v2[3],float v[3]);
void vec_addition(float v1[3],float v2[3],float v[3]);

float vec_angle(float v1[3],float v2[3],float f);
float solve_trangle(float a,float b,float c);
void vec_cross(float v1[3],float v2[3],float v[3]);
float calcu_angle(float sa,float ca,float cp2);


void MatrixmultPoint(float P0[3],float T[4][4],float P[3]);
void Matrixmult(float T1[4][4],float T2[4][4],float T[4][4]);
void MatrixT2MatrixI(float T[4][4],float I[4][4]);


}
#endif