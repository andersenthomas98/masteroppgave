#ifndef _MATRIX_OPERATIONS_H_
#define _MATRIX_OPERATIONS_H_

int choldc1(float * a, float * p, int n);

int choldcsl(float * A, float * a, float * p, int n);

int cholsl(float * A, float * a, float * p, int n);

void zeros(float * a, int m, int n);

void mulmat(float * a, float * b, float * c, int arows, int acols, int bcols); /* C <- A * B */

void mulvec(float * a, float * x, float * y, int m, int n);

void transpose(float * a, float * at, int m, int n); /* A <- A + B */

void accum(float * a, float * b, int m, int n); /* C <- A + B */

void add(float * a, float * b, float * c, int n); /* C <- A - B */

void sub(float * a, float * b, float * c, int n);

void negate(float * a, int m, int n);

void mat_addeye(float * a, int n);

#endif /*_MATRIX_OPERATIONS_H_*/