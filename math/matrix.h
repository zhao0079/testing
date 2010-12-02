/*
 * matrix.h
 *
 *  Created on: 18.11.2010
 *      Author: Laurens Mackay
 */

#ifndef MATRIX_H_
#define MATRIX_H_

typedef float m_elem;

typedef struct
{
	int rows;
	int cols;
	m_elem *a;
} matrix_t;

#define M(m,i,j) m.a[m.cols*i+j]

/*  This is the datatype used for the math and non-type specific ops.  */

matrix_t matrix_create(const int rows, const int cols, m_elem * a);
/*  matrix C = matrix A + matrix B , both of size m x n   */
void matrix_add(const matrix_t a, const matrix_t b, matrix_t c);

/*  matrix C = matrix A - matrix B , all of size m x n  */
void matrix_sub(const matrix_t a, const matrix_t b, matrix_t c);

/*  matrix C = matrix A x matrix B , A(a_rows x a_cols), B(a_cols x b_cols) */
void matrix_mult(const matrix_t a, const matrix_t b, matrix_t c);

void matrix_mult_scalar(const float f, const matrix_t a, matrix_t c);

void matrix_mult_element(const matrix_t a, const matrix_t b, matrix_t c);

#endif /* MATRIX_H_ */
