/*
 * matrix.h
 *
 *  Created on: 18.11.2010
 *      Author: mackayl
 */

#ifndef MATRIX_H_
#define MATRIX_H_

/*  This is the datatype used for the math and non-type specific ops.  */

typedef float m_elem;

/*  matrix C = matrix A + matrix B , both of size m x n   */
extern void mat_add( m_elem **A, m_elem **B, m_elem **C, int m, int n );

/*  matrix C = matrix A - matrix B , all of size m x n  */
extern void mat_sub( m_elem **A, m_elem **B, m_elem **C, int m, int n );

/*  matrix C = matrix A x matrix B , A(a_rows x a_cols), B(a_cols x b_cols) */
extern void mat_mult( m_elem **A, m_elem **B, m_elem **C,
		     int a_rows, int a_cols, int b_cols );

#endif /* MATRIX_H_ */
