/*
 * matrix.c
 *
 *  Created on: 18.11.2010
 *      Author: mackayl
 */
#include "matrix.h"
#include "debug.h"

matrix_t matrix_create(const int rows, const int cols, m_elem * a)
{
	matrix_t ret;
	ret.rows = rows;
	ret.cols = cols;
	ret.a = a;
	return ret;
}

void matrix_add(const matrix_t a, const matrix_t b, matrix_t c)
{
	if (a.rows != c.rows || a.cols != c.cols || b.rows != c.rows || b.cols
			!= c.cols)
	{
		debug_message_buffer("matrix_add: Dimension mismatch");
	}
	for (int i = 0; i < c.rows; i++)
	{
		for (int j = 0; j < c.cols; j++)
		{
			M(c, i, j) = M(a, i, j) + M(b, i, j);
		}

	}
}

void matrix_sub(const matrix_t a, const matrix_t b, matrix_t c)
{
	if (a.rows != c.rows || a.cols != c.cols || b.rows != c.rows || b.cols
			!= c.cols)
	{
		debug_message_buffer("matrix_sub: Dimension mismatch");
	}
	for (int i = 0; i < c.rows; i++)
	{
		for (int j = 0; j < c.cols; j++)
		{
			M(c, i, j) = M(a, i, j) - M(b, i, j);
		}

	}
}

void matrix_mult(const matrix_t a, const matrix_t b, matrix_t c)
{
	if (a.rows != c.rows || b.cols != c.cols || a.cols != b.rows)
	{
		debug_message_buffer("matrix_mult: Dimension mismatch");
	}
	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < b.cols; j++)
		{
			M(c, i, j) = 0;
			for (int k = 0; k < a.cols; k++)
			{
				M(c, i, j) += M(a, i, k) * M(b, k, j);
			}
		}

	}
}

void matrix_mult_scalar(const float f, const matrix_t a, matrix_t c)
{
	if (a.rows != c.rows || a.cols != c.cols)
	{
		debug_message_buffer("matrix_mult_scalar: Dimension mismatch");
	}
	for (int i = 0; i < c.rows; i++)
	{
		for (int j = 0; j < c.cols; j++)
		{
			M(c, i, j) = f * M(a, i, j);
		}

	}
}

/**********************************************************

 Matrix math functions

 */
//
//void mat_add(const m_elem *a, const m_elem *b, m_elem *c, int m, int n)
//{
//	int i, j;
//
//	for (i = 0; i < m; i++)//rows
//	{
//		for (j = 0; j < n; j++)//colls
//		{
//			M_ELEM_OF(c, n, i, j) = M_ELEM_OF(a, n, i, j) + M_ELEM_OF(b, n, i,
//					j);
//		}
//	}
//}
//
///* mat_sub()
// This function computes C = A - B, for matrices of size m x n  */
//
//void mat_sub(m_elem **a, m_elem **b, m_elem **c, int m, int n)
//{
//	int i, j;
//	m_elem *a_ptr, *b_ptr, *c_ptr;
//
//	for (j = 1; j <= m; j++)
//	{
//		a_ptr = &a[j][1];
//		b_ptr = &b[j][1];
//		c_ptr = &c[j][1];
//
//		for (i = 1; i <= n; i++)
//			*c_ptr++ = *a_ptr++ - *b_ptr++;
//	}
//}
//
///*  mat_mult
// This function performs a matrix multiplication.
// */
//void mat_mult(m_elem **a, m_elem **b, m_elem **c, int a_rows, int a_cols,
//		int b_cols)
//{
//	int i, j, k;
//	m_elem *a_ptr;
//	m_elem temp;
//
//	for (i = 1; i <= a_rows; i++)
//	{
//		a_ptr = &a[i][0];
//		for (j = 1; j <= b_cols; j++)
//		{
//			temp = 0.0;
//			for (k = 1; k <= a_cols; k++)
//				temp = temp + (a_ptr[k] * b[k][j]);
//			c[i][j] = temp;
//		}
//	}
//}
