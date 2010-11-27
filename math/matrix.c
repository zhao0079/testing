/*
 * matrix.c
 *
 *  Created on: 18.11.2010
 *      Author: mackayl
 */
#include "matrix.h"

/**********************************************************

  Matrix math functions

*/

void mat_add( m_elem **a, m_elem **b, m_elem **c, int m, int n )
{
  int i,j;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  for( j = 1; j <= m; j++)
    {
      a_ptr = &a[j][1];
      b_ptr = &b[j][1];
      c_ptr = &c[j][1];

      for( i = 1; i <= n; i++)
	*c_ptr++ = *a_ptr++ + *b_ptr++;
    }
}


/* mat_sub()
   This function computes C = A - B, for matrices of size m x n  */

void mat_sub( m_elem **a, m_elem **b, m_elem **c, int m, int n )
{
  int i,j;
  m_elem  *a_ptr, *b_ptr, *c_ptr;

  for( j = 1; j <= m; j++)
    {
      a_ptr = &a[j][1];
      b_ptr = &b[j][1];
      c_ptr = &c[j][1];

      for( i = 1; i <= n; i++)
	*c_ptr++ = *a_ptr++ - *b_ptr++;
    }
}

/*  mat_mult
    This function performs a matrix multiplication.
*/
void mat_mult( m_elem **a, m_elem **b, m_elem **c,
	      int a_rows, int a_cols, int b_cols )
{
  int i, j, k;
  m_elem  *a_ptr;
  m_elem  temp;

  for( i = 1; i <= a_rows; i++)
    {
      a_ptr = &a[i][0];
      for( j = 1; j <= b_cols; j++ )
	{
	  temp = 0.0;
	  for( k = 1; k <= a_cols; k++ )
	    temp = temp + (a_ptr[k] * b[k][j]);
	  c[i][j] = temp;
	}
    }
}
