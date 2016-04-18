#ifndef __MATRIX_H
#define __MATRIX_H

#define MAX_ROW 4
#define MAX_COL 4
typedef struct{
	short row, col; //store the actual size
	double mat[MAX_ROW][MAX_COL]; // to be simple, every matrix has the same size in memory; 
} matrix_t;

void matrix_multiply(matrix_t *A, matrix_t *B, matrix_t *C);
void matrix_add(matrix_t *A, matrix_t *B, matrix_t *C);
void matrix_multiply_scale(matrix_t *A, double k);
void matrix_transpose(matrix_t *A, matrix_t *B);
int matrix2_inverse(matrix_t *A, matrix_t *B);
void matrix_show(matrix_t *A); // one column vector will be shown as one row

#endif
