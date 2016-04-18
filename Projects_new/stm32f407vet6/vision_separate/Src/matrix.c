#include "main.h"
#include "matrix.h"

int matrix2_inverse(matrix_t *A, matrix_t *B){
	double det;
	if( (2 != A->row) || (2 != A->col) ){
		PRINTF("matrix_inverse error\r\n")
		while(1){
			#ifdef WATCH_DOG
			if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
			{
				Error_Handler();
			}
			#endif
		}
	}else{
		det = (A->mat[0][0])*(A->mat[1][1])-(A->mat[0][1])*(A->mat[1][0]);
		B->mat[0][0] = A->mat[1][1]/det;
		B->mat[1][1] = A->mat[0][0]/det;
		B->mat[0][1] = (-1.0f)*(A->mat[0][1])/det;
		B->mat[1][0] = (-1.0f)*(A->mat[1][0])/det;
		return 0;
	}
}

void matrix_multiply(matrix_t *A, matrix_t *B, matrix_t *C){
	int i, j, k;
	if( A->col != B->row ){
		PRINTF("matrix_multiply error\r\n")		
		while(1){
			#ifdef WATCH_DOG
			if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
			{
				Error_Handler();
			}
			#endif
		}
		return ;
	}else{
		C->row = A->row;
		C->col = B->col;
		for( i=0; i < C->row; i++ ){
			for( j=0; j < C->col; j++ ){
				C->mat[i][j] = 0;
				for( k=0; k < A->col; k++ ){
					C->mat[i][j] += (A->mat[i][k])*(B->mat[k][j]);
				}
			}
		}
		return ;
	}
}

void matrix_multiply_scale(matrix_t *C, double k){
	int i, j;
	for( i=0; i<C->row; i++ ){
		for( j=0; j<C->col; j++){
			C->mat[i][j] = C->mat[i][j]*k;
		}
	}
}

void matrix_add(matrix_t *A, matrix_t *B, matrix_t *C){
	int i, j;
	if( (A->col == B->col) && (A->row == B->row) ){
		C->row = A->row;
		C->col = A->col;
		for( i=0; i<C->row; i++ ){
			for( j=0; j<C->col; j++){
				C->mat[i][j] = A->mat[i][j] + B->mat[i][j];
			}
		}
		return;
	}else{
		PRINTF("matrix_add error\r\n")
		while(1);
		return;
	}
}

void matrix_transpose(matrix_t *A, matrix_t *B){
	int i, j;
	B->row = A->col;
	B->col = A->row;
	for( i=0; i<B->row; i++ ){
		for( j=0; j<B->col; j++ ){
			B->mat[i][j] = A->mat[j][i];
		}
	}
}

void matrix_show(matrix_t *A){
	int i, j;
	if( A->row <= 0 || A->col <= 0){
		return;
	}
	for( i=0; i<A->row; i++ ){
		for( j=0; j<A->col; j++){
			DISABLE_IRQ
			printf("%.12f\t", A->mat[i][j]);
			ENABLE_IRQ
		}
		if( 1 < A->col || i == (A->row - 1) ){ //more than one column or the last row
			printf("\r\n");
		}else{  // only one column and not the last row
			printf("\t");
		}
	}
}
