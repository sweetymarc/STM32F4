#include <stdio.h>
#include <stdlib.h>

#define MAX_ROW 4
#define MAX_COL 4
typedef struct{
	int row, col; //store the actual size
	double mat[MAX_ROW][MAX_COL]; // to be simple, every matrix has the same size in memory; 
} matrix_t;

float K_T, K_g = 9.8f, kp, kd;
matrix_t K_xe, K_F, K_G, K_u, K_K, K_H, K_Fxe, K_Gu, K_Hxe, K_err, K_Kerr;
matrix_t K_fb, K_y, K_ye, K_ye_old, K_derivative, K_pd; //K_y is the measurement value by camera, unit: m, not mm

int matrix_multiply(matrix_t *A, matrix_t *B, matrix_t *C);
int matrix_add(matrix_t *A, matrix_t *B, matrix_t *C);
void matrix_multiply_scale(matrix_t *C, double k);
void matrix_show(matrix_t *A);

char rtext[] = "%% PID parameter kp\r\np 1.4\r\n%% PID parameter kd\r\nd 0.9\r\n%% discrete time system period T\r\nT 0.016\r\n%% kalman filter gain K\r\nK 0.0 0.1 0.2 0.3\r\nK 0.0 0.1 0.2 0.3\r\nK 0.0 0.1 0.2 0.3\r\nK 0.0 0.1 0.2 0.3\r\n%% there must have a newline at the end of file\r\n";
int byteswritten, bytesread=sizeof(rtext); /* File write/read counts */
char *ptr0, *ptr1, *ptr_end;
//void kalman_filter(double t);

int main(int argc, char** argv){
	int i=0;
	float f1, f2;
	K_xe.row = 4; K_xe.col = 1;
	K_F.row = 4; K_F.col = 4;
	K_G.row = 4; K_G.col = 2;
	K_u.row = 2; K_u.col = 1;
	
	K_K.row = 4; K_K.col = 2;
	K_ye.row = 2; K_ye.col = 1;
	K_H.row = 2; K_H.col = 4;
	K_Fxe.row = 4; K_Fxe.col = 1;
	
	K_Gu.row = 4; K_Gu.col = 1;
	K_Hxe.row = 2; K_Hxe.col = 1;
	K_err.row = 2; K_err.col = 1;
	K_Kerr.row = 4; K_Kerr.col = 1;
	
	K_y.row = 2; K_y.col = 1;
	K_fb.row = 2; K_fb.col = 2;
	K_ye_old.row = 2; K_ye_old.col = 1;
	
	ptr0 = rtext;
	ptr_end = rtext + bytesread;
	printf(ptr0);
	while( ptr0 < ptr_end ){
		ptr1 = strchr((const char *)ptr0, '\n');
		if( ptr1 ){ // maybe '\n' is missing 
			*(ptr1 - 1) = '\0'; // '\r'
			*ptr1 = '\0';  // '\n'
		}
		switch( *ptr0 ){
			case '%' : // comments line, ignore it;
				break;
			case 'p' :
				sscanf(ptr0, "%*s%f", &kp);
				printf(ptr0);
				printf("kp set: %f\r\n", kp);
				break;
			case 'd' :
				sscanf(ptr0, "%*s%f", &kd);
				printf(ptr0);
				printf("kd set: %f\r\n", kd);
				break;
			case 'T' :
				sscanf(ptr0, "%*s%f", &K_T);
				printf(ptr0);
				printf("K_T set: %f\r\n", K_T);
				break;
			case 'K' :
				sscanf(ptr0, "%*s%f%f%*f%*f", &f1, &f2);
				K_K.mat[i][0] = (double)f1;
				K_K.mat[i][1] = (double)f2;
				printf(ptr0);
				printf("\r\n%f %f\r\n", f1, f2);
				if( 3 == i ){
					matrix_show(&K_K);
				}
				i++;
				break;
			default :
				printf("para.txt, format error");
		}
		if(ptr1){
			ptr0 = ptr1 + 1; // the first char of next line 
		}else{ // the '\n' is missed at the end of file
			break;
		}
	}
	K_xe.mat[0][0] = 0.1f;
	K_xe.mat[1][0] = 0.1f;
	K_xe.mat[2][0] = 1.0f;
	K_xe.mat[3][0] = 1.0f;
	printf("K_xe\r\n");
	matrix_show(&K_xe);
	
	K_y.mat[0][0] = 0.08f;
	K_y.mat[1][0] = 0.12f;
	K_ye_old.mat[0][0] = 0.099f;
	K_ye_old.mat[1][0] = 0.099f;
	
	K_u.mat[0][0] = -0.1f;
	K_u.mat[1][0] = 0.1f;
	
	K_F.mat[0][0] = 1.0f;
	K_F.mat[0][1] = 0.0f;
	K_F.mat[0][2] = K_T;
	K_F.mat[0][3] = 0.0f;
	
	K_F.mat[1][0] = 0.0f;
	K_F.mat[1][1] = 1.0f;
	K_F.mat[1][2] = 0.0f;
	K_F.mat[1][3] = K_T;
	
	K_F.mat[2][0] = 0.0f;
	K_F.mat[2][1] = 0.0f;
	K_F.mat[2][2] = 1.0f;
	K_F.mat[2][3] = 0.0f;
	
	K_F.mat[3][0] = 0.0f;
	K_F.mat[3][1] = 0.0f;
	K_F.mat[3][2] = 0.0f;
	K_F.mat[3][3] = 1.0f;
	printf("K_F\r\n");
	matrix_show(&K_F);
	
	K_G.mat[0][0] = 0.5f*K_g*K_T*K_T;
	K_G.mat[0][1] = 0.0f;
	K_G.mat[1][0] = 0.0f;
	K_G.mat[1][1] = -0.5f*K_g*K_T*K_T;
	K_G.mat[2][0] = K_g*K_T;
	K_G.mat[2][1] = 0.0f;
	K_G.mat[3][0] = 0.0f;
	K_G.mat[3][1] = -1.0f*K_g*K_T;
	printf("K_G\r\n");
	matrix_show(&K_G);
	
	K_H.mat[0][0] = 1.0f;
	K_H.mat[0][1] = 0.0f;
	K_H.mat[0][2] = 0.0f;
	K_H.mat[0][3] = 0.0f;
	
	K_H.mat[1][0] = 0.0f;
	K_H.mat[1][1] = 1.0f;
	K_H.mat[1][2] = 0.0f;
	K_H.mat[1][3] = 0.0f;
	printf("K_H\r\n");
	matrix_show(&K_H);
	//measurement gain is stable
	K_K.mat[0][0] = 0.0031f;
	K_K.mat[1][1] = 0.0031f;
	K_K.mat[2][0] = 0.0049f;
	K_K.mat[3][1] = 0.0049f;
	printf("K_K\r\n");
	matrix_show(&K_K);
	
	K_fb.mat[0][0] = -1.0f;
	K_fb.mat[1][1] = 1.0f;	
	printf("K_fb\r\n");
	matrix_show(&K_fb);
	
	
	matrix_multiply(&K_F, &K_xe, &K_Fxe);
	printf("K_Fxe\r\n");
	matrix_show(&K_Fxe);
	
	matrix_multiply(&K_G, &K_u, &K_Gu);
	printf("K_Gu\r\n");
	matrix_show(&K_Gu);
	
	matrix_add(&K_Fxe, &K_Gu, &K_xe);
	printf("K_xe\r\n");
	matrix_show(&K_xe);
	
	matrix_multiply(&K_H, &K_xe, &K_Hxe);
	printf("K_Hxe\r\n");
	matrix_show(&K_Hxe);
	
	matrix_multiply_scale(&K_Hxe, -1.0f);
	matrix_add(&K_y, &K_Hxe, &K_err);
	printf("K_err\r\n");
	matrix_show(&K_err);
	
	matrix_multiply(&K_K, &K_err, &K_Kerr);
	printf("K_Kerr\r\n");
	matrix_show(&K_Kerr);
	
	matrix_add(&K_xe, &K_Kerr, &K_xe);
	printf("K_xe\r\n");
	matrix_show(&K_xe);
	
	matrix_multiply(&K_H, &K_xe, &K_ye);
	printf("K_ye\r\n");
	matrix_show(&K_ye);
	
	matrix_multiply_scale(&K_ye_old, -1.0f);
	matrix_add(&K_ye, &K_ye_old, &K_derivative);
	//save current ye
	K_ye_old.mat[0][0] = K_ye.mat[0][0];
	K_ye_old.mat[1][0] = K_ye.mat[1][0];
	
	matrix_multiply_scale(&K_derivative, 1.0f/K_T);
	printf("K_derivative\r\n");
	matrix_show(&K_derivative);
	matrix_multiply_scale(&K_derivative, kd);
	matrix_add(&K_ye, &K_derivative, &K_pd);
	matrix_multiply_scale(&K_pd, kp);
	printf("K_pd\r\n");
	matrix_show(&K_pd);
	matrix_multiply(&K_fb, &K_pd, &K_u);	
	printf("K_u\r\n");
	matrix_show(&K_u);
}
int matrix2_inverse(matrix_t *A, matrix_t *B){
	double det;
	if( (2 != A->row) || (2 != A->col) ){
		return -1;
	}else{
		det = (A->mat[0][0])*(A->mat[1][1])-(A->mat[0][1])*(A->mat[1][0]);
		B->mat[0][0] = A->mat[1][1]/det;
		B->mat[1][1] = A->mat[0][0]/det;
		B->mat[0][1] = (-1.0f)*(A->mat[0][1])/det;
		B->mat[1][0] = (-1.0f)*(A->mat[1][0])/det;
		return 0;
	}
}
void kalman_filter(double t){

}
int matrix_multiply(matrix_t *A, matrix_t *B, matrix_t *C){
	int i, j, k;
	if( A->col != B->row ){
		return -1;
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
		return 0;
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

int matrix_add(matrix_t *A, matrix_t *B, matrix_t *C){
	int i, j;
	if( (A->col == B->col) && (A->row == B->row) ){
		C->row = A->row;
		C->col = A->col;
		for( i=0; i<C->row; i++ ){
			for( j=0; j<C->col; j++){
				C->mat[i][j] = A->mat[i][j] + B->mat[i][j];
			}
		}
		return 0;
	}else{
		return -1;
	}
}
void matrix_show(matrix_t *A){
	int i, j;
	if( A->row <= 0 || A->col <= 0){
		return;
	}
	for( i=0; i<A->row; i++ ){
		for( j=0; j<A->col; j++){
			printf("%.12f\t", A->mat[i][j]);
		}
		printf("\r\n");
	}
}
