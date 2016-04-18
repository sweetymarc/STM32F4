#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
// the object should be big enough, for (100, 300), my earphone case can be find easily, but my key often lost, u-disk just can be found
//f=h*d/(1-h) h=0.005155
// ratio: +/-   stardard coordinate: left is x+, forward is y+, up is z+; all position and velocity have been united;
vision_output_t v_out;
extern double real_roll, real_pitch;
extern IWDG_HandleTypeDef IwdgHandle;
extern void mavlink_parse(void);

linearSystem_t ls;
camera_parameter_t cp;
PID_t pid;
vision_t vision;

void vision_control_init(){
	cp.r0 = 120; cp.c0 = 160;
	cp.s = 12; //um // 
	cp.f = 1800; //um /*3752*/
	ls.d_limit = 2.0f;
	vision.lost_counter = vision.LOST_LIMIT;
	EXTILine1_Config();
	getParaFromSD();
	matrix_assign_initial_value(&ls, &pid, &vision);
	bin_DMA[0] = 0xAAAAAAAA;
}

void vision_control(){
	int i,j;
	uint32_t startTime;
	uint32_t *pBin;
	ls.dT = ls.K_T * (frame_count - vision.last_frame_count);
	
	mavlink_parse();
	startTime = HAL_GetTick();
	gray2bin(&(gray[0][0][0]), IMAGE_SIZE, vision.threshold, (vision.pBin0));
	//PRINTF("gray2bin\r\n");
	// assume not found
	vision.object_found = false; // find nothing
	vision.lost_counter ++;
	if( circle_search(bin, IMAGE_HEIGHT, IMAGE_WIDTH, vision.sd_threshold, &vision.pos_r, &vision.pos_c ) ){
		//mark
		for(i=vision.pos_r-5; i<=vision.pos_r+5; i++){
			pBin = vision.pBin0 + IMAGE_WIDTH * i;
			for(j=vision.pos_c-5; j<=vision.pos_c+5; j++){
				*(pBin + j) = 0x01;
			}
		}
		//PRINTF("circle found\r\n");
		position_calc(vision.pos_r, vision.pos_c, h_average, &cp, &vision.x, &vision.y);
		//PRINTF("position calc\r\n");
		lean_compensate(&vision, &real_roll, &real_pitch, 0);
		//PRINTF("lean\r\n");
		ls.K_y.mat[0][0] = vision.p_compensated.mat[0][0]*-1.0f;
		ls.K_y.mat[1][0] = vision.p_compensated.mat[1][0]*-1.0f;
		v_out.x_measure = ls.K_y.mat[0][0];
		v_out.y_measure = ls.K_y.mat[1][0];
		if( 0 == vision.lost_counter ){ // for safety, when two contiuous found check the velocity, at the begining or find again after some lost, ignore the velocity check
			if( 0 != ls.dT ){
				ls.d_x = fabs((ls.K_y.mat[0][0]-(ls.K_xe).mat[0][0]))/(ls.dT);
				ls.d_y = fabs((ls.K_y.mat[1][0]-(ls.K_xe).mat[1][0]))/(ls.dT);
				if( ls.d_x < ls.d_limit && ls.d_y < ls.d_limit ){ // find the right object
					vision.object_found = true;
					vision.lost_counter = 0;
				}
			}
		}else{ // after lost, the first time
			vision.object_found = true;
			vision.lost_counter = 0;
		}
	}
	v_out.searchCost = HAL_GetTick() - startTime;
	if( (1800 > rc_ch && vision.object_found) || (1800 < rc_ch && !vision.object_found) ){
		HAL_GPIO_WritePin( VISION_BEEP_PORT, VISION_BEEP_PIN, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin( VISION_BEEP_PORT, VISION_BEEP_PIN, GPIO_PIN_RESET);
	}
	// future work : divide to 2 step
/* if( vision.lost_counter < vision.LOST_LIMIT ){
		time_update();
		if( vision.object_found ){
		measurement_update();
	}else{
		reset filter
	}
	*/
	if( vision.object_found || vision.lost_counter < vision.LOST_LIMIT ){
		ls.K_u_real.mat[0][0] = real_roll;
		ls.K_u_real.mat[1][0] = -1.0f*real_pitch;
		kalman_filter(&ls, &ls.K_u_real, &ls.K_y, vision.object_found);
		ls.dT_counter += (frame_count - vision.last_frame_count);
		//PRINTF("kalman\r\n");
		if( 1800 < rc_ch ){
			PID_control(&pid, &ls.K_xe, &ls.K_u, &ls.dT);
			//PRINTF("pid\r\n");
			mavlink_rc(ls.K_u.mat[0][0], -1.0f*ls.K_u.mat[1][0], false);
			//PRINTF("pwm\r\n");
		}
	}else{
		ls.dT_counter = 0; // reset the system
		(ls.K_xe).mat[0][0] = 0.0f;
		(ls.K_xe).mat[1][0] = 0.0f;
		(ls.K_xe).mat[2][0] = 0.0f;
		(ls.K_xe).mat[3][0] = 0.0f;
		(ls.K_P).mat[0][0] = ls.c0; (ls.K_P).mat[0][1] = 0; (ls.K_P).mat[0][2] = 0; (ls.K_P).mat[0][3] = 0; 
		(ls.K_P).mat[1][0] = 0.0f; (ls.K_P).mat[1][1] = ls.c0; (ls.K_P).mat[1][2] = 0; (ls.K_P).mat[1][3] = 0; 
		(ls.K_P).mat[2][0] = 0.0f; (ls.K_P).mat[2][1] = 0; (ls.K_P).mat[2][2] = ls.cv0; (ls.K_P).mat[2][3] = 0; 
		(ls.K_P).mat[3][0] = 0.0f; (ls.K_P).mat[3][1] = 0; (ls.K_P).mat[3][2] = 0; (ls.K_P).mat[3][3] = ls.cv0; 
		
		pid.K_err_old.mat[0][0] = 0.0f;
		pid.K_err_old.mat[1][0] = 0.0f;
		vision.lost_counter = vision.LOST_LIMIT;
		pid.K_integrate.mat[0][0] = 0;
		pid.K_integrate.mat[1][0] = 0;
		mavlink_rc(0, 0, true);
		//PRINTF("pwm\r\n");
	}
	vision.last_frame_count = frame_count;

	v_out.height = h_average;
	v_out.rawHeight = sonar_height[h_i];
	v_out.wholeCost = HAL_GetTick() - startTime;
	v_out.count = frame_count;
	v_out.roll = real_roll; 
	v_out.pitch = real_pitch;
	v_out.x_est = (ls.K_xe).mat[0][0];
	v_out.y_est = (ls.K_xe).mat[1][0];
	v_out.vx_est = (ls.K_xe).mat[2][0];
	v_out.vy_est = (ls.K_xe).mat[3][0];
}

void gray2bin(uint8_t *_gray, int _size, uint8_t _threshold, uint32_t *_bin){
	const uint8_t *grayEnd = _gray + _size;
	
	for( ; _gray < grayEnd; _gray++, _bin++ ){
		if( *_gray < _threshold ){
			*_bin = 0x00;
		}else{
			*_bin = 0x01;
		}
	}
}

static uint16_t change[JUMP_SIZE], length[JUMP_SIZE], row[DIAMETER_COUNTER_SIZE], col[DIAMETER_COUNTER_SIZE];
int  diameterRow, diameterCol;
bool circle_search(const uint32_t *_bin, int _height, int _width, float sd_threshold, int *_pos_r, int *_pos_c){
	int i, j, k, m, length_change, lost_counter, find_counter, dLength;
	int sum_r, sum_c;
	int diameter_counter;
	static int r_row, c_row, r_col, c_col;
	const uint32_t *pBin0, *pBinStart, *pBin, *pBinNext, *pBinEnd;
	uint8_t *pBinByte;
	float sd, sd_threshold2 = vision.sd_threshold * 1.5;
	bool  diameter_found, diameter_start, diameter_end, row_ok, col_ok;
	
	pBin0 = (uint32_t *)(0x22000000 + ( (uint32_t)bin - 0x20000000 ) * 32);
	// row search
	diameter_counter = 0;
	find_counter = 0;
	lost_counter = 0;
	diameter_start = false;
	diameter_end = false;
	row_ok = false;
	for( i=10; i<_height;){ // every row
		k = 0;
		pBin = pBin0 + _width * i;
		pBinByte = (uint8_t *)_bin + (_width/8)*i;
		//1. detect jump(0->1 or 1->0), store the index between them in change
		for( m = 0; m < _width; ){
			if( 0 == m%8 ){ // fast search
				if( 0xff == *pBinByte || 0x00 == *pBinByte ){
					m += 7;
					pBin += 7;
				}
				pBinByte ++;
			}
			m++;
			pBinNext = pBin + 1;
			if( *pBin != *(pBinNext) ){
				change[k++] = m;
			}
			pBin = pBinNext;
			if( JUMP_SIZE - 1 == k ){
				PRINTF("JUMP_SIZE error\r\n");
				break;
			}
		}
		length_change = k;
		if( 10 > length_change ){//at least, 12 changes
			i += 2;
			continue;
		}
		/*DISABLE_IRQ
		printf("jump: %d\r\n", length_change);
		DISABLE_IRQ*/
		diameter_found = false; 
		//2. calc the length of every 000... or 1111... between two jump. 
		length[0] = change[0];
		for(j=1; j<length_change; j++){
			if( JUMP_SIZE == j ){
				PRINTF("length size samll\r\n");
				break;
			}
			length[j] = change[j] - change[j-1];
		}
		length[length_change] = _width - change[length_change-1];
		//3. search, if white(1) length is the same, that is I want.
		pBin = pBin0 + _width * i;
		for( j=1; j<(length_change+1)-9; j++ ){  //length size : length_change + 1
			//the first block should be black
			if( *(pBin + change[j-1]) ){
				//PRINTF("NOT BLACK")
				continue;
			}
			//compare the 5 black blocks
			sd = calc_sd_even( &length[j], 2 );	
			if(  sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+2], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+4], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+6], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			//compare the 4 white blocks	
			sd = calc_sd_even( &length[j+1], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}
			sd = calc_sd_even( &length[j+3], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}
			sd = calc_sd_even( &length[j+5], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}
			dLength = (change[j+8] - change[j-1]) - (length[j] + length[j+8])/2;
			diameterRow = (dLength * cp.s * h_average) / cp.f;
			v_out.diameterRow = diameterRow;
			if( abs(diameterRow-cp.realDiameter)*100 / cp.realDiameter > cp.diameterThreshold ){
				continue;
			}
			diameter_found = true;
			break; // only one diameter in one line, so break;
		}
		if( diameter_found ){
			lost_counter = 0;
			find_counter ++;
			row[diameter_counter] = i;
			col[diameter_counter] = (change[j+3]+change[j+4])/2;
			diameter_counter ++;
			if( DIAMETER_COUNTER_SIZE == diameter_counter ){
				PRINTF("DIAMETER_COUNTER_SIZE\r\n");
				diameter_end = true;
			}
		}else{
			lost_counter ++;
			find_counter = 0;
			if( !diameter_start ){
				diameter_counter = 0;
			}
		}
		if( !diameter_start && find_counter >= START_COUNTER ){
			diameter_start = true;
		}
		if( diameter_start && lost_counter > END_COUNTER ){
			diameter_end = true;
		}
		if( diameter_start && !diameter_end && diameter_found ){
		}
		if( diameter_end ){
			sum_r = 0; sum_c = 0;
			for(k=0; k < diameter_counter; k++){
				sum_r += row[k];
				sum_c += col[k];
			}
			r_row = sum_r / (diameter_counter);
			c_row = sum_c / (diameter_counter);
			row_ok = true;
			break;
		}
		if( diameter_found || diameter_start ){
			i += 1;
		}else{// go quickly
			i += 2;
		}
	}
	//column search
	diameter_counter = 0;
	find_counter = 0;
	lost_counter = 0;
	diameter_start = false;
	diameter_end = false;
	col_ok = false;
	for( i=10; i<_width;){ // i is colum, j is row
		k = 0;
		pBin = pBin0 + i;
		//1. detect jump(0->1 or 1->0), store the index between them in change
		for( m = 1; m < _height; m++){
			pBinNext = pBin + _width;
			if( *pBin != *pBinNext ){
				change[k++] = m;
			}
			pBin = pBinNext;
			if( JUMP_SIZE - 1 == k ){ // to avoid length overflow, so minus 1;
				PRINTF("JUMP_SIZE error\r\n");
				break;
			}
		}
		length_change = k;	
		if( 10 > length_change ){//at least, 10 changes
			i += 2;
			continue;
		}
		/*DISABLE_IRQ
		printf("jump: %d\r\n", length_change);
		DISABLE_IRQ*/
		diameter_found = false;
		//2. calc the length of every 000... or 1111... between two jump. 
		length[0] = change[0];
		for(j=1; j<length_change; j++){
			if( JUMP_SIZE == j ){
				PRINTF("length size samll\r\n");
				break;
			}
			length[j] = change[j] - change[j-1];
		}
		length[length_change] = _height - change[length_change - 1];
		//3. search, if two continuous length is the same, that is I want.
		pBin = pBin0 + i;
		for( j=1; j<(length_change+1)-9; j++ ){  //length size : length_change + 1
			//the first block should be black
			if( *(pBin + _width * change[j-1]) ){
				//PRINTF("NOT white")
				continue;
			}
			//compare the 5 black blocks
			sd = calc_sd_even( &length[j], 2 );	
			if(  sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+2], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+4], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			sd = calc_sd_even( &length[j+6], 2 );
			if( sd > sd_threshold ){
				continue;
			}
			//compare the 4 white blocks				
			if( 0x01 != *(pBin + _width * change[j]) ){
				//PRINTF("NOT white")
				continue;
			}
			sd = calc_sd_even( &length[j+1], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}
			sd = calc_sd_even( &length[j+3], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}
			sd = calc_sd_even( &length[j+5], 2 );
			if( sd > sd_threshold2 ){
				continue;
			}			
			dLength = (change[j+8] - change[j-1]) - (length[j] + length[j+8])/2;
			diameterCol = (dLength * cp.s * h_average) / cp.f;
			v_out.diameterCol = diameterCol;
			if( abs(diameterCol-cp.realDiameter)*100 / cp.realDiameter > cp.diameterThreshold ){
					continue;
			}
			diameter_found = true;
			break; // only one diameter in one line, so break;
		}
		if( diameter_found ){
			lost_counter = 0;
			find_counter ++;
			row[diameter_counter] = (change[j+3]+change[j+4])/2;
			col[diameter_counter] = i;
			diameter_counter ++;
			if( DIAMETER_COUNTER_SIZE == diameter_counter ){
				PRINTF("DIAMETER_COUNTER_SIZE\r\n");
				diameter_end = true;
			}
		}else{
			lost_counter ++;
			find_counter = 0;
			if( !diameter_start ){
				diameter_counter = 0;
			}
		}
		if( !diameter_start && find_counter >= START_COUNTER ){
			diameter_start = true;
		}
		if( diameter_start && lost_counter > END_COUNTER ){
			diameter_end = true;
		}
		if( diameter_end ){
			sum_r = 0; sum_c = 0;
			for(k=0; k < diameter_counter; k++){
				sum_r += row[k];
				sum_c += col[k];
			}
			r_col = sum_r / diameter_counter;
			c_col = sum_c / diameter_counter;
			col_ok = true;
			break;
		}
		if( diameter_found || diameter_start ){
			i += 1;
		}else{// go quickly
			i += 2;
		}
	}
	if( !row_ok && !col_ok ){
		return false;
	}else	if(row_ok && !col_ok){
		*_pos_r = r_row;
		*_pos_c = c_row;
		return true;
	}else if( !row_ok && col_ok ){
		*_pos_r = r_col;
		*_pos_c = c_col;
		return true;
	}else if( row_ok && col_ok ){
		/*if( vision.measure_valid ){ // choose the better
			if( abs(r_row - *_pos_r) < abs(r_col - *_pos_r) ){
				*_pos_r = r_row;
			}else{
				*_pos_r = r_col;
			}
			if( abs(c_row - *_pos_c) < abs(c_col - *_pos_c) ){
				*_pos_c = c_row;
			}else{
				*_pos_c = c_col;
			}
		}else{*/
		*_pos_r = r_row;
		*_pos_c = c_col;
		return true;
	}
}

void position_calc(uint16_t _pos_r, uint16_t _pos_c, uint16_t _height, camera_parameter_t *cp, int *x, int *y){
	/*x_sensor = (cp->kx*(_pos_c-160))/1000;  //um  1000
	y_sensor = (cp->ky*(120-_pos_r))/1000;  //um
	*x = ((x_sensor*((cp->f + _height*1000)/(cp->f)))/1000); // three 1000 vanished, so f is in um
	*y = ((y_sensor*((cp->f + _height*1000)/(cp->f)))/1000); // change um to mm,*/
	*x = (_height * ( _pos_c - cp->c0 ) * cp->s) / cp->f;
	*y = (_height * ( cp->r0 - _pos_r ) * cp->s) / cp->f;
}
/*
void low_pass_filter(){
	static int x_real[VISION_FILTER], y_real[VISION_FILTER];// mm, the position of the camera relative to the object. 
	static int x_i=0, y_i=0, d_x, d_y, d_limit=1000, camera_rotation=0, height_offset=50;
	d_x = abs(x_real[x_i] - *x_average) / dT;
	d_y = abs(y_real[y_i] - *y_average) / dT;
	if( search_count >= VISION_FILTER && (d_x > d_limit || d_y > d_limit) ){ // move too fast, must be error
		//object_found = false;
		return false;
	}else{
		x_i ++;
		y_i ++;
		x_i %= VISION_FILTER;
		y_i %= VISION_FILTER;
		search_count++;
		x_average = 0;
		y_average = 0;
		for(i_sum=0; i_sum<VISION_FILTER; i_sum++){
			*x_average += x_real[i_sum];
			*y_average += y_real[i_sum];
		}
		if( VISION_FILTER > search_count ){ // at the begining
			*x_average /= search_count; //mm, 
			*y_average /= search_count; 
		}else{
			*x_average /= VISION_FILTER; //mm, 
			*y_average /= VISION_FILTER; 
		}
	}
}*/

static	matrix_t K_tmp1;
void lean_compensate(vision_t *v, double *_roll, double *_pitch, double *_yaw){
	double tmp1, tmp2;
	vision.p_camera.mat[0][0] = vision.x/1000.0f; // right is +
	vision.p_camera.mat[1][0] = vision.y/1000.0f; // forward is +
	vision.p_camera.mat[2][0] = (-1.0f)*(h_average + cp.height_offset)/1000.0f; // up is +
	//R_z.col = 3; R_z.row = 3;
	/*
	tmp1 = cos(camera_rotation/180.0f);
	tmp2 = sin(camera_rotation/180.0f);
	R_z.mat[0][0] = tmp1;	R_z.mat[0][1] = -1.0f*tmp2;	R_z.mat[0][2] = 0.0f;	
	R_z.mat[1][0] = tmp2;	R_z.mat[1][1] = tmp1;	R_z.mat[1][2] = 0.0f;	
	R_z.mat[2][0] = 0.0f;	R_z.mat[2][1] = 0.0f;	R_z.mat[2][2] = 1.0f; */
	tmp1 = cos(*_pitch);
	tmp2 = sin(*_pitch);
	v->R_x.mat[0][0] = 1.0f;	v->R_x.mat[0][1] = 0.0f;	v->R_x.mat[0][2] = 0.0f;
	v->R_x.mat[1][0] = 0.0f;	v->R_x.mat[1][1] = tmp1;	v->R_x.mat[1][2] = -1.0f*tmp2;
	v->R_x.mat[2][0] = 0.0f;	v->R_x.mat[2][1] = tmp2;	v->R_x.mat[2][2] = tmp1;
	tmp1 = cos(*_roll);
	tmp2 = sin(*_roll);
	v->R_y.mat[0][0] = tmp1;	v->R_y.mat[0][1] = 0.0f;	v->R_y.mat[0][2] = tmp2;	
	v->R_y.mat[1][0] = 0.0f;	v->R_y.mat[1][1] = 1.0f;	v->R_y.mat[1][2] = 0.0f;	
	v->R_y.mat[2][0] = -1.0*tmp2;	v->R_y.mat[2][1] = 0.0f;	v->R_y.mat[2][2] = tmp1;	
	
	//matrix_multiply(&R_z, &p_camera, &p_compensated); //yaw
	matrix_multiply(&v->R_x, &v->p_camera, &K_tmp1); //pitch
	matrix_multiply(&v->R_y, &K_tmp1, &v->p_compensated); //roll
}

static matrix_t K_tmp2, K_tmp3;
void kalman_filter(linearSystem_t *ls, matrix_t *K_u_real, matrix_t *K_y, bool _object_found){ //x: left;  y: front;
	(ls->K_F).mat[0][2] = ls->dT;
	(ls->K_F).mat[1][3] = ls->dT;
	
	(ls->K_G).mat[0][0] = 0.5f*(ls->K_g)*(ls->dT)*(ls->dT); 	(ls->K_G).mat[0][1] = 0.0f;
	(ls->K_G).mat[1][0] = 0.0f;																(ls->K_G).mat[1][1] = 0.5f*(ls->K_g)*(ls->dT)*(ls->dT);
	(ls->K_G).mat[2][0] = (ls->K_g)*ls->dT;										(ls->K_G).mat[2][1] = 0.0f;
	(ls->K_G).mat[3][0] = 0.0f;																(ls->K_G).mat[3][1] = 1.0f*(ls->K_g)*ls->dT;
	
	if( _object_found ){ // it's used to store circle search result
		if( 0 != ls->dT_counter  ){// covariance of measurement
			ls->vx = ls->v0 + ls->k_velocity*ls->d_x + ls->k_lean*fabs(real_roll) + ls->k_height*abs(h_average);
			ls->vy = ls->v0 + ls->k_velocity*ls->d_y + ls->k_lean*fabs(real_pitch) + ls->k_height*abs(h_average);
		}else{ // the first time, have to trust the measure value;
			ls->vx = 0.0f;
			ls->vy = 0.0f;
		}
	}
	if( _object_found ){
		(ls->K_R).mat[0][0] = pow(ls->vx, 2.0f);
		(ls->K_R).mat[1][1] = pow(ls->vy, 2.0f);
		
		matrix_multiply( &(ls->K_F), &(ls->K_P), &K_tmp1);
		matrix_multiply( &K_tmp1, &ls->K_FT, &(ls->K_P) );
		matrix_multiply(&(ls->K_G), &(ls->K_Q), &K_tmp1);
		matrix_transpose(&(ls->K_G), &K_tmp2);
		matrix_multiply(&K_tmp1, &K_tmp2, &K_tmp3);
		matrix_add( &(ls->K_P), &K_tmp3, &(ls->K_P));
		
		matrix_multiply(&(ls->K_H), &(ls->K_P), &K_tmp1);
		matrix_multiply(&K_tmp1, &ls->K_HT, &K_tmp2);
		matrix_add(&K_tmp2, &(ls->K_R), &K_tmp1);
		matrix2_inverse(&K_tmp1, &K_tmp2);
		matrix_multiply(&ls->K_HT, &K_tmp2, &K_tmp1);
		matrix_multiply(&(ls->K_P), &K_tmp1, &(ls->K_K));
	}
	// time update
	matrix_multiply(&(ls->K_F), &(ls->K_xe), &K_tmp1);
	matrix_multiply(&(ls->K_G), K_u_real, &K_tmp2);
	matrix_add(&K_tmp1, &K_tmp2, &(ls->K_xe));
	v_out.x_sim = (ls->K_xe).mat[0][0];
	v_out.y_sim = (ls->K_xe).mat[1][0];
	
	#ifdef DEBUG_KALMAN_FILTER
	DISABLE_IRQ
	printf("K_Fxe\r\n");
	matrix_show(&K_Fxe);
	printf("K_Gu\r\n");
	matrix_show(&K_Gu);
	ENABLE_IRQ
	#endif
	if( _object_found ){// it's used to store circle search result
		matrix_multiply(&(ls->K_H), &(ls->K_xe), &K_tmp1);
		matrix_multiply_scale(&K_tmp1, -1.0f);
		matrix_add(K_y, &K_tmp1, &K_tmp2);
		matrix_multiply(&(ls->K_K), &K_tmp2, &K_tmp1);
		matrix_add(&(ls->K_xe), &K_tmp1, &(ls->K_xe));
	}
	
	#ifdef DEBUG_KALMAN_FILTER
	DISABLE_IRQ
	printf("K_err\r\n");
	matrix_show(&K_err);
	printf("K_Kerr\r\n");
	matrix_show(&K_Kerr);
	printf("K_xe\r\n");
	matrix_show(&(ls->K_xe));
	ENABLE_IRQ
	#endif
	
	
	if( _object_found ){
		matrix_multiply(&(ls->K_K), &(ls->K_H), &K_tmp1);
		matrix_multiply_scale(&K_tmp1, -1.0f);
		matrix_add(&(ls->K_I4), &K_tmp1, &K_tmp2);
		matrix_multiply(&K_tmp2, &(ls->K_P), &K_tmp1);
		matrix_transpose(&K_tmp2, &K_tmp3);
		matrix_multiply(&K_tmp1, &K_tmp3, &(ls->K_P)); // K_tmp1 - K_tmp3 is released, can't be reused;
		matrix_multiply(&(ls->K_K), &(ls->K_R), &K_tmp1);
		matrix_transpose(&(ls->K_K), &K_tmp2);
		matrix_multiply(&K_tmp1, &K_tmp2, &K_tmp3);
		matrix_add(&(ls->K_P), &K_tmp3, &(ls->K_P));
	}
}
void PID_control(PID_t *_pid, matrix_t *K_x, matrix_t *K_u, double *_dT){
	/*
	matrix_multiply_scale(&K_err_old, -1.0f);
	matrix_add(&K_err, &K_err_old, &K_derivative);
	//save current ye
	K_err_old.mat[0][0] = K_err.mat[0][0];
	K_err_old.mat[1][0] = K_err.mat[1][0];
	matrix_multiply_scale(&K_derivative, 1.0f*dT); 
 */
	//P
	_pid->K_err.mat[0][0] = -1.0f * K_x->mat[0][0];
	_pid->K_err.mat[1][0] = -1.0f * K_x->mat[1][0];
	K_tmp1.col = _pid->K_err.col; K_tmp1.row = _pid->K_err.row;
	K_tmp1.mat[0][0] = _pid->K_err.mat[0][0];
	K_tmp1.mat[1][0] = _pid->K_err.mat[1][0];
	matrix_multiply_scale(&_pid->K_err, _pid->kp);
	//I
	matrix_multiply_scale(&K_tmp1, *_dT);
	matrix_add(&_pid->K_integrate, &K_tmp1, &_pid->K_integrate);
	if( fabs(_pid->K_integrate.mat[0][0]) > _pid->i_limit ){
		if( _pid->K_integrate.mat[0][0] > 0 ){
			_pid->K_integrate.mat[0][0] = _pid->i_limit;
		}else{
			_pid->K_integrate.mat[0][0] = -1*(_pid->i_limit);
		}
	}
	if( fabs(_pid->K_integrate.mat[1][0]) > _pid->i_limit ){
		if( _pid->K_integrate.mat[1][0] > 0 ){
			_pid->K_integrate.mat[1][0] = _pid->i_limit;
		}else{
			_pid->K_integrate.mat[1][0] = -1*(_pid->i_limit);
		}
	}
	K_tmp1.col = _pid->K_integrate.col; K_tmp1.row = _pid->K_integrate.row;
	K_tmp1.mat[0][0] = _pid->K_integrate.mat[0][0];
	K_tmp1.mat[1][0] = _pid->K_integrate.mat[1][0];
	matrix_multiply_scale(&K_tmp1, _pid->ki);
	K_tmp1.mat[1][0] *= _pid->ratio_yx;
	//D
	_pid->K_derivative.mat[0][0] = -1.0f * K_x->mat[2][0];
	_pid->K_derivative.mat[1][0] = -1.0f * K_x->mat[3][0];
	//if the velocity is too small, it must be disturbance
	if( fabs(_pid->K_derivative.mat[0][0]) < _pid->v_limit_down ){ // too small 
		_pid->K_derivative.mat[0][0] = 0.0f;
	}else if( fabs(_pid->K_derivative.mat[0][0]) > _pid->v_limit_up ) { // too big
		if( _pid->K_derivative.mat[0][0] > 0 ){
			_pid->K_derivative.mat[0][0] = _pid->v_limit_up;
		}else if( _pid->K_derivative.mat[0][0] < 0 ){
			_pid->K_derivative.mat[0][0] = -1 * (_pid->v_limit_up);
		}
	}
	if( fabs(_pid->K_derivative.mat[1][0]) < _pid->v_limit_down ){ // too small 
		_pid->K_derivative.mat[1][0] = 0.0f;
	}else if( fabs(_pid->K_derivative.mat[1][0]) > _pid->v_limit_up ) { // too big
		if( _pid->K_derivative.mat[1][0] > 0 ){
			_pid->K_derivative.mat[1][0] = _pid->v_limit_up;
		}else if( _pid->K_derivative.mat[1][0] < 0 ){
			_pid->K_derivative.mat[1][0] = -1 * (_pid->v_limit_up);
		}
	}
	matrix_multiply_scale(&_pid->K_derivative, _pid->kd);
	matrix_add(&_pid->K_err, &_pid->K_derivative, K_u); // P + D
	K_u->mat[1][0] *= _pid->ratio_yx;
	if( K_u->mat[0][0] < 0 ){
		K_u->mat[0][0] *= _pid->ratio_x;
	}
	if( K_u->mat[1][0] < 0 ){
		K_u->mat[1][0] *= _pid->ratio_y;
	}
	/*DISABLE_IRQ
	printf("K_u: %f %f\r\n", K_u->mat[0][0], K_u->mat[1][0]);
	ENABLE_IRQ*/
	if( fabs(K_u->mat[0][0]) > _pid->u_limit ){
		if( K_u->mat[0][0] > 0 ){
			K_u->mat[0][0] = _pid->u_limit;
		}else{
			K_u->mat[0][0] = -1*_pid->u_limit;
		}
	}
	if( fabs(K_u->mat[1][0]) > _pid->u_limit ){
		if( K_u->mat[1][0] > 0 ){
			K_u->mat[1][0] = _pid->u_limit;
		}else{
			K_u->mat[1][0] = -1*_pid->u_limit;
		}
	}
	matrix_add(K_u, &K_tmp1, K_u); // + I
	v_out.ux = K_u->mat[0][0];
	v_out.uy = K_u->mat[1][0];
}

/**    ,..
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTILine1_Config(void){
  GPIO_InitTypeDef   GPIO_InitStructure;
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	EXTI->FTSR &= ~((uint32_t)0x01<<1);
	
  HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void matrix_assign_initial_value(linearSystem_t *ls, PID_t *_pid, vision_t *v){
	ls->K_g = 9.8f;
	(ls->K_G).row = 4; (ls->K_G).col = 2;
	(ls->K_R).row = 2; (ls->K_R).col = 2;
	(ls->K_K).row = 4; (ls->K_K).col = 2;
	
	(ls->K_F).row = 4; (ls->K_F).col = 4;
	(ls->K_F).mat[0][0] = 1.0f;	(ls->K_F).mat[0][1] = 0.0f;	(ls->K_F).mat[0][2] = 0;	(ls->K_F).mat[0][3] = 0.0f;	
	(ls->K_F).mat[1][0] = 0.0f;	(ls->K_F).mat[1][1] = 1.0f;	(ls->K_F).mat[1][2] = 0.0f;	(ls->K_F).mat[1][3] = 0;
	(ls->K_F).mat[2][0] = 0.0f;	(ls->K_F).mat[2][1] = 0.0f;	(ls->K_F).mat[2][2] = 1.0f;	(ls->K_F).mat[2][3] = 0.0f;	
	(ls->K_F).mat[3][0] = 0.0f;	(ls->K_F).mat[3][1] = 0.0f;	(ls->K_F).mat[3][2] = 0.0f;	(ls->K_F).mat[3][3] = 1.0f;
	matrix_transpose(&(ls->K_F), &ls->K_FT);
	
	(ls->K_xe).row = 4; (ls->K_xe).col = 1;
	(ls->K_xe).mat[0][0] = 0.0f;
	(ls->K_xe).mat[1][0] = 0.0f;
	(ls->K_xe).mat[2][0] = 0.0f;
	(ls->K_xe).mat[3][0] = 0.0f;
	printf("K_xe\r\n");
	matrix_show(&(ls->K_xe));
	HAL_Delay(10);
	
	(ls->K_H).row = 2; (ls->K_H).col = 4;
	(ls->K_H).mat[0][0] = 1.0f;	(ls->K_H).mat[0][1] = 0.0f;	(ls->K_H).mat[0][2] = 0.0f;	(ls->K_H).mat[0][3] = 0.0f;
	(ls->K_H).mat[1][0] = 0.0f;	(ls->K_H).mat[1][1] = 1.0f;	(ls->K_H).mat[1][2] = 0.0f;	(ls->K_H).mat[1][3] = 0.0f;
	matrix_transpose(&(ls->K_H), &ls->K_HT);
	printf("K_H\r\n");
	matrix_show(&(ls->K_H));
	HAL_Delay(10);
	
	(ls->K_I4).row = 4; (ls->K_I4).col = 4;
	(ls->K_I4).mat[0][0] = 1; (ls->K_I4).mat[0][1] = 0; (ls->K_I4).mat[0][2] = 0; (ls->K_I4).mat[0][3] = 0;
	(ls->K_I4).mat[1][0] = 0; (ls->K_I4).mat[1][1] = 1; (ls->K_I4).mat[1][2] = 0; (ls->K_I4).mat[1][3] = 0;
	(ls->K_I4).mat[2][0] = 0; (ls->K_I4).mat[2][1] = 0; (ls->K_I4).mat[2][2] = 1; (ls->K_I4).mat[2][3] = 0;
	(ls->K_I4).mat[3][0] = 0; (ls->K_I4).mat[3][1] = 0; (ls->K_I4).mat[3][2] = 0; (ls->K_I4).mat[3][3] = 1;
	
	(ls->K_P).row = 4; (ls->K_P).col = 4;
	(ls->K_P).mat[0][0] = ls->c0; (ls->K_P).mat[0][1] = 0; (ls->K_P).mat[0][2] = 0; (ls->K_P).mat[0][3] = 0; 
	(ls->K_P).mat[1][0] = 0.0f; (ls->K_P).mat[1][1] = ls->c0; (ls->K_P).mat[1][2] = 0; (ls->K_P).mat[1][3] = 0; 
	(ls->K_P).mat[2][0] = 0.0f; (ls->K_P).mat[2][1] = 0; (ls->K_P).mat[2][2] = ls->cv0; (ls->K_P).mat[2][3] = 0; 
	(ls->K_P).mat[3][0] = 0.0f; (ls->K_P).mat[3][1] = 0; (ls->K_P).mat[3][2] = 0; (ls->K_P).mat[3][3] = ls->cv0; 
	
	(ls->K_Q).row = 2; (ls->K_Q).col = 2;
	(ls->K_Q).mat[0][0] = pow(ls->w, 2.0); (ls->K_Q).mat[0][1] = 0;
	(ls->K_Q).mat[1][0] = 0; (ls->K_Q).mat[1][1] = pow(ls->w, 2.0);
	
	_pid->K_derivative.row = 2; _pid->K_derivative.col = 1;
	_pid->K_integrate.row = 2; _pid->K_integrate.col = 1;
	_pid->K_err.row = 2; _pid->K_err.col = 1;
	_pid->K_err_old.row = 2; _pid->K_err_old.col = 1;
	_pid->K_err_old.mat[0][0] = 0.0f;
	_pid->K_err_old.mat[1][0] = 0.0f;
	
	ls->K_u_real.row = 2; ls->K_u_real.col = 1;
	ls->K_y.row = 2; ls->K_y.col = 1;
	ls->K_u.row = 2; ls->K_u.col = 1;
	v->p_camera.col = 1; v->p_camera.row = 3;
	v->p_compensated.col = 1; v->p_compensated.row = 3;
	ls->K_y.row = 2; ls->K_y.col = 1;
	ls->K_y.mat[0][0] = 0.0f;
	ls->K_y.mat[1][0] = 0.0f;
	
	v->pBin0 = (uint32_t *)(0x22000000 + ( (uint32_t)bin - 0x20000000 ) * 32);
	v->last_frame_count = 0;
	v->R_y.col = 3; v->R_y.row = 3;
	v->R_x.col = 3; v->R_x.row = 3;
}

FIL MyFile;     /* File object */
FRESULT res;    /* FatFs function common result code */
void getParaFromSD(){
	char *rtext = (char *)gray; // borrow it, at the begining
	uint32_t bytesread; /* File write/read counts */
	char *ptr0, *ptr1, *ptr_end;
	//read intial value that stored in file "para.txt" on the SD card
	DISABLE_IRQ
	res = f_open(&MyFile, "para.txt", FA_READ);
	ENABLE_IRQ
	if( res != FR_OK){
		PRINTF("open error\r\n");
		while(1);
	}
	PRINTF("open successfully\r\n");
	DISABLE_IRQ
	res = f_read(&MyFile, rtext, 2048, (uint32_t *)&bytesread);
	ENABLE_IRQ
	if( res != FR_OK ){
		PRINTF("read error\r\n");
		while(1);
	}else{
		DISABLE_IRQ
		printf("%d bytes read\r\n", bytesread);
		ENABLE_IRQ
	}
	DISABLE_IRQ
	f_close(&MyFile);		
	ENABLE_IRQ
	/* parse the text, format example:
	% commments, every line start with a letter which is similar to the variable name
	K 0.1
	*/
	ptr0 = rtext;
	ptr_end = rtext + bytesread;
	while( ptr0 < ptr_end ){
		// search '\n' and replace '\r\n\' with '\0\0' to split the text to a short string
		ptr1 = strchr((const char *)ptr0, '\n');
		if( ptr1 ){ // maybe '\n' is missing 
			*(ptr1 - 1) = '\0'; // '\r'
			*ptr1 = '\0';  // '\n'
			set_parameter( ptr0 );
			ptr0 = ptr1 + 1; // the first char of next line 
		}else{ // the '\n' is missed at the end of file
			break;
		}
		HAL_Delay(50);
	}
}
void set_parameter(char *str){
	float f1, f2;
	int d, length;
	char *result = txBuffer8266 + 2;
	switch( *str ){
		case '%' : // comments line, ignore it;
			break;
		case 'H' :
			sscanf(str, "%*s%d", &cp.height_offset);
			length = sprintf(result, "height offset set: %d mm\r\n", cp.height_offset);
			break;
		//vision
		case 't' : 
			sscanf(str, "%*s%d", &vision.threshold);
			length = sprintf(result, "threshold set: %d\r\n", vision.threshold);
			break;
		case 's' :
			sscanf(str, "%*s%f", &vision.sd_threshold);
			length = sprintf(result, "sd_threshold set: %f\r\n", vision.sd_threshold);
			break;
		case 'b' :
			sscanf(str, "%*s%d", &d);
			vision.bin_send = (bool)d;
			if( vision.bin_send ){
				vision.gray_send = false;
			}
			length = sprintf(result, "bin_send set: %d\r\n", (int)vision.bin_send);
			break;
		case 'g' :
			sscanf(str, "%*s%d", &d);
			vision.gray_send = (bool)d;
			if( vision.gray_send ){
				vision.bin_send = false;
			}
			length = sprintf(result, "gray_send set: %d\r\n", (int)vision.gray_send);
			break;	
		case 'o' : 
			sscanf(str, "%*s%d", &vision.LOST_LIMIT);
			length = sprintf(result, "object lost limit set: %d\r\n", vision.LOST_LIMIT);
			break;
		// camera				
		case 'f' :  // focus length in um
			sscanf(str, "%*s%d", &cp.f);
			length = sprintf(result, "focus length set: %d\r\n", cp.f);
			break;
		case 'z' : //pixel size in um
			sscanf(str, "%*s%d", &cp.s);
			length = sprintf(result, "pixel size set: %d\r\n", cp.s);
			break;
		case 'R' : 
			sscanf(str, "%*s%d", &cp.r0);
			length = sprintf(result, "r0 set: %d\r\n", cp.r0);
			break;
		case 'C' : 
			sscanf(str, "%*s%d", &cp.c0);
			length = sprintf(result, "c0 set: %d\r\n", cp.c0);
			break;
		case 'L' : 
			sscanf(str, "%*s%d", &cp.realDiameter);
			length = sprintf(result, "realDiameter set: %d\r\n", cp.realDiameter);
			break;
		case 'O' : 
			sscanf(str, "%*s%d", &cp.diameterThreshold);
			length = sprintf(result, "diameterThreshold set: %d\r\n", cp.diameterThreshold);
			break;
		// linear system
		case 'T' :
			sscanf(str, "%*s%f", &f1);
			ls.K_T = (double)f1;
			length = sprintf(result, "K_T set: %f\r\n", ls.K_T);
			break;
		case 'c' : // initial covariance
			sscanf(str, "%*s%f%f", &f1, &f2);
			ls.c0 = (double)f1;
			ls.cv0 = (double)f2;
			length = sprintf(result, "c0 cv0 set: %f %f\r\n", ls.c0, ls.cv0);
			break;
		case 'w' :
			sscanf(str, "%*s%f", &f1);
			ls.w = (double)f1;
			length = sprintf(result, "w set : %f\r\n", ls.w);
			break;
		case 'v' :
			sscanf(str, "%*s%f", &f1);
			ls.v0 = (double)f1;
			length = sprintf(result, "v0 set : %f\r\n", ls.v0);
			break;
		case 'e' : 
			sscanf(str, "%*s%f", &f1);
			ls.k_velocity = (double)f1;
			length = sprintf(result, "k_velocity set: %f\r\n", ls.k_velocity);
			break;
		case 'l' : // k_lean
			sscanf(str, "%*s%f", &f1);
			ls.k_lean = (double)f1;
			length = sprintf(result, "k_lean set: %f\r\n", ls.k_lean);
			break;
		case 'h' : // k_height
			sscanf(str, "%*s%f", &f1);
			ls.k_height = (double)f1;
			length = sprintf(result, "k_height set: %f\r\n", ls.k_height);
			break;
		case 'D' : // d_limit
			sscanf(str, "%*s%f", &f1);
			ls.d_limit = (double)f1;
			length = sprintf(result, "d_limit set: %f\r\n", ls.d_limit);
			break;
		// pid
		case 'p' : // PID
			sscanf(str, "%*s%f", &f1);
			pid.kp = (double)f1;
			length = sprintf(result, "kp set: %f\r\n", pid.kp);
			break;
		case 'i' :
			sscanf(str, "%*s%f", &f1);
			pid.ki = (double)f1;
			length = sprintf(result, "ki set: %f\r\n", pid.ki);
			break;
		case 'd' :
			sscanf(str, "%*s%f", &f1);
			pid.kd = (double)f1;
			length = sprintf(result, "kd set: %f\r\n", pid.kd);
			break;
		case 'x' :
			sscanf(str, "%*s%f", &f1);
			pid.ratio_x = (double)f1;
			length = sprintf(result, "ratio_x set: %f\r\n", pid.ratio_x);
			break;
		case 'y' :
			sscanf(str, "%*s%f", &f1);
			pid.ratio_y = (double)f1;
			length = sprintf(result, "ratio_y set: %f\r\n", pid.ratio_y);
			break;
		case 'a' : // ratio: y/x
			sscanf(str, "%*s%f", &f1);
			pid.ratio_yx = (double)f1;
			length = sprintf(result, "ration_yx set: %f\r\n", pid.ratio_yx);
			break;
		case 'm' :
			sscanf(str, "%*s%f", &f1);
			pid.v_limit_up = (double)f1;
			length = sprintf(result, "v_limit_up set: %f\r\n", pid.v_limit_up);
			break;
		case 'n' :
			sscanf(str, "%*s%f", &f1);
			pid.v_limit_down = (double)f1;
			length = sprintf(result, "v_limit_down set: %f\r\n", pid.v_limit_down);
			break;
		case 'u' :
			sscanf(str, "%*s%f", &f1);
			pid.u_limit = (double)f1;
			length = sprintf(result, "u limit set: %f\r\n", pid.u_limit);
			break;
		case 'I' :
			sscanf(str, "%*s%f", &f1);
			pid.i_limit = (double)f1;
			length = sprintf(result, "i limit set: %f\r\n", pid.i_limit);
			break;
		case 'N' : // NED  roll
			sscanf( str, "%*s%d", &pid.rollC );
			length = sprintf(result, "roll center set: %d\r\n", pid.rollC);
			break;
		case 'E' : // NED pitch
			sscanf(str, "%*s%d", &pid.pitchC);
			length = sprintf(result, "pitch center set: %d\r\n", pid.pitchC);
			break;
		default :
			length = sprintf(result, "set error, no this cmd");
	}
	if( esp8266_exist && websocketConnected ){
		websocket_send(length);
	}else{
		DISABLE_IRQ
		printf(result);
		ENABLE_IRQ
	}
}

float calc_sd_even( const uint16_t *data, int length ){
	const uint16_t *dataEnd = data + length * 2;
	float sum=0, average;

	while( data < dataEnd ){
		sum += *data;
		data += 2;
	}
	average = sum/length;
	sum = 0;
	data = dataEnd - length*2;
	while( data < dataEnd ){
		sum += fabs( *data - average );
		data += 2;
	}
	return (sum / length)/average;
}
