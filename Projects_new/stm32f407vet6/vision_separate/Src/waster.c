
int total=0, threshold, search_count=0, out_divide=1;
int min_pixels = 10, max_pixels = 80;
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

bool object_search(uint8_t image[IMAGE_HEIGHT][IMAGE_WIDTH], int *pos_r, int *pos_c){
	int i, j, n, s_r, s_c;
	total = 0;
	//lean = true;
	object_found = false;
	for(n=0; n<2 && 0==total; n++){ // at most 2 times calculation to get the right result, Since this task will blocking other task, limiting its time is necessary
		// the measure to calc twice is to prevent jump between 0 and real value, sometimes, there is not a suitable threshold meet the criteria the big than min and less than max, will cause jump
		s_r = 0; // sum row
		s_c = 0; // sum col
		total = 0;
		for(i=0; i<IMAGE_HEIGHT; i++){
			for(j=0; j<IMAGE_WIDTH; j++){
				#ifdef SEARCH_BRIGHT
				if( image[i][j] > threshold ){
					s_r += i;
					s_c += j;
					total ++;
				}
				#else
				if( image[i][j] < threshold ){
					s_r += i;
					s_c += j;
					total ++;
				}
				#endif
			}
		}
		// adjust threshold
		#ifdef SEARCH_BRIGHT
		if( total < min_pixels ){
			threshold --;
		}else if( total > max_pixels ){
			threshold ++;
		}
		#else // search the dark
		if( total < min_pixels ){
			threshold ++;
		}else if( total > max_pixels ){
			threshold --;
		}
		#endif
	}
	if( 0 != total ){ // object found
		object_found = true;
		v_out.pixels = total;
		*pos_r = s_r / total;
		*pos_c = s_c / total;
		//show the result on the image
		for(i=*pos_r-5; i<=*pos_r+5; i++){
			for(j=*pos_c-5; j<=*pos_c+5; j++){
				image[i][j] = 0xff;
			}
		}
	}else{
		object_found = false;
		//pos_r = 0;
		//pos_c = 0;
	}
	/* Refresh IWDG: reload counter */
	#ifdef WATCH_DOG
	if(HAL_IWDG_Refresh(&IwdgHandle) != HAL_OK)
	{
		Error_Handler();
	}
	#endif
	return object_found;
}
		/*case '1' : // K_K1
			sscanf(str, "%*s%f%f", &f1, &f2);
			K_K.mat[i][0] = (double)f1;
			K_K.mat[i][1] = (double)f2;
			HAL_Delay(10);
			DISABLE_IRQ
			printf(str);
			printf("\r\n%f %f\r\n", K_K.mat[i][0], K_K.mat[i][1]);
			if( 3 == i ){
				matrix_show(&K_K);
			}
			ENABLE_IRQ
			HAL_Delay(10);
			i++;
			break;
			case '2' : //K_K2
			sscanf(str, "%*s%f%f", &f1, &f2);
			K_K2.mat[i2][0] = (double)f1;
			K_K2.mat[i2][1] = (double)f2;
			HAL_Delay(10);
			DISABLE_IRQ
			printf(str);
			printf("\r\n%f %f\r\n", K_K2.mat[i2][0], K_K2.mat[i2][1]);
			if( 3 == i2 ){
				matrix_show(&K_K2);
			}
			ENABLE_IRQ
			i2++;
			break;*/
			