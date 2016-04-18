#ifndef __POSITION_CALC_H
#define __POSITION_CALC_H
#include "main.h"
#include "matrix.h"
#include "stm32f429_camera.h"
// the MIN value should not be too little, it will casue this situation, for example: color max 201, threshold 200, if threshold++, the result is 0
// the interval should not be too narrow, for example, there just 151 pixels with the max color value, your interval is (100, 150), the result will jump between 0 and the right value
// after experience, the above two advices have drawback, if your object is small, many pixel on the background will be count in, if your object is two big ,there is no precision.
// so, the right solution is to make the MIN and MAX small, if there is no suitable threshold meets your criteria, break the rule.
//variables calculating position
#define SEARCH_BRIGHT
#define LOST_COUNTER 10
#define DIAMETER_COUNTER_SIZE 128
#define JUMP_SIZE 128
#define START_COUNTER 3
#define END_COUNTER 6

#define VISION_BEEP_CLK_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE
#define VISION_BEEP_PORT GPIOD
#define VISION_BEEP_PIN GPIO_PIN_13

typedef struct{
	int count;
	int pixels, height, rawHeight;
	double roll, pitch;
	double x_measure, y_measure;
	double x_sim, y_sim;
	double x_est, y_est;
	double vx_est, vy_est;
	double ux, uy;
	uint32_t period, searchCost, wholeCost;
	int diameterRow, diameterCol;
} vision_output_t;

typedef struct{
	int f; /**/ // in um
	int s; //length of each pixel, in um
	int r0, c0;
	int height_offset;
	int realDiameter;
	int diameterThreshold; //percent, not float 50 means 0.5
} camera_parameter_t;

typedef struct{
	matrix_t p_camera, p_compensated, R_x, R_y; // compensate lean
	uint32_t *pBin0;
	bool object_found;
	int lost_counter, LOST_LIMIT;
	int last_frame_count;
	int pos_r, pos_c;
	int x, y;
	float sd_threshold;
	bool bin_send;
	bool gray_send;
	bool status_send;
	uint8_t threshold;
} vision_t;

typedef struct{
	double dT, vx, vy, d_limit, d_x, d_y;
	double K_T, K_g, c0, cv0, w, v0,  k_velocity, k_lean, k_height;
	long dT_counter;
	matrix_t K_F, K_FT, K_G, K_P, K_Q, K_R, K_I4, K_xe, K_K, K_H, K_HT, K_y, K_u_real, K_u;
}linearSystem_t;

typedef struct{
	int rollC, pitchC;
	double kp, kd, ki, ratio_x, ratio_y, ratio_yx, i_limit, u_limit, v_limit_up, v_limit_down;
	matrix_t K_err, K_err_old, K_derivative, K_integrate;
}PID_t;

extern vision_t vision;
extern vision_output_t v_out;
extern void mavlink_rc(double x, double y, bool give_up);

void EXTILine1_Config(void);// soft int
void vision_control_init();
void getParaFromSD(void);
void set_parameter(char *str);
void matrix_assign_initial_value(linearSystem_t *ls, PID_t *pid, vision_t *v);
bool object_search();
void gray2bin(uint8_t *_gray, int _size, uint8_t _threshold, uint32_t *_bin);
bool circle_search(const uint32_t *_bin, int _height, int _width, float sd_threshold, int *_pos_r, int *_pos_c);
void position_calc(uint16_t _pos_r, uint16_t _pos_c, uint16_t _height, camera_parameter_t *cp, int *x, int *y);
void lean_compensate(vision_t *v, double *_roll, double *_pitch, double *_yaw);
void kalman_filter(linearSystem_t *ls, matrix_t *K_u_real, matrix_t *K_y, bool object_found);
void PID_control(PID_t *pid, matrix_t *K_x, matrix_t *K_u, double *_dT);
float calc_sd_even( const uint16_t data[], int length );



// double K_T=0.01332, K_g = 9.8f, c0=0.01, cv0=0.1, w=0.3, v0=0.005,  k_velocity=0.00001, k_lean=0.01, k_height=0;
// double kp=3, kd=0, ki=0, ratio_x=2, ratio_y=2, ratio_yx=0.5, i_limit=0.1, u_limit=0.2, v_limit_up=1.0f, v_limit_down=0.05;
#endif
