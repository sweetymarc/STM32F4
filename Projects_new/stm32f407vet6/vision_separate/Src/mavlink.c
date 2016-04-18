#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "v1.0/common/mavlink.h"

#define P_STX 0
#define P_LEN 1
#define P_SEQ 2
#define P_SYS 3
#define P_COMP 4
#define P_MSG 5

void mavlink_parse(void);
mavlink_attitude_t *payload;
double real_roll, real_pitch;
UART_HandleTypeDef Uart1Handle, Uart3Handle;
uint8_t mavlink_buffer[MAVLINK_BUFFER_SIZE];
uint8_t change_rate[] = {0xFE, 0x06, 0xBC, 0xFF, 0xBE, 0x42, 0xFF, 0x00, 0x01, 0x01, 0x0A, 0x01, 0x15, 0x03 };
extern PID_t pid;

void uart3Init(){
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  Uart3Handle.Instance          = USART3;
  
  Uart3Handle.Init.BaudRate     = 57600;
  Uart3Handle.Init.WordLength   = UART_WORDLENGTH_8B;
  Uart3Handle.Init.StopBits     = UART_STOPBITS_1;
  Uart3Handle.Init.Parity       = UART_PARITY_NONE;
  Uart3Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  Uart3Handle.Init.Mode         = UART_MODE_TX_RX;
  Uart3Handle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&Uart3Handle) != HAL_OK)
  {
    while(1);
  }
	HAL_UART_Transmit(&Uart3Handle, change_rate, sizeof(change_rate), 0xff);
	HAL_Delay(10);
	HAL_UART_Transmit(&Uart3Handle, change_rate, sizeof(change_rate), 0xff);
}
void uart1Init(){
	Uart1Handle.Instance = USART1;
	
	Uart1Handle.Init.BaudRate     = 57600;
  Uart1Handle.Init.WordLength   = UART_WORDLENGTH_8B;
  Uart1Handle.Init.StopBits     = UART_STOPBITS_1;
  Uart1Handle.Init.Parity       = UART_PARITY_NONE;
  Uart1Handle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  Uart1Handle.Init.Mode         = UART_MODE_TX_RX;
  Uart1Handle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_Init(&Uart1Handle) != HAL_OK)
  {
    while(1);
  }
	HAL_UART_Transmit(&Uart1Handle, change_rate, sizeof(change_rate), 0xff);
	HAL_Delay(10);
	HAL_UART_Transmit(&Uart1Handle, change_rate, sizeof(change_rate), 0xff);
}
/*
{0xA6, 0x09, 0x01, 0x00, 0x78, 0x87, 0xDA, 0xBE, 0xE1, 0x47, 
	0x83, 0x3F, 0x7D, 0x62, 0x1B, 0xC0, 0xE0, 0xCE, 0xE6, 0x39, 
	0x00, 0x64, 0xA2, 0x38, 0xC0, 0xBB, 0x7C, 0xB9, 0xBE, 0x7A,
	0xff, 0x00, 0x00, 0xcc, 0xFE, 0x1C, 0xE7, 0x01, 0x01, 0x1E, 
	0xA6, 0x09, 0x01, 0x00, 0x78, 0x87, 0xDA, 0xBE, 0xE1, 0x47, 
	0x83, 0x3F, 0x7D, 0x62, 0x1B, 0xC0, 0xE0, 0xCE, 0xE6, 0x39, 
	0x00, 0x64, 0xA2, 0x38, 0xC0, 0xBB, 0x7C, 0xB9, 0xBE, 0x7A, 
	0x33, 0x44, 0xFE, 0x1C, 0xE7, 0x01, 0x01, 0x1E
};*/
static uint8_t msg[64];
static int pos_front = 0, pos_back = 0, distance;

void mavlink_parse(){
	int payload_length = 0;
	uint8_t *p_checksum;
	uint16_t checksum, checksum_calc;
	uint8_t msg_id;
	bool payload_received;
	
	pos_front = MAVLINK_BUFFER_SIZE - __HAL_DMA_GET_COUNTER((Uart3Handle.hdmarx));
	//printf("%d\r\n", sizeof(mavlink_buffer));
	while(1){
		payload_received = false;
		if( pos_front >= pos_back ){
			distance = pos_front - pos_back;
		}else{
			distance = MAVLINK_BUFFER_SIZE + pos_front - pos_back;
		}
		if( distance >= 8 ){
			//printf("distance: %d\r\n", distance);
			payload_length = *(mavlink_buffer + (pos_back + P_LEN)%MAVLINK_BUFFER_SIZE );
			msg_id = *(mavlink_buffer + (pos_back + P_MSG)%MAVLINK_BUFFER_SIZE );
			if( 0xfe == *(mavlink_buffer + pos_back) && MAVLINK_MSG_ID_ATTITUDE == msg_id ){ // search sycn code and msg_id I want
				//printf("sync found: %d\r\n", pos_back);
				//printf("payload_length: %d\r\n", payload_length);
				//printf("msg_id: %x\r\n", msg_id);
				if( payload_length + 8 <= distance ){ // a full attitude payload has been received
					//printf("length enough\r\n");
					// copy the payload
					if( pos_back + 6 + payload_length < MAVLINK_BUFFER_SIZE ){
						memcpy( msg, mavlink_buffer + pos_back + 1, payload_length + 5 );
					}else{
						memcpy( msg, mavlink_buffer + pos_back + 1, MAVLINK_BUFFER_SIZE - (pos_back + 1) );
						memcpy( msg + MAVLINK_BUFFER_SIZE - (pos_back + 1), mavlink_buffer, (payload_length + 5) - (MAVLINK_BUFFER_SIZE - (pos_back + 1)) );
					}
					p_checksum = (mavlink_buffer + (pos_back + 6 + payload_length) % MAVLINK_BUFFER_SIZE);
					checksum = *p_checksum + 0x100 * ( *(p_checksum + 1) );
					//printf("checksum: %x\r\n", checksum);
					checksum_calc = crc_calculate( msg, payload_length + 5 );
					crc_accumulate(39, &checksum_calc);
					//printf("checksum: %x\r\n", checksum_calc);
					// crc check
					if ( checksum == checksum_calc){
						//printf("checksum ok\r\n");
						payload_received = true;
						payload = (mavlink_attitude_t *)((uint8_t *)msg + 5);
						real_roll = payload->roll;
						real_pitch = payload->pitch;
					}else{
						payload_received = false;
					}
				}else{
					//printf("length not enough\r\n");
					return; // length is not enough, so return;
				}
			}else{
				//printf("not found\r\n");
			}
		}else{
			return;// length is not enough, so return;
		}
		if( payload_received ){
			pos_back = ( pos_back + payload_length + 8 ) % MAVLINK_BUFFER_SIZE;
		}else{
			pos_back =( pos_back + 1 ) % MAVLINK_BUFFER_SIZE;
		}
	}
}

static mavlink_message_t tx_msg;
extern uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

uint16_t	roll, pitch;
void mavlink_rc(double x, double y, bool give_up){
	if( give_up ){ // 0 means give up
		roll = 0;
		pitch = 0;
	}else{
		roll	= pid.rollC + x*57.3f*10.0f;
		pitch = pid.pitchC + y*57.3f*10.0f;
	}
	// Pack the message
	mavlink_msg_rc_channels_override_pack(0xff, 0xbe, &tx_msg, 0x01, 0x01, roll, pitch, 0, 0, 0, 0, 0, 0);
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &tx_msg);
	if( Uart1Ready ){ //
		Uart1Ready = false;
		HAL_UART_Transmit_IT(&Uart1Handle, buf, len);
	}/*else{
		HAL_UART_DeInit(&Uart1Handle);
		uart1Init();
		Uart1Ready = true;
	}*/
}
