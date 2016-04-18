#include "stm32f4_esp8266.h"

extern UART_HandleTypeDef UartHandle;
extern volatile bool UartReady;

char aRxBuffer8266[RXBUFFERSIZE_8266];
int pos=0, pos_end;
bool esp8266_sending = false;

bool esp8266_init(){
	int length;
	// if the mcu and esp8266 are powered up at the same time, mcu will receive some info, have to ignore it.
  while(1){ // waiting until the receive buffer stop increase;
		HAL_Delay(100);
		length = update_length();
		if( length > 0 ){
			pos += length;
			pos %= RXBUFFERSIZE_8266;
			continue;
		}else{
			break;
		}
	}
	if( esp8266_cmd("\r\nAT\r\n", "\r\nOK\r\n", 1000) ){
		if( esp8266_cmd("AT+RST\r\n", "\r\nready\r\n", 2000) ){
			return true;
		}else{
			return false;
		}
	}else{
		return false;
	}
}
// in order to avoid push & pop and easy to debug, put the big data outside
static	char cmd_echo[16];
bool esp8266_cmd(char *cmd, char *echo, int timeout){
	int time, length=0, compare_length = strlen(echo);
	while(!UartReady);
	HAL_UART_Transmit( &UartHandle, (uint8_t *)cmd, strlen(cmd), 0xffff);
	time = HAL_GetTick();
	while( HAL_GetTick() - time < timeout ){ // waiting "xxx\r\nOK\r\n
		length = update_length();
		if( length > compare_length ){
			copy_buffer((uint8_t *)cmd_echo, -1*compare_length);
			if( 0 == strncmp(cmd_echo, echo, compare_length) ){
				pos += length;
				pos %= RXBUFFERSIZE_8266;
				return true;
			}
		}
	}
	// should move pos to the end, maybe the response is "no changed" which is the same as "OK", if you don't move it, the next cmd maybe fail
	pos += length;
	pos %= RXBUFFERSIZE_8266;
	return false;
}

bool esp8266_send(int socket, uint8_t *data, int length){
	char cmd[20];
	if( esp8266_sending ){
		return false;
	}
	sprintf(cmd, "AT+CIPSEND=%d,%d\r\n", socket, length);
	if( esp8266_cmd(cmd, "\r\n> ", 2000) ){
		while(!UartReady);
		if( (uint32_t)data < 0x20000000UL ){ // CCM memory
			while(1);
		}
		HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)data, length);
		UartReady = RESET;
		esp8266_sending = true;
		return true;
	}else{
		//while(1); // convenient to debug
		return false;
	}
}

void move_pos(int length){
	pos += length;
	pos %= RXBUFFERSIZE_8266;
}
int update_length(){
	int length;
	pos_end = RXBUFFERSIZE_8266 - __HAL_DMA_GET_COUNTER( UartHandle.hdmarx );
	if( pos_end >= pos ){
		length = pos_end - pos;
	}else{
		length = (RXBUFFERSIZE_8266 + pos_end) - pos;
	}
	//p1 = aRxBuffer + pos;
	//p2 = aRxBuffer + pos_end;
	return length;
}
void copy_buffer(uint8_t *tmp, int length){
	int pos_tmp;
	char *p1;
	if( length > 0){
		pos_tmp = (pos + length) % RXBUFFERSIZE_8266;
		p1 = aRxBuffer8266 + pos;
		if( pos_tmp > pos ){
			memcpy(tmp, p1, length);
		}else{
			memcpy( tmp, p1, RXBUFFERSIZE_8266-pos );
			memcpy( tmp + RXBUFFERSIZE_8266-pos, aRxBuffer8266, pos_tmp );
		}
	}else{
		length *= -1;
		pos_tmp = ( pos_end - length );
		if( pos_tmp < 0 ){
			pos_tmp += RXBUFFERSIZE_8266;
		}
		p1 = aRxBuffer8266 + pos_tmp;
		if( pos_end > pos_tmp ){
			memcpy(tmp, p1, length);
		}else{
			memcpy( tmp, p1, RXBUFFERSIZE_8266-pos_tmp );
			memcpy( tmp + RXBUFFERSIZE_8266-pos_tmp, aRxBuffer8266, pos_end );
		}
	}
}

static char header[16];
esp8266_event_t esp8266_parse(uint8_t *payload, int *payload_length){
	int length, length_expected; 
	int pos_tmp, pos_end2;
	char *p3, *p4;
	copy_buffer((uint8_t *)header, 6);
	length = update_length();
	if( length < 6 ){
		return NO_EVENT;
	}
	if( esp8266_sending ){ // sending
		if( length >= 11 ){
			pos_end2 = pos_end;
			if( pos_end2 < pos ){
				pos_end2 += RXBUFFERSIZE_8266;
			}
			for( pos_tmp = pos; pos_tmp <= pos_end2 - 10; pos_tmp++ ){ //"\r\nSEND OK\r\n"
				if( aRxBuffer8266[ pos_tmp%RXBUFFERSIZE_8266 ] == '\n' && aRxBuffer8266[ (pos_tmp + 7)%RXBUFFERSIZE_8266 ] == 'K' ){
					pos += pos_tmp-pos+10;
					pos %= RXBUFFERSIZE_8266;
					esp8266_sending = false;
					return SEND_OK;
				}
			}
		}
		return NO_EVENT;
	}
	if( length >=6 && (0 == strncmp(header, "Link", 4)) ){  // link
		pos += 6;
		pos %= RXBUFFERSIZE_8266;
		return LINK;
	}else if( length >=8 && (0 == strncmp(header, "Unlink", 6)) ){ // unlink
		pos += 8;
		pos %= RXBUFFERSIZE_8266;
		return UNLINK;
	}else if( length > HEADER_LENGTH && (0 == strncmp(header, "\r\n+IPD", 6)) ){ // "\r\n+IPD=0,478:xxx\r\nOK\r\n"
		copy_buffer((uint8_t *)header, 16);
		p3 = header+9;
		p4 = strchr( (char *)p3, ':' );
		*p4 = '\0';
		*payload_length = atoi(p3);
		length_expected = *payload_length + HEADER_LENGTH - 3 + strlen(p3);
		*(payload + *payload_length) = '\0';
		if( length < length_expected ){ //HEADER_LENGTH include "478", but the lenght is variable
			return NO_EVENT;
		}
		// now we have got a payload
		pos += (10 + strlen(p3));
		pos %= RXBUFFERSIZE_8266;
		copy_buffer( payload, *payload_length);
		pos += *payload_length + 6;
		pos %= RXBUFFERSIZE_8266;
		return PAYLOAD;
	}else{
		return NO_EVENT;
	}
}
