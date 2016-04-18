#ifndef ESP8266_H
#define ESP8266_H

#include "main.h"
#define RXBUFFERSIZE_8266 1024
#define HEADER_LENGTH 19
//pos : current postion, pos_end: right side of the last received character
extern int pos, pos_end;
extern char aRxBuffer8266[RXBUFFERSIZE_8266]; // DAM receiving, no interrupt,
// this var is set in esp8266_send(), and reset in esp8266_parse if sent ok, you don't need to set/reset it, just read it;
extern bool esp8266_sending;
extern uint8_t txBuffer8266[]; // if delare here, it maybe in CCM, can't be accessed by DMA; you can remove "extern" if you have enough memory
typedef enum {
	NO_EVENT,
	LINK,
	UNLINK,
	PAYLOAD,
	SEND_OK,
	SEND_ERROR,
	RST
}esp8266_event_t;

//send "AT\r\n" to test whether it exist and then reset it, if both action is succeed, return true;
bool esp8266_init(void);

// check whether there is new data come, if not ,return NO_EVENT; else parse the type of the new message;
// if there is a payload, the first parameter is used to store payload it received, the second is the length;
esp8266_event_t esp8266_parse( uint8_t *payload, int *payload_length );

void move_pos(int length);
//return how many characters have been received and update pos_end
int update_length(void);
// if len = -6, means copy the last 6 chars
void copy_buffer(uint8_t *tmp, int len);

//cmd is sent, echo is the last chars means OK.  if received extra chars after "OK\r\n" and the task is interrupted, it maybe fail, but this seldom happens
//you'd better make sure there is no message on the way before use it, otherwise the incoming message will be ignored
// this function need enhance
bool esp8266_cmd(char *cmd, char *echo, int timeout);

// cmd send in blocking mode, data is send in DMA mode, return before send ok; it will check esp8266_sending, you don't need to check;
bool esp8266_send(int socket, uint8_t *data, int length);

//not implemented
//bool esp8266_list_AP(void);

#endif
