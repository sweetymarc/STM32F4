#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include "main.h"

extern bool websocketConnected;
void rm_mask(uint8_t *payload, int length, uint8_t **context, int *context_length);
void web_soket_handshake(char *payload);

// the data should be store at txBuffer8266 + 2
void websocket_send(int length);
#endif
