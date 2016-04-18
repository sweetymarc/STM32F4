#include "websocket.h"
#include "stm32f4_esp8266.h"
#include "hash_sha1.h"
#include "base64.h"

#define KEY_LENGTH 60

char key[64+1], little2big[20], key_base64[32];
extern uint8_t txBuffer8266[]; // if delare here, it maybe in CCM, can't be accessed by DMA; you can remove "extern" if you have enough memory
static SHA1Context sha;
char *key_normal = "PlLHay4ixkG9p2in2h1BkA==258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
char *handshake_normal = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: WebSocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: JtkHP2HhbcUlENuiDpd7CtRL6Mw=\r\n\r\n";
bool websocketConnected = false;
void web_soket_handshake(char *payload){
	char *key_return;
	char *p;
	char *p3, *p4;
	int i, length;
	strcpy(key, key_normal);
	key[64] = '\0';
	length = strlen(handshake_normal);
	memcpy( txBuffer8266, handshake_normal, length+1 );
	key_return = (char*)txBuffer8266 + 97;
	
	p3 = payload;
	do{
		p4 = strchr(p3, '\n')+1;
		if( (p4 - p3) > 17 && (0 == strncmp(p3, "Sec-WebSocket-Key", 17)) ){
			break;
		}
		p3 = p4;
	}while( strchr(p3, '\n') );
	// NOT FOUND is not take into consideration
	p3 += 19; // point to key
	strncpy(key, p3, 24);
	
	SHA1Reset(&sha);
	SHA1Input(&sha, (const unsigned char *) key, KEY_LENGTH);
	if (!SHA1Result(&sha)){
		while(1);
	}else{
		p = (char*)(sha.Message_Digest);
		for(i=0; i<20; i++){
			little2big[i] = p[4*(i/4)+(3-i%4)];
		}
		base64_encode((uint8_t *)little2big, key_base64, 20);
		strncpy(key_return, key_base64, 28);
		//printf("%s\r\n", r);
	}
	// send handshake
	esp8266_send(0, txBuffer8266, length );
}

void rm_mask(uint8_t *payload, int length, uint8_t **context, int *context_length){
	uint8_t *mask;
	int i=0;
	mask = payload + 2;
	*context = payload + 6;
	*context_length = payload[1] & 0x7f;
	for(i=0; i<*context_length; i++){
		(*context)[i] ^= mask[i%4];
		//printf("%x ", payload[i]);
	}
}
// the data should be store at txBuffer8266 + 2
void websocket_send(int length){
	if( !websocketConnected ){
		return;
	}
	while(esp8266_sending);
	txBuffer8266[0] = 0x81;
	txBuffer8266[1] = length;
	esp8266_send(0, txBuffer8266, length+2);
}

