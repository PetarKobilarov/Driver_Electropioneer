#ifndef _CAN_H_INCLUDED
#define _CAN_H_INCLUDED

#define DRIVER_IDENTIFICATOR		0xC3
#define MAIN_BOARD_IDENTIFICATOR	0x01

#define RX_BUFFER_SIZE	50

void CAN_init(unsigned int);
void CAN_write(unsigned char *, unsigned int, unsigned char);
void CAN_read(unsigned char *);
unsigned char CAN_checkRX(void);
void CAN_getLastMessage(unsigned char *);

#endif
