
#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "driverlib/uart.h"


#define CFG_UART_BASE UART0_BASE

typedef struct {
    uint8_t packetData[255];
    uint8_t packetOpcode;
    uint8_t dataLen;
    uint8_t packetValid;
} strPacket_t ;

void rcvPacket(strPacket_t *packet);
void sendPacket(strPacket_t *p);

void UARTWriteBytes(uint32_t uart_base, uint8_t *buff, int len); 
void UARTWriteByte(uint32_t uart_base, char byte);
uint32_t bytesToU32(uint8_t *buff, int offset);

#endif /* PACKET_H_ */