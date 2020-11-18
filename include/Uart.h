/*
 * Uart.h
 *
 *  Created on: 2018/10/28
 *      Author: kennj
 */

#ifndef UART_H_
#define UART_H_

#include "system.h"
#include "Uart_conf.h"
#include "alt_types.h"

void Uart_send(unsigned char data);
void Uart_send_array(unsigned char *array,unsigned char size);
unsigned char Uart_receive(void);
unsigned char Uart_receive_array(unsigned char *array, unsigned char size, unsigned char TimeOut);
void Uart_ISR(void * context,alt_u32 id);
void Uart_ISR_init(void);
#endif /* UART_H_ */
