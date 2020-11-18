/*
 * Uart.c
 *
 *  Created on: 2018/10/28
 *      Author: kennj
 */
#include "Uart.h"
#include "altera_avalon_uart_regs.h"
#include "sys/alt_irq.h"

void Uart_send(unsigned char data) {
	while(!(IORD_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE)& ALTERA_AVALON_UART_STATUS_TRDY_MSK));//等待發送完成
	IOWR_ALTERA_AVALON_UART_TXDATA(QSYS_UART_BASE,data);
}

void Uart_send_array(unsigned char *array, unsigned char size) {
	for(;size>0;size--){
		Uart_send(*array);
		array++;
	}
}

unsigned char Uart_receive(void) {
	while(!(IORD_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE) & ALTERA_AVALON_UART_STATUS_RRDY_MSK));//等待接收完成
	return IORD_ALTERA_AVALON_UART_RXDATA(QSYS_UART_BASE);
}

unsigned char Uart_receive_array(unsigned char *array, unsigned char size, unsigned char TimeOut) {
	unsigned char cnt = 0;
	unsigned char TimeOut_cnt = 0;
	for(;size>0;size--,cnt++){
		TimeOut_cnt=0;
		while(       !(IORD_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE) & ALTERA_AVALON_UART_STATUS_RRDY_MSK)
				&&   TimeOut_cnt++ < TimeOut );
		if (TimeOut_cnt >= TimeOut) return cnt;
		array[cnt] = IORD_ALTERA_AVALON_UART_RXDATA(QSYS_UART_BASE);
	}
	return cnt;
}

void Uart_ISR(void * context,alt_u32 id) {

	int sr;
	unsigned char tmp;

	sr = IORD_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE);

	if(sr & ALTERA_AVALON_UART_STATUS_RRDY_MSK)

	{

		tmp = IORD_ALTERA_AVALON_UART_RXDATA(QSYS_UART_BASE);

		IOWR_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE, 0);
		printf("Uart Rx: %x\n",tmp);
		return;
	}else{
		printf("Uart other interrupt\n");
		return;
	}
//
//	if(sr & ALTERA_AVALON_UART_STATUS_TRDY_MSK)
//
//	{
//
//		if(IORD_ALTERA_AVALON_UART_CONTROL(QSYS_UART_BASE) & ALTERA_AVALON_UART_CONTROL_TRDY_MSK);
//
//		{
//
//			if (TxTail_2 != TxHead_2)
//
//			{
//
//				IOWR_ALTERA_AVALON_UART_TXDATA(QSYS_UART_BASE, tx_buffer_2[TxTail_2]);
//
//				if (++TxTail_2 > (TX_BUFFER_SIZE_2 -1)) TxTail_2 = 0;
//
//			}
//
//			else IOWR_ALTERA_AVALON_UART_CONTROL(QSYS_UART_BASE, ALTERA_AVALON_UART_CONTROL_RRDY_MSK);
//
//		}
//
//	}


}

void Uart_ISR_init(void) {
//	alt_ic_isr_register(QSYS_UART_IRQ_INTERRUPT_CONTROLLER_ID,
//			QSYS_UART_IRQ,
//			Uart_ISR,
//			p,
//			0);
	alt_irq_register(QSYS_UART_IRQ,0,Uart_ISR);

	IOWR_ALTERA_AVALON_UART_STATUS(QSYS_UART_BASE, 0x0);	//清空狀態暫存器
	IORD_ALTERA_AVALON_UART_RXDATA(QSYS_UART_BASE);			//讀空接受暫存器中的無效值
	//IOWR_ALTERA_AVALON_UART_DIVISOR(QSYS_UART_BASE, QSYS_UART_FREQ/QSYS_UART_BAUD - 1);//設定鮑率
	IOWR_ALTERA_AVALON_UART_CONTROL(QSYS_UART_BASE, ALTERA_AVALON_UART_CONTROL_RRDY_MSK);//接收中斷使能

}
