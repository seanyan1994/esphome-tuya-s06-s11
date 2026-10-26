#ifndef __CMT_SPI3_H
#define __CMT_SPI3_H

#include "Arduino.h"
//ugly hack but I don't want to rewrite the whole cmt2300a implementation
#include "globals.h"
#define t_nSysTickCount 	millis() //ͬ��ʱ�ӣ��û����Բ��ö���


#define CMT2300A_SPI_SCLK_PIN					tuya_sclk
#define CMT2300A_SPI_MOSI_PIN					tuya_mosi
#define CMT2300A_SPI_CSB_PIN					tuya_csb
#define CMT2300A_SPI_FCSB_PIN					tuya_fcsb

#define cmt_spi3_csb_out()   pinMode(CMT2300A_SPI_CSB_PIN, OUTPUT)
#define cmt_spi3_csb_1()	 digitalWrite(CMT2300A_SPI_CSB_PIN,HIGH)
#define cmt_spi3_csb_0()	 digitalWrite(CMT2300A_SPI_CSB_PIN,LOW)
#define cmt_spi3_fcsb_out()  pinMode(CMT2300A_SPI_FCSB_PIN, OUTPUT)
#define cmt_spi3_fcsb_1()	 digitalWrite(CMT2300A_SPI_FCSB_PIN,HIGH)
#define cmt_spi3_fcsb_0()	 digitalWrite(CMT2300A_SPI_FCSB_PIN,LOW)
#define cmt_spi3_sclk_out()  pinMode(CMT2300A_SPI_SCLK_PIN, OUTPUT)
#define cmt_spi3_sclk_1()	 digitalWrite(CMT2300A_SPI_SCLK_PIN,HIGH)
#define cmt_spi3_sclk_0()	 digitalWrite(CMT2300A_SPI_SCLK_PIN,LOW)
#define cmt_spi3_sdio_1()	 digitalWrite(CMT2300A_SPI_MOSI_PIN,HIGH)
#define cmt_spi3_sdio_0()	 digitalWrite(CMT2300A_SPI_MOSI_PIN,LOW)

#define cmt_spi3_sdio_in()	 pinMode(CMT2300A_SPI_MOSI_PIN, INPUT_PULLUP)
#define cmt_spi3_sdio_out()	 pinMode(CMT2300A_SPI_MOSI_PIN, OUTPUT)
#define cmt_spi3_sdio_read() digitalRead(CMT2300A_SPI_MOSI_PIN)

#define CMT2300A_DelayMs(ms)            delay(ms)
#define CMT2300A_DelayUs(us)            delayMicroseconds(us)
#define g_nSysTickCount 				t_nSysTickCount

/* ************************************************************************ */

void cmt_spi3_init(void);

void cmt_spi3_send(uint8_t data8);
uint8_t cmt_spi3_recv(void);

void cmt_spi3_write(uint8_t addr, uint8_t dat);
void cmt_spi3_read(uint8_t addr, uint8_t* p_dat);

void cmt_spi3_write_fifo(const uint8_t* p_buf, uint16_t len);
void cmt_spi3_read_fifo(uint8_t* p_buf, uint16_t len);

#endif
