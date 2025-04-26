#ifndef __LORA_TASK_H__
#define __LORA_TASK_H__
#include "stm32f1xx_hal.h"
extern unsigned char PARAM_MID;
extern unsigned char PARAM_SID;
extern uint16_t gSYS_100ms_cnt;
extern unsigned char gSYS_SEC_cnt;

extern unsigned char gRXPARAM_mid;
extern unsigned char gRXPARAM_seq;
extern unsigned char gRXPARAM_rssi;
extern unsigned char gRXPARAM_snr;

extern unsigned char gRXPARAM_seq_next;
extern unsigned int  gRXPARAM_errcnt;


int compose_beacon(unsigned char *buf,uint8_t fcnt,unsigned char seq );
int send_info_r(unsigned char *buf,unsigned char fcnt,unsigned char sid,unsigned char seq);
int decode_slvrxdata(unsigned char *rxbuf,int *p_ret,unsigned char *txbuf);
int conv_bye2txt(unsigned char bytein,unsigned char *outstr);
int compose_rsptxt(unsigned char * rawbyte,unsigned char *outstr,int);
#endif