#ifndef __SYSTEM_DEF_H__
#define __SYSTEM_DEF_H__



unsigned char gSYS_SEC_cnt =0;
uint16_t gSYS_100ms_cnt=0;
uint8_t  SYS_10ms_fcnt=0;
uint32_t gSYS_ACT_flag=0;

int gITemp2,gITemp3;
unsigned char PARAM_SYSSTATE;
unsigned char PARAM_MID = 0x10;
unsigned char PARAM_SID = 0x81;


unsigned char gRXPARAM_mid;
unsigned char gRXPARAM_seq;
unsigned char gRXPARAM_rssi=0;
unsigned char gRXPARAM_snr=0;
unsigned char gRXPARAM_seq_next=0;
unsigned int  gRXPARAM_errcnt=0;

unsigned char master_status=0;
//0:tx_idle
//1:tx_busy
//2:tx_done
//3:rx_idle
//4:rx_busy
//5:rx_done
unsigned char slave_status=3;
//0:rsptx_idle
//1:rsptx_busy
//2:rsptx_done
//3:rx_idle
//4:rx_busy
//5:rx_done
unsigned char slave_rsp_flag =100;
unsigned char slave_rx_flag =101;
int slave_rsp_leng =0;

#endif