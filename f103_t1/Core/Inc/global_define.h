#ifndef __GLOBAL_DEFINE_H__
#define __GLOBAL_DEFINE_H__

#define SPI_BUFSZ 32
#define UR1_BUFSZ 32

#define CMD_ECHO 0x55
#define CMD_REGW 0xA1
#define CMD_REGR 0x58
#define CMD_CTRL 0xB3
#define CMD_STAT 0x67

#define RSP_ECHO  0xAA
#define RSP_REGW  0x1A
#define RSP_REGR  0x85
//#define RSP_REGR4 0x84

#define RVAL_OK   0x40
#define RVAL_FAIL 0x4f

#define REGSEL_MEM   0x5
#define REGSEL_MEM2  0x7
#define CTRL_CLRMEM  0xc0
#define CTRL_UPDSCRN 0x1c

#define __EN_DIO0_INT 1
#endif
// rxd: [0]:         [1]  :           [2]:        [3]:      [4]:      [5]: | [0]:  [1]: [2]: [3]:      [4]:      [5]: 
//echo:0x55: length   0x04:            d0:         d1:       d2:        x: |0xaa: 0x04:  d0:  d1: 
//regw:0xa1: length   0x04:0x5(sel mem)  :  mem addrh:mem addrl:mem wdata: |0x1a:
//ctrl:0xb3: length   0x04:0xc0(clr mem) :x                  :         :
//ctrl:0xb3: length   0x04:0x1c(clr scrn):x                  :         :
//55 04 12 34 56 78 | AA 04 12 34 56 00 00 40 
//a1 04 05 03 02 aa | 1A 04 05 03 02 00 00 40  
//58 04 05 03 02 00 | 85 04 05 03 02 FF 00 40  