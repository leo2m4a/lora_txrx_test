#ifndef __CMD_DEF_H__
#define __CMD_DEF_H__

//MST <--> SLV
#define M2S_ECHO      0x55
#define M2S_PARAM     0x5a
#define M2S_RLYSCH    0x53
#define M2S_SENSCH    0x5c

#define S2M_ECHO      0x95
#define S2M_PARAM     0x9a
#define S2M_RLYSCH    0x93
#define S2M_SENSCH    0x9c

//subcmd
#define SC_BEACON     0xa5
#define SC_INFO_R     0xa3
#define SC_PING       0xa1
#define SC_CTRLSTS_W     0x11
#define SC_CTRLSTS_R     0x33

#define SC_RLYSCH_R4     0x11
#define SC_RLYSCH_R8     0x13
#define SC_RLYSCH_W8     0x44
#define SC_RLYSCH_W4     0x32
#define SC_RLYSCH_W1     0x33

//#define SC_PARAM_R     0x00
//#define SC_PARAM_W     0x01
//
//#define SC_RLYCTLSTAT_R 0x0
//#define SC_RLYCTLSTAT_W 0x1
//
//#define S2M_RSP_ECHO      0xaa
//#define S2M_RSP_SETUP     0xed
//#define S2M_RSP_CTRLSTS   0x18
//
//#define STAT_OK       0x4F
//#define STAT_FAIL     0x46
//
////HST <--> MST
//#define H2M_ECHO      0x5a
//#define H2M_SETUP     0x53
//#define H2M_R8        0x56
//#define H2M_W8        0x57
//#define H2M_IAPR      0x51
//#define H2M_IAPW      0x52
//#define H2M_IAPR8     0x54
//#define H2M_IAPW8     0x58
//#define H2M_IAPERASE  0x50
//#define H2M_PS        0x10
//
//#define M2H_RSP_ECHO  0xa5
//#define M2H_RSP_SETUP 0xac
//#define M2H_RSP_R8    0xa9
//#define M2H_RSP_W8    0xa8
//#define M2H_IAPR      0xae
//#define M2H_IAPW      0xad
//#define M2H_IAPR8     0xab
//#define M2H_IAPW8     0xa7
//#define M2H_IAPERASE  0xa0
//#define M2H_RSP_PS    0xe0

#endif
