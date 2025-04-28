#include "lora_task.h"
#include "cmd_def.h"

int compose_beacon(unsigned char *buf,uint8_t fcnt,unsigned char seq)
{

*(buf+0) = M2S_ECHO;
*(buf+1) = 12;
*(buf+2) = PARAM_MID;
*(buf+3) = 0xff;
//
*(buf+4) = SC_BEACON;
*(buf+5) = gSYS_SEC_cnt;
*(buf+6) = seq;
*(buf+7) = fcnt;
//
*(buf+8) = 0x08;
*(buf+9) = 0x09;
*(buf+10) = 0x0a;
*(buf+11) = 0x0b;
return 12;
}

int send_info_r(unsigned char *buf,unsigned char fcnt,unsigned char sid,unsigned char seq)
{
*(buf+0) = M2S_PARAM;
*(buf+1) = 12;
*(buf+2) = PARAM_MID;
*(buf+3) = sid;
//
*(buf+4) = SC_PING;
*(buf+5) = 0x15;
*(buf+6) = seq;
*(buf+7) = fcnt;
//
*(buf+8) = 0x18;
*(buf+9) = 0x19;
*(buf+10) = 0x1a;
*(buf+11) = 0x1b;
return 12;
  
}

int decode_slvrxdata(unsigned char *rxbuf,int *p_ret,unsigned char *txbuf)
{
  unsigned char hdr,rxsid,subcmd,uctemp1;
  int ret;
  
  int err=0;
  
  hdr = *(rxbuf+0);
  rxsid = *(rxbuf+3);
  subcmd = *(rxbuf+4);
  
  
  //check sid
  ret=0;err=0;
  //if(!(rxsid == PARAM_SID)) {err |=0x2;}
  //check hdr
  if(hdr == M2S_ECHO)
  {
    if(subcmd == SC_BEACON)
    {
		//6:fcont
		uctemp1= *(rxbuf+7);gSYS_100ms_cnt = uctemp1;
    uctemp1= *(rxbuf+5);gSYS_SEC_cnt = uctemp1;      
		
    }
    else
    {
      
      err |=0x4;
    }
  }
  else if(hdr == M2S_PARAM)
  {
    if(!(rxsid == PARAM_SID)) 
    {err |=0x2;}
    else
    {
      if(subcmd == SC_PING)
	    {
	    	//extract sequence number
	    	gRXPARAM_mid = *(rxbuf+2);
	    	gRXPARAM_seq = *(rxbuf+6);
	    	//compose info read data
	    	*(txbuf+0) = S2M_PARAM;
	    	*(txbuf+1) = 12;
	    	*(txbuf+2) = gRXPARAM_mid;
	    	*(txbuf+3) = PARAM_SID;
        //
	    	*(txbuf+4) = gRXPARAM_rssi;
        *(txbuf+5) = gRXPARAM_snr;
        *(txbuf+6) = 0;
	    	*(txbuf+7) = gRXPARAM_seq;
        //
	    	*(txbuf+8) = 0;
        *(txbuf+9) = 0;
        *(txbuf+10) = 0;
	    	*(txbuf+11) = 0;
      
	    	ret=12;
        if(!(gRXPARAM_seq_next == gRXPARAM_seq))
        {
          //handle overflow??
          if(gRXPARAM_errcnt >= 255)
          {gRXPARAM_errcnt = 255;}
          else
          {gRXPARAM_errcnt++;}
        }
        gRXPARAM_seq_next = gRXPARAM_seq+1;
	    }
      else
      {err |=0x4;}
    }
  }
  else
  {
    err |=0x1;
  }
  //check sid
  //check body
  *(p_ret+0) = ret;
  return err;
}

int conv_bye2txt(unsigned char bytein,unsigned char *outstr)
{
  unsigned char uctemp,uctemp2;
  
  uctemp = (bytein>>4)&0x0f;
  if(uctemp<10)  {uctemp2 = 0x30+uctemp;}
  else{uctemp2 = 0x41+uctemp-10;}
  *(outstr+0)=uctemp2;
  
  uctemp = (bytein>>0)&0x0f;
  if(uctemp<10)  {uctemp2 = 0x30+uctemp;}
  else{uctemp2 = 0x41+uctemp-10;}
  *(outstr+1)=uctemp2;
  return 2;  
}

int compose_rsptxt(unsigned char * rawbyte,unsigned char *outstr,int dataset_cnt,int mode)
{
  unsigned char uctemp;
  unsigned char uctemp_2b[2];
  int i,base,indx;
  
  base=0;
  for(i=0;i<dataset_cnt;i++)
  {
    uctemp = *(rawbyte+i);
    conv_bye2txt(uctemp,&uctemp_2b[0]);
    indx = base+0;*(outstr+indx) = uctemp_2b[0];
    indx = base+1;*(outstr+indx) = uctemp_2b[1];
    if(mode == 0)
    {
      if(i == (dataset_cnt-1))
      {base+=2;}
      else
      {
      indx = base+2;*(outstr+indx) = ',';
      base+=3;
      }
    }
    else
    {
      if(i == (dataset_cnt-1))
      {
      }
      else if((i == 1)||(i == 3)||(i == 5))
      {
        indx = base+2;*(outstr+indx) = ',';
        base+=3;
      }
      else
      {base+=2;}
    }
  }

  //uctemp = *(rawbyte+1);
  //conv_bye2txt(uctemp,&uctemp_2b[0]);
  //*(outstr+3) = uctemp_2b[0];
  //*(outstr+4) = uctemp_2b[1];
  //
  //*(outstr+5) = ',';
  //
  //uctemp = *(rawbyte+2);
  //conv_bye2txt(uctemp,&uctemp_2b[0]);
  //*(outstr+6) = uctemp_2b[0];
  //*(outstr+7) = uctemp_2b[1];
  //
  //*(outstr+8) = ',';
  //
  //uctemp = *(rawbyte+3);
  //conv_bye2txt(uctemp,&uctemp_2b[0]);
  //*(outstr+9) = uctemp_2b[0];
  //*(outstr+10) = uctemp_2b[1];

  indx = base+0;*(outstr+indx) = 0x0d;//'\n';
  indx = base+1;*(outstr+indx) = 0x0a;//'\n';
  
  return indx+1;
  
}

