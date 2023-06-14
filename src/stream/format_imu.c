/**
* @Function: IMU message functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/format_imu.h"

#define IMUPREAMB 0xFECB  /* IMU frame preamble */

int scale_inited = 0;
double acc_encode_factor;
double gyro_encode_factor;

/* init scale  --------------------------------------------------------*/
static void init_scale()
{
  acc_encode_factor=1.0/(9.8*24)*pow(2.0,20);
  gyro_encode_factor=R2D/4000.0*pow(2.0,20);
}

/* 8 bit parity  ------------------------------------------------------*/
static uint8_t crc8(const uint8_t *buff, int len)
{
  uint8_t crc=0;
  int i;
  
  trace(4,"crc8: len=%d\n",len);
  
  for (i=0;i<len;i++) crc^=buff[i];

  return crc;
}

/* decode imu message -------------------------------------------------*/
static int decode_imu(imu_t *imu)
{
  gtime_t time;
  int i=24,j,acc[3],gyro[3],sec;
  
  trace(3,"decode_imu: len=%3d\n",imu->len);

  if (!scale_inited) init_scale();

  time.time=getbitu(imu->buff,i,32); i+=32;
  sec    =getbitu(imu->buff,i,32); i+=32;
  for (j=0;j<3;j++) {
    acc[j]=getbits(imu->buff,i,20); i+=20;
  }
  for (j=0;j<3;j++) {
    gyro[j]=getbits(imu->buff,i,20); i+=20;
  }
  time.sec=(double)sec*1.0e-9;
  imu->time=time;
  for (j=0;j<3;j++) {
    imu->acc[j]=(double)acc[j]/acc_encode_factor;
    imu->gyro[j]=(double)gyro[j]/gyro_encode_factor;
  }
  
  return 1;
}

/* encode imu message -------------------------------------------------*/
extern int encode_imu(imu_t *imu)
{
  int i=24,j,sec=imu->time.sec*1.0e9;
  int32_t tmp;

  trace(3,"encode_imu\n");

  if (!scale_inited) init_scale();
  
  setbitu(imu->buff,i,32,imu->time.time); i+=32;
  setbitu(imu->buff,i,32,sec       ); i+=32;
  /* acceleration, range: -12g~12g, precision: 4.5e-4 m/s^2 */
  for (j=0;j<3;j++) {
    tmp = imu->acc[j]*acc_encode_factor;
    setbits(imu->buff,i,20,tmp); i+=20;
  }
  /* angular rate, range: -2000~2000 deg/s, precision: 3.8e-3 deg/s */
  for (j=0;j<3;j++) {
    tmp = imu->gyro[j]*gyro_encode_factor;
    setbits(imu->buff,i,20,tmp); i+=20;
  }

  imu->len=i/8;

  return 1;
}


/* input imu message from stream --------------------------------------------
* fetch next imu message and input a message from byte stream
* args   : imu_t *imu     IO  imu control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input imu data)
* notes  : 
*      imu message format:
*      +----------+-----------+--------------------+---------+
*      | preamble |  length   |    data message    | parity  |
*      +----------+-----------+--------------------+---------+
*      |<-- 16 -->|<--- 8 --->|<--- length x 8 --->|<-- 8 -->|
*      
*-----------------------------------------------------------------------------*/
extern int input_imu(imu_t *imu, uint8_t data)
{
  trace(5,"input_imu: data=%02x\n",data);
  
  /* synchronize frame */
  if (imu->nbyte==0) {
    if (data!=(uint8_t)((IMUPREAMB&0xFF00)>>8)) return 0;
    imu->buff[imu->nbyte++]=data;
    return 0;
  }
  if (imu->nbyte==1) {
    if (data!=(uint8_t)(IMUPREAMB&0x00FF)) {
      imu->nbyte = 0; return 0;
    }
    imu->buff[imu->nbyte++]=data;
    return 0;
  }
  imu->buff[imu->nbyte++]=data;
  
  if (imu->nbyte==3) {
    imu->len=getbitu(imu->buff,16,8)+3; /* length */
  }
  if (imu->nbyte<3||imu->nbyte<imu->len+1) return 0;
  imu->nbyte=0;

  /* check parity */
  if (crc8(imu->buff,imu->len)!=getbitu(imu->buff,imu->len*8,8)) {
    trace(2,"imu parity error: len=%d\n",imu->len);
    return 0;
  }

  /* decode imu message */
  return decode_imu(imu);
}

/* generate imu message -----------------------------------------------------
* generate imu message
* args   : imu_t *imu     IO  imu control struct
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int gen_imu(imu_t *imu)
{
  int i=0,crc=0;
  
  trace(4,"gen_imu\n");
  
  imu->nbyte=imu->len=0;
  
  /* set preamble and reserved */
  setbitu(imu->buff,i,16,IMUPREAMB); i+=16;
  setbitu(imu->buff,i,8 ,0    ); i+=8;
  
  /* encode imu message body */
  if (!encode_imu(imu)) return 0;

  /* message length without header and parity */
  setbitu(imu->buff,16,8,imu->len-3);
  
  /* crc-8q */
  crc=crc8(imu->buff,imu->len);
  i=imu->len*8;
  setbitu(imu->buff,i,8,crc);
  
  /* length total (bytes) */
  imu->nbyte=imu->len+1;
  
  return 1;
}

/* Initialize imu structure */
extern void init_imu(imu_t *imu)
{
  gtime_t time0 = {0};
  int i;

  imu->time = time0;
  imu->nmax = 65535;
  for (i = 0; i < 3; i++) {
    imu->acc[i] = 0.0;
    imu->gyro[i] = 0.0;
  }
  imu->nbyte = 0;
  imu->len = 0;
  if (!(imu->buff = (uint8_t*)malloc(sizeof(uint8_t)*imu->nmax))) {
    free_imu(imu);
    return;
  }
}

/* Free imu structure */
extern void free_imu(imu_t *imu)
{
  free(imu->buff); imu->buff = NULL;
}