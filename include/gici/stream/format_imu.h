/**
* @Function: IMU message functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#ifndef FORMAT_IMU_H
#define FORMAT_IMU_H

#include <stdint.h>
#include <math.h>

#include "gici/utility/rtklib_safe.h"

#ifdef __cplusplus
extern "C" {
#endif

// IMU stream handle
typedef struct {
  gtime_t time;     /* message time */
  int nmax;       /* max number of buffer */
  double acc[3];    /* acceleration data */
  double gyro[3];   /* gyroscope data */
  int nbyte;      /* number of bytes in message buffer */ 
  int len;      /* message length (bytes) */
  uint8_t *buff;    /* message buffer */
} imu_t;

/* input imu message from stream --------------------------------------------
* fetch next imu message and input a message from byte stream
* args   : imu_t *imu     IO  imu control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input imu data)
* notes  : 
*      imu message format:
*      +----------+-----------+--------------------+---------+
*      | preamble |  length   |  data message  | parity  |
*      +----------+-----------+--------------------+---------+
*      |<-- 16 -->|<--- 8 --->|<--- length x 8 --->|<-- 8 -->|
*      
*-----------------------------------------------------------------------------*/
extern int input_imu(imu_t *imu, uint8_t data);

/* generate imu message -----------------------------------------------------
* generate imu message
* args   : imu_t *imu     IO  imu control struct
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int gen_imu(imu_t *imu);

/* Initialize imu structure */
extern void init_imu(imu_t *imu);

/* Free imu structure */
extern void free_imu(imu_t *imu);

#ifdef __cplusplus
}
#endif

#endif