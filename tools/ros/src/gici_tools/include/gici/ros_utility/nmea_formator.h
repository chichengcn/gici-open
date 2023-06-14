/**
* @Function: Decode and encode GICI NMEA messages
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

#undef lock
#undef unlock

// Decode GNRMC message
int decodeRMC(char *buff, sol_t *sol);

// Decode GNGGA message
int decodeGGA(char *buff, sol_t *sol);

// ESA informations
typedef struct {
  gtime_t time;
  double vel[3];
  double att[3];
} esa_t;

// ESD informations
typedef struct {
  gtime_t time;
  double std_pos[3];
  double std_vel[3];
  double std_att[3];
} esd_t;

// Decode GNESA message
int decodeESA(char *buff, sol_t *sol, esa_t *esa);

// Decode GNESD message
int decodeESD(char *buff, sol_t *sol, esd_t *esd);

// Encode GNRMC message
int encodeRMC(const sol_t *sol, char *buff);

// Encode GNGGA message
int encodeGGA(const sol_t *sol, char *buff);

// Encode GNESA (self-defined Extended Speed and Attitude) message
// Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
int encodeESA(const sol_t *sol, const esa_t *esa, char* buf);

// Encode GNESD (self-defined Extended Speed and Attitude) message
// Format: $GNESD,tod,STD_Pe,STD_Pn,STD_Pu,STD_Ve,STD_Vn,STD_Vu,
//         STD_Ar,STD_Ap,STD_Py*checksum
int encodeESD(const sol_t *sol, const esd_t *esd, char* buf);

#ifdef __cplusplus
}

// One epoch NMEA message
typedef struct {
  sol_t sol;
  esa_t esa;
  esd_t esd;
} NmeaEpoch;

// Load and decode NMEA file
bool loadNmeaFile(char *path, std::vector<NmeaEpoch>& epochs);

// Encode and write NMEA file
bool writeNmeaFile(const std::vector<NmeaEpoch>& epochs, char *path);

#endif
