/**
* @Function: Some code and phase operations
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/utility/rtklib_safe.h"

// Rinex and code type maps
// Users should define the MAP operation
#define RINEX_TO_CODE_MAPS \
  MAP('G', "1C", CODE_L1C); \
  MAP('G', "1S", CODE_L1S); \
  MAP('G', "1L", CODE_L1L); \
  MAP('G', "1X", CODE_L1X); \
  MAP('G', "1P", CODE_L1P); \
  MAP('G', "1W", CODE_L1W); \
  MAP('G', "1Y", CODE_L1Y); \
  MAP('G', "1M", CODE_L1M); \
  MAP('G', "2C", CODE_L2C); \
  MAP('G', "2D", CODE_L2D); \
  MAP('G', "2S", CODE_L2S); \
  MAP('G', "2L", CODE_L2L); \
  MAP('G', "2X", CODE_L2X); \
  MAP('G', "2P", CODE_L2P); \
  MAP('G', "2W", CODE_L2W); \
  MAP('G', "2Y", CODE_L2Y); \
  MAP('G', "2M", CODE_L2M); \
  MAP('G', "5I", CODE_L5I); \
  MAP('G', "5Q", CODE_L5Q); \
  MAP('G', "5X", CODE_L5X); \
  MAP('R', "1C", CODE_L1C); \
  MAP('R', "1P", CODE_L1P); \
  MAP('R', "4A", CODE_L4A); \
  MAP('R', "4B", CODE_L4B); \
  MAP('R', "4X", CODE_L4X); \
  MAP('R', "2C", CODE_L2C); \
  MAP('R', "2P", CODE_L2P); \
  MAP('R', "6A", CODE_L6A); \
  MAP('R', "6B", CODE_L6B); \
  MAP('R', "6X", CODE_L6X); \
  MAP('R', "3I", CODE_L3I); \
  MAP('R', "3Q", CODE_L3Q); \
  MAP('R', "3X", CODE_L3X); \
  MAP('E', "1A", CODE_L1A); \
  MAP('E', "1B", CODE_L1B); \
  MAP('E', "1C", CODE_L1C); \
  MAP('E', "1X", CODE_L1X); \
  MAP('E', "1Z", CODE_L1Z); \
  MAP('E', "5I", CODE_L5I); \
  MAP('E', "5Q", CODE_L5Q); \
  MAP('E', "5X", CODE_L5X); \
  MAP('E', "7I", CODE_L7I); \
  MAP('E', "7Q", CODE_L7Q); \
  MAP('E', "7X", CODE_L7X); \
  MAP('E', "8I", CODE_L8I); \
  MAP('E', "8Q", CODE_L8Q); \
  MAP('E', "8X", CODE_L8X); \
  MAP('E', "6A", CODE_L6A); \
  MAP('E', "6B", CODE_L6B); \
  MAP('E', "6C", CODE_L6C); \
  MAP('E', "6X", CODE_L6X); \
  MAP('E', "6Z", CODE_L6Z); \
  MAP('C', "2I", CODE_L2I); \
  MAP('C', "2Q", CODE_L2Q); \
  MAP('C', "2X", CODE_L2X); \
  MAP('C', "1D", CODE_L1D); \
  MAP('C', "1P", CODE_L1P); \
  MAP('C', "1X", CODE_L1X); \
  MAP('C', "1S", CODE_L1S); \
  MAP('C', "1L", CODE_L1L); \
  MAP('C', "1Z", CODE_L1Z); \
  MAP('C', "5D", CODE_L5D); \
  MAP('C', "5P", CODE_L5P); \
  MAP('C', "5X", CODE_L5X); \
  MAP('C', "7I", CODE_L7I); \
  MAP('C', "7Q", CODE_L7Q); \
  MAP('C', "7X", CODE_L7X); \
  MAP('C', "7D", CODE_L7D); \
  MAP('C', "7P", CODE_L7P); \
  MAP('C', "7Z", CODE_L7Z); \
  MAP('C', "8D", CODE_L8D); \
  MAP('C', "8P", CODE_L8P); \
  MAP('C', "8X", CODE_L8X); \
  MAP('C', "6I", CODE_L6I); \
  MAP('C', "6Q", CODE_L6Q); \
  MAP('C', "6X", CODE_L6X); \
  MAP('C', "6Z", CODE_L6Z);

// Phase channels 
#define PHASE_NONE 0
#define PHASE_L1   1
#define PHASE_L2   2
#define PHASE_L5   3
#define PHASE_G1   1
#define PHASE_G1A  2
#define PHASE_G2   3
#define PHASE_G2A  4
#define PHASE_G3   5
#define PHASE_E1   1
#define PHASE_E5A  2
#define PHASE_E5B  3
#define PHASE_E5   4
#define PHASE_E6   5
#define PHASE_B1   1
#define PHASE_B1C  2
#define PHASE_B1A  3
#define PHASE_B2A  4
#define PHASE_B2   5
#define PHASE_B2B  6
#define PHASE_B2AB 7
#define PHASE_B3   8
#define PHASE_B3A  9

// Code type to phase channel maps
#define CODE_TO_PHASE_CHANNEL_MAPS \
  MAP('G', CODE_L1C, PHASE_L1); \
  MAP('G', CODE_L1S, PHASE_L1); \
  MAP('G', CODE_L1L, PHASE_L1); \
  MAP('G', CODE_L1X, PHASE_L1); \
  MAP('G', CODE_L1P, PHASE_L1); \
  MAP('G', CODE_L1W, PHASE_L1); \
  MAP('G', CODE_L1Y, PHASE_L1); \
  MAP('G', CODE_L1M, PHASE_L1); \
  MAP('G', CODE_L2C, PHASE_L2); \
  MAP('G', CODE_L2D, PHASE_L2); \
  MAP('G', CODE_L2S, PHASE_L2); \
  MAP('G', CODE_L2L, PHASE_L2); \
  MAP('G', CODE_L2X, PHASE_L2); \
  MAP('G', CODE_L2P, PHASE_L2); \
  MAP('G', CODE_L2W, PHASE_L2); \
  MAP('G', CODE_L2Y, PHASE_L2); \
  MAP('G', CODE_L2M, PHASE_L2); \
  MAP('G', CODE_L5I, PHASE_L5); \
  MAP('G', CODE_L5Q, PHASE_L5); \
  MAP('G', CODE_L5X, PHASE_L5); \
  MAP('R', CODE_L1C, PHASE_G1); \
  MAP('R', CODE_L1P, PHASE_G1); \
  MAP('R', CODE_L4A, PHASE_G1A); \
  MAP('R', CODE_L4B, PHASE_G1A); \
  MAP('R', CODE_L4X, PHASE_G1A); \
  MAP('R', CODE_L2C, PHASE_G2); \
  MAP('R', CODE_L2P, PHASE_G2); \
  MAP('R', CODE_L6A, PHASE_G2A); \
  MAP('R', CODE_L6B, PHASE_G2A); \
  MAP('R', CODE_L6X, PHASE_G2A); \
  MAP('R', CODE_L3I, PHASE_G3); \
  MAP('R', CODE_L3Q, PHASE_G3); \
  MAP('R', CODE_L3X, PHASE_G3); \
  MAP('E', CODE_L1A, PHASE_E1); \
  MAP('E', CODE_L1B, PHASE_E1); \
  MAP('E', CODE_L1C, PHASE_E1); \
  MAP('E', CODE_L1X, PHASE_E1); \
  MAP('E', CODE_L1Z, PHASE_E1); \
  MAP('E', CODE_L5I, PHASE_E5A); \
  MAP('E', CODE_L5Q, PHASE_E5A); \
  MAP('E', CODE_L5X, PHASE_E5A); \
  MAP('E', CODE_L7I, PHASE_E5B); \
  MAP('E', CODE_L7Q, PHASE_E5B); \
  MAP('E', CODE_L7X, PHASE_E5B); \
  MAP('E', CODE_L8I, PHASE_E5); \
  MAP('E', CODE_L8Q, PHASE_E5); \
  MAP('E', CODE_L8X, PHASE_E5); \
  MAP('E', CODE_L6A, PHASE_E6); \
  MAP('E', CODE_L6B, PHASE_E6); \
  MAP('E', CODE_L6C, PHASE_E6); \
  MAP('E', CODE_L6X, PHASE_E6); \
  MAP('E', CODE_L6Z, PHASE_E6); \
  MAP('C', CODE_L2I, PHASE_B1); \
  MAP('C', CODE_L2Q, PHASE_B1); \
  MAP('C', CODE_L2X, PHASE_B1); \
  MAP('C', CODE_L1D, PHASE_B1C); \
  MAP('C', CODE_L1P, PHASE_B1C); \
  MAP('C', CODE_L1X, PHASE_B1C); \
  MAP('C', CODE_L1S, PHASE_B1A); \
  MAP('C', CODE_L1L, PHASE_B1A); \
  MAP('C', CODE_L1Z, PHASE_B1A); \
  MAP('C', CODE_L5D, PHASE_B2A); \
  MAP('C', CODE_L5P, PHASE_B2A); \
  MAP('C', CODE_L5X, PHASE_B2A); \
  MAP('C', CODE_L7I, PHASE_B2); \
  MAP('C', CODE_L7Q, PHASE_B2); \
  MAP('C', CODE_L7X, PHASE_B2); \
  MAP('C', CODE_L7D, PHASE_B2B); \
  MAP('C', CODE_L7P, PHASE_B2B); \
  MAP('C', CODE_L7Z, PHASE_B2B); \
  MAP('C', CODE_L8D, PHASE_B2AB); \
  MAP('C', CODE_L8P, PHASE_B2AB); \
  MAP('C', CODE_L8X, PHASE_B2AB); \
  MAP('C', CODE_L6I, PHASE_B3); \
  MAP('C', CODE_L6Q, PHASE_B3); \
  MAP('C', CODE_L6X, PHASE_B3); \
  MAP('C', CODE_L6Z, PHASE_B3A);
  
// Phase channel to default code 
#define PHASE_CHANNEL_TO_DEFAULT_CODE \
  MAP('G', PHASE_L1, CODE_L1W); \
  MAP('G', PHASE_L2, CODE_L2W); \
  MAP('G', PHASE_L5, CODE_L5Q); \
  MAP('R', PHASE_G1, CODE_L1P); \
  MAP('R', PHASE_G1A, CODE_L4A); \
  MAP('R', PHASE_G2, CODE_L2P); \
  MAP('R', PHASE_G2A, CODE_L6A); \
  MAP('R', PHASE_G3, CODE_L3I); \
  MAP('E', PHASE_E1, CODE_L1C); \
  MAP('E', PHASE_E5A, CODE_L5Q); \
  MAP('E', PHASE_E5B, CODE_L7Q); \
  MAP('E', PHASE_E5, CODE_L8Q); \
  MAP('E', PHASE_E6, CODE_L6C); \
  MAP('C', PHASE_B1, CODE_L2I); \
  MAP('C', PHASE_B1C, CODE_L1P); \
  MAP('C', PHASE_B1A, CODE_L1L); \
  MAP('C', PHASE_B2A, CODE_L5P); \
  MAP('C', PHASE_B2, CODE_L7I); \
  MAP('C', PHASE_B2B, CODE_L7P); \
  MAP('C', PHASE_B2AB, CODE_L8P); \
  MAP('C', PHASE_B3, CODE_L6I); \
  MAP('C', PHASE_B3A, CODE_L6Z); 

// Phase channel to phase frequency maps
#define PHASE_CHANNEL_TO_FREQUENCY_MAPS \
  MAP('G', PHASE_L1, FREQ1); \
  MAP('G', PHASE_L2, FREQ2); \
  MAP('G', PHASE_L2, FREQ2); \
  MAP('G', PHASE_L5, FREQ5); \
  MAP('R', PHASE_G1, FREQ1_GLO); \
  MAP('R', PHASE_G1A, FREQ1a_GLO); \
  MAP('R', PHASE_G2, FREQ2_GLO); \
  MAP('R', PHASE_G2A, FREQ2a_GLO); \
  MAP('R', PHASE_G3, FREQ3_GLO); \
  MAP('E', PHASE_E1, FREQ1); \
  MAP('E', PHASE_E5A, FREQ5); \
  MAP('E', PHASE_E5B, FREQ7); \
  MAP('E', PHASE_E5, FREQ8); \
  MAP('E', PHASE_E6, FREQ6); \
  MAP('C', PHASE_B1, FREQ1_CMP); \
  MAP('C', PHASE_B1C, FREQ1); \
  MAP('C', PHASE_B1A, FREQ1); \
  MAP('C', PHASE_B2A, FREQ5); \
  MAP('C', PHASE_B2, FREQ2_CMP); \
  MAP('C', PHASE_B2B, FREQ2_CMP); \
  MAP('C', PHASE_B2AB, FREQ8); \
  MAP('C', PHASE_B3, FREQ3_CMP); \
  MAP('C', PHASE_B3A, FREQ3_CMP); 

// Phase options string to phase channel maps
#define PHASE_CHANNEL_TO_STR_MAPS \
  MAP('G', PHASE_L1, "L1"); \
  MAP('G', PHASE_L2, "L2"); \
  MAP('G', PHASE_L5, "L5"); \
  MAP('R', PHASE_G1, "G1"); \
  MAP('R', PHASE_G1A, "G1A"); \
  MAP('R', PHASE_G2, "G2"); \
  MAP('R', PHASE_G2A, "G2A"); \
  MAP('R', PHASE_G3, "G3"); \
  MAP('E', PHASE_E1, "E1"); \
  MAP('E', PHASE_E5A, "E5A"); \
  MAP('E', PHASE_E5B, "E5B"); \
  MAP('E', PHASE_E5, "E5"); \
  MAP('E', PHASE_E6, "E6"); \
  MAP('C', PHASE_B1, "B1"); \
  MAP('C', PHASE_B1C, "B1C"); \
  MAP('C', PHASE_B1A, "B1A"); \
  MAP('C', PHASE_B2A, "B2A"); \
  MAP('C', PHASE_B2, "B2"); \
  MAP('C', PHASE_B2B, "B2B"); \
  MAP('C', PHASE_B2AB, "B2AB"); \
  MAP('C', PHASE_B3, "B3"); \
  MAP('C', PHASE_B3A, "B3A"); 
  