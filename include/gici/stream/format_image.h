/**
* @Function: Camera message functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#ifndef FORMAT_IMAGE_H
#define FORMAT_IMAGE_H

#include <stdint.h>

#include "gici/utility/rtklib_safe.h"

#ifdef __cplusplus
extern "C" {
#endif

// Image stream handle
typedef struct {
  gtime_t time;     /* message time */
  int width;      /* image width */ 
  int height;     /* image height */ 
  int step;       /* image step */
  int nmax;       /* max number of buffer */
  uint8_t *image;   /* image data */
  int nbyte;      /* number of bytes in message buffer */ 
  int len;      /* message length (bytes) */
  uint8_t *buff;    /* message buffer */
} img_t;

/* input image message from stream --------------------------------------------
* fetch next image message and input a message from byte stream
* args   : img_t *img     IO  image control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input image data)
* notes  : 
*      image message format:
*      +----------+-----------+--------------------+
*      | preamble |  length   |  data message  |
*      +----------+-----------+--------------------+
*      |<-- 16 -->|<-- 24 --->|<--- length x 8 --->|
*      
*-----------------------------------------------------------------------------*/
extern int input_image(img_t *img, uint8_t data);

/* input image message from v4l2 --------------------------------------------
* fetch image message
* args   : img_t *img     IO  image control struct
*      uint8_t *buf   I   stream buffer (full frame)
*      int n      I   length of buffer
* return : status (-1: error message, 0: no message, 1: input image data)
*-----------------------------------------------------------------------------*/
extern int input_image_v4l2(img_t *img, const uint8_t *buf, int n);

/* generate image message -----------------------------------------------------
* generate image message
* args   : img_t *img     IO  image control struct
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int gen_img(img_t *img);

/* Initialize image structure */
extern void init_img(img_t *img, int width, int height, int step);

/* Free image structure */
extern void free_img(img_t *img);

#ifdef __cplusplus
}
#endif


#endif