/**
* @Function: Camera message functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/format_image.h"

/* image frame preamble */
const char img_preamb[] = { 0xFE, 0xCA, 0x00, 0xFF, 0x00, 0xFF };
/* image frame tail */
const char img_tail[] = { 0xCC, 0xFE, 0xFF, 0x00, 0xFF, 0x00 };

/* decode image message -------------------------------------------------*/
static int decode_image(img_t *img)
{
  gtime_t time;
  int i=72,j,width,height,step,sec;
  
  trace(3,"decode_image: len=%3d\n",img->len);

  time.time=getbitu(img->buff,i,32); i+=32;
  sec    =getbitu(img->buff,i,32); i+=32;
  width  =getbitu(img->buff,i,16); i+=16;
  height   =getbitu(img->buff,i,16); i+=16;
  step   =getbitu(img->buff,i,8); i+=8;
  time.sec=(double)sec*1.0e-9;
  if (width*height*step!=img->len-9-13-6) return -1;

  img->time=time;
  img->width=width;
  img->height=height;
  img->step=step;
  memcpy(img->image, img->buff+9+13, width*height*step);
  
  return 1;
}

/* encode image message -------------------------------------------------*/
extern int encode_img(img_t *img)
{
  int i=72,j,sec=img->time.sec*1.0e9;

  trace(3,"encode_img\n");

  if (img->width<=0||img->height<=0||img->step<=0||img->image==NULL) return 0;
  
  setbitu(img->buff,i,32,img->time.time); i+=32;
  setbitu(img->buff,i,32,sec       ); i+=32;
  setbitu(img->buff,i,16,img->width  ); i+=16;
  setbitu(img->buff,i,16,img->height   ); i+=16;
  setbitu(img->buff,i,8,img->step   ); i+=8;
  memcpy(img->buff+9+13, img->image, img->width*img->height*img->step);

  img->len=i/8+img->width*img->height*img->step;

  return 1;
}


/* input image message from stream --------------------------------------------
* fetch next image message and input a message from byte stream
* args   : img_t *img     IO  image control struct
*      uint8_t data   I   stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input image data)
* notes  : 
*      image message format:
*      +----------+-----------+--------------------+----------+
*      | preamble |  length   |   data message     |   tail   |
*      +----------+-----------+--------------------+----------+
*      |<-- 48 -->|<-- 24 --->|<--- length x 8 --->|<-- 48 -->|
*
*-----------------------------------------------------------------------------*/
extern int input_image(img_t *img, uint8_t data)
{
  trace(5,"input_image: data=%02x\n",data);
  
  /* synchronize frame */
  for (int i = 0; i < 6; i++) {
    if (img->nbyte == i) {
      if (data!=(uint8_t)(img_preamb[i])) {
        if (i > 0) img->nbyte = 0;
        return 0;
      }
      img->buff[img->nbyte++]=data;
      
      return 0;
    }
  }
  img->buff[img->nbyte++]=data;
  
  if (img->nbyte==9) {
    img->len=getbitu(img->buff,48,24)+9; /* length */
  }

  /* check tail */
  int is_end = 0;
  if (data == (uint8_t)(img_tail[5])) {
    is_end = 1;
    uint8_t *tail = img->buff + (img->nbyte - 6);
    for (int i = 0; i < 6; i++) {
      if (tail[i] != (uint8_t)(img_tail[i])) {
        is_end = 0; break;
      }
    }
  }

  /* bad package */
  if (is_end) {
    if (img->nbyte != img->len) {
      trace(3, "Invalid byte size: %d vs %d\r\n", img->nbyte, img->len);
      img->nbyte=0; return 0;
    }
  }
  if (img->nbyte >= img->nmax) {
    trace(3, "Exceeded byte size: %d vs %d\r\n", img->nbyte, img->nmax);
    img->nbyte = 0; return 0;
  }
  if (img->nbyte<9||img->nbyte<img->len) {
    return 0;
  }

  img->nbyte=0;
  
  /* decode image message */
  return decode_image(img);
}

/* input image message from v4l2 --------------------------------------------
* fetch image message
* args   : img_t *img     IO  image control struct
*      uint8_t *buf   I   stream buffer (full frame)
*      int n      I   length of buffer
* return : status (-1: error message, 0: no message, 1: input image data)
*-----------------------------------------------------------------------------*/
extern int input_image_v4l2(img_t *img, const uint8_t *buf, int n)
{
  int i;

  trace(4,"input_image_v4l2\n");

  if (buf==NULL) return 0;
  if (n<img->width*img->height) return -1;
  memcpy(img->image, buf, img->width*img->height);
  
  return 1;
}

/* generate image message -----------------------------------------------------
* generate image message
* args   : img_t *img     IO  image control struct
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int gen_img(img_t *img)
{
  int i=0;
  
  trace(4,"gen_img\n");
  
  img->nbyte=img->len=0;
  
  /* set preamble and reserved */
  for (int k=0;k<6;k++) {
    uint32_t a = (uint8_t)*(img_preamb+k);
    setbitu(img->buff,i,8,(uint8_t)*(img_preamb+k)); i+=8;
  }
  setbitu(img->buff,i,24,0    ); i+=24;
  
  /* encode image message body */
  if (!encode_img(img)) return 0;

  /* message tail */
  i=img->len*8;
  for (int k=0;k<6;k++) {
    setbitu(img->buff,i,8,(uint8_t)*(img_tail+k)); i+=8;
  }
  img->len+=6;

  /* message length without header */
  setbitu(img->buff,48,24,img->len-9);

  /* length total (bytes) */
  img->nbyte=img->len;
  
  return 1;
}

/* Initialize image structure */
extern void init_img(img_t *img, int width, int height, int step)
{
  gtime_t time0 = {0};
  int i;

  img->time = time0;
  img->width = width;
  img->height = height;
  img->step = step;
  int image_length = width * height * step;
  img->nmax = image_length + 512;
  img->len = 0;
  img->nbyte = 0;
  if (!(img->buff = (uint8_t*)malloc(sizeof(uint8_t)*img->nmax)) ||
    !(img->image = (uint8_t*)malloc(sizeof(uint8_t)*image_length))) {
    free_img(img);
    return;
  }
}

/* Free image structure */
extern void free_img(img_t *img)
{
  free(img->buff); img->buff = NULL;
  free(img->image); img->image = NULL;
}
