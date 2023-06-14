/**
* @Function: Append ionosphere parameter at the front of the novatel format bin file
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "edit_timestamp_utility.h"

using namespace gici;

#define OEM4SYNC1       0xAA    /* oem7/6/4 message start sync code 1 */
#define OEM4SYNC2       0x44    /* oem7/6/4 message start sync code 2 */
#define OEM4SYNC3       0x12    /* oem7/6/4 message start sync code 3 */
#define OEM4HLEN        28      /* oem7/6/4 message header length (bytes) */
#define ID_IONUTC       8       /* oem7/6/4 iono and utc data */

inline void U2(uint16_t d, uint8_t *p) {memcpy(p, &d, 2);}
inline void U4(uint32_t d, uint8_t *p) {memcpy(p, &d, 4);}
inline void I4(int32_t  d, uint8_t *p) {memcpy(p, &d, 4);}
inline void R4(float    d, uint8_t *p) {memcpy(p, &d, 4);}
inline void R8(double   d, uint8_t *p) {memcpy(p, &d, 8);}

const double ionosphere_paraemter[] = {
  2.9802E-08,  7.4506E-09, -1.7881E-07,  0.0000E+00,
  1.3312E+05,  0.0000E+00 ,-2.6214E+05,  2.6214E+05};
const double duration = 5.0;  // seconds

int main(int argc, char** argv)
{
  // Get file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-bin-file>" << std::endl;
    return -1;
  }
  std::string file_path = argv[1];
  std::string out_folder, out_file_name;
  size_t split_point = 0;
  for (size_t i = 0; i < file_path.size(); i++) {
    if (file_path[i] == '/') split_point = i;
  }
  out_folder = file_path.substr(0, split_point);
  std::reverse(out_file_name.begin(), out_file_name.end());
  std::string out_file_path = out_folder + "/ionosphere_parameter.bin";
  
  // Open file
  file_t *in_file = NULL, *out_file = NULL;
  in_file = openfile(file_path.data(), FILE_MODE_INPUT);
  out_file = openfile(out_file_path.data(), FILE_MODE_OUTPUT);

  // Read, append, and write
  // write output tag header
  double start_timestamp;
  char tagh[TIMETAGH_LEN+1]="";
  sprintf(tagh,"TIMETAG RTKLIB %s",VER_RTKLIB);
  out_file->tick=out_file->tick_f=in_file->tick_f;
  memcpy(tagh+TIMETAGH_LEN-4,&out_file->tick_f,sizeof(out_file->tick_f));
  out_file->time=in_file->time;
  start_timestamp = gnss_common::gtimeToDouble(out_file->time);
  uint32_t time_time=(uint32_t)out_file->time.time;
  double time_sec=out_file->time.sec;
  fwrite(&tagh,1,TIMETAGH_LEN,out_file->fp_tag);
  fwrite(&time_time,1,sizeof(time_time),out_file->fp_tag);
  fwrite(&time_sec ,1,sizeof(time_sec ),out_file->fp_tag);
  fflush(out_file->fp_tag);

  // body
  uint32_t fpos_4B;
  uint64_t fpos_8B;
  long out_fpos;
  const int buf_length = 65535;
  uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * buf_length);
  gtime_t last_wrote_time = {0, 0.0};
  while (fread(&in_file->tick_n, sizeof(uint32_t), 1, in_file->fp_tag) && 
         fread((in_file->size_fpos==4)?(void *)&fpos_4B:(void *)&fpos_8B, 
         in_file->size_fpos, 1, in_file->fp_tag)) 
  {
    gtime_t time = timeadd(in_file->time, (double)in_file->tick_n * 1.0e-3);
    if (timediff(time, last_wrote_time) < duration) continue;
    last_wrote_time = time;

    // write ionosphere parameter
    int week;
    double tow = time2gpst(in_file->time, &week);
    int data_length = 108;
    memset(buf, 0, data_length + OEM4HLEN + 4);
    buf[0] = OEM4SYNC1;
    buf[1] = OEM4SYNC2;
    buf[2] = OEM4SYNC3;
    U2(ID_IONUTC, buf + 4);
    U2((uint16_t)data_length, buf + 8);
    U2((uint16_t)week, buf + 14);
    U4((uint32_t)(tow * 1000.0), buf + 16);
    for (int i = 0; i < 8; i++) {
      R8(ionosphere_paraemter[i], buf + OEM4HLEN + i * 8);
    }
    uint32_t crc = rtk_crc32(buf, data_length + OEM4HLEN);
    U4(crc, buf + data_length + OEM4HLEN);
    fwrite(buf, 1, data_length + OEM4HLEN + 4, out_file->fp);
    out_fpos = ftell(out_file->fp);
    fflush(out_file->fp);

    // write tag
    fwrite(&in_file->tick_n, 1, sizeof(uint32_t), out_file->fp_tag);
    if (out_file->size_fpos==4) {
        fpos_4B=(uint32_t)out_fpos;
        fwrite(&fpos_4B,1,sizeof(fpos_4B),out_file->fp_tag);
    }
    else {
        fpos_8B=(uint64_t)out_fpos;
        fwrite(&fpos_8B,1,sizeof(fpos_8B),out_file->fp_tag);
    }
    fflush(out_file->fp_tag);
  }

  free(buf);
  closefile(in_file);
  closefile(out_file);

  return 0;
}