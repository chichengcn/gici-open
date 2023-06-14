/**
* @Function: Shift replay tag
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "rtklib.h"

#define USE_4BIT_TAG 1
#define TIMETAGH_LEN 64          /* time tag file header length */

int main(int argc, char** argv)
{
  // Get tag file
  if (argc != 3) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-tag-file> <seconds-to-shift>" << std::endl;
    return -1;
  }
  std::string tag_file_path = argv[1];
  std::string modified_tag_file_path = tag_file_path + ".modified";
  double shift_seconds = -atof(argv[2]);

  // Process tag file
  FILE *fp_tag = fopen(tag_file_path.data(), "r");
  FILE *fp_modified_tag = fopen(modified_tag_file_path.data(), "w");
  // header
  double time_sec;
  uint32_t time_time, tick_f;
  gtime_t start_time;
  char tagh[TIMETAGH_LEN+1]="";
  if (fread(&tagh,TIMETAGH_LEN,1,fp_tag)==1&&
    fread(&time_time,sizeof(time_time),1,fp_tag)==1&&
    fread(&time_sec ,sizeof(time_sec ),1,fp_tag)==1) {
    memcpy(&tick_f,tagh+TIMETAGH_LEN-4,sizeof(tick_f));
    start_time.time=(time_t)time_time;
    start_time.sec =time_sec;

    tick_f += (int)(shift_seconds * 1e3);
    start_time = timeadd(start_time, shift_seconds);
    memcpy(tagh+TIMETAGH_LEN-4,&tick_f,sizeof(tick_f));
    time_time=(uint32_t)start_time.time;
    time_sec=start_time.sec;
    fwrite(&tagh,1,TIMETAGH_LEN,fp_modified_tag);
    fwrite(&time_time,1,sizeof(time_time),fp_modified_tag);
    fwrite(&time_sec ,1,sizeof(time_sec ),fp_modified_tag);
  }
  else return -1;
  // body
  while (1) 
  {
    uint32_t tick_n, tick, fpos_4B;
    uint64_t fpos_8B;
    int size_fpos = USE_4BIT_TAG ? 4 : 8;
    if (fread(&tick_n,sizeof(tick),1,fp_tag)<1||
      fread((USE_4BIT_TAG)?(void *)&fpos_4B:(void *)&fpos_8B,
            size_fpos,1,fp_tag)<1) {
      break;
    }
    
    // tick_n += (int)(shift_seconds * 1e3);  // do not need
    fwrite(&tick_n,sizeof(tick),1,fp_modified_tag);
    fwrite((USE_4BIT_TAG)?(void *)&fpos_4B:(void *)&fpos_8B,
            size_fpos,1,fp_modified_tag);
  }
  fclose(fp_tag);
  fclose(fp_modified_tag);

  return 0;
}