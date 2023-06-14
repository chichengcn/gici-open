/**
* @Function: Correct IMU timestamp for unstable version GICI-board data
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "edit_timestamp_utility.h"

using namespace gici;

const double timestamp_shift = 0.0;

int main(int argc, char** argv)
{
  // Get file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-bin-file>" << std::endl;
    return -1;
  }
  std::string file_path = argv[1];
  std::string corrected_file_path = file_path + ".modified.bin";

  // Open files
  file_t *in_file = NULL, *out_file = NULL;
  in_file = openfile(file_path.data(), FILE_MODE_INPUT);
  out_file = openfile(corrected_file_path.data(), FILE_MODE_OUTPUT);

  // Read, correct timestamp, and write
  std::shared_ptr<FormatorBase> in_formator, out_formator;
  IMUPackFormator::Option formator_options;
  in_formator = makeFormator(formator_options);
  out_formator = makeFormator(formator_options);
  const int buf_length = 32768;
  uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * buf_length);
  std::vector<std::shared_ptr<DataCluster>> dataset;

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
  imu_t imu, out_imu;
  init_imu(&imu);
  init_imu(&out_imu);
  double last_timestamp = 0.0;
  int cnt = 0;
  while (fread(&in_file->tick_n, sizeof(uint32_t), 1, in_file->fp_tag) && 
         fread((in_file->size_fpos==4)?(void *)&fpos_4B:(void *)&fpos_8B, 
         in_file->size_fpos, 1, in_file->fp_tag)) 
  {
    in_file->fpos_n = (long)((in_file->size_fpos==4)?fpos_4B:fpos_8B);
    int n_read = in_file->fpos_n - ftell(in_file->fp);
    if (n_read > buf_length) {
      std::cerr << "WARN: Max buffer length exceeded!" << std::endl;
      n_read = buf_length;
    }
    if (n_read <= 0) continue;
    int n = (int)fread(buf, 1, n_read, in_file->fp);
    long out_fpos;
    for (int i = 0; i < n; i++) {
      // decode
      if (!input_imu(&imu, buf[i])) continue;
      cnt++;
      out_imu.time = imu.time;
      double timestamp = gnss_common::gtimeToDouble(imu.time);
      // if (cnt < 40000) continue;
      double corrected_timestamp = timestamp;
      // correct timestamp
      if (last_timestamp != 0.0 && fabs(timestamp - last_timestamp) > 0.5) {
        double dt = timestamp - last_timestamp;
        double dt_round = round(dt);
        if (fabs(dt - dt_round) > 0.1) {
          std::cout << "Detected non-integer timestamp error at " << std::fixed
            << timestamp << ". Last timestamp is " << last_timestamp << "." << std::endl;
        }
        corrected_timestamp = timestamp - dt_round;
        out_imu.time = gnss_common::doubleToGtime(corrected_timestamp);
        std::cout << "Corrected IMU timestamp from " << std::fixed 
          << timestamp << " to " << corrected_timestamp << "." << std::endl;
      }
      last_timestamp = corrected_timestamp;
      // write bin
      out_imu.time = timeadd(out_imu.time, timestamp_shift);
      memcpy(out_imu.acc, imu.acc, sizeof(double) * 3);
      memcpy(out_imu.gyro, imu.gyro, sizeof(double) * 3);
      CHECK(gen_imu(&out_imu));
      fwrite(out_imu.buff, 1, out_imu.nbyte, out_file->fp);
      out_fpos = ftell(out_file->fp);
      fflush(out_file->fp);
    }
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

  closefile(in_file);
  closefile(out_file);
  free_imu(&imu);
  free_imu(&out_imu);

  // Modify file names
  std::string cmd;
  cmd = "cp " + file_path + " " + file_path + ".store";
  system(cmd.data());
  cmd = "cp " + file_path + ".tag " + file_path + ".tag.store";
  system(cmd.data());
  cmd = "mv " + corrected_file_path + " " + file_path;
  system(cmd.data());
  cmd = "mv " + corrected_file_path + ".tag " + file_path + ".tag";
  system(cmd.data());

  return 0;
}