/**
* @Function: Convert file from GICI IMU pack to IE IMR
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/format_imu.h"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  char imu_pack_path[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(imu_pack_path, argv[1]);
	}

  FILE *fp_imu_pack = fopen(imu_pack_path, "r");
  char buf[1034];
  sprintf(buf, "%s.imr", imu_pack_path);
  FILE *fp_imr = fopen(buf, "w");

  // Scan frequency, we need this parameter to fill in IMR header
  imu_t imu;
  init_imu(&imu);
  int imu_data_cnt = 0;
  double start_time = 0.0, end_time = 0.0;
  while (size_t n = fread(buf, 1, 1034, fp_imu_pack))
  {
    for (size_t i = 0; i < n; i++) {
      if (!(input_imu(&imu, buf[i]) == 1)) continue;
      end_time = static_cast<double>(imu.time.time) + imu.time.sec;
      if (start_time == 0.0) start_time = end_time;
      imu_data_cnt++;
    }
  }
  double time_range = end_time - start_time;
  double frequency = static_cast<double>(imu_data_cnt) / time_range;
  double min_delta_frequency = 1e6;
  double integer_frequecy = 0.0;
  for (int i = 0; i < 100; i++) {
    double f0 = 100.0 * static_cast<double>(i);
    double delta_frequency = fabs(frequency - f0);
    if (delta_frequency < min_delta_frequency) {
      integer_frequecy = f0;
      min_delta_frequency = delta_frequency;
    }
  }
  fclose(fp_imu_pack);
  fp_imu_pack = fopen(imu_pack_path, "r");

  // IMR header 
  // https://docs.novatel.com/Waypoint/Content/Data_Formats/IMR_File.htm
  const double acc_encode_factor = 1.0e-6;
  const double gyro_encode_factor = 1.0e-5;
  uint8_t head_buf[512];
  double double_tmp;
  int32_t int32_tmp;
  int idx = 0;
  sprintf((char *)head_buf, "$IMURAW"); idx += 8;
  setbits(head_buf, idx*8, 1*8, 0); idx += 1;
  double_tmp = 8.8;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  double_tmp = integer_frequecy;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  memcpy(head_buf + idx, &gyro_encode_factor, 8); idx += 8;
  memcpy(head_buf + idx, &acc_encode_factor, 8); idx += 8;
  int32_tmp = 1;
  memcpy(head_buf + idx, &int32_tmp, 4); idx += 4;  // use UTC time
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;  // Unknown, will default to corrected time
  double_tmp = 0.0;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  sprintf((char *)(head_buf + idx), "gici-imu"); idx += 32;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  sprintf((char *)(head_buf + idx), "gici"); idx += 32;
  memset(head_buf + idx, 0, 12); idx += 12;
  setbits(head_buf, idx*8, 1*8, 0); idx += 1;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4; 
  memset(head_buf + idx, 0, 354); idx += 354;
  fwrite(head_buf, sizeof(uint8_t), 512, fp_imr);
  
  // IMR body
  double last_tow = 0.0;
  uint8_t body_buf[32];
  while (size_t n = fread(buf, 1, 1034, fp_imu_pack))
  {
    for (size_t i = 0; i < n; i++) {
      if (!(input_imu(&imu, buf[i]) == 1)) continue;
      idx = 0;
      double tow = time2gpst(imu.time, NULL);
      if (last_tow != 0.0 && tow - last_tow > 0.1) {
        std::cout << "WARNING: Throughing IMU data at tow " << std::fixed << tow 
          << " because of time jumping! Last tow is " << last_tow << std::endl;
        continue;
      }
      if (last_tow != 0.0 && tow - last_tow == 0.0) {
        std::cout << "WARNING: Throughing IMU data at tow " << std::fixed << tow 
          << " because of time duplicating! Last tow is " << last_tow << std::endl;
        continue;
      }
      last_tow = tow;
      memcpy(body_buf + idx, &tow, 8); idx += 8;
      int32_tmp = round(imu.gyro[0] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.gyro[1] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.gyro[2] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[0] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[1] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[2] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      fwrite(body_buf, sizeof(uint8_t), 32, fp_imr);
    }
  }
  
  free_imu(&imu);
  fclose(fp_imu_pack);
  fclose(fp_imr);

  std::cout << "Converted " << imu_data_cnt 
    << " IMU data. Frequency is " << integer_frequecy << " Hz." << std::endl;

  return 0;
}