/**
* @Function: Convert file from GICI IMU pack to text
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
  sprintf(buf, "%s.txt", imu_pack_path);
  FILE *fp_text = fopen(buf, "w");

  // Write text
  fprintf(fp_text, "Timestamp\tAcc-X\tAcc-Y\tAcc-Z\tGyro-X\tGyro-Y\tGyro-Z\t\r\n");
  imu_t imu;
  init_imu(&imu);
  while (size_t n = fread(buf, 1, 1034, fp_imu_pack))
  {
    for (size_t i = 0; i < n; i++) {
      if (!(input_imu(&imu, buf[i]) == 1)) continue;
      fprintf(fp_text, "%13.3lf\t%12.8f\t%12.8f\t%12.8f\t%13.9f\t%13.9f\t%13.9f\r\n",
        (double)imu.time.time + imu.time.sec,
        imu.acc[0], imu.acc[1], imu.acc[2], imu.gyro[0], imu.gyro[1], imu.gyro[2]);
    }
  }
  
  free_imu(&imu);
  fclose(fp_imu_pack);
  fclose(fp_text);

  return 0;
}