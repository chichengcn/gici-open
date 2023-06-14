/**
* @Function: Convert file from IE IMR to text
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <rtklib.h>
#include <iostream>
#include <string>
#include <vector>

// Imr IMU header
typedef struct
{
  char szHeader[8];
  int8_t bIsIntelOrMotorola;
  double dVersionNumber;
  int32_t bDeltaTheta;
  int32_t bDeltaVelocity;
  double dDataRateHz;
  double dGyroScaleFactor;
  double dAccelScaleFactor;
  int32_t iUtcOrGpsTime;
  int32_t iRcvTimeOrCorrTime;
  double dTimeTagBias;
  char szImuName[32];
  uint8_t reserved1[4];
  char szProgramName[32];
  int bLeverArmValid;
  int32_t lXoffset;
  int32_t lYoffset;
  int32_t lZoffset;
  int8_t Reserved[354];
} ImrImuHeader;

// Imr IMU data format
typedef struct {
  double tow;
  double omegaX;
  double omegaY;
  double omegaZ;
  double accX;
  double accY;
  double accZ;     
} ImrImuData;

// Imr data loading and synchronizing handle
typedef struct {
  int week;
  FILE* fp_imr;
  ImrImuHeader imu_header;
  std::vector<ImrImuData> imus;
} ImrDataLoader;

// Read IMU file header
int readImuHeader(ImrDataLoader* loader)
{
  char buffer[512];
  size_t bytesRead = fread(buffer, sizeof(char), 512, loader->fp_imr);
  ImrImuHeader* h = &loader->imu_header;

  // 0 = LE, 1 = BE
  memcpy(&h->bIsIntelOrMotorola, &buffer[8], 1);

  // 0 = Data to follow will be read as scaled angular rates
  // 1 = (default), data to follow will be read as delta thetas, meaning angular
  // increments(i.e.scale and multiply by dDataRateHz to get degrees / second)
  memcpy(&h->bDeltaTheta, &buffer[16], 4);

  // 0 = Data to follow will be read as scaled accelerations
  // 1 = (default), data to follow will be read as delta velocities, meaning
  // velocity increments (i.e. scale and multiply by dDataRateHz to get m/s2)
  memcpy(&h->bDeltaVelocity, &buffer[20], 4);

  // The data rate of the IMU in Hz. e.g. 0.01 second data rate is 100 Hz
  memcpy(&h->dDataRateHz, &buffer[25], 8);

  // If bDeltaTheta == 0, multiply the gyro measurements by this to get degrees/second
  // If bDeltaTheta == 1, multiply the gyro measurements by this to get degrees, then multiply
  // by dDataRateHz to get degrees/second
  memcpy(&h->dGyroScaleFactor, &buffer[33], 8);

  // If bDeltaVelocity == 0, multiply the accel measurements by this to get m/s2
  // If bDeltaVelocity == 1, multiply the accel measurements by this to get m/s, then multiply by
  // dDataRateHz to get m/s2
  memcpy(&h->dAccelScaleFactor, &buffer[41], 8);

  // Defines the time tags as GPS or UTC seconds of the week
  // 0 = Unknown, will default to GPS
  // 1 = Time tags are UTC seconds of week
  // 2 = Time tags are GPS seconds of week
  memcpy(&h->iUtcOrGpsTime, &buffer[49], 4);

  // Defines whether the time tags are on the nominal top of the second or are corrected for
  // receiver time bias
  // 0 = Unknown, will default to corrected time
  // 1 = Time tags are top of the second
  // 2 = Time tags are corrected for receiver clock bias
  memcpy(&h->iRcvTimeOrCorrTime, &buffer[53], 4);

  // If you have a known bias between your GPS and IMU time tags enter it here
  memcpy(&h->dTimeTagBias, &buffer[57], 8);

  return 1;
}

// Read IMU data
int readImuData(ImrDataLoader* loader)
{
	// Read new data
  unsigned char byte[1];
  size_t bytesRead;
  char data[32];
  int32_t ax = 0;
  int32_t ay = 0;
  int32_t az = 0;
  int32_t gx = 0;
  int32_t gy = 0;
  int32_t gz = 0;
  double tow = 0;
  ImrImuData imuData;
  const int week = loader->week;
  FILE* filp = loader->fp_imr;
  ImrImuHeader* h = &loader->imu_header;
	while (bytesRead = fread(data, sizeof(char), 32, filp))
	{
    loader->imus.push_back(ImrImuData());
    ImrImuData* imu = &loader->imus.back();

    memcpy(&tow, &data[0], 8); 
    memcpy(&gx, &data[8], 4);
    memcpy(&gy, &data[12], 4);
    memcpy(&gz, &data[16], 4);

    if (h->bDeltaTheta == 1) 
    {
      imuData.omegaX = (double)(gx) * h->dDataRateHz * h->dGyroScaleFactor * PI / 180;
      imuData.omegaY = (double)(gy) * h->dDataRateHz * h->dGyroScaleFactor * PI / 180;
      imuData.omegaZ = (double)(gz) * h->dDataRateHz * h->dGyroScaleFactor * PI / 180;
    }
    else 
    {
      imuData.omegaX = (double)(gx) * h->dGyroScaleFactor * PI / 180;
      imuData.omegaY = (double)(gy) * h->dGyroScaleFactor * PI / 180;
      imuData.omegaZ = (double)(gz) * h->dGyroScaleFactor * PI / 180;
    }

    memcpy(&ax, &data[20], 4);
    memcpy(&ay, &data[24], 4);
    memcpy(&az, &data[28], 4);

    if (h->bDeltaVelocity == 1)
    {
      imuData.accX = (double)(ax) * h->dDataRateHz * h->dAccelScaleFactor;
      imuData.accY = (double)(ay) * h->dDataRateHz * h->dAccelScaleFactor;
      imuData.accZ = (double)(az) * h->dDataRateHz * h->dAccelScaleFactor;
    }
    else
    {
      imuData.accX = (double)(ax) * h->dAccelScaleFactor;
      imuData.accY = (double)(ay) * h->dAccelScaleFactor;
      imuData.accZ = (double)(az) * h->dAccelScaleFactor;
    }
    imuData.tow = tow;

    *imu = imuData;
	}

  return loader->imus.size();
}

int main(int argc, char ** argv)
{
  char imr_path[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(imr_path, argv[1]);
	}

  ImrDataLoader imr_loader;

  imr_loader.fp_imr = fopen(imr_path, "r");
  char buf[1034];
  sprintf(buf, "%s.txt", imr_path);
  FILE *fp_text = fopen(buf, "w");

  // IMR header 
  readImuHeader(&imr_loader);
  
  // IMR body
  readImuData(&imr_loader);

  // Write text
  fprintf(fp_text, "Tow\tAcc-X\tAcc-Y\tAcc-Z\tGyro-X\tGyro-Y\tGyro-Z\t\r\n");
  for (size_t i = 0; i < imr_loader.imus.size(); i++) {
    ImrImuData& imu = imr_loader.imus[i];
    fprintf(fp_text, "%10.3f\t%12.8f\t%12.8f\t%12.8f\t%13.9f\t%13.9f\t%13.9f\r\n",
      imu.tow, imu.accX, imu.accY, imu.accZ, imu.omegaX, imu.omegaY, imu.omegaZ);
  }
  
  fclose(imr_loader.fp_imr);
  fclose(fp_text);

  return 0;
}