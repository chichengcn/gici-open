#include "rtklib.h"

int main(int argc, char ** argv)
{
  double ecef[3];
	if (argc != 4) {
		return -1;
	} 
  for (int i = 0; i < 3; i++) {
    ecef[i] = atof(argv[i + 1]);
  }

  double lla[3];
  ecef2pos(ecef, lla);
  double dms_lat[3], dms_lon[3];
  deg2dms(lla[0] * R2D, dms_lat, 7);
  deg2dms(lla[1] * R2D, dms_lon, 7);
  printf("LLA in rad: %14.10lf %14.10lf %10.4lf\r\n", lla[0], lla[1], lla[2]);
  printf("LLA in deg: %14.8lf %14.8lf %10.4lf\r\n", lla[0] * R2D, lla[1] * R2D, lla[2]);
  printf("LLA in deg dms: %4.0f %02.0f %08.5f  %4.0f %02.0f %08.5f  %10.4lf\r\n", 
    dms_lat[0], dms_lat[1], dms_lat[2], dms_lon[0], dms_lon[1], dms_lon[2],
    lla[2]);

  return 0;
}