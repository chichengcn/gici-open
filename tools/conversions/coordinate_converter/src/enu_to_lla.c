#include "rtklib.h"

// Input format: E N U LatRef LonRef AltRef (m and deg)
int main(int argc, char ** argv)
{
  double enu[3], lla_ref[3];
	if (argc != 7) {
		return -1;
	} 
  for (int i = 0; i < 3; i++) {
    enu[i] = atof(argv[i + 1]);
    lla_ref[i] = atof(argv[i + 4]);
  }

  lla_ref[0] *= D2R; lla_ref[1] *= D2R;
  double decef[3], ecef[3], ecef_ref[3], lla[3];
  enu2ecef(lla_ref, enu, decef);
  pos2ecef(lla_ref, ecef_ref);
  for (int i = 0; i < 3; i++) ecef[i] = ecef_ref[i] + decef[i];
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