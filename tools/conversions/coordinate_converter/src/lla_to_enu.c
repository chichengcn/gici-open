#include "rtklib.h"

// Input format: Lat Lon Alt LatRef LonRef AltRef (deg)
int main(int argc, char ** argv)
{
  double lla[3], lla_ref[3];
	if (argc != 7) {
		return -1;
	} 
  for (int i = 0; i < 3; i++) {
    lla[i] = atof(argv[i + 1]);
    lla_ref[i] = atof(argv[i + 4]);
  }

  lla[0] *= D2R; lla[1] *= D2R;
  lla_ref[0] *= D2R; lla_ref[1] *= D2R;
  double enu[3], ecef[3], ecef_ref[3], decef[3];
  pos2ecef(lla, ecef);
  pos2ecef(lla_ref, ecef_ref);
  for (int i = 0; i < 3; i++) decef[i] = ecef[i] - ecef_ref[i];
  ecef2enu(lla_ref, decef, enu);
  
  printf("ENU in meter: %.4lf %.4lf %.4f\r\n", enu[0], enu[1], enu[2]);

  return 0;
}