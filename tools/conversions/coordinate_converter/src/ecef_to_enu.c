#include "rtklib.h"

// Input format: Lat Lon Alt LatRef LonRef AltRef (deg)
int main(int argc, char ** argv)
{
  double ecef[3], ecef_ref[3];
	if (argc != 7) {
		return -1;
	} 
  for (int i = 0; i < 3; i++) {
    ecef[i] = atof(argv[i + 1]);
    ecef_ref[i] = atof(argv[i + 4]);
  }

  double enu[3], lla_ref[3], decef[3];
  ecef2pos(ecef_ref, lla_ref);
  for (int i = 0; i < 3; i++) decef[i] = ecef[i] - ecef_ref[i];
  ecef2enu(lla_ref, decef, enu);
  
  printf("ENU in meter: %.4lf %.4lf %.4f\r\n", enu[0], enu[1], enu[2]);

  return 0;
}