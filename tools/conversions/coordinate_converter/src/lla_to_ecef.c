#include "rtklib.h"

// Input should in deg
int main(int argc, char ** argv)
{
  double lla[3];
	if (argc != 4) {
		return -1;
	} 
  for (int i = 0; i < 3; i++) {
    lla[i] = atof(argv[i + 1]);
  }

  lla[0] *= D2R; lla[1] *= D2R;
  double ecef[3];
  pos2ecef(lla, ecef);
  printf("ECEF in meter: %.4lf %.4lf %.4f\r\n", ecef[0], ecef[1], ecef[2]);

  return 0;
}