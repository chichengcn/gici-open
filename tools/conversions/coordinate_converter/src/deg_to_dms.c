#include "rtklib.h"

int main(int argc, char ** argv)
{
	if (argc != 2) {
		return -1;
	} 
  double deg = atof(argv[1]);

  double dms[3];
  deg2dms(deg, dms, 7);
  printf("%4.0f %02.0f %08.5f\r\n", dms[0], dms[1], dms[2]);

  return 0;
}