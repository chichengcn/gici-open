#include "rtklib.h"

int main(int argc, char ** argv)
{
	if (argc != 2) {
		return -1;
	} 
  double time = atof(argv[1]);

  gtime_t gtime;
  gtime.time = (int)floor(time);
  gtime.sec = time - floor(time);
  double ep[6];
  time2epoch(gtime, ep);
  printf("Epoch time: %04d-%02.0f-%02.0f %02.0f:%02.0f:%04.1f\r\n",
          (int)ep[0],ep[1],ep[2],ep[3],ep[4],ep[5]);

  return 0;
}