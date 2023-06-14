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
  int week;
  double tow = time2gpst(gtime, &week);
  printf("GPS time: %d, %.6lf\r\n", week, tow);

  return 0;
}