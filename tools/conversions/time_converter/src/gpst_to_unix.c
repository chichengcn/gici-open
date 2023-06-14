#include "rtklib.h"

int main(int argc, char ** argv)
{
	if (argc != 3) {
		return -1;
	} 
  int week = atoi(argv[1]);
  double tow = atof(argv[2]);

  gtime_t time = gpst2time(week, tow);
  printf("Unix time: %.6lf\r\n", (double)time.time + time.sec);

  return 0;
}