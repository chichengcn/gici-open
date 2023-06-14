/**
* @Function: Align NMEA file with high rate to the low rate file
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "rtklib.h"
#include "nmea_formator.h"

// Align NMEA file with high rate to the low rate file
int main(int argc, char ** argv)
{
  char nmea_high_rate[1024], nmea_low_rate[1024];
	if (argc < 3) {
		return -1;
	} else if (argc == 3) {
		strcpy(nmea_high_rate, argv[1]);
    strcpy(nmea_low_rate, argv[2]);
	}
  
  // Load
  std::vector<NmeaEpoch> epochs_high, epochs_low;
  loadNmeaFile(nmea_high_rate, epochs_high);
  loadNmeaFile(nmea_low_rate, epochs_low);
  
  // check frequency
  double freq_high = (double)epochs_high.size() / timediff(epochs_high.back().sol.time, epochs_high.front().sol.time);
  double freq_low = (double)epochs_low.size() / timediff(epochs_low.back().sol.time, epochs_low.front().sol.time);
  if (freq_high < freq_low - 0.5) {
    printf("The frequency of the first input file should be larger than or equal to the sencond one!\r\n");
    printf("The frequencies of the two files are: %.3f and %.3f.\r\n", freq_high, freq_low);
    return -1;
  }

  // Interpolate
  std::vector<NmeaEpoch> epochs_high_aligned;
  int i = 0;
  for (int k = 0; k < epochs_low.size(); k++) {
    double time, time_next;
    time = (double)epochs_high[i].sol.time.time + epochs_high[i].sol.time.sec;
    if (i + 1 >= epochs_high.size()) {
      continue;
    }
    else {
      time_next = (double)epochs_high[i + 1].sol.time.time + epochs_high[i + 1].sol.time.sec;
    }
    double require_time = (double)epochs_low[k].sol.time.time + epochs_low[k].sol.time.sec;
    if (time > require_time) {
      continue;
    }
    else if (time <= require_time && time_next > require_time) {
      epochs_high_aligned.push_back(NmeaEpoch());
      sol_t *sol = &epochs_high_aligned.back().sol;
      esa_t *esa = &epochs_high_aligned.back().esa;
      *sol = epochs_high[i].sol;
      sol->time.time = (time_t)floor(require_time);
      sol->time.sec = require_time - floor(require_time);
      esa->time = sol->time;
      double interval = time_next - time;
      double dt = require_time - time;
      const double r = dt / interval;
      for (int m = 0; m < 6; m++) {
        sol->rr[m] = (1.0 - r) * epochs_high[i].sol.rr[m] + r * epochs_high[i + 1].sol.rr[m];
      }
      for (int m = 0; m < 3; m++) {
        esa->att[m] = (1.0 - r) * epochs_high[i].esa.att[m] + r * epochs_high[i + 1].esa.att[m];
        esa->vel[m] = (1.0 - r) * epochs_high[i].esa.vel[m] + r * epochs_high[i + 1].esa.vel[m];
      }
    }
    else {
      i++; k--;
    }
  }

  // Output
  char aligned_path[1034];
  sprintf(aligned_path, "%s.aligned", nmea_high_rate);
  writeNmeaFile(epochs_high_aligned, aligned_path);

  return 0;
}