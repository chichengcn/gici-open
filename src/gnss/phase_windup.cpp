/**
* @Function: Phase wind-up handler
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/phase_windup.h"

#include "gici/utility/rtklib_safe.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// Get phase-windup
double PhaseWindup::get(const double time, const std::string& prn, 
    const Eigen::Vector3d& satellite_position,
    const Eigen::Vector3d& receiver_position)
{
  auto it_windup = windups_.find(prn);
  if (it_windup == windups_.end()) {
    it_windup = windups_.insert(std::make_pair(prn, 0.0)).first;
  }

  windupcorr(time, satellite_position.data(), 
    receiver_position.data(), &it_windup->second);
  return it_windup->second;
}

// Phase windup model
void PhaseWindup::windupcorr(const double time, 
  const double* rs, const double* rr, double* phw)
{
  double ek[3], exs[3], eys[3], ezs[3], ess[3], exr[3], eyr[3], eks[3], ekr[3], E[9];
  double dr[3], ds[3], drs[3], r[3], pos[3], rsun[3], cosp, ph, erpv[5] = { 0 };
  int i;
  gtime_t gtime = gnss_common::doubleToGtime(time);

  /* sun position in ecef */
  sunmoonpos(gtime, erpv, rsun, NULL, NULL);

  /* unit vector satellite to receiver */
  for (i = 0; i < 3; i++) r[i] = rr[i] - rs[i];
  if (!normv3(r, ek)) return;

  /* unit vectors of satellite antenna */
  for (i = 0; i < 3; i++) r[i] = -rs[i];
  if (!normv3(r, ezs)) return;
  for (i = 0; i < 3; i++) r[i] = rsun[i] - rs[i];
  if (!normv3(r, ess)) return;
  cross3(ezs, ess, r);
  if (!normv3(r, eys)) return;
  cross3(eys, ezs, exs);

  /* unit vectors of receiver antenna */
  ecef2pos(rr, pos);
  xyz2enu(pos, E);
  exr[0] = E[1]; exr[1] = E[4]; exr[2] = E[7]; /* x = north */
  eyr[0] = -E[0]; eyr[1] = -E[3]; eyr[2] = -E[6]; /* y = west  */

  /* phase windup effect */
  cross3(ek, eys, eks);
  cross3(ek, eyr, ekr);
  for (i = 0; i < 3; i++) {
      ds[i] = exs[i] - ek[i] * dot(ek, exs, 3) - eks[i];
      dr[i] = exr[i] - ek[i] * dot(ek, exr, 3) + ekr[i];
  }
  cosp = dot(ds, dr, 3) / norm(ds, 3) / norm(dr, 3);
  if (cosp < -1.0) cosp = -1.0;
  else if (cosp > 1.0) cosp = 1.0;
  ph = acos(cosp) / 2.0 / PI;
  cross3(ds, dr, drs);
  if (dot(ek, drs, 3) < 0.0) ph = -ph;

  *phw = ph + floor(*phw - ph + 0.5); /* in cycle */
}

}