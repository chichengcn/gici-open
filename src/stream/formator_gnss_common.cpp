/**
* @Function: Stream functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/formator.h"

#include <glog/logging.h>

#include "gici/gnss/gnss_common.h"

namespace gici {

namespace gnss_common {

// Realloc and add ephemeris
extern int add_eph(nav_t *nav, const eph_t *eph)
{
  eph_t *nav_eph;
  
  if (nav->nmax<=nav->n) {
    nav->nmax+=1024;
    if (!(nav_eph=(eph_t *)realloc(nav->eph,sizeof(eph_t)*nav->nmax))) {
      free(nav->eph); nav->eph=NULL; nav->n=nav->nmax=0;
      return 0;
    }
    nav->eph=nav_eph;
  }
  nav->eph[nav->n++]=*eph;
  return 1;
}

// Realloc and add glonass ephemeris
extern int add_geph(nav_t *nav, const geph_t *geph)
{
  geph_t *nav_geph;
  
  if (nav->ngmax<=nav->ng) {
    nav->ngmax+=1024;
    if (!(nav_geph=(geph_t *)realloc(nav->geph,sizeof(geph_t)*nav->ngmax))) {
      free(nav->geph); nav->geph=NULL; nav->ng=nav->ngmax=0;
      return 0;
    }
    nav->geph=nav_geph;
  }
  nav->geph[nav->ng++]=*geph;
  return 1;
}

// Compare ephemeris
static int cmpeph(const void *p1, const void *p2)
{
  eph_t *q1=(eph_t *)p1,*q2=(eph_t *)p2;
  return q1->ttr.time!=q2->ttr.time?(int)(q1->ttr.time-q2->ttr.time):
          (q1->toe.time!=q2->toe.time?(int)(q1->toe.time-q2->toe.time):
          q1->sat-q2->sat);
}

// Sort and unique ephemeris
extern void uniqeph(nav_t *nav)
{
  eph_t *nav_eph;
  int i,j;
  
  if (nav->n<=0) return;
  
  qsort(nav->eph,nav->n,sizeof(eph_t),cmpeph);
  
  for (i=1,j=0;i<nav->n;i++) {
    if (nav->eph[i].sat!=nav->eph[j].sat||
      nav->eph[i].iode!=nav->eph[j].iode) {
      nav->eph[++j]=nav->eph[i];
    }
  }
  nav->n=j+1;
  
  if (!(nav_eph=(eph_t *)realloc(nav->eph,sizeof(eph_t)*nav->n))) {
    free(nav->eph); nav->eph=NULL; nav->n=nav->nmax=0;
    return;
  }
  nav->eph=nav_eph;
  nav->nmax=nav->n;
}

// Compare glonass ephemeris
static int cmpgeph(const void *p1, const void *p2)
{
  geph_t *q1=(geph_t *)p1,*q2=(geph_t *)p2;
  return q1->tof.time!=q2->tof.time?(int)(q1->tof.time-q2->tof.time):
          (q1->toe.time!=q2->toe.time?(int)(q1->toe.time-q2->toe.time):
          q1->sat-q2->sat);
}

// Sort and unique glonass ephemeris 
extern void uniqgeph(nav_t *nav)
{
  geph_t *nav_geph;
  int i,j;
  
  if (nav->ng<=0) return;
  
  qsort(nav->geph,nav->ng,sizeof(geph_t),cmpgeph);
  
  for (i=j=0;i<nav->ng;i++) {
    if (nav->geph[i].sat!=nav->geph[j].sat||
      nav->geph[i].toe.time!=nav->geph[j].toe.time||
      nav->geph[i].svh!=nav->geph[j].svh) {
      nav->geph[++j]=nav->geph[i];
    }
  }
  nav->ng=j+1;
  
  if (!(nav_geph=(geph_t *)realloc(nav->geph,sizeof(geph_t)*nav->ng))) {
    free(nav->geph); nav->geph=NULL; nav->ng=nav->ngmax=0;
    return;
  }
  nav->geph=nav_geph;
  nav->ngmax=nav->ng;
}

// Update observation data
extern void updateObservation(
  obs_t *obs, std::shared_ptr<DataCluster::GNSS>& gnss_data)
{
  int n = 0;
  for (int i = 0; i < obs->n; i++) {
    gnss_data->observation[0].data[n++] = obs->data[i];
  }
  gnss_data->observation[0].n = n;
  sortobs(&gnss_data->observation[0]);
}

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, std::shared_ptr<DataCluster::GNSS>& gnss_data)
{
  int prn;
  if (satsys(sat, &prn) != SYS_GLO) {
    add_eph(gnss_data->ephemeris, nav->eph+sat-1);
  }
  else {
    add_geph(gnss_data->ephemeris, nav->geph+prn-1);
  }
}

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data)
{
  matcpy(gnss_data->ephemeris->utc_gps, nav->utc_gps,8,1);
  matcpy(gnss_data->ephemeris->utc_glo, nav->utc_glo,8,1);
  matcpy(gnss_data->ephemeris->utc_gal, nav->utc_gal,8,1);
  matcpy(gnss_data->ephemeris->utc_qzs, nav->utc_qzs,8,1);
  matcpy(gnss_data->ephemeris->utc_cmp, nav->utc_cmp,8,1);
  matcpy(gnss_data->ephemeris->utc_irn, nav->utc_irn,9,1);
  matcpy(gnss_data->ephemeris->utc_sbs, nav->utc_sbs,4,1);
  matcpy(gnss_data->ephemeris->ion_gps, nav->ion_gps,8,1);
  matcpy(gnss_data->ephemeris->ion_gal, nav->ion_gal,4,1);
  matcpy(gnss_data->ephemeris->ion_qzs, nav->ion_qzs,8,1);
  matcpy(gnss_data->ephemeris->ion_cmp, nav->ion_cmp,8,1);
  matcpy(gnss_data->ephemeris->ion_irn, nav->ion_irn,8,1);
}

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, std::shared_ptr<DataCluster::GNSS>& gnss_data)
{
  if (sta == NULL) {
    LOG(ERROR) << "Antenna position parameter has NULL pointer!";
    return;
  }
  for (int i = 0; i < 3; i++) {
    gnss_data->antenna->pos[i] = sta->pos[i];
  }
  double pos[3], del[3] = {0}, dr[3];
  ecef2pos(sta->pos, pos);
  if (sta->deltype == 1) { // ECEF
    del[2] = sta->hgt;
    enu2ecef(pos, del, dr);
    for (int i = 0; i < 3; i++) {
      gnss_data->antenna->pos[i] += sta->del[i] + dr[i];
    }
  }
  else {  // ENU
    enu2ecef(pos, sta->del, dr);
    for (int i = 0; i < 3; i++) {
      gnss_data->antenna->pos[i] += dr[i];
    }
  }
}

// Update ssr corrections
extern void updateSsr(
  ssr_t *ssr, std::shared_ptr<DataCluster::GNSS>& gnss_data,
  std::vector<UpdateSsrType> type, bool reset_ssr_status)
{
  if (ssr == NULL) {
    LOG(ERROR) << "SSR parameter has NULL pointer!";
    return;
  }
  for (int i = 0; i < MAXSAT; i++) 
  {
    if (!ssr[i].update) continue;
    if (reset_ssr_status) ssr[i].update = 0;
    
    // update ephemeris
    if (std::find(type.begin(), type.end(), 
        UpdateSsrType::Ephemeris) != type.end()) 
    {
      // check consistency between iods of orbit and clock
      if (ssr[i].iod[0] != ssr[i].iod[1]) continue;

      // common
      gnss_data->ephemeris->ssr[i].iode = ssr[i].iode;
      gnss_data->ephemeris->ssr[i].iodcrc = ssr[i].iodcrc;
      gnss_data->ephemeris->ssr[i].refd = ssr[i].refd;
      
      // ephemeris
      if (ssr[i].t0[0].time) {
        gnss_data->ephemeris->ssr[i].t0[0] = ssr[i].t0[0];
        gnss_data->ephemeris->ssr[i].udi[0] = ssr[i].udi[0];
        gnss_data->ephemeris->ssr[i].iod[0] = ssr[i].iod[0];
        for (int j = 0; j < 3; j++) {
          gnss_data->ephemeris->ssr[i].deph[j] = ssr[i].deph[j];
          gnss_data->ephemeris->ssr[i].ddeph[j] = ssr[i].ddeph[j];
        }
      }

      // clock
      if (ssr[i].t0[1].time) {
        gnss_data->ephemeris->ssr[i].t0[1] = ssr[i].t0[1];
        gnss_data->ephemeris->ssr[i].udi[1] = ssr[i].udi[1];
        gnss_data->ephemeris->ssr[i].iod[1] = ssr[i].iod[1];
        for (int j = 0; j < 3; j++) {
          gnss_data->ephemeris->ssr[i].dclk[j] = ssr[i].dclk[j];
        }
      }

      // high rate clock
      if (ssr[i].t0[2].time) {
        gnss_data->ephemeris->ssr[i].t0[2] = ssr[i].t0[2];
        gnss_data->ephemeris->ssr[i].udi[2] = ssr[i].udi[2];
        gnss_data->ephemeris->ssr[i].iod[2] = ssr[i].iod[2];
        gnss_data->ephemeris->ssr[i].hrclk = ssr[i].hrclk;
      }

      // ura
      if (ssr[i].t0[3].time) {
        gnss_data->ephemeris->ssr[i].t0[3] = ssr[i].t0[3];
        gnss_data->ephemeris->ssr[i].udi[3] = ssr[i].udi[3];
        gnss_data->ephemeris->ssr[i].iod[3] = ssr[i].iod[3];
        gnss_data->ephemeris->ssr[i].hrclk = ssr[i].hrclk;
        gnss_data->ephemeris->ssr[i].ura = ssr[i].ura;
      }
    }
    // update code bias
    if (std::find(type.begin(), type.end(), 
        UpdateSsrType::CodeBias) != type.end() && 
        ssr[i].t0[4].time) 
    {
      gnss_data->ephemeris->ssr[i].t0[4] = ssr[i].t0[4];
      gnss_data->ephemeris->ssr[i].udi[4] = ssr[i].udi[4];
      memcpy(gnss_data->ephemeris->ssr[i].cbias, 
            ssr[i].cbias, sizeof(float) * MAXCODE);
      gnss_data->ephemeris->ssr[i].isdcb = ssr[i].isdcb;
    }
    // update phase bias
    if (std::find(type.begin(), type.end(), 
        UpdateSsrType::PhaseBias) != type.end() && 
        ssr[i].t0[5].time) 
    {
      gnss_data->ephemeris->ssr[i].t0[5] = ssr[i].t0[5];
      gnss_data->ephemeris->ssr[i].udi[5] = ssr[i].udi[5];
      memcpy(gnss_data->ephemeris->ssr[i].pbias, 
            ssr[i].pbias, sizeof(double) * MAXCODE);
      memcpy(gnss_data->ephemeris->ssr[i].stdpb, 
            ssr[i].stdpb, sizeof(float) * MAXCODE);
      gnss_data->ephemeris->ssr[i].yaw_ang = ssr[i].yaw_ang;
      gnss_data->ephemeris->ssr[i].yaw_rate = ssr[i].yaw_rate;
      gnss_data->ephemeris->ssr[i].isdpb = ssr[i].isdpb;
    }

    if (type.size() > 0) {
      gnss_data->ephemeris->ssr[i].update = 1;
    }
  }
}

// Select data from GNSS stream
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data)
{
  GnssDataType type = static_cast<GnssDataType>(ret);
  // Observation data
  if (type == GnssDataType::Observation) {
    updateObservation(obs, gnss_data[iobs]);
  }
  // Ephemeris data
  else if (type == GnssDataType::Ephemeris) {
    updateEphemeris(nav, sat, gnss_data[0]);
  }
  // Ionosphere parameters
  else if (type == GnssDataType::IonAndUtcPara) {
    updateIonAndUTC(nav, gnss_data[0]);
  }
  // Antenna position parameters
  else if (type == GnssDataType::AntePos) {
    updateAntennaPosition(sta, gnss_data[0]);
  }
  // SSR (precise ephemeris, DCBs, etc..)
  else if (type == GnssDataType::SSR) {
    updateSsr(ssr, gnss_data[0]);
  }
}

}

}