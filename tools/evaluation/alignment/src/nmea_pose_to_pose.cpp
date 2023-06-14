/**
* @Function: Convert pose from A frame to B frame
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "rtklib.h"
#include "nmea_formator.h"
#include "gici/utility/transform.h"
#include "gici/utility/svo.h"

using namespace gici;

const Eigen::Vector3d t_AB_A(-0.00509546,    0.320372,   0.0669864);
const Eigen::Vector3d rot_AB(179.928, -0.55273, -179.265);

int main(int argc, char ** argv)
{
  char nmea_buf[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(nmea_buf, argv[1]);
	}

  std::vector<NmeaEpoch> epochs;
  loadNmeaFile(nmea_buf, epochs);

  Transformation T_AB(t_AB_A, eulerAngleToQuaternion(rot_AB * D2R));

  std::vector<NmeaEpoch> to_gnss_epochs;
  Eigen::Vector3d ref_ecef = Eigen::Vector3d::Zero();
  for (int i = 0; i < epochs.size(); i++)
  {
    NmeaEpoch& epoch = epochs[i];
    esa_t& esa = epoch.esa;
    sol_t& sol = epoch.sol;

    Eigen::Quaterniond q = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(esa.att));
    Eigen::Vector3d p_enu = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_ecef(sol.rr);
    if (ref_ecef == Eigen::Vector3d::Zero()) {
      ref_ecef = Eigen::Map<Eigen::Vector3d>(sol.rr);
    }
    double ref_position[3];
    ecef2pos(ref_ecef.data(), ref_position);
    Eigen::Vector3d dp = p_ecef - ref_ecef;
    ecef2enu(ref_position, dp.data(), p_enu.data());
    Transformation T_WA(p_enu, q);

    Transformation T_WB = T_WA * T_AB;
    p_enu = T_WB.getPosition();
    q = T_WB.getEigenQuaternion();

    enu2ecef(ref_position, p_enu.data(), dp.data());
    p_ecef = ref_ecef + dp;
    for (int i = 0; i < 3; i++) sol.rr[i] = p_ecef(i);
    
    Eigen::Map<Eigen::Vector3d>(esa.att) = quaternionToEulerAngle(q);

    NmeaEpoch to_gnss_epoch;
    to_gnss_epoch.sol = sol;
    to_gnss_epoch.esa.time = sol.time;
    for (int j = 0; j < 3; j++) {
      to_gnss_epoch.esa.vel[j] = 0.0;
      to_gnss_epoch.esa.att[j] = esa.att[j];
    }
    to_gnss_epochs.push_back(to_gnss_epoch);
  }

  char to_gnss_nmea_path[1040];
  sprintf(to_gnss_nmea_path, "%s.transformed", nmea_buf);
  writeNmeaFile(to_gnss_epochs, to_gnss_nmea_path);

  return 0;
}