/**
* @Function: Estimate pose extrinsics between two NMEA files
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "nmea_formator.h"
#include "gici/estimate/pose_parameter_block.h"
#include "gici/estimate/pose_error.h"
#include "gici/utility/transform.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/estimate/graph.h"
#include "gici/estimate/estimator_types.h"

using namespace gici;

// Estimate pose extrinsics between two NMEA files
// The frequency of the first input file should be no less than the second one.
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
  std::cout << "Loading files..." << std::endl;
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
  std::vector<Transformation> T_ABs;
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

      Eigen::Vector3d t_A_ECEF = Eigen::Map<Eigen::Vector3d>(sol->rr);
      Eigen::Vector3d t_B_ECEF = Eigen::Map<Eigen::Vector3d>(epochs_low[k].sol.rr);
      GeoCoordinate coordinate(t_B_ECEF, GeoType::ECEF);
      Eigen::Vector3d t_A = coordinate.convert(t_A_ECEF, GeoType::ECEF, GeoType::ENU);
      Eigen::Vector3d t_B = coordinate.convert(t_B_ECEF, GeoType::ECEF, GeoType::ENU);
      Eigen::Quaterniond q_WA = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(esa->att));
      Eigen::Quaterniond q_WB = eulerAngleToQuaternion(Eigen::Map<Eigen::Vector3d>(epochs_low[k].esa.att));
      Transformation T_WA(t_A, q_WA);
      Transformation T_WB(t_B, q_WB);
      Transformation T_AB = T_WA.inverse() * T_WB;
      T_ABs.push_back(T_AB);
    }
    else {
      i++; k--;
    }
  }

  // Estimate extrinsics
  std::cout << "Estimating..." << std::endl;
  std::unique_ptr<Graph> graph = std::make_unique<Graph>();
  // add pose parameter block
  BackendId id = createNFrameId(0);
  std::shared_ptr<PoseParameterBlock> pose_parameter_block = 
    std::make_shared<PoseParameterBlock>(T_ABs.front(), id.asInteger());
  CHECK(graph->addParameterBlock(pose_parameter_block, Graph::Pose6d));
  for (size_t i = 0; i < T_ABs.size(); i++) {
    Transformation& T_AB = T_ABs[i];
    // add pose residual block
    Eigen::Matrix<double, 6, 6> information;
    information.setIdentity();
    std::shared_ptr<PoseError> pose_error = 
      std::make_shared<PoseError>(T_AB, information);
    ceres::ResidualBlockId residual_id = 
      graph->addResidualBlock(pose_error, nullptr,
        graph->parameterBlockPtr(id.asInteger()));
  }
  // solve
  graph->options.linear_solver_type = ceres::DENSE_QR;
  graph->options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  graph->options.max_num_iterations = 100;
  graph->options.minimizer_progress_to_stdout = true;
  graph->solve();
  std::cout << graph->summary.BriefReport() << std::endl;


  // Get solution
  std::shared_ptr<PoseParameterBlock> block_ptr =
      std::static_pointer_cast<PoseParameterBlock>(
        graph->parameterBlockPtr(id.asInteger()));
  CHECK(block_ptr != nullptr);
  Transformation T_AB = block_ptr->estimate();
  std::cout << "T_AB = " << std::endl << T_AB << std::endl;
  std::cout << "t_AB_A = " << T_AB.getPosition().transpose() << std::endl;
  std::cout << "rot_AB = " << quaternionToEulerAngle(T_AB.getEigenQuaternion()).transpose() * R2D << std::endl;

  return 0;
}