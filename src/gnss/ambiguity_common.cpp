/**
* @Function: Ambiguity common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/ambiguity_common.h"

#include "gici/gnss/gnss_common.h"

namespace gici {

#define LOG_CYCLE_SLIP 1

// Cycle slip detection
void cycleSlipDetection(GnssMeasurement& measurement_pre, 
                        GnssMeasurement& measurement_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre,
                        const Eigen::Vector3d position_cur)
{
  // Detect by LLI
  cycleSlipDetectionLLI(measurement_pre, measurement_cur);

  // Detect by MW
  cycleSlipDetectionMW(measurement_pre, measurement_cur, options.mw_slip_thres);

  // Detect by GF
  cycleSlipDetectionGF(measurement_pre, measurement_cur, options.gf_slip_thres);

  // Detect by relative position
  if (!(checkZero(position_pre) && checkZero(position_cur))) {
    // TODO
    // cycleSlipDetectionPosition(measurement_pre, measurement_cur,
    //     position_pre, position_cur, option_tools.?);
  }

  // Detect by time gap for single frequency
  cycleSlipDetectionTimeGap(measurement_pre, measurement_cur, options.period * 1.5);
}

// Cycle slip detection after single difference
void cycleSlipDetectionSD(GnssMeasurement& measurement_rov_pre, 
                        GnssMeasurement& measurement_ref_pre, 
                        GnssMeasurement& measurement_rov_cur,
                        GnssMeasurement& measurement_ref_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre,
                        const Eigen::Vector3d position_cur)
{
  // Detect by LLI
  cycleSlipDetectionLLI(measurement_rov_pre, measurement_rov_cur);
  cycleSlipDetectionLLI(measurement_ref_pre, measurement_ref_cur);

  // Apply single difference 
  GnssMeasurementSDIndexPairs pairs_pre = 
    gnss_common::formPhaserangeSDPair(measurement_rov_pre, measurement_ref_pre);
  GnssMeasurement measurement_sd_pre;
  measurement_sd_pre.timestamp = measurement_rov_pre.timestamp;
  for (size_t i = 0; i < pairs_pre.size(); i++) {
    Observation& observation_rov = measurement_rov_pre.getObs(pairs_pre[i].rov);
    Observation& observation_ref = measurement_ref_pre.getObs(pairs_pre[i].ref);
    Satellite& satellite_rov = measurement_rov_pre.getSat(pairs_pre[i].rov);
    double dpseudorange = observation_rov.pseudorange - observation_ref.pseudorange;
    double dphaserange = observation_rov.phaserange - observation_ref.phaserange;
    
    Observation observation_sd;
    observation_sd.pseudorange = dpseudorange;
    observation_sd.phaserange = dphaserange;
    observation_sd.wavelength = observation_rov.wavelength;
    observation_sd.LLI = 0;
    observation_sd.slip = false;
    observation_sd.SNR = observation_rov.SNR;
    observation_sd.raw_code = observation_rov.raw_code;

    // force insert here
    std::string prn = pairs_pre[i].rov.prn;
    int code_type = pairs_pre[i].rov.code_type;
    Satellite& satellite_sd_pre = measurement_sd_pre.satellites[prn];
    satellite_sd_pre.prn = satellite_rov.prn;
    satellite_sd_pre.sat_type = satellite_rov.sat_type;
    satellite_sd_pre.sat_position = satellite_rov.sat_position;
    satellite_sd_pre.sat_clock = satellite_rov.sat_clock;
    satellite_sd_pre.observations[code_type] = observation_sd;
    measurement_sd_pre.position.setZero();
  }

  GnssMeasurementSDIndexPairs pairs_cur = 
    gnss_common::formPhaserangeSDPair(measurement_rov_cur, measurement_ref_cur);
  GnssMeasurement measurement_sd_cur;
  measurement_sd_cur.timestamp = measurement_rov_cur.timestamp;
  for (size_t i = 0; i < pairs_cur.size(); i++) {
    Observation& observation_rov = measurement_rov_cur.getObs(pairs_cur[i].rov);
    Observation& observation_ref = measurement_ref_cur.getObs(pairs_cur[i].ref);
    Satellite& satellite_rov = measurement_rov_cur.getSat(pairs_cur[i].rov);
    double dpseudorange = observation_rov.pseudorange - observation_ref.pseudorange;
    double dphaserange = observation_rov.phaserange - observation_ref.phaserange;
    
    Observation observation_sd;
    observation_sd.pseudorange = dpseudorange;
    observation_sd.phaserange = dphaserange;
    observation_sd.wavelength = observation_rov.wavelength;
    observation_sd.LLI = 0;
    observation_sd.slip = false;
    observation_sd.SNR = observation_rov.SNR;
    observation_sd.raw_code = observation_rov.raw_code;

    std::string prn = pairs_cur[i].rov.prn;
    int code_type = pairs_cur[i].rov.code_type;
    Satellite& satellite_sd_cur = measurement_sd_cur.satellites[prn];
    satellite_sd_cur.prn = satellite_rov.prn;
    satellite_sd_cur.sat_type = satellite_rov.sat_type;
    satellite_sd_cur.sat_position = satellite_rov.sat_position;
    satellite_sd_cur.sat_clock = satellite_rov.sat_clock;
    satellite_sd_cur.observations[code_type] = observation_sd;
    measurement_sd_cur.position.setZero();
  }

  // Detect by GF
  cycleSlipDetectionGF(measurement_sd_pre, measurement_sd_cur, options.gf_sd_slip_thres);

  // Detect by time gap for single frequency
  cycleSlipDetectionTimeGap(measurement_sd_pre, measurement_sd_cur, options.period * 1.5);

  // Put slip flags
  for (size_t i = 0; i < pairs_cur.size(); i++) {
    Observation& observation_sd = measurement_sd_cur.getObs(pairs_cur[i].rov);
    Observation& observation_rov = measurement_rov_cur.getObs(pairs_cur[i].rov);
    Observation& observation_ref = measurement_ref_cur.getObs(pairs_cur[i].ref);
    observation_rov.slip |= observation_sd.slip;
    observation_ref.slip |= observation_sd.slip;
  }

  // Detect by relative position
  if (!(checkZero(position_pre) && checkZero(position_cur))) {
    // TODO
    // cycleSlipDetectionPosition(measurement_rov_pre, measurement_rov_cur,
    //     position_pre, position_cur, option_tools.?);
  }
}

// Cycle slip detection by Loss of Lock Indicator (LLI)
void cycleSlipDetectionLLI(GnssMeasurement& measurement_pre, 
                           GnssMeasurement& measurement_cur)
{
  for (auto& sat : measurement_cur.satellites) {
    for (auto& obs : sat.second.observations) {
      Observation& observation = obs.second;
      if (observation.LLI & 1) {
        observation.slip = true;
        continue;
      }

      // detect slip by parity unknown flag transition in LLI
      uint8_t LLI_cur = observation.LLI;
      auto it_sat = measurement_pre.satellites.find(sat.first);
      if (it_sat == measurement_pre.satellites.end()) continue;
      auto it_obs = it_sat->second.observations.find(obs.first);
      if (it_obs == it_sat->second.observations.end()) continue;
      uint8_t LLI_pre = it_obs->second.LLI;
      if (((LLI_pre & 2) && !(LLI_cur & 2)) || (!(LLI_pre & 2) && (LLI_cur & 2))) {
        observation.slip = true;
#if LOG_CYCLE_SLIP
        LOG(INFO) << "Detected cycle slip by LLI at " << sat.second.prn << ".";
#endif
      }
    }
  }
}

// Cycle slip detection by Melbourne-Wubbena (MW) combination
void cycleSlipDetectionMW(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold)
{
  GnssMeasurementSDIndexPairs pairs = 
    gnss_common::formPhaserangeSDPair(measurement_pre, measurement_cur);

  // Find valid frequencies for each satellite
  std::vector<std::vector<int>> pair_indexes;
  std::string last_prn = "";
  for (size_t i = 0; i < pairs.size(); i++) {
    std::string prn = pairs[i].rov.prn;
    if (prn != last_prn) {
      std::vector<int> indexes; indexes.push_back(i);
      pair_indexes.push_back(indexes);
      last_prn = prn;
    }
    else {
      pair_indexes[pair_indexes.size() - 1].push_back(i);
    }
  }

  // Form MW combinations and detect
  for (size_t i = 0; i < pair_indexes.size(); i++) {
    if (pair_indexes[i].size() < 2) continue;

    for (size_t j = 1; j < pair_indexes[i].size(); j++) {
      GnssMeasurementIndex index_pre_0 = pairs[pair_indexes[i][0]].rov;
      GnssMeasurementIndex index_pre_1 = pairs[pair_indexes[i][j]].rov;
      GnssMeasurementIndex index_cur_0 = pairs[pair_indexes[i][0]].ref;
      GnssMeasurementIndex index_cur_1 = pairs[pair_indexes[i][j]].ref;
      Observation& observation_pre_0 = measurement_pre.getObs(index_pre_0);
      Observation& observation_pre_1 = measurement_pre.getObs(index_pre_1);
      Observation& observation_cur_0 = measurement_cur.getObs(index_cur_0);
      Observation& observation_cur_1 = measurement_cur.getObs(index_cur_1);

      double mw_pre = gnss_common::combinationMW(observation_pre_0, observation_pre_1);
      double mw_cur = gnss_common::combinationMW(observation_cur_0, observation_cur_1);

      if (fabs(mw_pre - mw_cur) > threshold) {
        observation_cur_0.slip = true;
        observation_cur_1.slip = true;
#if LOG_CYCLE_SLIP
        LOG(INFO) << "Detected cycle slip by MW at " << 
          measurement_cur.getSat(index_cur_0).prn << ".";
#endif
      }
    }
  }
}

// Cycle slip detection by Geometry-Free (GF) combination
void cycleSlipDetectionGF(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold)
{
  GnssMeasurementSDIndexPairs pairs = 
    gnss_common::formPhaserangeSDPair(measurement_pre, measurement_cur);

  // Find valid frequencies for each satellite
  std::vector<std::vector<int>> pair_indexes;
  std::string last_prn = "";
  for (size_t i = 0; i < pairs.size(); i++) {
    std::string prn = pairs[i].rov.prn;
    if (prn != last_prn) {
      std::vector<int> indexes; indexes.push_back(i);
      pair_indexes.push_back(indexes);
      last_prn = prn;
    }
    else {
      pair_indexes[pair_indexes.size() - 1].push_back(i);
    }
  }

  // Form MW combinations and detect
  for (size_t i = 0; i < pair_indexes.size(); i++) {
    if (pair_indexes[i].size() < 2) continue;

    for (size_t j = 1; j < pair_indexes[i].size(); j++) {
      GnssMeasurementIndex index_pre_0 = pairs[pair_indexes[i][0]].rov;
      GnssMeasurementIndex index_pre_1 = pairs[pair_indexes[i][j]].rov;
      GnssMeasurementIndex index_cur_0 = pairs[pair_indexes[i][0]].ref;
      GnssMeasurementIndex index_cur_1 = pairs[pair_indexes[i][j]].ref;
      Observation& observation_pre_0 = measurement_pre.getObs(index_pre_0);
      Observation& observation_pre_1 = measurement_pre.getObs(index_pre_1);
      Observation& observation_cur_0 = measurement_cur.getObs(index_cur_0);
      Observation& observation_cur_1 = measurement_cur.getObs(index_cur_1);

      double gf_pre = gnss_common::combinationGF(observation_pre_0, observation_pre_1);
      double gf_cur = gnss_common::combinationGF(observation_cur_0, observation_cur_1);

      if (fabs(gf_pre - gf_cur) > threshold) {
        observation_cur_0.slip = true;
        observation_cur_1.slip = true;
#if LOG_CYCLE_SLIP
        LOG(INFO) << "Detected cycle slip by GF at " << 
          measurement_cur.getSat(index_cur_0).prn << ".";
#endif
      }
    }
  }
}

// Cycle slip detection by relative position
void cycleSlipDetectionPosition(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          const Eigen::Vector3d position_pre,
                          const Eigen::Vector3d position_cur,
                          double threshold)
{
  // TODO: Detect by relative position
  // how to fix clock jump?
  return;
}

// Cycle slip detection by time gap for single frequency receiver
void cycleSlipDetectionTimeGap(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double max_time_gap)
{
#ifndef NDEBUG
  return; // disable in debug mode
#endif

  if (measurement_cur.timestamp - measurement_pre.timestamp < max_time_gap) {
    return;
  }

  // Check if single frequency
  std::vector<bool> is_single_frequency;
  for (auto sat : measurement_cur.satellites) {
    auto& satellite = sat.second;
    int num_phases = 0;
    for (auto obs : satellite.observations) {
      if (gnss_common::checkObservationValid(measurement_cur, 
          GnssMeasurementIndex(sat.first, obs.first), 
          ObservationType::Phaserange)) {
        num_phases++;
      }
    }
    if (num_phases > 1) is_single_frequency.push_back(false);
    else is_single_frequency.push_back(true);
  }

  // set slip flag
  size_t index = 0;
  for (auto& sat : measurement_cur.satellites) {
    if (is_single_frequency[index]) {
      auto& satellite = sat.second;
      for (auto& obs : satellite.observations) {
        obs.second.slip = true;
#if LOG_CYCLE_SLIP
        LOG(INFO) << "Detected cycle slip by time-gap at " << sat.second.prn << ".";
#endif
      }
    }
    index++;
  }
}

// Compute initial ambiguity for single differenced measurements
double getInitialAmbiguitySD(const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssMeasurementIndex& index_rov,
                            const GnssMeasurementIndex& index_ref)
{
  auto& observation_1 = measurement_rov.satellites.at(index_rov.prn).
                        observations.at(index_rov.code_type);
  double pseudorange_1 = observation_1.pseudorange;
  double phaserange_1 = observation_1.phaserange;

  auto& observation_2 = measurement_ref.satellites.at(index_ref.prn).
                        observations.at(index_ref.code_type);
  double pseudorange_2 = observation_2.pseudorange;
  double phaserange_2 = observation_2.phaserange;

  return phaserange_1 - phaserange_2 - (pseudorange_1 - pseudorange_2);
}

// Solve integer ambiguity by LAMBDA
bool solveAmbiguityLambda(const Eigen::VectorXd& float_ambiguities,
                          const Eigen::MatrixXd& covariance, 
                          const double ratio_threshold, 
                          Eigen::VectorXd& fixed_ambiguities,
                          double& ratio)
{
  CHECK(float_ambiguities.size() == covariance.cols());
  CHECK(covariance.rows() == covariance.cols());

  const int length = float_ambiguities.size();
  fixed_ambiguities.resize(length);
  fixed_ambiguities.setZero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> 
    fixed_template = Eigen::MatrixXd::Zero(length, 2);
  Eigen::Vector2d residuals;
  // Lambda search
  lambda(float_ambiguities.size(), 2, float_ambiguities.data(), 
         covariance.data(), fixed_template.data(), residuals.data());
  ratio = residuals[0] > 0 ? (residuals[1] / residuals[0]) : 0.0;

  // Check ratio
  if (ratio > ratio_threshold) {
    fixed_ambiguities = fixed_template.leftCols(1);
    return true;
  }
  else return false;
}

}