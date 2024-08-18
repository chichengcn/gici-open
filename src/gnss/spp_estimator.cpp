/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/spp_estimator.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
SppEstimator::SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options) :
  spp_options_(options), 
  GnssEstimatorBase(gnss_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());
  can_compute_covariance_ = true;
}

SppEstimator::SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options) :
  spp_options_(options), 
  GnssEstimatorBase(gnss_base_options, EstimatorBaseOptions()),
  EstimatorBase(EstimatorBaseOptions())
{
  base_options_.max_iteration = 10;
  base_options_.max_solver_time = 0.05;
  base_options_.solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  base_options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  base_options_.verbose_output = false;

  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());
}

SppEstimator::SppEstimator(const GnssEstimatorBaseOptions& gnss_base_options) :
  spp_options_(SppEstimatorOptions()), 
  GnssEstimatorBase(gnss_base_options, EstimatorBaseOptions()),
  EstimatorBase(EstimatorBaseOptions())
{
  base_options_.max_iteration = 10;
  base_options_.max_solver_time = 0.05;
  base_options_.solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  base_options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  base_options_.verbose_output = false;

  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());
}

// The default destructor
SppEstimator::~SppEstimator()
{}

// Add measurement
bool SppEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
    return addGnssMeasurementAndState(*measurement.gnss);
  }

  return false;
}

// Add GNSS measurements and state
bool SppEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get prior position
  Eigen::Vector3d position_prior = Eigen::Vector3d::Zero();
  if (!isFirstEpoch()) {
    position_prior = getPositionEstimate();
  }
  else if (coordinate_) {
    position_prior = coordinate_->getZero(GeoType::ECEF);
  }
  else {
    // compute from ephemerides
    int num_sum_satellites = 0;
    Eigen::Vector3d sum_satellite_position = Eigen::Vector3d::Zero();
    for (auto& sat : measurement.satellites) {
      auto& satellite = sat.second;
      if (checkZero(satellite.sat_position)) continue;
      sum_satellite_position += satellite.sat_position;
      num_sum_satellites++;
    }
    Eigen::Vector3d avg_satellite_position = sum_satellite_position / 
      static_cast<double>(num_sum_satellites);
    Eigen::Vector3d avg_lla;
    ecef2pos(avg_satellite_position.data(), avg_lla.data());
    avg_lla(2) = 0.0;
    pos2ecef(avg_lla.data(), position_prior.data());
  }

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;

  // Erase non-base-frequency measurements and non-dual-frequency satellites
  if (spp_options_.use_dual_frequency) arrangeDualFrequency(curGnss());
  // Correct code bias for single frequency mode
  else correctCodeBias(curGnss());

  // Compute ionosphere delays
  computeIonosphereDelay(curGnss(), !spp_options_.use_dual_frequency);

  // Add parameter blocks
  double timestamp = measurement.timestamp;
  curState().timestamp = timestamp;
  // position block
  BackendId position_id = addGnssPositionParameterBlock(curGnss().id, position_prior);
  curState().id = position_id;
  curState().id_in_graph = position_id;
  // clock block
  int num_valid_system = 0;
  addClockParameterBlocks(curGnss(), curGnss().id, num_valid_system, 
    std::map<char, double>(), true);
  
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(
    curGnss(), curState(), num_valid_satellite, true);

  // Check if insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    return false;
  }

  num_satellites_ = num_valid_satellite;

  // Erase all parameters in previous states
  Graph::ParameterBlockCollection parameters = graph_->parameters();
  for (auto parameter : parameters) {
    if (BackendId(parameter.first).bundleId() == curState().id.bundleId()) continue;
    graph_->removeParameterBlock(parameter.first);
  }

  return true;
}

// Solve current graph
bool SppEstimator::estimate()
{
  status_ = EstimatorStatus::Converged;

  // Optimize with FDE
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();
    // reject outlier
    if (!rejectPseudorangeOutlier(curState(), 
        gnss_base_options_.reject_one_outlier_once)) break;
  }
  // Optimize without FDE
  else {
    optimize();
  }

  // Check DOP
  bool dop_valid = true;
  curGnss().position = getPositionEstimate(curState());
  if (!checkZero(curGnss().position)) {
    updateGdop(curGnss());
    if (gdop_ > gnss_base_options_.common.max_gdop) {
      LOG(INFO) << "High GDOP! Our tolerant is " 
        << gnss_base_options_.common.max_gdop << " in maximum, but current GDOP is "
        << gdop_ << "!";
      // erase parameters
      Graph::ParameterBlockCollection parameters = graph_->parameters();
      for (auto parameter : parameters) graph_->removeParameterBlock(parameter.first);
      states_.clear(); 
      dop_valid = false;
    }
  }

  // Check chi-square
  int n_residual = 0;
  int n_parameter = graph_->parameters().size();
  double chi_square = 0.0;
  auto residual_blocks = graph_->residuals();
  for (auto residual_block : residual_blocks) {
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPseudorangeError)) continue;
    double residual[1];
    n_residual++;
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    chi_square += square(residual[0]);
  }
  int chisqr_degree = n_residual - n_parameter - 1;
  if (chisqr_degree > 0 && chi_square > chisqr[n_residual - n_parameter - 1]) {
    LOG(INFO) << "High chi-square! Chi-square is " << chi_square
      << ". Chi-square threshold is " << chisqr[n_residual - n_parameter - 1] << ".";
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_      
      << ", GDOP: " << std::setprecision(1) << std::fixed << gdop_;
  }

  // Estimate velocity
  if (spp_options_.estimate_velocity && dop_valid) 
  {
    // Add parameter blocks
    int num_valid_doppler_system = 0;
    // velocity block
    addGnssVelocityParameterBlock(curGnss().id);
    // frequency block
    addFrequencyParameterBlocks(curGnss(), curGnss().id, num_valid_doppler_system, 
      std::map<char, double>(), true);

    // Add doppler residual blocks
    int num_valid_doppler_satellite = 0;
    addDopplerResidualBlocks(curGnss(), curState(), num_valid_doppler_satellite, true);
    if (!checkSufficientSatellite(
        num_valid_doppler_satellite, num_valid_doppler_system, false)) {
      eraseGnssVelocityParameterBlock(curState());
      eraseFrequencyParameterBlocks(curState());
      has_velocity_estimate_ = false;
    }
    else {
      if (!base_options_.compute_covariance) {
        // To speed-up the optimization, we do the following things:
        // erase all pseudorange residuals
        erasePseudorangeResidualBlocks(curState());
        // set position and clock parameters as constant
        graph_->setParameterBlockConstant(curState().id.asInteger());
        for (auto system : getGnssSystemList()) {
          BackendId clock_id = changeIdType(curState().id, IdType::gClock, system);
          if (graph_->parameterBlockExists(clock_id.asInteger())) {
            graph_->setParameterBlockConstant(clock_id.asInteger());
          }
        }
      }

      optimize();
      has_velocity_estimate_ = true;
    }
  }

  // Shift memory
  states_.push_back(State());
  while (states_.size() > 2) states_.pop_front();

  if (!dop_valid) status_ = EstimatorStatus::InvalidEpoch;
  return dop_valid;
}

// Erase non-base-frequency measurements and non-dual-frequency satellites
void SppEstimator::arrangeDualFrequency(GnssMeasurement& measurement)
{
  CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
  for (auto it_sat = measurement.satellites.begin(); 
       it_sat != measurement.satellites.end(); ) {
    char system = it_sat->first[0];
    std::pair<int, int> base_pair = bases.at(system);
    int phase_id_base_lhs = 
      gnss_common::getPhaseID(system, base_pair.first);
    int phase_id_base_rhs = 
      gnss_common::getPhaseID(system, base_pair.second);
    int valid_cnt = 0;
    for (auto it_obs = it_sat->second.observations.begin(); 
         it_obs != it_sat->second.observations.end(); ) {
      int phase_id = gnss_common::getPhaseID(system, it_obs->first);
      if ((phase_id != phase_id_base_lhs && phase_id != phase_id_base_rhs) || 
          !checkObservationValid(measurement, 
          GnssMeasurementIndex(it_sat->first, it_obs->first))) {
        it_obs = it_sat->second.observations.erase(it_obs);
      }
      else { valid_cnt++; it_obs++; }
    }
    if (valid_cnt != 2) {
      it_sat = measurement.satellites.erase(it_sat);
    }
    else it_sat++;
  }
}

// Get position estimate in ECEF
Eigen::Vector3d SppEstimator::getPositionEstimate(const State& state)
{
  std::shared_ptr<PositionParameterBlock> block_ptr =
      std::static_pointer_cast<PositionParameterBlock>(
        graph_->parameterBlockPtr(state.id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Get clock estimate
std::map<char, double> SppEstimator::getClockEstimate(const State& state)
{
  std::map<char, double> system_to_clock;
  for (auto system : getGnssSystemList()) {
    BackendId clock_id = changeIdType(state.id, IdType::gClock, system);
    if (!graph_->parameterBlockExists(clock_id.asInteger())) continue;
    std::shared_ptr<ClockParameterBlock> block_ptr =
        std::static_pointer_cast<ClockParameterBlock>(
          graph_->parameterBlockPtr(clock_id.asInteger()));
    CHECK(block_ptr != nullptr);
    system_to_clock.insert(std::make_pair(system, block_ptr->estimate()[0]));
  }
  return system_to_clock;
}

// Get velocity estimate in ECEF
Eigen::Vector3d SppEstimator::getVelocityEstimate(const State& state)
{
  if (!spp_options_.estimate_velocity) {
    return Eigen::Vector3d::Zero();
  }

  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (!graph_->parameterBlockExists(velocity_id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }
  std::shared_ptr<VelocityParameterBlock> block_ptr =
      std::static_pointer_cast<VelocityParameterBlock>(
        graph_->parameterBlockPtr(velocity_id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Get frequency estimate
std::map<char, double> SppEstimator::getFrequencyEstimate(const State& state)
{
  std::map<char, double> system_to_freq;
  if (!spp_options_.estimate_velocity) {
    return system_to_freq;
  }

  for (auto system : getGnssSystemList()) {
    BackendId freq_id = changeIdType(state.id, IdType::gFrequency, system);
    if (!graph_->parameterBlockExists(freq_id.asInteger())) continue;
    std::shared_ptr<FrequencyParameterBlock> block_ptr =
        std::static_pointer_cast<FrequencyParameterBlock>(
          graph_->parameterBlockPtr(freq_id.asInteger()));
    CHECK(block_ptr != nullptr);
    system_to_freq.insert(std::make_pair(system, block_ptr->estimate()[0]));
  }
  return system_to_freq;
}

// Get velocity covariance in ECEF
Eigen::Matrix3d SppEstimator::getVelocityCovariance(const State& state)
{
  if (!spp_options_.estimate_velocity) {
    return Eigen::Matrix3d::Zero();
  }

  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (!graph_->parameterBlockExists(velocity_id.asInteger())) {
    return Eigen::Matrix3d::Zero();
  }
  std::vector<size_t> parameter_block_ids;
  parameter_block_ids.push_back(velocity_id.asInteger());

  // compute covariance
  Eigen::MatrixXd covariance;
  CHECK(graph_->computeCovariance(parameter_block_ids, covariance));
  CHECK_EQ(covariance.cols(), 3);
  return covariance;
}

};