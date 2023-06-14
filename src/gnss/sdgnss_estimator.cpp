/**
* @Function: Single-differenced pseudorange positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/sdgnss_estimator.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
SdgnssEstimator::SdgnssEstimator(const SdgnssEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options) :
  sdgnss_options_(options), 
  GnssEstimatorBase(gnss_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::Sdgnss;
  can_compute_covariance_ = true;

  // SPP estimator for setting initial states
  spp_estimator_.reset(new SppEstimator(gnss_base_options));

  states_.push_back(State());
  gnss_measurement_pairs_.push_back(
    std::make_pair(GnssMeasurement(), GnssMeasurement()));
}

// The default destructor
SdgnssEstimator::~SdgnssEstimator()
{}

// Add measurement
bool SdgnssEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  GnssMeasurement rov, ref;
  meausrement_align_.add(measurement);
  if (meausrement_align_.get(sdgnss_options_.max_age, rov, ref)) {
    return addGnssMeasurementAndState(rov, ref);
  }

  return false;
}

// Add GNSS measurements and state
bool SdgnssEstimator::addGnssMeasurementAndState(
  const GnssMeasurement& measurement_rov, 
  const GnssMeasurement& measurement_ref)
{
  // Get prior states
  if (!spp_estimator_->addGnssMeasurementAndState(measurement_rov)) {
    return false;
  }
  if (!spp_estimator_->estimate()) {
    return false;
  }
  Eigen::Vector3d position_prior = spp_estimator_->getPositionEstimate();
  Eigen::Vector3d velocity_prior = spp_estimator_->getVelocityEstimate();
  std::map<char, double> frequency_prior = spp_estimator_->getFrequencyEstimate();
  curState().status = GnssSolutionStatus::Single;

  // Set to local measurement handle
  curGnssRov() = measurement_rov;
  curGnssRov().position = position_prior;
  curGnssRef() = measurement_ref;

  // Form single difference pair
  // select single difference pairs
  GnssMeasurementSDIndexPairs index_pairs = gnss_common::formPseudorangeSDPair(
    curGnssRov(), curGnssRef(), gnss_base_options_.common);

  // Add parameter blocks
  double timestamp = curGnssRov().timestamp;
  curState().timestamp = timestamp;
  // position block
  BackendId position_id = addGnssPositionParameterBlock(curGnssRov().id, position_prior);
  curState().id = position_id;
  curState().id_in_graph = position_id;
  // clock block
  int num_valid_system = 0;
  addSdClockParameterBlocks(curGnssRov(), curGnssRef(), 
    index_pairs, curGnssRov().id, num_valid_system);
  if (sdgnss_options_.estimate_velocity) {
    // velocity block
    addGnssVelocityParameterBlock(curGnssRov().id, velocity_prior);
    // frequency block
    int num_valid_doppler_system = 0;
    addFrequencyParameterBlocks(curGnssRov(), curGnssRov().id, 
      num_valid_doppler_system, frequency_prior);
  }
  
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addSdPseudorangeResidualBlocks(curGnssRov(), curGnssRef(), 
    index_pairs, curState(), num_valid_satellite);

  // Check if insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    return false;
  }
  num_satellites_ = num_valid_satellite;

  // Add doppler residual blocks
  if (sdgnss_options_.estimate_velocity) {
    addDopplerResidualBlocks(curGnssRov(), curState(), num_valid_satellite);
    if (checkSufficientSatellite(num_valid_satellite, num_valid_system, false)) {
      has_velocity_estimate_ = true;
    }
    else {
      has_velocity_estimate_ = false;
    }
  }

  // Erase all parameters in previous states
  Graph::ParameterBlockCollection parameters = graph_->parameters();
  for (auto parameter : parameters) {
    if (BackendId(parameter.first).bundleId() == curState().id.bundleId()) continue;
    graph_->removeParameterBlock(parameter.first);
  }

  return true;
}

// Solve current graph
bool SdgnssEstimator::estimate()
{
  // Optimize with FDE
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();
    // reject outlier
    if (!rejectPseudorangeOutlier(curState(),
        gnss_base_options_.reject_one_outlier_once) && 
        !rejectDopplerOutlier(curState(), 
        gnss_base_options_.reject_one_outlier_once)) break;
  }
  // Optimize without FDE
  else {
    optimize();
  }

  curState().status = GnssSolutionStatus::DGNSS;

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_;
  }

  // Shift memory
  states_.push_back(State());
  while (states_.size() > 2) states_.pop_front();

  return true;
}

};