/**
* @Function: Precise Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/ppp_estimator.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// The default constructor
PppEstimator::PppEstimator(const PppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  ppp_options_(options), 
  GnssEstimatorBase(gnss_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::Ppp;
  is_verbose_model_ = true;
  is_ppp_ = true;
  is_use_phase_ = true;
  if (options.estimate_velocity) has_velocity_estimate_ = true;
  can_compute_covariance_ = true;
  shiftMemory();

  // SPP estimator for setting initial states
  spp_estimator_.reset(new SppEstimator(gnss_base_options));

  // Phase wind-up control
  phase_windup_.reset(new PhaseWindup());

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));

  // Open intermediate data logging files
  if (base_options_.log_intermediate_data) {
    const std::string& directory = base_options_.log_intermediate_data_directory;
    createAmbiguityLogger(directory);
    createIonosphereLogger(directory);
    createPseudorangeResidualLogger(directory);
    createPhaserangeResidualLogger(directory);
  }
}

// The default destructor
PppEstimator::~PppEstimator()
{
  // Close intermediate data logging files
  if (base_options_.log_intermediate_data) {
    freeAmbiguityLogger();
    freeIonosphereLogger();
    freePseudorangeResidualLogger();
    freePhaserangeResidualLogger();
  }
}

// Add measurement
bool PppEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
    return addGnssMeasurementAndState(*measurement.gnss);
  }

  return false;
}

// Add GNSS measurements and state
bool PppEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get prior states
  if (!spp_estimator_->addGnssMeasurementAndState(measurement)) {
    return false;
  }
  if (!spp_estimator_->estimate()) {
    return false;
  }
  Eigen::Vector3d position_prior = spp_estimator_->getPositionEstimate();
  Eigen::Vector3d velocity_prior = spp_estimator_->getVelocityEstimate();
  std::map<char, double> clock_prior = spp_estimator_->getClockEstimate();
  std::map<char, double> frequency_prior = spp_estimator_->getFrequencyEstimate();
  curState().status = GnssSolutionStatus::Single;

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;
  curGnss().phase_windup = phase_windup_;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnss(), false);

  // Correct code bias
  correctCodeBias(curGnss(), false);

  // Correct phase bias
  if (curGnss().phase_bias->valid()) correctPhaseBias(curGnss());

  // Correct BDS satellite multipath
  correctBdsSatelliteMultipath(curGnss());

  // Compute ionosphere delays
  computeIonosphereDelay(curGnss());

  // Cycle-slip detection
  if (!isFirstEpoch()) {
    cycleSlipDetection(lastGnss(), curGnss(), gnss_base_options_.common);
  }
  
  // Add parameter blocks
  double timestamp = measurement.timestamp;
  curState().timestamp = timestamp;
  // position block
  BackendId position_id = addGnssPositionParameterBlock(curGnss().id, position_prior);
  curState().id = position_id;
  curState().id_in_graph = position_id;
  // clock block
  int num_valid_system = 0;
  addClockParameterBlocks(curGnss(), curGnss().id, num_valid_system, clock_prior);
  // troposphere block
  addTroposphereParameterBlock(curGnss().id);
  // ionosphere blocks
  addIonosphereParameterBlocks(curGnss(), curGnss().id, curIonosphereState());
  // ambiguity blocks
  addAmbiguityParameterBlocks(curGnss(), curGnss().id, curAmbiguityState());
  // inter-frequency bias (IFB) blocks
  addIfbParameterBlocks(curGnss(), curGnss().id);
  if (ppp_options_.estimate_velocity) {
    // velocity block
    addGnssVelocityParameterBlock(curGnss().id, velocity_prior);
    // frequency block
    int num_valid_doppler_system = 0;
    addFrequencyParameterBlocks(curGnss(), curGnss().id, 
      num_valid_doppler_system, frequency_prior);
  }
  
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(curGnss(), curState(), num_valid_satellite);

  // Check if insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    // erase parameters in current state
    eraseGnssPositionParameterBlock(curState());
    eraseClockParameterBlocks(curState());
    eraseTroposphereParameterBlock(curState());
    eraseIonosphereParameterBlocks(curIonosphereState());
    eraseAmbiguityParameterBlocks(curAmbiguityState());
    if (ppp_options_.estimate_velocity) {
      eraseGnssVelocityParameterBlock(curState());
      eraseFrequencyParameterBlocks(curState());
    }

    return false;
  }
  num_satellites_ = num_valid_satellite;

  // Add phaserange residual blocks
  addPhaserangeResidualBlocks(curGnss(), curState());

  // Add doppler residual blocks
  if (ppp_options_.estimate_velocity) {
    addDopplerResidualBlocks(curGnss(), curState(), num_valid_satellite);
  }

  // Add relative errors
  if (!isFirstEpoch()) {
    if (!ppp_options_.estimate_velocity) {
      // position
      addRelativePositionResidualBlock(lastState(), curState());
    }
    else {
      // position and velocity
      addRelativePositionAndVelocityBlock(lastState(), curState());
      // frequency
      addRelativeFrequencyBlock(lastState(), curState());
    }
    // troposphere
    addRelativeTroposphereResidualBlock(lastState(), curState());
    // ionosphere
    addRelativeIonosphereResidualBlock(
      lastIonosphereState(), curIonosphereState());
    // ambiguity
    addRelativeAmbiguityResidualBlock(
      lastGnss(), curGnss(), lastAmbiguityState(), curAmbiguityState());
  }

  return true;
}

// Solve current graph
bool PppEstimator::estimate()
{
  // Optimize with FDE
  size_t n_pseudorange = numPseudorangeError(curState());
  size_t n_phaserange = numPhaserangeError(curState());
  size_t n_doppler = numDopplerError(curState());
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();

    // reject outlier
    if (!rejectPseudorangeOutlier(curState(), curAmbiguityState(),
        gnss_base_options_.reject_one_outlier_once) && 
        !rejectDopplerOutlier(curState(), 
        gnss_base_options_.reject_one_outlier_once) && 
        !rejectPhaserangeOutlier(curState(), curAmbiguityState(),
        gnss_base_options_.reject_one_outlier_once)) break;
    if (!gnss_base_options_.reject_one_outlier_once) break;  
  }
  // Optimize without FDE
  else {
    optimize();
  }

  // Check if we rejected too many residuals
  double ratio_pseudorange = n_pseudorange == 0.0 ? 0.0 : 1.0 - 
    getDivide(numPseudorangeError(curState()), n_pseudorange);
  double ratio_phaserange = n_phaserange == 0.0 ? 0.0 : 1.0 - 
    getDivide(numPhaserangeError(curState()), n_phaserange);
  double ratio_doppler = n_doppler == 0.0 ? 0.0 : 1.0 - 
    getDivide(numDopplerError(curState()), n_doppler);
  const double thr = gnss_base_options_.diverge_max_reject_ratio;
  if (isGnssGoodObservation() && 
      (ratio_pseudorange > thr || ratio_phaserange > thr || ratio_doppler > thr)) {
    num_cotinuous_reject_gnss_++;
  }
  else num_cotinuous_reject_gnss_ = 0;
  if (num_cotinuous_reject_gnss_ > 
      gnss_base_options_.diverge_min_num_continuous_reject) {
    LOG(WARNING) << "Estimator diverge: Too many outliers rejected!";
    status_ = EstimatorStatus::Diverged;
    num_cotinuous_reject_gnss_ = 0;
  }

  // Ambiguity resolution
  curState().status = GnssSolutionStatus::Float;
  if (ppp_options_.use_ambiguity_resolution) {
    // get covariance of ambiguities
    Eigen::MatrixXd ambiguity_covariance;
    std::vector<uint64_t> parameter_block_ids;
    for (auto id : curAmbiguityState().ids) {
      parameter_block_ids.push_back(id.asInteger());
    }
    graph_->computeCovariance(parameter_block_ids, ambiguity_covariance);
    // solve
    AmbiguityResolution::Result ret = ambiguity_resolution_->solvePpp(
      curState().id, curAmbiguityState().ids, ambiguity_covariance, curGnss());
    if (ret == AmbiguityResolution::Result::NlFix) {
      curState().status = GnssSolutionStatus::Fixed;
    }
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_
      << ", Fix status: " << std::setw(1) << static_cast<int>(curState().status);
  }

  // Apply marginalization
  marginalization();

  // Shift memory for states and measurements
  shiftMemory();

  return true;
}

// Log intermediate data
void PppEstimator::logIntermediateData()
{
  if (!base_options_.log_intermediate_data) return;
  logAmbiguityEstimate();
  logIonosphereEstimate();
  logPseudorangeResidual();
  logPhaserangeResidual();
}

// Marginalization
bool PppEstimator::marginalization()
{
  // Check if we need marginalization
  if (states_.size() < ppp_options_.max_window_length) {
    return true;
  }

  // Erase old marginalization item
  if (!eraseOldMarginalization()) return false;

  // Add marginalization items
  // position
  addGnssPositionMarginBlockWithResiduals(oldestState());
  // clock
  addClockMarginBlocksWithResiduals(oldestState());
  // troposphere
  addTroposphereMarginBlockWithResiduals(oldestState());
  // ionosphere
  addIonosphereMarginBlocksWithResiduals(oldestIonosphereState());
  // ambiguity
  addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
  if (ppp_options_.estimate_velocity) {
    // velocity
    addGnssVelocityMarginBlockWithResiduals(oldestState());
    // frequency
    addFrequencyMarginBlocksWithResiduals(oldestState());
  }

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};