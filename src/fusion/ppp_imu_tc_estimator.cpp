/**
* @Function: PPP/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/ppp_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// The default constructor
PppImuTcEstimator::PppImuTcEstimator(
               const PppImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const PppEstimatorOptions ppp_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  tc_options_(options), ppp_options_(ppp_options),
  GnssEstimatorBase(gnss_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::PppImuTc;
  is_verbose_model_ = true;
  is_ppp_ = true;
  is_use_phase_ = true;
  shiftMemory();

  // SPP estimator for setting initial states
  SppEstimatorOptions spp_options;
  spp_options.use_dual_frequency = true;
  spp_estimator_.reset(new SppEstimator(spp_options, gnss_base_options));

  // Phase wind-up control
  phase_windup_.reset(new PhaseWindup());

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));

  // Initialization control
  initializer_sub_estimator_.reset(new PppEstimator(
    ppp_options, gnss_base_options, base_options, ambiguity_options));
  initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_, initializer_sub_estimator_));

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
PppImuTcEstimator::~PppImuTcEstimator()
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
bool PppImuTcEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  // Initialization
  if (coordinate_ == nullptr || !gravity_setted_) return false;
  if (!initializer_->finished()) {
    if (initializer_->getCoordinate() == nullptr) {
      initializer_->setCoordinate(coordinate_);
      initializer_sub_estimator_->setCoordinate(coordinate_);
      initializer_->setGravity(imu_base_options_.imu_parameters.g);
    }
    if (initializer_->addMeasurement(measurement)) {
      initializer_->estimate();
      // set result to estimator
      setInitializationResult(initializer_);
    }
    return false;
  }

  // Add IMU
  if (measurement.imu && measurement.imu_role == ImuRole::Major) {
    addImuMeasurement(*measurement.imu);
  }

  // Add GNSS
  if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
    return addGnssMeasurementAndState(*measurement.gnss);
  }

  return false;
}

// Add GNSS measurements and state
bool PppImuTcEstimator::addGnssMeasurementAndState(
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

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;
  curGnss().phase_windup = phase_windup_;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnss(), true);

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
  double timestamp = curGnss().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnss().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  CHECK(index == states_.size() - 1);
  curState().status = GnssSolutionStatus::Single;
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());
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
  // frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnss(), curGnss().id, 
    num_valid_doppler_system, frequency_prior);

  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(curGnss(), curState(), num_valid_satellite);
  
  // We do not need to check if the number of satellites is sufficient in tightly fusion.
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    // do nothing
  }
  num_satellites_ = num_valid_satellite;

  // No satellite
  if (num_satellites_ == 0) {
    // erase parameters in current state
    eraseImuState(curState());
    eraseClockParameterBlocks(curState());
    eraseTroposphereParameterBlock(curState());
    eraseIonosphereParameterBlocks(curIonosphereState());
    eraseAmbiguityParameterBlocks(curAmbiguityState());
    eraseFrequencyParameterBlocks(curState());

    return false;
  }

  // Add phaserange residual blocks
  addPhaserangeResidualBlocks(curGnss(), curState());

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnss(), curState(), num_valid_satellite);

  // Add position and velocity prior constraints
  if (isFirstEpoch()) {
    addGnssPositionResidualBlock(curState(), 
      position_prior, gnss_base_options_.error_parameter.initial_position);
    addGnssVelocityResidualBlock(curState(), 
      velocity_prior, gnss_base_options_.error_parameter.initial_velocity);
  }

  // Add relative errors
  if (!isFirstEpoch()) {
    // frequency
    addRelativeFrequencyResidualBlock(lastState(), curState());
    // isb
    // addRelativeIsbResidualBlock(lastState(), curState());
    // troposphere
    addRelativeTroposphereResidualBlock(lastState(), curState());
    // ionosphere
    addRelativeIonosphereResidualBlock(
      lastIonosphereState(), curIonosphereState());
    // ambiguity
    addRelativeAmbiguityResidualBlock(
      lastGnss(), curGnss(), lastAmbiguityState(), curAmbiguityState());
  }

  // ZUPT
  addZUPTResidualBlock(curState());

  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(curState());
    // non-holonomic constraint
    addNHCResidualBlock(curState());
  }

  return true;
}

// Solve current graph
bool PppImuTcEstimator::estimate()
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

  // Check if we rejected too many GNSS residuals
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
    LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
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

  // Check if we continuously cannot fix ambiguity, while we have good observations
  if (ppp_options_.use_ambiguity_resolution) {
    const double thr = gnss_base_options_.good_observation_max_reject_ratio;
    if (isGnssGoodObservation() && ratio_pseudorange < thr && 
        ratio_phaserange < thr && ratio_doppler < thr) {
      if (curState().status != GnssSolutionStatus::Fixed) num_continuous_unfix_++;
      else num_continuous_unfix_ = 0; 
    }
    else num_continuous_unfix_ = 0;
    if (num_continuous_unfix_ > 
        gnss_base_options_.reset_ambiguity_min_num_continuous_unfix) {
      LOG(INFO) << "Continuously unfix under good observations. Clear current ambiguities.";
      resetAmbiguityEstimation();
      num_continuous_unfix_ = 0;
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

// Set initializatin result
void PppImuTcEstimator::setInitializationResult(
  const std::shared_ptr<MultisensorInitializerBase>& initializer)
{
  CHECK(initializer->finished());

  // Cast to desired initializer
  std::shared_ptr<GnssImuInitializer> gnss_imu_initializer = 
    std::static_pointer_cast<GnssImuInitializer>(initializer);
  CHECK_NOTNULL(gnss_imu_initializer);
  
  // Arrange to window length
  ImuMeasurements imu_measurements;
  std::deque<GnssSolution> gnss_solution_measurements;
  gnss_imu_initializer->arrangeToEstimator(
    tc_options_.max_window_length, marginalization_error_, states_, 
    marginalization_residual_id_, gnss_extrinsics_id_, 
    gnss_solution_measurements, imu_measurements);
  for (auto it = imu_measurements.rbegin(); it != imu_measurements.rend(); it++) {
    imu_mutex_.lock();
    imu_measurements_.push_front(*it);
    imu_mutex_.unlock();
  }
  gnss_measurements_.resize(gnss_solution_measurements.size());
  ambiguity_states_.resize(states_.size());
  ionosphere_states_.resize(states_.size());

  // Shift memory for states and measurements
  shiftMemory();

  // Set flags
  can_compute_covariance_ = true;
}

// Marginalization
bool PppImuTcEstimator::marginalization()
{
  // Check if we need marginalization
  if (states_.size() < tc_options_.max_window_length) {
    return true;
  }

  // Erase old marginalization item
  if (!eraseOldMarginalization()) return false;

  // Add marginalization items
  // IMU states and residuals
  addImuStateMarginBlockWithResiduals(oldestState());
  // clock
  addClockMarginBlocksWithResiduals(oldestState());
  // troposphere
  addTroposphereMarginBlockWithResiduals(oldestState());
  // ionosphere
  addIonosphereMarginBlocksWithResiduals(oldestIonosphereState());
  // ambiguity
  addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
  // frequency
  addFrequencyMarginBlocksWithResiduals(oldestState());

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};