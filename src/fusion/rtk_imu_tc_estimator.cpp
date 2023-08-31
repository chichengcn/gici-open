/**
* @Function: RTK/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// The default constructor
RtkImuTcEstimator::RtkImuTcEstimator(
               const RtkImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const RtkEstimatorOptions rtk_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options,
               const AmbiguityResolutionOptions& ambiguity_options) :
  tc_options_(options), rtk_options_(rtk_options),
  GnssEstimatorBase(gnss_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::RtkImuTc;
  is_use_phase_ = true;
  shiftMemory();

  // Ambiguity resolution
  ambiguity_resolution_.reset(new AmbiguityResolution(ambiguity_options, graph_));

  // Initialization control
  initializer_sub_estimator_.reset(new RtkEstimator(
    rtk_options, gnss_base_options, base_options, ambiguity_options));
  initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_, initializer_sub_estimator_));
}

// The default destructor
RtkImuTcEstimator::~RtkImuTcEstimator()
{}

// Add measurement
bool RtkImuTcEstimator::addMeasurement(const EstimatorDataCluster& measurement)
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
  if (measurement.gnss) {
    GnssMeasurement rov, ref;
    meausrement_align_.add(measurement);
    if (meausrement_align_.get(rtk_options_.max_age, rov, ref)) {
      return addGnssMeasurementAndState(rov, ref);
    }
  }

  return false;
}

// Add GNSS measurements and state
bool RtkImuTcEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref)
{
  // Get prior states
  Eigen::Vector3d position_prior = coordinate_->convert(
    getPoseEstimate().getPosition(), GeoType::ENU, GeoType::ECEF);

  // Set to local measurement handle
  curGnssRov() = measurement_rov;
  curGnssRov().position = position_prior;
  curGnssRef() = measurement_ref;

  // Erase duplicated phases, arrange to one observation per phase
  gnss_common::rearrangePhasesAndCodes(curGnssRov());
  gnss_common::rearrangePhasesAndCodes(curGnssRef());

  // Form double difference pair
  GnssMeasurementDDIndexPairs code_index_pairs = gnss_common::formPseudorangeDDPair(
    curGnssRov(), curGnssRef(), gnss_base_options_.common);
  GnssMeasurementDDIndexPairs phase_index_pairs = gnss_common::formPhaserangeDDPair(
    curGnssRov(), curGnssRef(), gnss_base_options_.common);

  // Cycle-slip detection
  if (!isFirstEpoch()) {
    cycleSlipDetectionSD(lastGnssRov(), lastGnssRef(), 
      curGnssRov(), curGnssRef(), gnss_base_options_.common);
  }

  // Add parameter blocks
  double timestamp = curGnssRov().timestamp;
  // pose and speed and bias block
  const int32_t bundle_id = curGnssRov().id;
  BackendId pose_id = createGnssPoseId(bundle_id);
  size_t index = insertImuState(timestamp, pose_id);
  CHECK(index == states_.size() - 1);
  curState().status = GnssSolutionStatus::Single;
  // GNSS extrinsics, it should be added at initialization step
  CHECK(gnss_extrinsics_id_.valid());
  // ambiguity blocks
  addSdAmbiguityParameterBlocks(curGnssRov(), 
    curGnssRef(), phase_index_pairs, curGnssRov().id, curAmbiguityState());
  // frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnssRov(), curGnssRov().id, num_valid_doppler_system);

  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addDdPseudorangeResidualBlocks(curGnssRov(), 
    curGnssRef(), code_index_pairs, curState(), num_valid_satellite);
  
  // We do not need to check if the number of satellites is sufficient in tightly fusion.
  if (!checkSufficientSatellite(num_valid_satellite, 0)) {
    // do nothing
  }
  num_satellites_ = num_valid_satellite;

  // No satellite
  if (num_satellites_ == 0) {
    // erase parameters in current state
    eraseFrequencyParameterBlocks(curState());
    eraseImuState(curState());
    eraseAmbiguityParameterBlocks(curAmbiguityState());
    return false;
  }

  // Add phaserange residual blocks
  addDdPhaserangeResidualBlocks(
    curGnssRov(), curGnssRef(), phase_index_pairs, curState());

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnssRov(), curState(), num_valid_satellite, 
    false, getImuMeasurementNear(timestamp).angular_velocity);

  // Add relative errors
  if (!isFirstEpoch()) {
    // frequency
    addRelativeFrequencyResidualBlock(lastState(), curState());
    // ambiguity
    addRelativeAmbiguityResidualBlock(
      lastGnssRov(), curGnssRov(), lastAmbiguityState(), curAmbiguityState());
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

  // Compute DOP
  updateGdop(curGnssRov(), code_index_pairs);

  return true;
}

// Solve current graph
bool RtkImuTcEstimator::estimate()
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
  if (rtk_options_.use_ambiguity_resolution) {
    // get covariance of ambiguities
    Eigen::MatrixXd ambiguity_covariance;
    std::vector<uint64_t> parameter_block_ids;
    for (auto id : curAmbiguityState().ids) {
      parameter_block_ids.push_back(id.asInteger());
    }
    graph_->computeCovariance(parameter_block_ids, ambiguity_covariance);
    // solve
    AmbiguityResolution::Result ret = ambiguity_resolution_->solveRtk(
      curState().id, curAmbiguityState().ids, 
      ambiguity_covariance, gnss_measurement_pairs_.back());
    if (ret == AmbiguityResolution::Result::NlFix) {
      curState().status = GnssSolutionStatus::Fixed;
    }
  }

  // Check if we continuously cannot fix ambiguity, while we have good observations
  if (rtk_options_.use_ambiguity_resolution) {
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
void RtkImuTcEstimator::setInitializationResult(
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
  gnss_measurement_pairs_.resize(gnss_solution_measurements.size());
  ambiguity_states_.resize(states_.size());

  // Shift memory for states and measurements
  shiftMemory();

  // Set flags
  can_compute_covariance_ = true;
}

// Marginalization
bool RtkImuTcEstimator::marginalization()
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
  // ambiguity
  addAmbiguityMarginBlocksWithResiduals(oldestAmbiguityState());
  // frequency
  addFrequencyMarginBlocksWithResiduals(oldestState());

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};