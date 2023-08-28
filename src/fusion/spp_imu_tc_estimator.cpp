/**
* @Function: SPP/IMU tightly couple estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/spp_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"

namespace gici {

// The default constructor
SppImuTcEstimator::SppImuTcEstimator(
               const SppImuTcEstimatorOptions& options, 
               const GnssImuInitializerOptions& init_options, 
               const SppEstimatorOptions spp_options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const GnssLooseEstimatorBaseOptions& gnss_loose_base_options, 
               const ImuEstimatorBaseOptions& imu_base_options,
               const EstimatorBaseOptions& base_options) :
  tc_options_(options), spp_options_(spp_options),
  GnssEstimatorBase(gnss_base_options, base_options),
  ImuEstimatorBase(imu_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::SppImuTc;
  shiftMemory();

  // Initialization control
  initializer_sub_estimator_.reset(new SppEstimator(
    spp_options, gnss_base_options, base_options));
  initializer_.reset(new GnssImuInitializer(
    init_options, gnss_loose_base_options, imu_base_options, 
    base_options, graph_, initializer_sub_estimator_));
}

// The default destructor
SppImuTcEstimator::~SppImuTcEstimator()
{}

// Add measurement
bool SppImuTcEstimator::addMeasurement(const EstimatorDataCluster& measurement)
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
bool SppImuTcEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get prior states
  Eigen::Vector3d position_prior = coordinate_->convert(
    getPoseEstimate().getPosition(), GeoType::ENU, GeoType::ECEF);

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;

  // Correct code bias
  correctCodeBias(curGnss());

  // Compute ionosphere delays
  computeIonosphereDelay(curGnss(), true);

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
  addClockParameterBlocks(curGnss(), curGnss().id, num_valid_system);
  // frequency block
  int num_valid_doppler_system = 0;
  addFrequencyParameterBlocks(curGnss(), curGnss().id, num_valid_doppler_system);

  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(curGnss(), curState(), num_valid_satellite, true);
  
  // We do not need to check if the number of satellites is sufficient in tightly fusion.
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    // do nothing
  }
  num_satellites_ = num_valid_satellite;

  // No satellite
  if (num_satellites_ == 0) {
    // erase parameters in current state
    eraseClockParameterBlocks(curState());
    eraseFrequencyParameterBlocks(curState());
    eraseImuState(curState());
    return false;
  }

  // Add doppler residual blocks
  addDopplerResidualBlocks(curGnss(), curState(), num_valid_satellite, 
    true, getImuMeasurementNear(timestamp).angular_velocity);

  // Add relative errors
  if (!isFirstEpoch()) {
    // frequency
    addRelativeFrequencyResidualBlock(lastState(), curState());
  }

  // Car motion
  if (imu_base_options_.car_motion) {
    // heading measurement constraint
    addHMCResidualBlock(curState());
    // non-holonomic constraint
    addNHCResidualBlock(curState());
  }

  // Compute DOP
  updateGdop(curGnss());

  return true;
}

// Solve current graph
bool SppImuTcEstimator::estimate()
{
  // Optimize with FDE
  size_t n_pseudorange = numPseudorangeError(curState());
  size_t n_doppler = numDopplerError(curState());
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();

    // reject outlier
    if (!rejectPseudorangeOutlier(curState(),
        gnss_base_options_.reject_one_outlier_once) && 
        !rejectDopplerOutlier(curState(), 
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
  double ratio_doppler = n_doppler == 0.0 ? 0.0 : 1.0 - 
    getDivide(numDopplerError(curState()), n_doppler);
  const double thr = gnss_base_options_.diverge_max_reject_ratio;
  if (isGnssGoodObservation() && 
      (ratio_pseudorange > thr || ratio_doppler > thr)) {
    num_cotinuous_reject_gnss_++;
  }
  else num_cotinuous_reject_gnss_ = 0;
  if (num_cotinuous_reject_gnss_ > 
      gnss_base_options_.diverge_min_num_continuous_reject) {
    LOG(WARNING) << "Estimator diverge: Too many GNSS outliers rejected!";
    status_ = EstimatorStatus::Diverged;
    num_cotinuous_reject_gnss_ = 0;
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_;
  }

  // Apply marginalization
  marginalization();

  // Shift memory for states and measurements
  shiftMemory();

  return true;
}

// Set initializatin result
void SppImuTcEstimator::setInitializationResult(
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

  // Shift memory for states and measurements
  shiftMemory();

  // Set flags
  can_compute_covariance_ = true;
}

// Marginalization
bool SppImuTcEstimator::marginalization()
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
  // frequency
  addFrequencyMarginBlocksWithResiduals(oldestState());

  // Apply marginalization and add the item into graph
  return applyMarginalization();
}

};