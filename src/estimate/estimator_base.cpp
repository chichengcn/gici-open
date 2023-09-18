/**
* @Function: Base class of estimator
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/estimate/estimator_base.h"

#include "gici/gnss/gnss_parameter_blocks.h"

namespace gici {

// The default constructor
EstimatorBase::EstimatorBase(const EstimatorBaseOptions& options) :
  base_options_(options), graph_(std::make_shared<Graph>()),
  cauchy_loss_function_(new ceres::CauchyLoss(1)),
  huber_loss_function_(new ceres::HuberLoss(1)),
  marginalization_residual_id_(0), status_(EstimatorStatus::Initializing)
{
  marginalization_error_.reset(new MarginalizationError(*graph_.get()));
}

// The default destructor
EstimatorBase::~EstimatorBase()
{}

// Apply ceres optimization
void EstimatorBase::optimize()
{
  graph_->options.linear_solver_type = base_options_.solver_type;
  graph_->options.trust_region_strategy_type = base_options_.trust_region_strategy_type;
  graph_->options.num_threads = base_options_.num_threads;
  graph_->options.max_num_iterations = base_options_.max_iteration;
#ifdef NDEBUG
  graph_->options.max_solver_time_in_seconds = base_options_.max_solver_time;
#endif

  if (base_options_.verbose_output) {
    // graph_->options.minimizer_progress_to_stdout = true;
  }
  else {
    graph_->options.logging_type = ceres::LoggingType::SILENT;
    graph_->options.minimizer_progress_to_stdout = false;
  }

  // call solver
  graph_->solve();

  // update covariance 
  if (base_options_.compute_covariance && can_compute_covariance_) {
    updateCovariance(latestState());
  }
}

// Erase old marginalization item
bool EstimatorBase::eraseOldMarginalization()
{
  marginalization_parameter_ids_.clear();
  marginalization_keep_parameter_blocks_.clear();
  if (marginalization_error_ && marginalization_residual_id_)
  {
    bool ret = graph_->removeResidualBlock(marginalization_residual_id_);
    CHECK(ret) << "Could not remove marginalization error";
    marginalization_residual_id_ = 0;
    if (!ret) return false;
  }

  return true;
}

// Apply new marginalization
bool EstimatorBase::applyMarginalization()
{
  // Check if there are any residuals we forgot to add
  for (auto id : marginalization_parameter_ids_) {
    auto residuals = graph_->residuals(id.asInteger());
    for (auto residual : residuals) {
      marginalization_error_->addResidualBlock(residual.residual_block_id);
    }
  }

  // Apply marginalization
  std::vector<uint64_t> parameter_blocks_to_be_marginalized;
  for (auto margin_id : marginalization_parameter_ids_) {
    parameter_blocks_to_be_marginalized.push_back(margin_id.asInteger());
  }
  marginalization_error_->marginalizeOut(parameter_blocks_to_be_marginalized,
                                         marginalization_keep_parameter_blocks_);

  // update error computation
  if(parameter_blocks_to_be_marginalized.size() > 0) {
    marginalization_error_->updateErrorComputation();
  }                              

  // add the marginalization term again
  if(marginalization_error_->num_residuals()==0)
  {
    marginalization_error_.reset();
  }
  if (marginalization_error_)
  {
    std::vector<std::shared_ptr<ParameterBlock> > parameter_block_ptrs;
    marginalization_error_->getParameterBlockPtrs(parameter_block_ptrs);
    marginalization_residual_id_ = graph_->addResidualBlock(
          marginalization_error_, nullptr, parameter_block_ptrs);
    CHECK(marginalization_residual_id_)
        << "could not add marginalization error";
    if (!marginalization_residual_id_) return false;
  }

  return true;
}

// Get pose
Transformation EstimatorBase::getPoseEstimate()
{
  State& state = getLast(states_);
  if (!graph_->parameterBlockExists(state.id_in_graph.asInteger())) {
    return Transformation();
  }
  return getPoseEstimate(state);
}

// Get pose estimate at given timestmap
bool EstimatorBase::getPoseEstimateAt(
  const double timestamp, Transformation& T_WS)
{
  for (size_t i = 0; i < states_.size() - 1; i++) {
    if (checkEqual(timestamp, states_[i].timestamp)) {
      T_WS = getPoseEstimate(states_[i]);
      return true;
    }
  }
  return false;
}

// Get latest speed and bias
SpeedAndBias EstimatorBase::getSpeedAndBiasEstimate()
{
  State& state = getLast(states_);
  if (!graph_->parameterBlockExists(state.id_in_graph.asInteger())) {
    return SpeedAndBias::Zero();
  }
  return getSpeedAndBiasEstimate(state);
}

// Get speed and bias estimate at given timestmap
bool EstimatorBase::getSpeedAndBiasEstimateAt(
  const double timestamp, SpeedAndBias& speed_and_bias)
{
  for (size_t i = 0; i < states_.size() - 1; i++) {
    if (checkEqual(timestamp, states_[i].timestamp)) {
      speed_and_bias = getSpeedAndBiasEstimate(states_[i]);
      return true;
    }
  }
  return false;
}

// Get latest minimal covariance 
Eigen::Matrix<double, 15, 15> EstimatorBase::getCovariance()
{
  State& state = getLast(states_);
  if (!graph_->parameterBlockExists(state.id_in_graph.asInteger())) {
    return Eigen::Matrix<double, 15, 15>::Zero();
  }
  Eigen::Matrix<double, 15, 15> covariance = getCovariance(state);
  return covariance;
}

// Get covariance at given timestamp
bool EstimatorBase::getCovarianceAt(
  const double timestamp, Eigen::Matrix<double, 15, 15>& covariance)
{
  for (size_t i = 0; i < states_.size() - 1; i++) {
    if (checkEqual(timestamp, states_[i].timestamp)) {
      covariance = getCovariance(states_[i]);
      return true;
    }
  }
  return false;
}

// Get pose estimate at a given state
Transformation EstimatorBase::getPoseEstimate(const State& state)
{
  BackendId id = state.id_in_graph;
  if (!graph_->parameterBlockExists(id.asInteger())) {
    LOG(ERROR) << "Parameter block with id " << id << " does not exist!";
    return Transformation();
  }

  // GNSS position state
  if (id.type() == IdType::gPosition)
  {
    std::shared_ptr<PositionParameterBlock> block_ptr =
        std::static_pointer_cast<PositionParameterBlock>(
          graph_->parameterBlockPtr(id.asInteger()));
    CHECK(block_ptr != nullptr);
    Eigen::Vector3d p = block_ptr->estimate().head<3>();

    // transform to local coordinate
    if (!coordinate_) {
      LOG(ERROR) << "Coordinate not setted!";
      return Transformation();
    }
    Eigen::Vector3d p_enu = coordinate_->convert(p, GeoType::ECEF, GeoType::ENU);
    Transformation T_WS = Transformation(p_enu, Eigen::Quaterniond::Identity());
    convertStateAndCovarianceToBody(&T_WS, nullptr, nullptr);
    return T_WS;
  }
  // Pose state
  else
  {
    std::shared_ptr<PoseParameterBlock> block_ptr =
        std::static_pointer_cast<PoseParameterBlock>(
          graph_->parameterBlockPtr(id.asInteger()));
    CHECK(block_ptr != nullptr);
    Transformation T_WS = block_ptr->estimate();
    convertStateAndCovarianceToBody(&T_WS, nullptr, nullptr);
    return T_WS;
  }
}

// Get latest speed and bias at a given state
SpeedAndBias EstimatorBase::getSpeedAndBiasEstimate(const State& state)
{
  BackendId id = state.id_in_graph;
  if (!graph_->parameterBlockExists(id.asInteger())) {
    LOG(ERROR) << "Parameter block with id " << id << " does not exist!";
    return SpeedAndBias::Zero();
  }

  // GNSS position state
  if (id.type() == IdType::gPosition)
  {
    BackendId speed_id = changeIdType(id, IdType::gVelocity);

    // check if we have velocity estimate
    if (!graph_->parameterBlockExists(speed_id.asInteger())) {
      return SpeedAndBias::Zero();
    }

    std::shared_ptr<VelocityParameterBlock> block_ptr =
        std::static_pointer_cast<VelocityParameterBlock>(
          graph_->parameterBlockPtr(speed_id.asInteger()));
    CHECK(block_ptr != nullptr);
    Eigen::Vector3d v = block_ptr->estimate().head<3>();

    // transform to local coordinate
    if (!coordinate_) {
      LOG(ERROR) << "Coordinate not setted!";
      return SpeedAndBias::Zero();
    }
    Eigen::Vector3d v_enu = coordinate_->rotate(v, GeoType::ECEF, GeoType::ENU);
    SpeedAndBias speed_and_bias;
    speed_and_bias.setZero();
    speed_and_bias.head<3>() = v_enu;
    convertStateAndCovarianceToBody(nullptr, &speed_and_bias, nullptr);
    return speed_and_bias;
  }
  // IMU state
  else
  {
    BackendId speed_and_bias_id = changeIdType(id, IdType::ImuStates);
    std::shared_ptr<SpeedAndBiasParameterBlock> block_ptr =
        std::static_pointer_cast<SpeedAndBiasParameterBlock>(
          graph_->parameterBlockPtr(speed_and_bias_id.asInteger()));
    CHECK(block_ptr != nullptr);
    SpeedAndBias speed_and_bias = block_ptr->estimate();
    convertStateAndCovarianceToBody(nullptr, &speed_and_bias, nullptr);
    return speed_and_bias;
  }
}

// Get minimal covariance at a given state
Eigen::Matrix<double, 15, 15> EstimatorBase::getCovariance(const State& state)
{
  base_options_.compute_covariance = true;

  const double timestamp = state.timestamp;
  auto it = covariances_.find(timestamp);
  if (it == covariances_.end()) return Eigen::Matrix<double, 15, 15>::Identity() * 1.0e6;
  else return it->second;
}

// Compute and get minimal covariance at a given state
Eigen::Matrix<double, 15, 15> EstimatorBase::computeAndGetCovariance(const State& state)
{
  BackendId id = state.id_in_graph;
  Eigen::Matrix<double, 15, 15> full_covariance;

  // GNSS position state
  if (id.type() == IdType::gPosition)
  {
    if (!coordinate_) {
      LOG(ERROR) << "Coordinate not setted!";
      return Eigen::Matrix<double, 15, 15>::Zero();
    }

    std::vector<size_t> parameter_block_ids;
    parameter_block_ids.push_back(id.asInteger());

    // check if we have velocity estimate
    bool has_speed = false;
    BackendId speed_id = changeIdType(id, IdType::gVelocity);
    if (graph_->parameterBlockExists(speed_id.asInteger())) {
      parameter_block_ids.push_back(speed_id.asInteger());
      has_speed = true;
    }

    // compute covariance
    Eigen::MatrixXd covariance;
    if (!graph_->computeCovariance(parameter_block_ids, covariance)) {
      return Eigen::Matrix<double, 15, 15>::Identity() * 1.0e6;
    }

    // only position
    if (!has_speed) {
      CHECK_EQ(covariance.cols(), 3);
      Eigen::Matrix3d p_covariance = covariance.topLeftCorner(3, 3);

      // transform to local coordinate
      Eigen::Matrix3d p_covariance_enu = coordinate_->convertCovariance(
        p_covariance, GeoType::ECEF, GeoType::ENU);
      full_covariance.setZero();
      full_covariance.topLeftCorner(3, 3) = p_covariance_enu;
    }
    else {
      CHECK_EQ(covariance.cols(), 6);
      Eigen::Matrix3d rot = coordinate_->rotationMatrix(
        GeoType::ECEF, GeoType::ENU);
      Eigen::Matrix<double, 6, 6> rot_extend;
      rot_extend.setZero();
      rot_extend.topLeftCorner(3, 3) = rot;
      rot_extend.bottomRightCorner(3, 3) = rot;
      Eigen::Matrix<double, 6, 6> covariance_enu = 
        rot_extend * covariance * rot_extend.transpose();
      full_covariance.setZero();
      full_covariance.block<3, 3>(0, 0) = covariance_enu.block<3, 3>(0, 0);
      full_covariance.block<3, 3>(6, 6) = covariance_enu.block<3, 3>(3, 3);
      full_covariance.block<3, 3>(0, 6) = covariance_enu.block<3, 3>(0, 3);
      full_covariance.block<3, 3>(6, 0) = covariance_enu.block<3, 3>(3, 0);
    }
  }
  // IMU state
  else
  {
    std::vector<size_t> parameter_block_ids;
    parameter_block_ids.push_back(id.asInteger());
    BackendId speed_and_bias_id = changeIdType(id, IdType::ImuStates);
    parameter_block_ids.push_back(speed_and_bias_id.asInteger());

    // comppute covariance
    Eigen::MatrixXd covariance;
    if (!graph_->computeCovariance(parameter_block_ids, covariance)) {
      return Eigen::Matrix<double, 15, 15>::Identity() * 1.0e6;
    }
    CHECK_EQ(covariance.cols(), 16);

    // convert to minimal
    Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
    Eigen::Matrix<double, 7, 1> parameters;
    const Transformation T_WS = getPoseEstimate(state);
    parameters << T_WS.getPosition().x(), T_WS.getPosition().y(), 
      T_WS.getPosition().z(), T_WS.getRotation().x(), T_WS.getRotation().y(), 
      T_WS.getRotation().z(), T_WS.getRotation().w();
    PoseLocalParameterization::liftJacobian(parameters.data(), J_lift.data());
    Eigen::Matrix<double, 15, 16> J_lift_full;
    J_lift_full.setZero();
    J_lift_full.topLeftCorner(6, 7) = J_lift;
    J_lift_full.bottomRightCorner(9, 9).setIdentity();
    full_covariance = J_lift_full * covariance * J_lift_full.transpose();
  }

  convertStateAndCovarianceToBody(nullptr, nullptr, &full_covariance);
  return full_covariance;
}

// Update covariance storage
void EstimatorBase::updateCovariance(const State& state)
{
  // erase old covariances
  const double oldest_timestamp = oldestState().timestamp;
  for (auto it = covariances_.begin(); it != covariances_.end();) {
    if (it->first < oldest_timestamp) it = covariances_.erase(it);
    else it++;
  }

  // update covariances
  auto it = covariances_.find(state.timestamp);
  if (it != covariances_.end()) {
    it->second = computeAndGetCovariance(state);
  }
  else {
    covariances_.insert(
      std::make_pair(state.timestamp, computeAndGetCovariance(state)));
  }
}

}