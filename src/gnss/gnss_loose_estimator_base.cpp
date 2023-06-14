/**
* @Function: Base class for GNSS loose loosely coupling with other sensors
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/gnss_loose_estimator_base.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_const_errors.h"
#include "gici/gnss/position_error.h"
#include "gici/gnss/velocity_error.h"

namespace gici {

// The default constructor
GnssLooseEstimatorBase::GnssLooseEstimatorBase(
                    const GnssLooseEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options) :
  gnss_loose_base_options_(options), EstimatorBase(base_options)
{}

// The default destructor
GnssLooseEstimatorBase::~GnssLooseEstimatorBase()
{}

// Convert solution to GNSS solution
GnssSolution GnssLooseEstimatorBase::convertSolutionToGnssSolution(
  const Solution& solution, const SolutionRole& role)
{
  GnssSolution gnss_solution;
  static int32_t id_static = 0;

  gnss_solution.timestamp = solution.timestamp;
  gnss_solution.id = ++id_static;
  gnss_solution.position = solution.coordinate->convert(
    solution.pose.getPosition(), GeoType::ENU, GeoType::ECEF);
  Eigen::Matrix3d R_ecef_enu = solution.coordinate->rotationMatrix(
    GeoType::ENU, GeoType::ECEF);
  gnss_solution.velocity = R_ecef_enu * solution.speed_and_bias.segment<3>(0);
  Eigen::Matrix<double, 6, 6> R_ecef_enu_double;
  R_ecef_enu_double.setZero();
  R_ecef_enu_double.topLeftCorner(3, 3) = R_ecef_enu;
  R_ecef_enu_double.bottomRightCorner(3, 3) = R_ecef_enu;
  Eigen::Matrix<double, 6, 6> covariance_position_velocity;
  covariance_position_velocity.topLeftCorner(3, 3) = 
    solution.covariance.block<3, 3>(0, 0);
  covariance_position_velocity.bottomRightCorner(3, 3) = 
    solution.covariance.block<3, 3>(6, 6);
  covariance_position_velocity.topRightCorner(3, 3) = 
    solution.covariance.block<3, 3>(0, 6);
  covariance_position_velocity.bottomLeftCorner(3, 3) = 
    solution.covariance.block<3, 3>(6, 0);
  gnss_solution.covariance = R_ecef_enu_double * 
    covariance_position_velocity * R_ecef_enu_double.transpose();
  gnss_solution.status = solution.status;
  gnss_solution.num_satellites = solution.num_satellites;
  gnss_solution.differential_age = solution.differential_age;
  if (role == SolutionRole::Position || role == SolutionRole::PositionAndVelocity) {
    gnss_solution.has_position = true;
  }
  if (role == SolutionRole::Velocity || role == SolutionRole::PositionAndVelocity) {
    gnss_solution.has_velocity = true;
  }
  
  // adjust covariance
  // if (gnss_solution.status == GnssSolutionStatus::Float) {
  //   gnss_solution.covariance.topLeftCorner(3, 3) *= 1.0e2;
  // }

  return gnss_solution;
}

// Compute coarse velocity from relative position
bool GnssLooseEstimatorBase::computeCoarseVelocityFromPosition(
  const GnssSolution& last_gnss_solution,
  const GnssSolution& cur_gnss_solution, 
  Eigen::Vector3d& velocity)
{
  // avoid crazy jump
  if (last_gnss_solution.status != cur_gnss_solution.status) {
    return false;
  }
  double dt = cur_gnss_solution.timestamp - last_gnss_solution.timestamp;
  if (dt <= 0.0) return false;
  velocity = (cur_gnss_solution.position - last_gnss_solution.position) / dt;
  return true;
}

// Add GNSS extrinsics block to graph
BackendId GnssLooseEstimatorBase::addGnssExtrinsicsParameterBlock(
  const int32_t id, 
  const Eigen::Vector3d& t_SR_S_prior)
{
  BackendId pose_id = createGnssPoseId(id);
  BackendId gnss_extrinsics_id = changeIdType(pose_id, IdType::gExtrinsics);
  std::shared_ptr<PositionParameterBlock> gnss_extrinsic_parameter_block = 
    std::make_shared<PositionParameterBlock>(
      t_SR_S_prior, gnss_extrinsics_id.asInteger());
  CHECK(graph_->addParameterBlock(gnss_extrinsic_parameter_block));
  return gnss_extrinsics_id;
}

// Add position block to graph
void GnssLooseEstimatorBase::addGnssPositionResidualBlock(
  const GnssSolution& measurement,
  const State& state)
{
  CHECK(gnss_extrinsics_id_.valid());
  if (!measurement.has_position) return;

  std::shared_ptr<PositionError<7, 3>> position_error = 
    std::make_shared<PositionError<7, 3>>(measurement.position, 
    measurement.covariance.topLeftCorner(3, 3).inverse());
  position_error->setCoordinate(coordinate_);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(position_error, 
      nullptr,   
      graph_->parameterBlockPtr(state.id_in_graph.asInteger()),
      graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
}

// Add velocity block to graph
void GnssLooseEstimatorBase::addGnssVelocityResidualBlock(
  const GnssSolution& measurement,
  const State& state,
  const Eigen::Vector3d& angular_velocity)
{
  CHECK(state.id.type() == IdType::gPose);
  CHECK(gnss_extrinsics_id_.valid());
  if (!measurement.has_velocity) return;

  BackendId pose_id = state.id_in_graph;
  BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
  std::shared_ptr<VelocityError<7, 9, 3>> velocity_error = 
    std::make_shared<VelocityError<7, 9, 3>>(measurement.velocity, 
    measurement.covariance.bottomRightCorner(3, 3).inverse(), angular_velocity);
  velocity_error->setCoordinate(coordinate_);
  ceres::ResidualBlockId residual_id = 
    graph_->addResidualBlock(velocity_error, 
      nullptr,   
      graph_->parameterBlockPtr(pose_id.asInteger()),
      graph_->parameterBlockPtr(speed_and_bias_id.asInteger()), 
      graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
}

// Add initial GNSS extrinsics error
void GnssLooseEstimatorBase::addGnssExtrinsicsResidualBlock(
  const BackendId& gnss_extrinsics_id, 
  const Eigen::Vector3d& t_SR_S_prior, 
  const Eigen::Vector3d& std)
{
  // set the parameter as constant
  if (std[0] * std[1] * std[2] == 0.0) {
    graph_->setParameterBlockConstant(gnss_extrinsics_id.asInteger());
    return;
  }

  Eigen::Vector3d info;
  for (size_t i = 0; i < 3; i++) info(i) = square(1.0 / std(i));
  std::shared_ptr<PositionError<3>> extrinsic_error = 
    std::make_shared<PositionError<3>>(t_SR_S_prior, info.asDiagonal());
  graph_->addResidualBlock(extrinsic_error, nullptr,
    graph_->parameterBlockPtr(gnss_extrinsics_id.asInteger()));
}

// Add GNSS position residual block to marginalizer
void GnssLooseEstimatorBase::addGnssPositionResidualMarginBlock(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kPositionError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add GNSS velocity residual block to marginalizer
void GnssLooseEstimatorBase::addGnssVelocityResidualMarginBlock(const State& state)
{
  BackendId speed_and_bias_id = changeIdType(state.id_in_graph, IdType::ImuStates);
  CHECK(graph_->parameterBlockExists(speed_and_bias_id.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(speed_and_bias_id.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kVelocityError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add all GNSS loosely coupled residual blocks to marginalier
void GnssLooseEstimatorBase::addGnssLooseResidualMarginBlocks(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    if (residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kPositionError && 
        residuals[r].error_interface_ptr->typeInfo() 
        != ErrorType::kVelocityError) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Erase GNSS extrinsics
void GnssLooseEstimatorBase::eraseGnssExtrinsicsParameterBlock(BackendId& extrinsics_id)
{
  CHECK(extrinsics_id.type() == IdType::gExtrinsics);
  CHECK(graph_->parameterBlockExists(extrinsics_id.asInteger()));
  graph_->removeParameterBlock(extrinsics_id.asInteger());
  extrinsics_id = BackendId(0);
}

// Erase GNSS position and velocity residual block
void GnssLooseEstimatorBase::eraseGnssLooseResidualBlocks(const State& state)
{
  const BackendId& parameter_id = state.id_in_graph;

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (auto residual_block : residual_blocks) {
    if (residual_block.error_interface_ptr->typeInfo() == 
        ErrorType::kPositionError || 
        residual_block.error_interface_ptr->typeInfo() == 
        ErrorType::kVelocityError) {
      graph_->removeResidualBlock(residual_block.residual_block_id);
    }
  }
}

// Get extrinsics estimate
Eigen::Vector3d GnssLooseEstimatorBase::getGnssExtrinsicsEstimate()
{
  std::shared_ptr<PositionParameterBlock> block_ptr =
      std::static_pointer_cast<PositionParameterBlock>(
        graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
  CHECK(block_ptr != nullptr);
  Eigen::Vector3d p = block_ptr->estimate().head<3>();
  return p;
}

}