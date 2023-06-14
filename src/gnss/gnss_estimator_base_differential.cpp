/**
* @Function: Base class for GNSS estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/gnss_estimator_base.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_const_errors.h"
#include "gici/gnss/pseudorange_error_sd.h"
#include "gici/gnss/phaserange_error_sd.h"
#include "gici/gnss/pseudorange_error_dd.h"
#include "gici/gnss/phaserange_error_dd.h"

namespace gici {

// Note that we do not need to check the observation validity in all the functions below 
// because we have already done this during forming the SD or DD pairs.

// Add clock blocks to graph for single differenced measurements
void GnssEstimatorBase::addSdClockParameterBlocks(
  const GnssMeasurement& measurement_rov, 
  const GnssMeasurement& measurement_ref, 
  const GnssMeasurementSDIndexPairs& index_pairs,
  const int32_t id, 
  int& num_valid_system,
  const std::map<char, double>& prior)
{
  // Add clock parameter for each system
  for (size_t i = 0; i < getGnssSystemList().size(); i++) 
  {
    const char system = getGnssSystemList()[i];
    BackendId clock_id = createGnssClockId(system, id);
    if (gnss_common::useSystem(gnss_base_options_.common, system) && 
        !graph_->parameterBlockExists(clock_id.asInteger())) 
    {
      Eigen::Matrix<double, 1, 1> clock_init;
      clock_init.setZero();
      if (prior.find(system) != prior.end()) clock_init[0] = prior.at(system);
      std::shared_ptr<ClockParameterBlock> clock_parameter_block = 
        std::make_shared<ClockParameterBlock>(clock_init, clock_id.asInteger());
      CHECK(graph_->addParameterBlock(clock_parameter_block));
    }
  }

  // Check if any system does not have valid satellite
  std::map<char, int> system_observation_cnt;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    const char system = getGnssSystemList()[i];
    if (!gnss_common::useSystem(gnss_base_options_.common, system)) continue;
    system_observation_cnt.insert(std::make_pair(system, 0));
  }
  for (auto& index_pair : index_pairs) 
  {
    const GnssMeasurementIndex& index = index_pair.rov;
    const Satellite& satellite = measurement_rov.getSat(index);
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(gnss_base_options_.common, system)) continue;
    system_observation_cnt.at(system)++;
  }

  // add pseudo-measurement for all systems to avoid rank deficiancy
  num_valid_system = 0;
  for (auto it_system : system_observation_cnt)
  {
    char system = it_system.first;
    bool valid = it_system.second;
    if (valid) {
      num_valid_system++;
    }

    BackendId clock_id = createGnssClockId(system, id);
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(1);
    if (prior.find(system) != prior.end()) measurement[0] = prior.at(system);
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(1, 1) * 1e-6;
    std::shared_ptr<ClockError> clock_error = 
      std::make_shared<ClockError>(measurement, information);
    graph_->addResidualBlock(clock_error, nullptr, 
      graph_->parameterBlockPtr(clock_id.asInteger()));
  }
}

// Add ambiguity blocks to graph
void GnssEstimatorBase::addSdAmbiguityParameterBlocks(
  const GnssMeasurement& measurement_rov, 
  const GnssMeasurement& measurement_ref, 
  const GnssMeasurementDDIndexPairs& index_pairs,
  const int32_t id,
  AmbiguityState& state)
{
  state.clear();
  state.timestamp = measurement_rov.timestamp;

  for (auto index_pair : index_pairs) 
  {
    const Satellite& satellite = 
      measurement_rov.getSat(index_pair.rov);
    const Satellite& satellite_base = 
      measurement_rov.getSat(index_pair.rov_base);
    const Satellite& satellite_ref = 
      measurement_ref.getSat(index_pair.ref);
    const Satellite& satellite_ref_base = 
      measurement_ref.getSat(index_pair.ref_base);
    const Observation& observation = 
      satellite.observations.at(index_pair.rov.code_type);
    const Observation& observation_base = 
      satellite_base.observations.at(index_pair.rov_base.code_type);
    const Observation& observation_ref = 
      satellite_ref.observations.at(index_pair.ref.code_type);
    const Observation& observation_ref_base = 
      satellite_ref_base.observations.at(index_pair.ref_base.code_type);
    char system = satellite.getSystem();
    std::vector<Observation> observations_frequency;

    // add ambiguity parameter block
    double code = index_pair.rov.code_type;
    double wavelength = observation.wavelength;
    double phase_id = gnss_common::getPhaseID(system, code);
    BackendId ambiguity_id = createGnssAmbiguityId(satellite.prn, phase_id, id);
    CHECK(!graph_->parameterBlockExists(ambiguity_id.asInteger())) 
      << "Ambiguity parameter for " << satellite.prn << " in phase " 
      << phase_id << " has already been added!";

    // compute initial ambiguity state
    double phaserange = observation.phaserange;
    double pseudorange = observation.pseudorange;
    double phaserange_ref = observation_ref.phaserange;
    double pseudorange_ref = observation_ref.pseudorange;
    double ambiguity = phaserange - phaserange_ref - 
                      (pseudorange - pseudorange_ref);

    Eigen::Matrix<double, 1, 1> init;
    init[0] = ambiguity;
    std::shared_ptr<AmbiguityParameterBlock> ambiguity_parameter_block = 
      std::make_shared<AmbiguityParameterBlock>(init, ambiguity_id.asInteger());
    CHECK(graph_->addParameterBlock(ambiguity_parameter_block));
    state.ids.push_back(ambiguity_id);

    // add ambiguity parameter block for base satellite
    BackendId ambiguity_base_id = 
      createGnssAmbiguityId(satellite_base.prn, phase_id, id);
    if (!graph_->parameterBlockExists(ambiguity_base_id.asInteger())) 
    {
      phaserange = observation_base.phaserange;
      pseudorange = observation_base.pseudorange;
      phaserange_ref = observation_ref_base.phaserange;
      pseudorange_ref = observation_ref_base.pseudorange;
      ambiguity = phaserange - phaserange_ref - 
                        (pseudorange - pseudorange_ref);
      Eigen::Matrix<double, 1, 1> init_base;
      init_base[0] = ambiguity;
      std::shared_ptr<AmbiguityParameterBlock> ambiguity_base_parameter_block = 
        std::make_shared<AmbiguityParameterBlock>(
          init_base, ambiguity_base_id.asInteger());
      CHECK(graph_->addParameterBlock(ambiguity_base_parameter_block));
      state.ids.push_back(ambiguity_base_id);

      // add initial prior measurement
      addAmbiguityResidualBlock(ambiguity_base_id, init_base[0], 
        gnss_base_options_.error_parameter.initial_ambiguity);
    }
  }
}

// Add single-differenced pseudorange residual block to graph
void GnssEstimatorBase::addSdPseudorangeResidualBlocks(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurement& measurement_ref,
  const GnssMeasurementSDIndexPairs& index_pairs,
  const State& state,
  int& num_valid_satellite,
  bool use_single_frequency)
{
  CHECK(!(use_single_frequency && is_verbose_model_));
  num_valid_satellite = 0;
  const BackendId parameter_id = state.id;

  // Normal mode.  
  if (!is_verbose_model_) 
  {
    std::unordered_map<std::string, int> num_code_used;
    for (auto& index_pair : index_pairs) 
    {
      const GnssMeasurementIndex& index = index_pair.rov;
      const Satellite& satellite = measurement_rov.getSat(index);
      std::vector<Observation> observations_frequency;
      char system = satellite.getSystem();
      std::string prn = satellite.prn;

      // Add residuals
      if (num_code_used.find(prn) == num_code_used.end()) {
        num_code_used.insert(std::make_pair(prn, 0));
      }
      if (use_single_frequency && num_code_used.at(prn) > 0) continue;

      BackendId clock_id = createGnssClockId(
        satellite.getSystem(), measurement_rov.id);

      // position in ECEF for standalone 
      if (parameter_id.type() == IdType::gPosition) {
        is_state_pose_ = false;
        std::shared_ptr<PseudorangeErrorSD<3, 1>> pseudorange_error = 
          std::make_shared<PseudorangeErrorSD<3, 1>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, gnss_base_options_.error_parameter);
        graph_->addResidualBlock(pseudorange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(parameter_id.asInteger()),
          graph_->parameterBlockPtr(clock_id.asInteger()));
      }
      // pose in ENU for fusion
      else {
        is_state_pose_ = true;
        BackendId pose_id = state.id_in_graph;
        std::shared_ptr<PseudorangeErrorSD<7, 3, 1>> pseudorange_error = 
          std::make_shared<PseudorangeErrorSD<7, 3, 1>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, gnss_base_options_.error_parameter); 
        pseudorange_error->setCoordinate(coordinate_);
        graph_->addResidualBlock(pseudorange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(pose_id.asInteger()), 
          graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()),
          graph_->parameterBlockPtr(clock_id.asInteger()));
      }
        
      num_code_used.at(prn)++;
    }
    num_valid_satellite = num_code_used.size();
  }
  // Precise mode.
  else 
  {
    LOG(FATAL) << "Not supported yet!";
  }
}

// Add double-differenced pseudorange residual block to graph
void GnssEstimatorBase::addDdPseudorangeResidualBlocks(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurement& measurement_ref,
  const GnssMeasurementDDIndexPairs& index_pairs,
  const State& state,
  int& num_valid_satellite,
  bool use_single_frequency)
{
  CHECK(!(use_single_frequency && is_verbose_model_));
  num_valid_satellite = 0;
  const BackendId parameter_id = state.id;

  // Normal mode.  
  if (!is_verbose_model_) 
  {
    std::unordered_map<std::string, int> num_code_used;
    std::unordered_map<char, int> num_system_used;
    for (auto& index_pair : index_pairs) 
    {
      const GnssMeasurementIndex& index = index_pair.rov;
      const Satellite& satellite = measurement_rov.getSat(index);
      std::vector<Observation> observations_frequency;
      char system = satellite.getSystem();
      std::string prn = satellite.prn;

      // Add residuals
      if (num_code_used.find(prn) == num_code_used.end()) {
        num_code_used.insert(std::make_pair(prn, 0));
      }
      if (num_system_used.find(system) == num_system_used.end()) {
        num_system_used.insert(std::make_pair(system, 0));
      }
      if (use_single_frequency && num_code_used.at(prn) > 0) continue;

      // position in ECEF for standalone 
      if (parameter_id.type() == IdType::gPosition) {
        is_state_pose_ = false;
        std::shared_ptr<PseudorangeErrorDD<3>> pseudorange_error = 
          std::make_shared<PseudorangeErrorDD<3>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, index_pair.rov_base, index_pair.ref_base,
          gnss_base_options_.error_parameter);
        graph_->addResidualBlock(pseudorange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(parameter_id.asInteger()));
      }
      // pose in ENU for fusion
      else {
        is_state_pose_ = true;
        BackendId pose_id = state.id_in_graph;
        std::shared_ptr<PseudorangeErrorDD<7, 3>> pseudorange_error = 
          std::make_shared<PseudorangeErrorDD<7, 3>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, index_pair.rov_base, index_pair.ref_base,
          gnss_base_options_.error_parameter); 
        pseudorange_error->setCoordinate(coordinate_);
        graph_->addResidualBlock(pseudorange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(pose_id.asInteger()), 
          graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
      }
      
      num_code_used.at(prn)++;
      num_system_used.at(system)++;
    }
    num_valid_satellite = num_code_used.size() + num_system_used.size();
  }
  // Precise mode.
  else 
  {
    LOG(FATAL) << "Not supported yet!";
  }
}

// Add double-differenced phaserange residual blocks to graph
void GnssEstimatorBase::addDdPhaserangeResidualBlocks(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurement& measurement_ref,
  const GnssMeasurementDDIndexPairs& index_pairs,
  const State& state)
{
  const BackendId parameter_id = state.id;

  // Normal mode.  
  if (!is_verbose_model_) 
  {
    for (auto& index_pair : index_pairs) 
    {
      const GnssMeasurementIndex& index = index_pair.rov;
      const GnssMeasurementIndex& index_base = index_pair.rov_base;
      const Satellite& satellite = measurement_rov.getSat(index);
      const Satellite& satellite_base = measurement_rov.getSat(index_base);
      std::vector<Observation> observations_frequency;
      char system = satellite.getSystem();

      // Add residuals
      int code = index_pair.rov.code_type;
      double phase_id = gnss_common::getPhaseID(system, code);
      BackendId ambiguity_id = createGnssAmbiguityId(
        satellite.prn, phase_id, parameter_id.bundleId());
      BackendId ambiguity_base_id = createGnssAmbiguityId(
        satellite_base.prn, phase_id, parameter_id.bundleId());

      // position in ECEF for standalone 
      if (parameter_id.type() == IdType::gPosition) {
        is_state_pose_ = false;
        std::shared_ptr<PhaserangeErrorDD<3, 1, 1>> phaserange_error = 
          std::make_shared<PhaserangeErrorDD<3, 1, 1>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, index_pair.rov_base, index_pair.ref_base,
          gnss_base_options_.error_parameter);
        graph_->addResidualBlock(phaserange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(parameter_id.asInteger()), 
          graph_->parameterBlockPtr(ambiguity_id.asInteger()), 
          graph_->parameterBlockPtr(ambiguity_base_id.asInteger()));
      }
      // pose in ENU for fusion
      else {
        is_state_pose_ = true;
        BackendId pose_id = state.id_in_graph;
        std::shared_ptr<PhaserangeErrorDD<7, 3, 1, 1>> phaserange_error = 
          std::make_shared<PhaserangeErrorDD<7, 3, 1, 1>>(
          measurement_rov, measurement_ref, 
          index_pair.rov, index_pair.ref, index_pair.rov_base, index_pair.ref_base,
          gnss_base_options_.error_parameter); 
        phaserange_error->setCoordinate(coordinate_);
        graph_->addResidualBlock(phaserange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(pose_id.asInteger()), 
          graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()), 
          graph_->parameterBlockPtr(ambiguity_id.asInteger()), 
          graph_->parameterBlockPtr(ambiguity_base_id.asInteger()));
      }
    }
  }
  // Precise mode.
  else 
  {
    LOG(FATAL) << "Not supported yet!";
  }
}

// Get GNSS measurement index from error interface
GnssMeasurementSDIndexPair GnssEstimatorBase::getGnssMeasurementSDIndexPairFromErrorInterface(
  const std::shared_ptr<ErrorInterface>& error_interface)
{
  GnssMeasurementSDIndexPair index;
  if (error_interface->typeInfo() == ErrorType::kPseudorangeErrorSD) 
  {
    if (!is_verbose_model_) {
      if (!is_state_pose_) {
        const std::shared_ptr<PseudorangeErrorSD<3, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeErrorSD<3, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
      else {
        const std::shared_ptr<PseudorangeErrorSD<7, 3, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeErrorSD<7, 3, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
    }
  }

  return index;
}

// Get GNSS measurement index from error interface
GnssMeasurementDDIndexPair GnssEstimatorBase::getGnssMeasurementDDIndexPairFromErrorInterface(
  const std::shared_ptr<ErrorInterface>& error_interface)
{
  GnssMeasurementDDIndexPair index;
  if (error_interface->typeInfo() == ErrorType::kPseudorangeErrorDD) 
  {
    if (!is_verbose_model_) {
      if (!is_state_pose_) {
        const std::shared_ptr<PseudorangeErrorDD<3>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeErrorDD<3>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
      else {
        const std::shared_ptr<PseudorangeErrorDD<7, 3>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeErrorDD<7, 3>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
    }
  }
  if (error_interface->typeInfo() == ErrorType::kPhaserangeErrorDD) 
  {
    if (!is_verbose_model_) {
      if (!is_state_pose_) {
        const std::shared_ptr<PhaserangeErrorDD<3, 1, 1>> phaserange_error = 
          std::static_pointer_cast<PhaserangeErrorDD<3, 1, 1>>(error_interface);
        index = phaserange_error->getGnssMeasurementIndex();
      }
      else {
        const std::shared_ptr<PhaserangeErrorDD<7, 3, 1, 1>> phaserange_error = 
          std::static_pointer_cast<PhaserangeErrorDD<7, 3, 1, 1>>(error_interface);
        index = phaserange_error->getGnssMeasurementIndex();
      }
    }
  }

  return index;
}

}