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
#include "gici/gnss/pseudorange_error.h"
#include "gici/gnss/phaserange_error.h"
#include "gici/gnss/doppler_error.h"
#include "gici/gnss/gnss_relative_errors.h"
#include "gici/gnss/code_phase_maps.h"

namespace gici {

// Static variables
int32_t GnssEstimatorBase::solution_id = 0;

// The default constructor
GnssEstimatorBase::GnssEstimatorBase(
                    const GnssEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options) :
  gnss_base_options_(options), EstimatorBase(base_options)
{}

// The default destructor
GnssEstimatorBase::~GnssEstimatorBase()
{}

// Add GNSS position block to graph
BackendId GnssEstimatorBase::addGnssPositionParameterBlock(
  const int32_t id, const Eigen::Vector3d& prior)
{
  BackendId position_id = createGnssPositionId(id);
  std::shared_ptr<PositionParameterBlock> position_parameter_block = 
    std::make_shared<PositionParameterBlock>(prior, position_id.asInteger());
  CHECK(graph_->addParameterBlock(position_parameter_block));
  return position_id;
}

// Add GNSS velocity block to graph
BackendId GnssEstimatorBase::addGnssVelocityParameterBlock(
  const int32_t id, 
  const Eigen::Vector3d& prior)
{
  BackendId velocity_id = createGnssVelocityId(id);
  std::shared_ptr<VelocityParameterBlock> velocity_parameter_block = 
    std::make_shared<VelocityParameterBlock>(prior, velocity_id.asInteger());
  CHECK(graph_->addParameterBlock(velocity_parameter_block));
  return velocity_id;
}

// Add GNSS extrinsics block to graph
BackendId GnssEstimatorBase::addGnssExtrinsicsParameterBlock(
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

// Add GNSS clock blocks to graph
void GnssEstimatorBase::addClockParameterBlocks(
  const GnssMeasurement& measurement, 
  const int32_t id, int& num_valid_system,
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
  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(gnss_base_options_.common, system)) continue;
    for (auto obs : satellite.observations) {
      if (checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) {
        system_observation_cnt.at(system)++;
      }
    }
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

// Add Inter-Frequency Bias (IFB) to graph
void GnssEstimatorBase::addIfbParameterBlocks(
  GnssMeasurement& measurement, 
  const int32_t id)
{
  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();

    if (!gnss_common::useSystem(gnss_base_options_.common, system) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
    if (satellite.ionosphere_type == IonoType::None) continue;

    for (auto& obs : satellite.observations) {
      std::pair<char, int> ifb(system, obs.first);
      if (std::find(ifbs_.begin(), ifbs_.end(), ifb) != ifbs_.end()) continue;
      if (!checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) continue;
      
      // check if base frequency
      bool is_base = false;
      double wavelength = obs.second.wavelength;
      CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
      std::pair<int, int> base_pair = bases.at(system);
      int phase_id_base_lhs = 
        gnss_common::getPhaseID(system, base_pair.first);
      int phase_id_base_rhs = 
        gnss_common::getPhaseID(system, base_pair.second);
      int phase_id = 
        gnss_common::getPhaseID(system, obs.first);
      if (phase_id == phase_id_base_lhs || phase_id == phase_id_base_rhs) {
        is_base = true;
      }

      // if not base, we compute its initial value
      double init_ifb = 0.0;
      if (!is_base) {
        // find the first base frequency
        double pseudorange_base, wavelength_base;
        bool found = false;
        for (auto& j_obs : satellite.observations) {
          if (!checkObservationValid(measurement, 
              GnssMeasurementIndex(satellite.prn, j_obs.first))) continue;
          int j_phase_id = gnss_common::getPhaseID(system, j_obs.first);
          if (j_phase_id == phase_id_base_lhs) {
            pseudorange_base = j_obs.second.pseudorange;
            wavelength_base = j_obs.second.wavelength;
            found = true;
            break;
          }
        }
        if (!found) continue;
        double ionosphere = satellite.ionosphere;
        init_ifb = obs.second.pseudorange - pseudorange_base - 
          gnss_common::ionosphereConvertFromBase(ionosphere, wavelength) +
          gnss_common::ionosphereConvertFromBase(ionosphere, wavelength_base);
      }
      
      BackendId ifb_id = createGnssIfbId(satellite.getSystem(), obs.first);
      if (!graph_->parameterBlockExists(ifb_id.asInteger())) {
        Eigen::Matrix<double, 1, 1> init = 
          Eigen::Matrix<double, 1, 1>::Identity() * init_ifb;
        std::shared_ptr<IfbParameterBlock> ifb_parameter_block = 
          std::make_shared<IfbParameterBlock>(init, ifb_id.asInteger());
        CHECK(graph_->addParameterBlock(ifb_parameter_block));
        ifbs_.push_back(ifb);

        // if base frequency, set as constant
        if (is_base) {
          graph_->setParameterBlockConstant(ifb_id.asInteger());
        }
      }
    }
  }
}

// Add frequency blocks to graph
void GnssEstimatorBase::addFrequencyParameterBlocks(
  const GnssMeasurement& measurement, 
  const int32_t id, 
  int& num_valid_system,
  const std::map<char, double>& prior)
{
  // Add frequency parameter for each system
  for (size_t i = 0; i < getGnssSystemList().size(); i++) 
  {
    char system = getGnssSystemList()[i];
    BackendId freq_id = createGnssFrequencyId(system, id);
    if (gnss_common::useSystem(gnss_base_options_.common, system) && 
        !graph_->parameterBlockExists(freq_id.asInteger())) 
    {
      Eigen::Matrix<double, 1, 1> freq_init;
      freq_init.setZero();
      if (prior.find(system) != prior.end()) freq_init[0] = prior.at(system);
      std::shared_ptr<FrequencyParameterBlock> freq_parameter_block = 
        std::make_shared<FrequencyParameterBlock>(freq_init, freq_id.asInteger());
      CHECK(graph_->addParameterBlock(freq_parameter_block));
    }
  }

  // Check if any system does not have valid satellite
  std::map<char, int> system_observation_cnt;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    const char system = getGnssSystemList()[i];
    if (!gnss_common::useSystem(gnss_base_options_.common, system)) continue;
    system_observation_cnt.insert(std::make_pair(system, 0));
  }
  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(gnss_base_options_.common, system)) continue;
    for (auto obs : satellite.observations) {
      const Observation& observation = 
        measurement.getObs(GnssMeasurementIndex(satellite.prn, obs.first));
      if (checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first), true) && 
          !observation.slip) {
        system_observation_cnt.at(system)++;
      }
    }
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

    BackendId freq_id = createGnssFrequencyId(system, id);
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(1);
    if (prior.find(system) != prior.end()) measurement[0] = prior.at(system);
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(1, 1) * 1e-6;
    std::shared_ptr<FrequencyError> freq_error = 
      std::make_shared<FrequencyError>(measurement, information);
    graph_->addResidualBlock(freq_error, nullptr, 
      graph_->parameterBlockPtr(freq_id.asInteger()));
  }
}

// Add troposphere block to graph
void GnssEstimatorBase::addTroposphereParameterBlock(const int32_t id)
{
  BackendId tropo_id = createGnssTroposphereId(id);
  Eigen::Matrix<double, 1, 1> tropo_init;
  tropo_init.setZero();
  std::shared_ptr<TroposphereParameterBlock> tropo_parameter_block = 
    std::make_shared<TroposphereParameterBlock>(tropo_init, tropo_id.asInteger());
  CHECK(graph_->addParameterBlock(tropo_parameter_block));

  // Add initial prior measurement
  if (isFirstEpoch()) {
    addTroposphereResidualBlock(tropo_id, tropo_init[0], 
      gnss_base_options_.error_parameter.initial_troposphere);
  }
}

// Add ionosphere blocks to graph
void GnssEstimatorBase::addIonosphereParameterBlocks(
  const GnssMeasurement& measurement, 
  const int32_t id,
  IonosphereState& state)
{
  state.clear();
  state.timestamp = measurement.timestamp;

  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(gnss_base_options_.common, satellite.getSystem()) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
    if (satellite.ionosphere_type == IonoType::None) continue;

    // add ionosphere parameter blocks
    BackendId iono_id = createGnssIonosphereId(satellite.prn, id);
    Eigen::Matrix<double, 1, 1> init;
    init[0] = satellite.ionosphere;
    std::shared_ptr<IonosphereParameterBlock> iono_parameter_block = 
      std::make_shared<IonosphereParameterBlock>(init, iono_id.asInteger());
    CHECK(graph_->addParameterBlock(iono_parameter_block));
    state.ids.push_back(iono_id);

    // add initial prior measurement
    if (isFirstEpoch()) {
      addIonosphereResidualBlock(iono_id, init[0], 
        gnss_base_options_.error_parameter.initial_ionosphere);
    }
  }
}

// Add ambiguity blocks to graph
void GnssEstimatorBase::addAmbiguityParameterBlocks(
  const GnssMeasurement& measurement, 
  const int32_t id,
  AmbiguityState& state)
{
  state.clear();
  state.timestamp = measurement.timestamp;

  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(gnss_base_options_.common, system) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
    if (satellite.ionosphere_type == IonoType::None) continue;

    // add ambiguity parameter blocks
    for (auto obs : satellite.observations) {
      if (!checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) continue;
      
      double code = obs.first;
      double wavelength = obs.second.wavelength;
      double phase_id = gnss_common::getPhaseID(system, code);
      BackendId ambiguity_id = createGnssAmbiguityId(satellite.prn, phase_id, id);
      CHECK(!graph_->parameterBlockExists(ambiguity_id.asInteger())) 
        << "Ambiguity parameter for " << satellite.prn << " in phase " 
        << phase_id << " has already been added!";

      // compute initial ambiguity state
      double ionosphere_delay = satellite.ionosphere;
      double ambiguity = obs.second.phaserange - obs.second.pseudorange + 
        2.0 * gnss_common::ionosphereConvertFromBase(ionosphere_delay, wavelength);

      Eigen::Matrix<double, 1, 1> init;
      init[0] = ambiguity;
      std::shared_ptr<AmbiguityParameterBlock> ambiguity_parameter_block = 
        std::make_shared<AmbiguityParameterBlock>(init, ambiguity_id.asInteger());
      CHECK(graph_->addParameterBlock(ambiguity_parameter_block));
      state.ids.push_back(ambiguity_id);

      // add initial prior measurement
      if (isFirstEpoch()) {
        addAmbiguityResidualBlock(ambiguity_id, init[0], 
          gnss_base_options_.error_parameter.initial_ambiguity);
      }
    }
  }
}

// Correct code biases
void GnssEstimatorBase::correctCodeBias(
  GnssMeasurement& measurement, const bool accept_coarse)
{
  CodeBiasPtr code_bias = measurement.code_bias;
  for (auto& sat : measurement.satellites) {
    Satellite& satellite = sat.second;
    std::string prn = satellite.prn;
    for (auto& obs : satellite.observations) {
      int code = obs.first;
      Observation& observation = obs.second;
      if (observation.pseudorange == 0.0) continue;
      double bias = code_bias->getCodeBias(prn, code, accept_coarse);
      if (bias == 0.0) {
        // code bias not availible
        observation.pseudorange = 0.0;
      }
      else {
        observation.pseudorange += bias;
      }
    }
  }
}

// Correct phase biases
void GnssEstimatorBase::correctPhaseBias(GnssMeasurement& measurement)
{
  PhaseBiasPtr phase_bias = measurement.phase_bias;
  const AmbiguityResolutionOptions& options = ambiguity_resolution_->getOptions();
  const std::vector<char>& system_exclude = options.system_exclude;
  const std::vector<std::string>& satellite_exclude = options.satellite_exclude;
  const std::vector<std::pair<char, int>>& phase_exclude = options.phase_exclude;
  for (auto& sat : measurement.satellites) {
    Satellite& satellite = sat.second;
    std::string prn = satellite.prn;
    char system = prn[0];
    if (!gnss_common::useSystem(system_exclude, system) || 
        !gnss_common::useSatellite(satellite_exclude, prn)) continue;

    for (auto& obs : satellite.observations) {
      int code = obs.first;
      Observation& observation = obs.second;
      if (observation.phaserange == 0.0) continue;
      int phase_id = gnss_common::getPhaseID(prn[0], code);
      if (!gnss_common::usePhase(phase_exclude, system, phase_id)) continue;
      double bias = phase_bias->getPhaseBias(prn, phase_id);
      if (bias == 0.0) {
        // phase bias not availible
        observation.phaserange = 0.0;
      }
      else {
        observation.phaserange += bias;
      }
    }
  }
}

// Correct BDS satellite multipath
void GnssEstimatorBase::correctBdsSatelliteMultipath(GnssMeasurement& measurement)
{
  for (auto& sat : measurement.satellites) {
    Satellite& satellite = sat.second;
    std::string prn = satellite.prn;
    char system = prn[0];
    if (system != 'C') continue;
    if (checkZero(satellite.sat_position)) return;
    double elevation = gnss_common::satelliteElevation(
        satellite.sat_position, measurement.position);
    for (auto& obs : satellite.observations) {
      int code = obs.first;
      if (obs.second.pseudorange == 0.0) continue;
      obs.second.pseudorange += 
        gnss_common::getBdsSatelliteMultipath(prn, elevation, code);
    }
  }
}

// Compute and set ionosphere delays using dual-frequency or model
void GnssEstimatorBase::computeIonosphereDelay(
  GnssMeasurement& measurement, bool use_single_frequency)
{
  for (auto& sat : measurement.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    std::vector<Observation> dual_frequency_observations;
    std::vector<Observation> valid_observations;

    if (satellite.ionosphere_type == IonoType::Augmentation) continue;
    CHECK(satellite.ionosphere == 0.0);

    if (!gnss_common::useSystem(gnss_base_options_.common, satellite.getSystem()) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;

    // Check if we can get initial ionosphere with dual-frequency
    double ionosphere_delay;
    IonoType type = IonoType::None;
    for (auto obs : satellite.observations) {
      if (!checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) continue;
      
      // Add to valid
      bool found = false;
      for (auto& it : valid_observations) {
        if (checkEqual(it.wavelength, obs.second.wavelength)) {
          found = true; break;
        }
      }
      if (!found) valid_observations.push_back(obs.second);

      // only use bases frequencies to apply dual-frequency ionosphere calculation, 
      // or it may contain large IFB
      CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
      std::pair<int, int> base_pair = bases.at(system);
      int phase_id_base_lhs = 
        gnss_common::getPhaseID(system, base_pair.first);
      int phase_id_base_rhs = 
        gnss_common::getPhaseID(system, base_pair.second);
      int phase_id = 
        gnss_common::getPhaseID(system, obs.first);
      if (phase_id != phase_id_base_lhs && phase_id != phase_id_base_rhs) {
        continue;
      }

      // ensure different frequencies
      found = false;
      for (auto& it : dual_frequency_observations) {
        if (checkEqual(it.wavelength, obs.second.wavelength)) {
          found = true; break;
        }
      }
      if (!found) dual_frequency_observations.push_back(obs.second);
    }

    // no valid observations
    if (valid_observations.size() == 0) continue;

    // compute ionosphere delay using dual-frequency observation
    bool computed = false;
    if (!use_single_frequency && dual_frequency_observations.size() > 1) 
    {
      // use the maximum and minimum wavelengths
      std::sort(dual_frequency_observations.begin(), 
                dual_frequency_observations.end(),
        [](Observation& lhs, Observation& rhs) 
        { return lhs.wavelength < rhs.wavelength; });

      // check error ratio
      double ratio = fabs(1.0 / (1.0 - square(
        dual_frequency_observations.back().wavelength / 
        dual_frequency_observations.front().wavelength)));
      if (ratio < 3.0) {
        // compute 
        double ionosphere = gnss_common::ionosphereDualFrequency(
          dual_frequency_observations.front(), dual_frequency_observations.back());
        ionosphere_delay = gnss_common::ionosphereConvertToBase(
          ionosphere, dual_frequency_observations.front().wavelength);
        type = IonoType::DualFrequency;
        computed = true;
      }
    }
    // compute ionosphere delay using model
    if (!computed) {
      double timestamp = measurement.timestamp;
      Eigen::Vector3d position = measurement.position;
      if (checkZero(position)) {
        // For initial SPP
        ionosphere_delay = 0.0;
      }
      else {
        double azimuth = gnss_common::satelliteAzimuth(
          satellite.sat_position, position);
        double elevation = gnss_common::satelliteElevation(
          satellite.sat_position, position);
        double wavelength = valid_observations[0].wavelength;
        Eigen::VectorXd iono_parameters = measurement.ionosphere_parameters;
        ionosphere_delay = gnss_common::ionosphereBroadcast(timestamp, position, 
          azimuth, elevation, wavelength, iono_parameters);
        ionosphere_delay = gnss_common::ionosphereConvertToBase(
          ionosphere_delay, wavelength);
      }
      type = IonoType::Broadcast;
    }

    // add to measurement
    satellite.ionosphere_type = type;
    satellite.ionosphere = ionosphere_delay;
  }
}

// Add pseudorange residual blocks to graph
void GnssEstimatorBase::addPseudorangeResidualBlocks(
  const GnssMeasurement& measurement,
  const State& state,
  int& num_valid_satellite,
  bool use_single_frequency)
{
  CHECK(!(use_single_frequency && is_verbose_model_));
  num_valid_satellite = 0;
  const BackendId parameter_id = state.id;

  // None precise mode. 
  // Do not estimate atmosphere, we correct them by models or measurements. 
  // Ignore IFBs, because they are not significant for meter-level positioning. 
  if (!is_verbose_model_) 
  {
    for (auto& sat : measurement.satellites) 
    {
      const Satellite& satellite = sat.second;
      std::vector<Observation> observations_frequency;
      char system = satellite.getSystem();

      if (!gnss_common::useSystem(gnss_base_options_.common, system) || 
          !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
      if (satellite.ionosphere_type == IonoType::None) continue;

      // Add residuals
      bool has_valid = false;
      for (auto obs : satellite.observations) {
        if (!checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) continue;
        
        // check single frequency
        if (use_single_frequency) {
          CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
          std::pair<int, int> base_pair = bases.at(system);
          if (system == 'C') base_pair.first = CODE_L2I;  // use B1I for BDS
          int phase_id_base = gnss_common::getPhaseID(system, base_pair.first);
          int phase_id = gnss_common::getPhaseID(system, obs.first);
          if (phase_id_base != phase_id) continue;
        }

        BackendId clock_id = createGnssClockId(satellite.getSystem(), measurement.id);

        // position in ECEF for standalone 
        ceres::ResidualBlockId residual_id;
        if (parameter_id.type() == IdType::gPosition) {
          is_state_pose_ = false;
          std::shared_ptr<PseudorangeError<3, 1>> pseudorange_error = 
            std::make_shared<PseudorangeError<3, 1>>(measurement, 
            GnssMeasurementIndex(satellite.prn, obs.first), 
            gnss_base_options_.error_parameter);
          residual_id = graph_->addResidualBlock(pseudorange_error, 
            huber_loss_function_ ? huber_loss_function_.get() : nullptr,
            graph_->parameterBlockPtr(parameter_id.asInteger()),
            graph_->parameterBlockPtr(clock_id.asInteger()));
        }
        // pose in ENU for fusion
        else {
          is_state_pose_ = true;
          BackendId pose_id = state.id_in_graph;
          std::shared_ptr<PseudorangeError<7, 3, 1>> pseudorange_error = 
            std::make_shared<PseudorangeError<7, 3, 1>>(measurement, 
            GnssMeasurementIndex(satellite.prn, obs.first), 
            gnss_base_options_.error_parameter);
          pseudorange_error->setCoordinate(coordinate_);
          residual_id = graph_->addResidualBlock(pseudorange_error, 
            huber_loss_function_ ? huber_loss_function_.get() : nullptr,
            graph_->parameterBlockPtr(pose_id.asInteger()), 
            graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()),
            graph_->parameterBlockPtr(clock_id.asInteger()));
        }
        
        has_valid = true;
      }
      if (has_valid) num_valid_satellite++;
    }
  }
  // Precise mode.
  // Estimate atmosphere and IFBs
  else 
  {
    for (auto& sat : measurement.satellites) 
    {
      const Satellite& satellite = sat.second;
      std::vector<Observation> observations_frequency;

      if (!gnss_common::useSystem(gnss_base_options_.common, satellite.getSystem()) || 
          !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
      if (satellite.ionosphere_type == IonoType::None) continue;

      bool has_valid = false;
      for (auto obs : satellite.observations) {
        if (!checkObservationValid(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first))) continue;

        has_valid = true;
        BackendId clock_id = createGnssClockId(satellite.getSystem(), measurement.id);
        BackendId troposphere_id = createGnssTroposphereId(measurement.id);
        BackendId ionosphere_id = createGnssIonosphereId(satellite.prn, measurement.id);
        BackendId ifb_id = createGnssIfbId(satellite.prn[0], obs.first);

        // IFB not yet initialized
        if (!graph_->parameterBlockExists(ifb_id.asInteger())) {
          LOG(WARNING) << "IFB for code " << obs.first << " for system " 
            << satellite.prn[0] << " has not been initialized!";
          continue;
        }

        // position in ECEF for standalone 
        ceres::ResidualBlockId residual_id;
        if (parameter_id.type() == IdType::gPosition) {
          is_state_pose_ = false;
          std::shared_ptr<PseudorangeError<3, 1, 1, 1, 1>> pseudorange_error = 
            std::make_shared<PseudorangeError<3, 1, 1, 1, 1>>(measurement, 
            GnssMeasurementIndex(satellite.prn, obs.first), 
            gnss_base_options_.error_parameter);
          residual_id = graph_->addResidualBlock(pseudorange_error, 
            huber_loss_function_ ? huber_loss_function_.get() : nullptr,
            graph_->parameterBlockPtr(parameter_id.asInteger()),
            graph_->parameterBlockPtr(clock_id.asInteger()), 
            graph_->parameterBlockPtr(ifb_id.asInteger()), 
            graph_->parameterBlockPtr(troposphere_id.asInteger()),
            graph_->parameterBlockPtr(ionosphere_id.asInteger()));
        }
        // pose in ENU for fusion
        else {
          is_state_pose_ = true;
          BackendId pose_id = state.id_in_graph;
          std::shared_ptr<PseudorangeError<7, 3, 1, 1, 1, 1>> pseudorange_error = 
            std::make_shared<PseudorangeError<7, 3, 1, 1, 1, 1>>(measurement, 
            GnssMeasurementIndex(satellite.prn, obs.first), 
            gnss_base_options_.error_parameter);
          pseudorange_error->setCoordinate(coordinate_);
          residual_id = graph_->addResidualBlock(pseudorange_error, 
            huber_loss_function_ ? huber_loss_function_.get() : nullptr,
            graph_->parameterBlockPtr(pose_id.asInteger()), 
            graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()),
            graph_->parameterBlockPtr(clock_id.asInteger()),
            graph_->parameterBlockPtr(ifb_id.asInteger()), 
            graph_->parameterBlockPtr(troposphere_id.asInteger()),
            graph_->parameterBlockPtr(ionosphere_id.asInteger()));
        }
      }
      if (has_valid) num_valid_satellite++;
    }
  }
}

// Add phaserange residual blocks to graph
void GnssEstimatorBase::addPhaserangeResidualBlocks(
  const GnssMeasurement& measurement,
  const State& state)
{
  CHECK(is_verbose_model_);
  const BackendId parameter_id = state.id;

  for (auto& sat : measurement.satellites) 
  {
    const Satellite& satellite = sat.second;
    char system = satellite.getSystem();
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(gnss_base_options_.common, system) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;
    if (satellite.ionosphere_type == IonoType::None) continue;

    for (auto obs : satellite.observations) {
      if (!checkObservationValid(measurement, 
        GnssMeasurementIndex(satellite.prn, obs.first))) continue;

      int code = obs.first;
      double wavelength = obs.second.wavelength;
      double phase_id = gnss_common::getPhaseID(system, code);
      BackendId clock_id = createGnssClockId(system, measurement.id);
      BackendId ambiguity_id = createGnssAmbiguityId(
        satellite.prn, phase_id, parameter_id.bundleId());
      BackendId troposphere_id = createGnssTroposphereId(measurement.id);
      BackendId ionosphere_id = createGnssIonosphereId(satellite.prn, measurement.id);

      // position in ECEF for standalone 
      ceres::ResidualBlockId residual_id;
      if (parameter_id.type() == IdType::gPosition) {
        is_state_pose_ = false;
        std::shared_ptr<PhaserangeError<3, 1, 1, 1, 1>> phaserange_error = 
          std::make_shared<PhaserangeError<3, 1, 1, 1, 1>>(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first), 
          gnss_base_options_.error_parameter);
        residual_id = graph_->addResidualBlock(phaserange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(parameter_id.asInteger()),
          graph_->parameterBlockPtr(clock_id.asInteger()), 
          graph_->parameterBlockPtr(ambiguity_id.asInteger()), 
          graph_->parameterBlockPtr(troposphere_id.asInteger()),
          graph_->parameterBlockPtr(ionosphere_id.asInteger()));
      }
      // pose in ENU for fusion
      else {
        is_state_pose_ = true;
        BackendId pose_id = state.id_in_graph;
        std::shared_ptr<PhaserangeError<7, 3, 1, 1, 1, 1>> phaserange_error = 
          std::make_shared<PhaserangeError<7, 3, 1, 1, 1, 1>>(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first), 
          gnss_base_options_.error_parameter);
        phaserange_error->setCoordinate(coordinate_);
        residual_id = graph_->addResidualBlock(phaserange_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(pose_id.asInteger()), 
          graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()),
          graph_->parameterBlockPtr(clock_id.asInteger()),
          graph_->parameterBlockPtr(ambiguity_id.asInteger()), 
          graph_->parameterBlockPtr(troposphere_id.asInteger()),
          graph_->parameterBlockPtr(ionosphere_id.asInteger()));
      }
    }
  }
}

// Add doppler residual blocks to graph
void GnssEstimatorBase::addDopplerResidualBlocks(
  const GnssMeasurement& measurement,
  const State& state,
  int& num_valid_satellite,
  bool use_single_frequency,
  const Eigen::Vector3d& angular_velocity)
{
  num_valid_satellite = 0;
  const BackendId parameter_id = state.id;

  for (auto& sat : measurement.satellites) 
  {
    const Satellite& satellite = sat.second;
    char system = satellite.getSystem();
    std::vector<Observation> observations_frequency;

    if (!gnss_common::useSystem(gnss_base_options_.common, system) || 
        !gnss_common::useSatellite(gnss_base_options_.common, satellite.prn)) continue;

    bool has_valid = false;
    for (auto obs : satellite.observations) {
      if (!checkObservationValid(measurement, 
        GnssMeasurementIndex(satellite.prn, obs.first), true)) continue;
      
      // check single frequency
      if (use_single_frequency) {
        CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
        std::pair<int, int> base_pair = bases.at(system);
        int phase_id_base = gnss_common::getPhaseID(system, base_pair.first);
        int phase_id = gnss_common::getPhaseID(system, obs.first);
        if (phase_id_base != phase_id) continue;
      }

      const Observation& observation = 
        measurement.getObs(GnssMeasurementIndex(satellite.prn, obs.first));
      if (observation.slip) continue;

      // position in ECEF for standalone 
      if (parameter_id.type() == IdType::gPosition) {
        is_state_pose_ = false;
        BackendId velocity_id = changeIdType(parameter_id, IdType::gVelocity);
        BackendId freq_id = createGnssFrequencyId(system, measurement.id);
        std::shared_ptr<DopplerError<3, 3, 1>> doppler_error = 
          std::make_shared<DopplerError<3, 3, 1>>(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first), 
          gnss_base_options_.error_parameter);
        graph_->addResidualBlock(doppler_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(parameter_id.asInteger()),
          graph_->parameterBlockPtr(velocity_id.asInteger()),
          graph_->parameterBlockPtr(freq_id.asInteger()));
      }
      // pose in ENU for fusion
      else {
        is_state_pose_ = true;
        BackendId pose_id = state.id_in_graph;
        BackendId speed_and_bias_id = changeIdType(pose_id, IdType::ImuStates);
        BackendId freq_id = createGnssFrequencyId(system, measurement.id);
        std::shared_ptr<DopplerError<7, 9, 3, 1>> doppler_error = 
          std::make_shared<DopplerError<7, 9, 3, 1>>(measurement, 
          GnssMeasurementIndex(satellite.prn, obs.first), 
          gnss_base_options_.error_parameter, angular_velocity);
        doppler_error->setCoordinate(coordinate_);
        graph_->addResidualBlock(doppler_error, 
          huber_loss_function_ ? huber_loss_function_.get() : nullptr,
          graph_->parameterBlockPtr(pose_id.asInteger()), 
          graph_->parameterBlockPtr(speed_and_bias_id.asInteger()), 
          graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()),
          graph_->parameterBlockPtr(freq_id.asInteger()));
      }
      
      has_valid = true;
    }

    if (has_valid) num_valid_satellite++;
  }
}

// Add troposphere residual block to graph
void GnssEstimatorBase::addTroposphereResidualBlock(
  const BackendId& tropo_id,
  const double value, const double std)
{
  CHECK(tropo_id.type() == IdType::gTroposphere);
  Eigen::Matrix<double, 1, 1> covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(std);
  const Eigen::Map<const Eigen::VectorXd> tropo_init(&value, 1);
  std::shared_ptr<TroposphereError> tropo_error = 
    std::make_shared<TroposphereError>(tropo_init, covariance.inverse());
  graph_->addResidualBlock(tropo_error, nullptr,
    graph_->parameterBlockPtr(tropo_id.asInteger()));
}

// Add ionosphere residual block to graph
void GnssEstimatorBase::addIonosphereResidualBlock(
  const BackendId& iono_id,
  const double value, const double std)
{
  CHECK(iono_id.type() == IdType::gIonosphere);
  Eigen::Matrix<double, 1, 1> covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(std);
  const Eigen::Map<const Eigen::VectorXd> iono_init(&value, 1);
  std::shared_ptr<IonosphereError> iono_error = 
    std::make_shared<IonosphereError>(iono_init, covariance.inverse());
  graph_->addResidualBlock(iono_error, nullptr,
    graph_->parameterBlockPtr(iono_id.asInteger()));
}

// Add ambiguity residual block to graph
void GnssEstimatorBase::addAmbiguityResidualBlock(
  const BackendId& amb_id,
  const double value, const double std)
{
  CHECK(amb_id.type() == IdType::gAmbiguity);
  Eigen::Matrix<double, 1, 1> covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(std);
  const Eigen::Map<const Eigen::VectorXd> amb_init(&value, 1);
  std::shared_ptr<SingleAmbiguityError> amb_error = 
    std::make_shared<SingleAmbiguityError>(amb_init, covariance.inverse());
  graph_->addResidualBlock(amb_error, nullptr,
    graph_->parameterBlockPtr(amb_id.asInteger()));
}

// Add relative position block to graph
void GnssEstimatorBase::addRelativePositionResidualBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;
  CHECK(last_id.type() == IdType::gPosition);
  CHECK(cur_id.type() == IdType::gPosition);

  double dt = cur_state.timestamp - last_state.timestamp;
  CHECK(dt >= 0.0);
  Eigen::Vector3d dp_error = gnss_base_options_.error_parameter.relative_position;
  for (size_t i = 0; i < 3; i++) CHECK(dp_error(i) != 0.0);
  Eigen::Matrix3d dp_covariance = (cwiseSquare(dp_error) * dt).asDiagonal();
  dp_covariance = coordinate_->convertCovariance(
    dp_covariance, GeoType::ENU, GeoType::ECEF);

  std::shared_ptr<RelativePositionError> relative_position_error = 
    std::make_shared<RelativePositionError>(dp_covariance.inverse());
  graph_->addResidualBlock(relative_position_error, nullptr,
    graph_->parameterBlockPtr(last_id.asInteger()),
    graph_->parameterBlockPtr(cur_id.asInteger()));
}

// Add relative position and velocity block to graph
void GnssEstimatorBase::addRelativePositionAndVelocityBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;
  CHECK(last_id.type() == IdType::gPosition);
  CHECK(cur_id.type() == IdType::gPosition);
  const BackendId last_velocity_id = changeIdType(last_id, IdType::gVelocity);
  const BackendId cur_velocity_id = changeIdType(cur_id, IdType::gVelocity);
  CHECK(graph_->parameterBlockExists(last_velocity_id.asInteger()));
  CHECK(graph_->parameterBlockExists(cur_velocity_id.asInteger()));

  double dt = cur_state.timestamp - last_state.timestamp;
  CHECK(dt >= 0.0);
  Eigen::Vector3d dv_error = gnss_base_options_.error_parameter.relative_velocity;
  for (size_t i = 0; i < 3; i++) CHECK(dv_error(i) != 0.0);
  Eigen::Matrix3d dv_psd = cwiseSquare(dv_error).asDiagonal();
  dv_psd = coordinate_->convertCovariance(dv_psd, GeoType::ENU, GeoType::ECEF);

  std::shared_ptr<RelativePositionAndVelocityError> relative_error = 
    std::make_shared<RelativePositionAndVelocityError>(dv_psd, dt);
  graph_->addResidualBlock(relative_error, nullptr, 
    graph_->parameterBlockPtr(last_id.asInteger()), 
    graph_->parameterBlockPtr(cur_id.asInteger()),
    graph_->parameterBlockPtr(last_velocity_id.asInteger()),
    graph_->parameterBlockPtr(cur_velocity_id.asInteger()));
}

// Add relative frequency block to graph
void GnssEstimatorBase::addRelativeFrequencyBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  for (auto system : getGnssSystemList()) {
    BackendId last_freq_id = changeIdType(last_id, IdType::gFrequency, system);
    BackendId cur_freq_id = changeIdType(cur_id, IdType::gFrequency, system);

    if (!graph_->parameterBlockExists(last_freq_id.asInteger())) continue;
    if (!graph_->parameterBlockExists(cur_freq_id.asInteger())) continue;

    double dt = cur_state.timestamp - last_state.timestamp;
    CHECK(dt >= 0.0);
    double dfreq_error = gnss_base_options_.error_parameter.relative_frequency;
    CHECK(dfreq_error != 0.0);
    Eigen::Matrix<double, 1, 1> dfreq_covariance = 
      Eigen::Matrix<double, 1, 1>::Identity() * square(dfreq_error) * dt;
    
    std::shared_ptr<RelativeFrequencyError> relative_freq_error = 
      std::make_shared<RelativeFrequencyError>(dfreq_covariance.inverse());
    graph_->addResidualBlock(relative_freq_error, nullptr,
      graph_->parameterBlockPtr(last_freq_id.asInteger()),
      graph_->parameterBlockPtr(cur_freq_id.asInteger()));

    // reset initial value of current state
    *graph_->parameterBlockPtr(cur_freq_id.asInteger())->parameters() = 
      *graph_->parameterBlockPtr(last_freq_id.asInteger())->parameters();
    graph_->setParameterBlockVariable(cur_freq_id.asInteger());
  }
}

// Add relative troposphere block to graph
void GnssEstimatorBase::addRelativeTroposphereResidualBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  BackendId last_tropo_id = changeIdType(last_id, IdType::gTroposphere);
  BackendId cur_tropo_id = changeIdType(cur_id, IdType::gTroposphere);

  double dt = cur_state.timestamp - last_state.timestamp;
  CHECK(dt >= 0.0);
  double dtropo_error = gnss_base_options_.error_parameter.relative_troposphere;
  CHECK(dtropo_error != 0.0);
  Eigen::Matrix<double, 1, 1> dtropo_covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(dtropo_error) * dt;
  
  std::shared_ptr<RelativeTroposphereError> relative_tropo_error = 
    std::make_shared<RelativeTroposphereError>(dtropo_covariance.inverse());
  graph_->addResidualBlock(relative_tropo_error, nullptr,
    graph_->parameterBlockPtr(last_tropo_id.asInteger()),
    graph_->parameterBlockPtr(cur_tropo_id.asInteger()));

  // reset initial value of current state
  *graph_->parameterBlockPtr(cur_tropo_id.asInteger())->parameters() = 
    *graph_->parameterBlockPtr(last_tropo_id.asInteger())->parameters();
  graph_->setParameterBlockVariable(cur_tropo_id.asInteger());
}

// Add relative ionosphere block to graph
void GnssEstimatorBase::addRelativeIonosphereResidualBlock(
  const IonosphereState& last_state, const IonosphereState& cur_state)
{
  double dt = cur_state.timestamp - last_state.timestamp;
  CHECK(dt >= 0.0);
  double diono_error = gnss_base_options_.error_parameter.relative_ionosphere;
  CHECK(diono_error != 0.0);
  Eigen::Matrix<double, 1, 1> diono_covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(diono_error) * dt;

  for (size_t i = 0; i < cur_state.ids.size(); i++) {
    bool has_last = false;
    for (size_t j = 0; j < last_state.ids.size(); j++) {
      if (!sameIonosphere(last_state.ids[j], cur_state.ids[i])) continue;

      std::shared_ptr<RelativeIonosphereError> relative_iono_error = 
        std::make_shared<RelativeIonosphereError>(diono_covariance.inverse());
      graph_->addResidualBlock(relative_iono_error, nullptr,
        graph_->parameterBlockPtr(last_state.ids[j].asInteger()),
        graph_->parameterBlockPtr(cur_state.ids[i].asInteger()));

      // reset initial value
      *graph_->parameterBlockPtr(cur_state.ids[i].asInteger())->parameters() = 
        *graph_->parameterBlockPtr(last_state.ids[j].asInteger())->parameters();
      graph_->setParameterBlockVariable(cur_state.ids[i].asInteger());

      has_last = true;
      break;
    }

    // Add initial prior
    if (!has_last) {
      addIonosphereResidualBlock(cur_state.ids[i], 
        *graph_->parameterBlockPtr(cur_state.ids[i].asInteger())->parameters(), 
        gnss_base_options_.error_parameter.initial_ionosphere);
    }
  }
}

// Add relative ambiguity block to graph
void GnssEstimatorBase::addRelativeAmbiguityResidualBlock(
  GnssMeasurement& last_measurement,
  GnssMeasurement& cur_measurement,
  const AmbiguityState& last_state, const AmbiguityState& cur_state)
{
  double dt = cur_state.timestamp - last_state.timestamp;
  CHECK(dt >= 0.0);
  double damb_error = gnss_base_options_.error_parameter.relative_ambiguity;
  CHECK(damb_error != 0.0);
  Eigen::Matrix<double, 1, 1> damb_covariance = 
    Eigen::Matrix<double, 1, 1>::Identity() * square(damb_error) * dt;

  for (size_t i = 0; i < cur_state.ids.size(); i++) {
    bool has_last = false;
    for (size_t j = 0; j < last_state.ids.size(); j++) {
      if (!sameAmbiguity(last_state.ids[j], cur_state.ids[i])) continue;

      // check cycle slip
      std::string prn = cur_state.ids[i].gPrn();
      int phase_id = cur_state.ids[i].gPhaseId();
      Satellite& satellite = cur_measurement.getSat(prn);
      bool slip = false;

      for (auto obs : satellite.observations) {
        if (gnss_common::getPhaseID(satellite.getSystem(), obs.first) == phase_id) {
          slip = obs.second.slip;
        }
      }
      // if slip happened, we do not add ambiguity time constraint
      if (slip) continue;

      // Add noise for GPS L5 to absorb Inter-Frequency Clock Bias (IFCB)
      if (satellite.getSystem() == 'G' && phase_id == PHASE_L5) {
        damb_covariance += Eigen::Matrix<double, 1, 1>::Identity() * 
          square(gnss_base_options_.error_parameter.relative_gps_ifcb) * dt;
      }

      std::shared_ptr<RelativeAmbiguityError> relative_amb_error = 
        std::make_shared<RelativeAmbiguityError>(damb_covariance.inverse());
      graph_->addResidualBlock(relative_amb_error, nullptr,
        graph_->parameterBlockPtr(last_state.ids[j].asInteger()),
        graph_->parameterBlockPtr(cur_state.ids[i].asInteger()));

      // reset initial value
      *graph_->parameterBlockPtr(cur_state.ids[i].asInteger())->parameters() = 
        *graph_->parameterBlockPtr(last_state.ids[j].asInteger())->parameters();
      graph_->setParameterBlockVariable(cur_state.ids[i].asInteger());

      has_last = true;
      break;
    }

    // Add initial prior
    if (!has_last) {
      addAmbiguityResidualBlock(cur_state.ids[i], 
        *graph_->parameterBlockPtr(cur_state.ids[i].asInteger())->parameters(), 
        gnss_base_options_.error_parameter.initial_ambiguity);
    }
  }
}

// Number of pseudorange errors
size_t GnssEstimatorBase::numPseudorangeError(const State& state)
{
  size_t num = 0;
  const BackendId& parameter_id = state.id_in_graph;
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPseudorangeError || 
          type == ErrorType::kPseudorangeErrorSD || 
          type == ErrorType::kPseudorangeErrorDD)) continue;
    num++;
  }
  return num;
}

// Number of phaserange errors
size_t GnssEstimatorBase::numPhaserangeError(const State& state)
{
  size_t num = 0;
  const BackendId& parameter_id = state.id_in_graph;
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPhaserangeError || 
          type == ErrorType::kPhaserangeErrorSD || 
          type == ErrorType::kPhaserangeErrorDD)) continue;
    num++;
  }
  return num;
}

// Number of doppler errors
size_t GnssEstimatorBase::numDopplerError(const State& state)
{
  size_t num = 0;
  const BackendId& parameter_id = state.id_in_graph;
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kDopplerError)) continue;
    num++;
  }
  return num;
}

// Reject pseudorange outlier
size_t GnssEstimatorBase::rejectPseudorangeOutlier(const State& state, bool reject_one)
{
  const BackendId& parameter_id = state.id_in_graph;

  // check residual
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  std::vector<double> residuals;
  std::unordered_map<size_t, ceres::ResidualBlockId> residual_index_to_id;
  std::unordered_map<size_t, std::shared_ptr<ErrorInterface>> residual_index_to_interface;
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPseudorangeError || 
          type == ErrorType::kPseudorangeErrorSD || 
          type == ErrorType::kPseudorangeErrorDD)) continue;
    double residual[1];
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    interface->deNormalizeResidual(residual);
    residuals.push_back(*residual);
    residual_index_to_id.insert(std::make_pair(
      residuals.size() - 1, residual_block.residual_block_id)); 
    residual_index_to_interface.insert(std::make_pair(
      residuals.size() - 1, residual_block.error_interface_ptr));
  }
  if (residuals.size() == 0) return 0;
  double residuals_median = getMedian(residuals);
  for (size_t i = 0; i < residuals.size(); i++) {
    residuals[i] -= residuals_median;
  } 
  
  std::vector<int> outlier_indexes;
  for (size_t i = 0; i < residuals.size(); i++) {
    if (fabs(residuals[i]) > gnss_base_options_.max_pesudorange_error) {
      outlier_indexes.push_back(i);
    }
  }
  // outlier detected, remove this residual block and re-optimize
  if (outlier_indexes.size() > 0) {
    std::vector<size_t> indexes_to_remove;
    // only reject one outlier
    if (reject_one) {
      // find a largest outlier
      size_t largest_outlier_index = 0;
      double largest_outlier = 0.0;
      for (auto index : outlier_indexes) {
        double residual = fabs(residuals[index]);
        if (residual > largest_outlier) {
          largest_outlier = residual;
          largest_outlier_index = index;
        }
      }
      indexes_to_remove.push_back(largest_outlier_index);
    }
    // reject all outliers
    else {
      for (auto index : outlier_indexes) {
        indexes_to_remove.push_back(index);
      }
    }
    // apply rejection
    for (auto index : indexes_to_remove) {
      graph_->removeResidualBlock(
        residual_index_to_id.at(static_cast<size_t>(index)));
      if (base_options_.verbose_output) 
      {
        std::string info_message;
        std::shared_ptr<ErrorInterface> error_interface = 
          residual_index_to_interface.at(static_cast<size_t>(index));
        ErrorType type = error_interface->typeInfo();
        char system;
        int code_type;
        std::string code_str;
#define MAP(S, R, C) \
  if (system == S && code_type == C) { code_str = R; }
        if (type == ErrorType::kPseudorangeError) {
          GnssMeasurementIndex index = 
            getGnssMeasurementIndexFromErrorInterface(error_interface);
          system = index.prn[0];
          code_type = index.code_type;
          RINEX_TO_CODE_MAPS;
          info_message += " at " + index.prn + "|" + code_str;
        }
        if (type == ErrorType::kPseudorangeErrorSD) {
          GnssMeasurementSDIndexPair index = 
            getGnssMeasurementSDIndexPairFromErrorInterface(error_interface);
          system = index.rov.prn[0];
          code_type = index.rov.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_rov = code_str;
          code_type = index.ref.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_ref = code_str;
          info_message += " at " + index.rov.prn + "|" + 
            code_str_rov + "&" + code_str_ref;
        }
        if (type == ErrorType::kPseudorangeErrorDD) {
          GnssMeasurementDDIndexPair index = 
            getGnssMeasurementDDIndexPairFromErrorInterface(error_interface);
          system = index.rov.prn[0];
          code_type = index.rov.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_rov = code_str;
          code_type = index.ref.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_ref = code_str;
          code_type = index.rov_base.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_rov_base = code_str;
          code_type = index.ref_base.code_type;
          RINEX_TO_CODE_MAPS;
          std::string code_str_ref_base = code_str;
          info_message += " at " + index.rov.prn + "|" + 
            code_str_rov + "&" + code_str_ref + "-" + index.rov_base.prn + 
            "|" + code_str_rov_base + "&" + code_str_ref_base;
        }
#undef MAP
        LOG(INFO) << "Rejected pseudorange outlier" << info_message
                  << ": residual = " << std::fixed << residuals[index];
      }
    }
    return indexes_to_remove.size();
  }

  // no outlier
  return 0;
}

// Reject pseudorange outlier together with corresponding ambiguity parameter
size_t GnssEstimatorBase::rejectPseudorangeOutlier(
  const State& state, AmbiguityState& ambiguity_state, 
  bool reject_one)
{
  if (ambiguity_state.ids.size() == 0) {
    return rejectPseudorangeOutlier(state, reject_one);
  }

  const BackendId& parameter_id = state.id_in_graph;

  // check residual
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  std::vector<double> residuals;
  std::unordered_map<size_t, ceres::ResidualBlockId> residual_index_to_id;
  std::unordered_map<size_t, std::shared_ptr<ErrorInterface>> residual_index_to_interface;
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPseudorangeError || 
          type == ErrorType::kPseudorangeErrorSD || 
          type == ErrorType::kPseudorangeErrorDD)) continue;
    double residual[1];
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    interface->deNormalizeResidual(residual);
    residuals.push_back(*residual);
    residual_index_to_id.insert(std::make_pair(
      residuals.size() - 1, residual_block.residual_block_id)); 
    residual_index_to_interface.insert(std::make_pair(
      residuals.size() - 1, residual_block.error_interface_ptr));
  }
  if (residuals.size() == 0) return 0;
  double residuals_median = getMedian(residuals);
  for (size_t i = 0; i < residuals.size(); i++) {
    residuals[i] -= residuals_median;
  } 
  
  std::vector<int> outlier_indexes;
  for (size_t i = 0; i < residuals.size(); i++) {
    if (fabs(residuals[i]) > gnss_base_options_.max_pesudorange_error) {
      outlier_indexes.push_back(i);
    }
  }
  // outlier detected, remove this residual block and re-optimize
  if (outlier_indexes.size() > 0) {
    std::vector<size_t> indexes_to_remove;
    // only reject one outlier
    if (reject_one) {
      // find a largest outlier
      size_t largest_outlier_index = 0;
      double largest_outlier = 0.0;
      for (auto index : outlier_indexes) {
        double residual = fabs(residuals[index]);
        if (residual > largest_outlier) {
          largest_outlier = residual;
          largest_outlier_index = index;
        }
      }
      indexes_to_remove.push_back(largest_outlier_index);
    }
    // reject all outliers
    else {
      for (auto index : outlier_indexes) {
        indexes_to_remove.push_back(index);
      }
    }
    // apply rejection
    for (auto index : indexes_to_remove) {
      graph_->removeResidualBlock(
        residual_index_to_id.at(static_cast<size_t>(index)));

      // erase corresponding ambiguity parameter(s)
      std::string info_message;
      std::shared_ptr<ErrorInterface> error_interface = 
        residual_index_to_interface.at(static_cast<size_t>(index));
      ErrorType type = error_interface->typeInfo();
      char system;
      int code_type;
      std::string code_str;
#define MAP(S, R, C) \
  if (system == S && code_type == C) { code_str = R; }
      if (type == ErrorType::kPseudorangeError) {
        GnssMeasurementIndex index = 
          getGnssMeasurementIndexFromErrorInterface(error_interface);
        system = index.prn[0];
        code_type = index.code_type;
        RINEX_TO_CODE_MAPS;
        info_message += " at " + index.prn + "|" + code_str;

        int phase_id = gnss_common::getPhaseID(system, code_type);
        for (auto it = ambiguity_state.ids.begin(); it != ambiguity_state.ids.end(); ) {
          auto& ambiguity_id = *it;
          if (!sameAmbiguity(ambiguity_id, 
              createGnssAmbiguityId(index.prn, phase_id, 0))) {
            it++; continue;
          }
          if (graph_->parameterBlockExists(ambiguity_id.asInteger())) {
            graph_->removeParameterBlock(ambiguity_id.asInteger());
          }
          it = ambiguity_state.ids.erase(it);
        }
      }
      if (type == ErrorType::kPseudorangeErrorSD) {
        GnssMeasurementSDIndexPair index = 
          getGnssMeasurementSDIndexPairFromErrorInterface(error_interface);
        system = index.rov.prn[0];
        code_type = index.rov.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_rov = code_str;
        code_type = index.ref.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_ref = code_str;
        info_message += " at " + index.rov.prn + "|" + 
          code_str_rov + "&" + code_str_ref;

        int phase_id = gnss_common::getPhaseID(system, code_type);
        for (auto it = ambiguity_state.ids.begin(); it != ambiguity_state.ids.end(); ) {
          auto& ambiguity_id = *it;
          if (!sameAmbiguity(ambiguity_id, 
              createGnssAmbiguityId(index.rov.prn, phase_id, 0))) {
            it++; continue;
          }
          if (graph_->parameterBlockExists(ambiguity_id.asInteger())) {
            graph_->removeParameterBlock(ambiguity_id.asInteger());
          }
          it = ambiguity_state.ids.erase(it);
        }
      }
      if (type == ErrorType::kPseudorangeErrorDD) {
        GnssMeasurementDDIndexPair index = 
          getGnssMeasurementDDIndexPairFromErrorInterface(error_interface);
        system = index.rov.prn[0];
        code_type = index.rov.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_rov = code_str;
        code_type = index.ref.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_ref = code_str;
        code_type = index.rov_base.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_rov_base = code_str;
        code_type = index.ref_base.code_type;
        RINEX_TO_CODE_MAPS;
        std::string code_str_ref_base = code_str;
        info_message += " at " + index.rov.prn + "|" + 
          code_str_rov + "&" + code_str_ref + "-" + index.rov_base.prn + 
          "|" + code_str_rov_base + "&" + code_str_ref_base;

        int phase_id = gnss_common::getPhaseID(system, code_type);
        for (auto it = ambiguity_state.ids.begin(); it != ambiguity_state.ids.end(); ) {
          auto& ambiguity_id = *it;
          if (!sameAmbiguity(ambiguity_id, 
              createGnssAmbiguityId(index.rov.prn, phase_id, 0))) {
            it++; continue;
          }
          if (graph_->parameterBlockExists(ambiguity_id.asInteger())) {
            graph_->removeParameterBlock(ambiguity_id.asInteger());
          }
          it = ambiguity_state.ids.erase(it);
        }
      }
#undef MAP

      if (base_options_.verbose_output) {
          LOG(INFO) << "Rejected pseudorange outlier with ambiguities" << info_message
                    << ": residual = " << std::fixed << residuals[index];
      }
    }

    return indexes_to_remove.size();
  }

  // no outlier
  return 0;
}

// Reject phaserange outlier
size_t GnssEstimatorBase::rejectPhaserangeOutlier(
  const State& state, AmbiguityState& ambiguity_state, bool reject_one)
{
  const BackendId& parameter_id = state.id_in_graph;

  // check residual
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  std::vector<double> residuals;
  std::unordered_map<size_t, ceres::ResidualBlockId> residual_index_to_id;
  std::unordered_map<size_t, std::shared_ptr<ErrorInterface>> residual_index_to_interface;
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPhaserangeError || 
          type == ErrorType::kPhaserangeErrorSD || 
          type == ErrorType::kPhaserangeErrorDD)) continue;
    double residual[1];
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    interface->deNormalizeResidual(residual);
    residuals.push_back(*residual);
    residual_index_to_id.insert(std::make_pair(
      residuals.size() - 1, residual_block.residual_block_id)); 
    residual_index_to_interface.insert(std::make_pair(
      residuals.size() - 1, residual_block.error_interface_ptr));
  }
  if (residuals.size() == 0) return 0;
  double residuals_median = getMedian(residuals);
  for (size_t i = 0; i < residuals.size(); i++) {
    residuals[i] -= residuals_median;
  } 
  std::vector<int> outlier_indexes;
  for (size_t i = 0; i < residuals.size(); i++) {
    if (fabs(residuals[i]) > gnss_base_options_.max_phaserange_error) {
      outlier_indexes.push_back(i);
    }
  }
  // outlier detected, remove this residual block and the corresponding ambiguity 
  // parameter and re-optimize
  if (outlier_indexes.size() > 0) {
    std::vector<size_t> indexes_to_remove;
    // only reject one outlier
    if (reject_one) {
      // find a largest outlier
      size_t largest_outlier_index = 0;
      double largest_outlier = 0.0;
      for (auto index : outlier_indexes) {
        double residual = fabs(residuals[index]);
        if (residual > largest_outlier) {
          largest_outlier = residual;
          largest_outlier_index = index;
        }
      }
      indexes_to_remove.push_back(largest_outlier_index);
    }
    // reject all outliers
    else {
      for (auto index : outlier_indexes) {
        indexes_to_remove.push_back(index);
      }
    }
    // apply rejection
    for (auto index : indexes_to_remove) {
      // get corresponding ambiguity parameter
      Graph::ParameterBlockCollection parameters = graph_->parameters(
        residual_index_to_id.at(static_cast<size_t>(index)));
      BackendId id;
      for (auto& parameter : parameters) {
        id = BackendId(parameter.first);
        if (id.type() != IdType::gAmbiguity) continue; 

        // check if reference satellite
        Graph::ResidualBlockCollection residuals = graph_->residuals(id.asInteger());
        CHECK(residuals.size() > 0);
        int num_phaserange_block = 0;
        for (size_t r = 0; r < residuals.size(); ++r) {
          ErrorType type = residuals[r].error_interface_ptr->typeInfo();
          if (type == ErrorType::kPhaserangeError ||
              type == ErrorType::kPhaserangeErrorSD ||
              type == ErrorType::kPhaserangeErrorDD) {
            num_phaserange_block++;
          }
        }
        if (num_phaserange_block > 1) continue;

        graph_->removeParameterBlock(id.asInteger());
        for (auto it = ambiguity_state.ids.begin(); 
            it != ambiguity_state.ids.end(); it++) {
          if (*it == id) {
            it = ambiguity_state.ids.erase(it); break;
          }
        }
      }

      // remove residual block. We do not need to call removeResidualBlock() here because the 
      // residual block has already been removed when calling removeParameterBlock()
      if (base_options_.verbose_output) 
      {
        std::string info_message;
        std::shared_ptr<ErrorInterface> error_interface = 
          residual_index_to_interface.at(static_cast<size_t>(index));
        ErrorType type = error_interface->typeInfo();
        char system;
        int phase_id;
        std::string phase_str;
#define MAP(S, P, PS) \
    if (system == S && phase_id == P) { phase_str = PS; }
        if (type == ErrorType::kPhaserangeError) {
          GnssMeasurementIndex index = 
            getGnssMeasurementIndexFromErrorInterface(error_interface);
          system = index.prn[0];
          phase_id = gnss_common::getPhaseID(system, index.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          info_message += " at " + index.prn + "|" + phase_str;
        }
        if (type == ErrorType::kPhaserangeErrorSD) {
          GnssMeasurementSDIndexPair index = 
            getGnssMeasurementSDIndexPairFromErrorInterface(error_interface);
          system = index.rov.prn[0];
          phase_id = gnss_common::getPhaseID(system, index.rov.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_rov = phase_str;
          phase_id = gnss_common::getPhaseID(system, index.ref.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_ref = phase_str;
          info_message += " at " + index.rov.prn + "|" + 
            phase_str_rov + "&" + phase_str_ref;
        }
        if (type == ErrorType::kPhaserangeErrorDD) {
          GnssMeasurementDDIndexPair index = 
            getGnssMeasurementDDIndexPairFromErrorInterface(error_interface);
          system = index.rov.prn[0];
          phase_id = gnss_common::getPhaseID(system, index.rov.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_rov = phase_str;
          phase_id = gnss_common::getPhaseID(system, index.ref.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_ref = phase_str;
          phase_id = gnss_common::getPhaseID(system, index.rov_base.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_rov_base = phase_str;
          phase_id = gnss_common::getPhaseID(system, index.ref_base.code_type);
          PHASE_CHANNEL_TO_STR_MAPS;
          std::string phase_str_ref_base = phase_str;
          info_message += " at " + index.rov.prn + "|" + 
            phase_str_rov + "&" + phase_str_ref + "-" + index.rov_base.prn + 
            "|" + phase_str_rov_base + "&" + phase_str_ref_base;
        }
#undef MAP
        LOG(INFO) << "Rejected phaserange outlier" << info_message
                  << ": residual = " << std::fixed << residuals[index];
      }
    }

    // Check if any ambiguity parameter do not have residual blocks now
    for (auto it = ambiguity_state.ids.begin(); 
        it != ambiguity_state.ids.end(); ) {
      BackendId id = *it;
      Graph::ResidualBlockCollection residuals = graph_->residuals(id.asInteger());
      CHECK(residuals.size() > 0);
      int num_phaserange_block = 0;
      for (size_t r = 0; r < residuals.size(); ++r) {
        ErrorType type = residuals[r].error_interface_ptr->typeInfo();
        if (type == ErrorType::kPhaserangeError ||
            type == ErrorType::kPhaserangeErrorSD ||
            type == ErrorType::kPhaserangeErrorDD) {
          num_phaserange_block++;
        }
      }
      if (num_phaserange_block == 0) {
        graph_->removeParameterBlock(it->asInteger());
        it = ambiguity_state.ids.erase(it);
      }
      else it++;
    }

    return indexes_to_remove.size();
  }
  
  // no outlier
  return 0;
}

// Reject doppler outlier
size_t GnssEstimatorBase::rejectDopplerOutlier(const State& state, bool reject_one)
{
  const BackendId& parameter_id = state.id_in_graph;

  // check residual
  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  std::vector<double> residuals;
  std::unordered_map<size_t, ceres::ResidualBlockId> residual_index_to_id;
  std::unordered_map<size_t, std::shared_ptr<ErrorInterface>> residual_index_to_interface;
  for (size_t i = 0; i < residual_blocks.size(); i++) {
    auto& residual_block = residual_blocks[i];
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kDopplerError)) continue;
    double residual[1];
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    interface->deNormalizeResidual(residual);
    residuals.push_back(*residual);
    residual_index_to_id.insert(std::make_pair(
      residuals.size() - 1, residual_block.residual_block_id)); 
    residual_index_to_interface.insert(std::make_pair(
      residuals.size() - 1, residual_block.error_interface_ptr));
  }
  if (residuals.size() == 0) return 0;
  double residuals_median = getMedian(residuals);
  for (size_t i = 0; i < residuals.size(); i++) {
    residuals[i] -= residuals_median;
  } 
  
  std::vector<int> outlier_indexes;
  for (size_t i = 0; i < residuals.size(); i++) {
    if (fabs(residuals[i]) > gnss_base_options_.max_doppler_error) {
      outlier_indexes.push_back(i);
    }
  }
  // outlier detected, remove this residual block and re-optimize
  if (outlier_indexes.size() > 0) {
    std::vector<size_t> indexes_to_remove;
    // only reject one outlier
    if (reject_one) {
      // find a largest outlier
      size_t largest_outlier_index = 0;
      double largest_outlier = 0.0;
      for (auto index : outlier_indexes) {
        double residual = fabs(residuals[index]);
        if (residual > largest_outlier) {
          largest_outlier = residual;
          largest_outlier_index = index;
        }
      }
      indexes_to_remove.push_back(largest_outlier_index);
    }
    // reject all outliers
    else {
      for (auto index : outlier_indexes) {
        indexes_to_remove.push_back(index);
      }
    }
    // apply rejection
    for (auto index : indexes_to_remove) {
      graph_->removeResidualBlock(
        residual_index_to_id.at(static_cast<size_t>(index)));
      if (base_options_.verbose_output) 
      {
        std::string info_message;
        std::shared_ptr<ErrorInterface> error_interface = 
          residual_index_to_interface.at(static_cast<size_t>(index));
        ErrorType type = error_interface->typeInfo();
        char system;
        int code_type;
        std::string code_str;
#define MAP(S, R, C) \
  if (system == S && code_type == C) { code_str = R; }
        if (type == ErrorType::kDopplerError) {
          GnssMeasurementIndex index = 
            getGnssMeasurementIndexFromErrorInterface(error_interface);
          system = index.prn[0];
          code_type = index.code_type;
          RINEX_TO_CODE_MAPS;
          info_message += " at " + index.prn + "|" + code_str;
        }
#undef MAP
        LOG(INFO) << "Rejected doppler outlier" << info_message
                  << ": residual = " << std::fixed << residuals[index];
      }
    }
    return indexes_to_remove.size();
  }

  // no outlier
  return 0;
}

// Add position block to marginalizer
void GnssEstimatorBase::addGnssPositionMarginBlockWithResiduals(const State& state, bool keep)
{
  BackendId position_id = state.id;
  CHECK(position_id.type() == IdType::gPosition);
  if (graph_->parameterBlockExists(position_id.asInteger())) {
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(position_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_->addResidualBlock(
            residuals[r].residual_block_id);
    }
    marginalization_parameter_ids_.push_back(position_id);
    marginalization_keep_parameter_blocks_.push_back(keep);
  }
}

// Add velocity block to marginalizer
void GnssEstimatorBase::addGnssVelocityMarginBlockWithResiduals(const State& state, bool keep)
{
  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (graph_->parameterBlockExists(velocity_id.asInteger())) {
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(velocity_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_->addResidualBlock(
            residuals[r].residual_block_id);
    }
    marginalization_parameter_ids_.push_back(velocity_id);
    marginalization_keep_parameter_blocks_.push_back(keep);
  }
}

// Add clock blocks to marginalizer
void GnssEstimatorBase::addClockMarginBlocksWithResiduals(const State& state, bool keep)
{
  const BackendId& parameter_id = state.id;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) 
  {
    const char system = getGnssSystemList()[i];
    BackendId clock_id = changeIdType(parameter_id, IdType::gClock, system);
    if (graph_->parameterBlockExists(clock_id.asInteger())) {
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(clock_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_error_->addResidualBlock(
              residuals[r].residual_block_id);
      }
      marginalization_parameter_ids_.push_back(clock_id);
      marginalization_keep_parameter_blocks_.push_back(keep);
    }
  }
}

// Add frequency blocks to marginalizer
void GnssEstimatorBase::addFrequencyMarginBlocksWithResiduals(const State& state, bool keep)
{
  const BackendId& parameter_id = state.id;
  for (auto system : getGnssSystemList()) {
    BackendId freq_id = changeIdType(parameter_id, IdType::gFrequency, system);
    if (graph_->parameterBlockExists(freq_id.asInteger())) {
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(freq_id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_error_->addResidualBlock(
              residuals[r].residual_block_id);
      }
      marginalization_parameter_ids_.push_back(freq_id);
      marginalization_keep_parameter_blocks_.push_back(keep);
    }
  }
}

// Add troposphere block to marginalizer
void GnssEstimatorBase::addTroposphereMarginBlockWithResiduals(const State& state, bool keep)
{
  const BackendId& parameter_id = state.id;
  BackendId tropo_id = changeIdType(parameter_id, IdType::gTroposphere);
  if (graph_->parameterBlockExists(tropo_id.asInteger())) {
    Graph::ResidualBlockCollection residuals = 
      graph_->residuals(tropo_id.asInteger());
    for (size_t r = 0; r < residuals.size(); ++r) {
      marginalization_error_->addResidualBlock(
            residuals[r].residual_block_id);
    }
    marginalization_parameter_ids_.push_back(tropo_id);
    marginalization_keep_parameter_blocks_.push_back(keep);
  }
}

// Add ionosphere blocks to marginalizer
void GnssEstimatorBase::addIonosphereMarginBlocksWithResiduals(
  const IonosphereState& state, bool keep)
{
  for (auto id : state.ids) {
    if (graph_->parameterBlockExists(id.asInteger())) {
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_error_->addResidualBlock(
              residuals[r].residual_block_id);
      }
      marginalization_parameter_ids_.push_back(id);
      marginalization_keep_parameter_blocks_.push_back(keep);
    }
  }
}

// Add ambiguity blocks and its residuals to marginalizer
void GnssEstimatorBase::addAmbiguityMarginBlocksWithResiduals(
  const AmbiguityState& state, bool keep)
{
  for (auto id : state.ids) {
    if (graph_->parameterBlockExists(id.asInteger())) {
      Graph::ResidualBlockCollection residuals = 
        graph_->residuals(id.asInteger());
      for (size_t r = 0; r < residuals.size(); ++r) {
        marginalization_error_->addResidualBlock(
              residuals[r].residual_block_id);
      }
      marginalization_parameter_ids_.push_back(id);
      marginalization_keep_parameter_blocks_.push_back(keep);
    }
  }
}

// Add GNSS measurement residual blocks to marginalizer
void GnssEstimatorBase::addGnssMeasurementResidualMarginBlocks(const State& state)
{
  const std::unordered_set<int> gnss_error_types = {
    static_cast<int>(ErrorType::kPseudorangeError),
    static_cast<int>(ErrorType::kPseudorangeErrorSD),
    static_cast<int>(ErrorType::kPseudorangeErrorDD),
    static_cast<int>(ErrorType::kPhaserangeError),
    static_cast<int>(ErrorType::kPhaserangeErrorSD),
    static_cast<int>(ErrorType::kPhaserangeErrorDD),
    static_cast<int>(ErrorType::kDopplerError)};

  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));
  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    int type = static_cast<int>(residuals[r].error_interface_ptr->typeInfo());
    if (gnss_error_types.find(type) == gnss_error_types.end()) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add all GNSS residual blocks to marginalizer
void GnssEstimatorBase::addGnssResidualMarginBlocks(const State& state)
{
  CHECK(graph_->parameterBlockExists(state.id_in_graph.asInteger()));

  const std::unordered_set<int> gnss_error_types = {
    static_cast<int>(ErrorType::kPseudorangeError),
    static_cast<int>(ErrorType::kPseudorangeErrorSD),
    static_cast<int>(ErrorType::kPseudorangeErrorDD),
    static_cast<int>(ErrorType::kPhaserangeError),
    static_cast<int>(ErrorType::kPhaserangeErrorSD),
    static_cast<int>(ErrorType::kPhaserangeErrorDD),
    static_cast<int>(ErrorType::kDopplerError),
    static_cast<int>(ErrorType::kAmbiguityError),
    static_cast<int>(ErrorType::kClockError),
    static_cast<int>(ErrorType::kFrequencyError),
    static_cast<int>(ErrorType::kTroposphereError),
    static_cast<int>(ErrorType::kIonosphereError),
    static_cast<int>(ErrorType::kRelativePositionAndVelocityError),
    static_cast<int>(ErrorType::kRelativePositionError),
    static_cast<int>(ErrorType::kRelativeFrequencyError),
    static_cast<int>(ErrorType::kRelativeTroposphereError),
    static_cast<int>(ErrorType::kRelativeIonosphereError),
    static_cast<int>(ErrorType::kRelativeAmbiguityError) };

  Graph::ResidualBlockCollection residuals = 
    graph_->residuals(state.id_in_graph.asInteger());
  for (size_t r = 0; r < residuals.size(); ++r) {
    int type = static_cast<int>(residuals[r].error_interface_ptr->typeInfo());
    if (gnss_error_types.find(type) == gnss_error_types.end()) continue;
    marginalization_error_->addResidualBlock(
          residuals[r].residual_block_id);
  }
}

// Add all GNSS loosely coupled residual blocks to marginalier
void GnssEstimatorBase::addGnssLooseResidualMarginBlocks(const State& state)
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

// Erase GNSS position block
void GnssEstimatorBase::eraseGnssPositionParameterBlock(const State& state)
{
  const BackendId& parameter_id = state.id;
  CHECK(parameter_id.type() == IdType::gPosition);
  if (graph_->parameterBlockExists(parameter_id.asInteger())) {
    graph_->removeParameterBlock(parameter_id.asInteger());
  }
}

// Erase velocity block from graph
void GnssEstimatorBase::eraseGnssVelocityParameterBlock(const State& state)
{
  const BackendId& parameter_id = state.id;
  BackendId velocity_id = changeIdType(parameter_id, IdType::gVelocity);
  if (graph_->parameterBlockExists(velocity_id.asInteger())) {
    graph_->removeParameterBlock(velocity_id.asInteger());
  }
}

// Erase GNSS extrinsics
void GnssEstimatorBase::eraseGnssExtrinsicsParameterBlock(BackendId& extrinsics_id)
{
  CHECK(extrinsics_id.type() == IdType::gExtrinsics);
  CHECK(graph_->parameterBlockExists(extrinsics_id.asInteger()));
  graph_->removeParameterBlock(extrinsics_id.asInteger());
  extrinsics_id = BackendId(0);
}

// Erase clock blocks
void GnssEstimatorBase::eraseClockParameterBlocks(const State& state)
{
  const BackendId& parameter_id = state.id;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) 
  {
    const char system = getGnssSystemList()[i];
    BackendId clock_id = changeIdType(parameter_id, IdType::gClock, system);
    if (graph_->parameterBlockExists(clock_id.asInteger())) {
      graph_->removeParameterBlock(clock_id.asInteger());
    }
  }
}

// Erase IFB blocks
void GnssEstimatorBase::eraseIfbParameterBlocks(std::vector<std::pair<char, int>>& ifbs)
{
  for (auto ifb : ifbs) {
    BackendId ifb_id = createGnssIfbId(ifb.first, ifb.second);
    if (graph_->parameterBlockExists(ifb_id.asInteger())) {
      graph_->removeParameterBlock(ifb_id.asInteger());
    }
  }
  ifbs.clear();
}

// Erase frequency blocks
void GnssEstimatorBase::eraseFrequencyParameterBlocks(
  const State& state, bool reform)
{
  const BackendId& parameter_id = state.id;
  for (auto system : getGnssSystemList()) {
    BackendId freq_id = changeIdType(parameter_id, IdType::gFrequency, system);
    if (graph_->parameterBlockExists(freq_id.asInteger())) {
      graph_->removeParameterBlock(freq_id.asInteger());
    }
  }
  
  // Reform connection if needed
  int index_lhs = -1, index_rhs = -1;
  for (size_t i = 0; i < states_.size(); i++) {
    if (!states_[i].valid()) continue;
    if (states_[i].id.type() != IdType::gPosition && 
        states_[i].id.type() != IdType::gPose) continue;
    if (!checkLargerEqual(states_[i].timestamp, state.timestamp)) {
      index_lhs = i;
    }
    if (!checkLessEqual(states_[i].timestamp, state.timestamp)) {
      index_rhs = i; break;
    }
  }
  if (reform && index_lhs != -1 && index_rhs != -1) {
    addRelativeFrequencyBlock(states_[index_lhs], states_[index_rhs]);
  }
}

// Erase troposphere blocks
void GnssEstimatorBase::eraseTroposphereParameterBlock(
  const State& state, bool reform)
{
  const BackendId& parameter_id = state.id;
  BackendId tropo_id = changeIdType(parameter_id, IdType::gTroposphere);
  if (graph_->parameterBlockExists(tropo_id.asInteger())) {
    graph_->removeParameterBlock(tropo_id.asInteger());
  }

  // Reform connection if needed
  int index_lhs = -1, index_rhs = -1;
  for (size_t i = 0; i < states_.size(); i++) {
    if (!states_[i].valid()) continue;
    if (states_[i].id.type() != IdType::gPosition && 
        states_[i].id.type() != IdType::gPose) continue;
    if (!checkLargerEqual(states_[i].timestamp, state.timestamp)) {
      index_lhs = i;
    }
    if (!checkLessEqual(states_[i].timestamp, state.timestamp)) {
      index_rhs = i; break;
    }
  }
  if (reform && index_lhs != -1 && index_rhs != -1) {
    addRelativeTroposphereResidualBlock(states_[index_lhs], states_[index_rhs]);
  }
}

// Erase ionosphere blocks
void GnssEstimatorBase::eraseIonosphereParameterBlocks(
  IonosphereState& state, bool reform)
{
  for (auto id : state.ids) {
    if (graph_->parameterBlockExists(id.asInteger())) {
      graph_->removeParameterBlock(id.asInteger());
    }
  }
  state.ids.clear();

  // Reform connection if needed
  int index_lhs = -1, index_rhs = -1;
  for (size_t i = 0; i < ionosphere_states_.size(); i++) {
    if (!checkLargerEqual(ionosphere_states_[i].timestamp, state.timestamp)) {
      index_lhs = i;
    }
    if (!checkLessEqual(ionosphere_states_[i].timestamp, state.timestamp)) {
      index_rhs = i; break;
    }
  }
  if (reform && index_lhs != -1 && index_rhs != -1) {
    addRelativeIonosphereResidualBlock(
      ionosphere_states_[index_lhs], ionosphere_states_[index_rhs]);
  }
}

// Erase ambiguity blocks
void GnssEstimatorBase::eraseAmbiguityParameterBlocks(
  AmbiguityState& state, bool reform)
{
  // Reform connection if needed. we should reform at first 
  // because we need to access the residuals connected to current state.
  int index_lhs = -1, index_rhs = -1;
  for (size_t i = 0; i < ambiguity_states_.size(); i++) {
    if (!checkLargerEqual(ambiguity_states_[i].timestamp, state.timestamp)) {
      index_lhs = i;
    }
    if (!checkLessEqual(ambiguity_states_[i].timestamp, state.timestamp)) {
      index_rhs = i; break;
    }
  }
  if (index_lhs != -1 && index_rhs != -1) {
    double dt = ambiguity_states_[index_rhs].timestamp - 
                ambiguity_states_[index_lhs].timestamp;
    CHECK(dt >= 0.0);
    double damb_error = gnss_base_options_.error_parameter.relative_ambiguity;
    CHECK(damb_error != 0.0);
    Eigen::Matrix<double, 1, 1> damb_covariance = 
      Eigen::Matrix<double, 1, 1>::Identity() * square(damb_error) * dt;

    for (auto id : state.ids) {
      if (!graph_->parameterBlockExists(id.asInteger())) continue;
      const auto& residuals = graph_->residuals(id.asInteger());
      std::vector<uint64_t> connected_ids;
      for (const auto& residual : residuals) {
        if (residual.error_interface_ptr->typeInfo() != 
            ErrorType::kRelativeAmbiguityError) continue;
        const auto& parameters = graph_->parameters(residual.residual_block_id);
        CHECK(parameters.size() == 2);
        for (const auto& parameter : parameters) {
          if (parameter.first != id.asInteger()) {
            connected_ids.push_back(parameter.first);
            break;
          }
        }
      }
      CHECK(connected_ids.size() <= 2);
      // has bilateral connections, we reconnect them
      if (reform && connected_ids.size() == 2) {
        // add noise for GPS L5 to absorb Inter-Frequency Clock Bias (IFCB)
        char system = BackendId(id).gSystem();
        int phase_id = BackendId(id).gPhaseId();
        if (system == 'G' && phase_id == PHASE_L5) {
          damb_covariance += Eigen::Matrix<double, 1, 1>::Identity() * 
            square(gnss_base_options_.error_parameter.relative_gps_ifcb) * dt;
        }

        uint64_t last_id, cur_id;
        if (BackendId(connected_ids[0]).bundleId() < 
            BackendId(connected_ids[1]).bundleId()) {
          last_id = connected_ids[0]; cur_id = connected_ids[1];
        }
        else {
          last_id = connected_ids[1]; cur_id = connected_ids[0];
        }

        std::shared_ptr<RelativeAmbiguityError> relative_amb_error = 
          std::make_shared<RelativeAmbiguityError>(damb_covariance.inverse());
        graph_->addResidualBlock(relative_amb_error, nullptr,
          graph_->parameterBlockPtr(last_id),
          graph_->parameterBlockPtr(cur_id));

        // reset initial value
        *graph_->parameterBlockPtr(cur_id)->parameters() = 
          *graph_->parameterBlockPtr(last_id)->parameters();
        graph_->setParameterBlockVariable(cur_id);
      }
    }
  }

  // Erase states
  for (auto id : state.ids) {
    if (graph_->parameterBlockExists(id.asInteger())) {
      graph_->removeParameterBlock(id.asInteger());
    }
  }
  state.ids.clear();
}

// Erase all pseudorange residual blocks
void GnssEstimatorBase::erasePseudorangeResidualBlocks(const State& state)
{
  const BackendId& parameter_id = state.id;

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (auto residual_block : residual_blocks) {
    if (residual_block.error_interface_ptr->typeInfo() == 
        ErrorType::kPseudorangeError || 
        residual_block.error_interface_ptr->typeInfo() == 
        ErrorType::kPseudorangeErrorSD || 
        residual_block.error_interface_ptr->typeInfo() == 
        ErrorType::kPseudorangeErrorDD) {
      graph_->removeResidualBlock(residual_block.residual_block_id);
    }
  }
}

// Erase GNSS measurement residual blocks
void GnssEstimatorBase::eraseGnssMeasurementResidualBlocks(const State& state)
{
  const std::unordered_set<int> gnss_error_types = {
    static_cast<int>(ErrorType::kPseudorangeError),
    static_cast<int>(ErrorType::kPseudorangeErrorSD),
    static_cast<int>(ErrorType::kPseudorangeErrorDD),
    static_cast<int>(ErrorType::kPhaserangeError),
    static_cast<int>(ErrorType::kPhaserangeErrorSD),
    static_cast<int>(ErrorType::kPhaserangeErrorDD),
    static_cast<int>(ErrorType::kDopplerError)};

  const BackendId& parameter_id = state.id;

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(parameter_id.asInteger());
  for (auto residual_block : residual_blocks) {
    int type = static_cast<int>(residual_block.error_interface_ptr->typeInfo());
    if (gnss_error_types.find(type) == gnss_error_types.end()) continue;
    graph_->removeResidualBlock(residual_block.residual_block_id);
  }
}

// Erase GNSS position and velocity residual block
void GnssEstimatorBase::eraseGnssLooseResidualBlocks(const State& state)
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

// Erase relative position residual blocks
void GnssEstimatorBase::eraseRelativePositionResidualBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(cur_id.asInteger());
  for (auto residual_block : residual_blocks) {
    if (residual_block.error_interface_ptr->typeInfo() != 
        ErrorType::kRelativePositionError) continue;
    auto parameters = graph_->parameters(residual_block.residual_block_id);
    bool found = false;
    for (auto parameter : parameters) {
      if (parameter.first == last_id.asInteger()) {
        found = true; break;
      }
    }
    if (!found) continue;
    graph_->removeResidualBlock(residual_block.residual_block_id);
  }
}

// Erase relative position and velocity residual blocks
void GnssEstimatorBase::eraseRelativePositionAndVelocityBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(cur_id.asInteger());
  for (auto residual_block : residual_blocks) {
    if (residual_block.error_interface_ptr->typeInfo() != 
        ErrorType::kRelativePositionAndVelocityError) continue;
    auto parameters = graph_->parameters(residual_block.residual_block_id);
    bool found = false;
    for (auto parameter : parameters) {
      if (parameter.first == last_id.asInteger()) {
        found = true; break;
      }
    }
    if (!found) continue;
    graph_->removeResidualBlock(residual_block.residual_block_id);
  }
}

// Erase relative frequency residual blocks
void GnssEstimatorBase::eraseRelativeFrequencyBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  for (auto system : getGnssSystemList()) {
    BackendId last_freq_id = changeIdType(last_id, IdType::gFrequency, system);
    BackendId cur_freq_id = changeIdType(cur_id, IdType::gFrequency, system);

    if (!graph_->parameterBlockExists(last_freq_id.asInteger())) continue;
    if (!graph_->parameterBlockExists(cur_freq_id.asInteger())) continue;

    Graph::ResidualBlockCollection residual_blocks = 
      graph_->residuals(cur_freq_id.asInteger());
    for (auto residual_block : residual_blocks) {
      if (residual_block.error_interface_ptr->typeInfo() != 
          ErrorType::kRelativeFrequencyError) continue;
      auto parameters = graph_->parameters(residual_block.residual_block_id);
      bool found = false;
      for (auto parameter : parameters) {
        if (parameter.first == last_freq_id.asInteger()) {
          found = true; break;
        }
      }
      if (!found) continue;
      graph_->removeResidualBlock(residual_block.residual_block_id);
    }
  }
}

// Erase relative troposphere residual blocks
void GnssEstimatorBase::eraseRelativeTroposphereResidualBlock(
  const State& last_state, const State& cur_state)
{
  const BackendId last_id = last_state.id;
  const BackendId cur_id = cur_state.id;

  BackendId last_tropo_id = changeIdType(last_id, IdType::gTroposphere);
  BackendId cur_tropo_id = changeIdType(cur_id, IdType::gTroposphere);

  Graph::ResidualBlockCollection residual_blocks = 
    graph_->residuals(cur_tropo_id.asInteger());
  for (auto residual_block : residual_blocks) {
    if (residual_block.error_interface_ptr->typeInfo() != 
        ErrorType::kRelativeTroposphereError) continue;
    auto parameters = graph_->parameters(residual_block.residual_block_id);
    bool found = false;
    for (auto parameter : parameters) {
      if (parameter.first == last_tropo_id.asInteger()) {
        found = true; break;
      }
    }
    if (!found) continue;
    graph_->removeResidualBlock(residual_block.residual_block_id);
  }
}

// Erase relative ionosphere residual blocks
void GnssEstimatorBase::eraseRelativeIonosphereResidualBlock(
  const IonosphereState& last_state, const IonosphereState& cur_state)
{
  if (last_state.ids.size() == 0) return;
  for (size_t i = 0; i < cur_state.ids.size(); i++) {
    Graph::ResidualBlockCollection residual_blocks = 
      graph_->residuals(cur_state.ids[i].asInteger());
    for (auto residual_block : residual_blocks) {
      if (residual_block.error_interface_ptr->typeInfo() != 
          ErrorType::kRelativeIonosphereError) continue;
      auto parameters = graph_->parameters(residual_block.residual_block_id);
      bool found = false;
      for (auto parameter : parameters) {
        if (BackendId(parameter.first).bundleId() == 
            last_state.ids[0].bundleId()) {
          found = true; break;
        }
      }
      if (!found) continue;
      graph_->removeResidualBlock(residual_block.residual_block_id);
    }
  }
}

// Erase relative ambiguity residual blocks
void GnssEstimatorBase::eraseRelativeAmbiguityResidualBlock(
  const AmbiguityState& last_state, const AmbiguityState& cur_state)
{
  if (last_state.ids.size() == 0) return;
  for (size_t i = 0; i < cur_state.ids.size(); i++) {
    Graph::ResidualBlockCollection residual_blocks = 
      graph_->residuals(cur_state.ids[i].asInteger());
    for (auto residual_block : residual_blocks) {
      if (residual_block.error_interface_ptr->typeInfo() != 
          ErrorType::kRelativeAmbiguityError) continue;
      auto parameters = graph_->parameters(residual_block.residual_block_id);
      bool found = false;
      for (auto parameter : parameters) {
        if (BackendId(parameter.first).bundleId() == 
            last_state.ids[0].bundleId()) {
          found = true; break;
        }
      }
      if (!found) continue;
      graph_->removeResidualBlock(residual_block.residual_block_id);
    }
  }
}

// Convert from estimated states (in ENU) to body states
void GnssEstimatorBase::convertStateAndCovarianceToBody(
  Transformation* T_WS, SpeedAndBias* speed_and_bias, 
  Eigen::Matrix<double, 15, 15>* covariance)
{
  const Eigen::Vector3d pco = gnss_base_options_.common.receiver_pco;
  if (T_WS) {
    T_WS->getPosition() -= pco;
  }
}

// Get extrinsics estimate
Eigen::Vector3d GnssEstimatorBase::getGnssExtrinsicsEstimate()
{
  std::shared_ptr<PositionParameterBlock> block_ptr =
      std::static_pointer_cast<PositionParameterBlock>(
        graph_->parameterBlockPtr(gnss_extrinsics_id_.asInteger()));
  CHECK(block_ptr != nullptr);
  Eigen::Vector3d p = block_ptr->estimate().head<3>();
  return p;
}

// Get GNSS measurement index from error interface
GnssMeasurementIndex GnssEstimatorBase::getGnssMeasurementIndexFromErrorInterface(
  const std::shared_ptr<ErrorInterface>& error_interface)
{
  GnssMeasurementIndex index;
  if (error_interface->typeInfo() == ErrorType::kPseudorangeError)
  {
    if (!is_verbose_model_) {
      if (!is_state_pose_) {
        const std::shared_ptr<PseudorangeError<3, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeError<3, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
      else {
        const std::shared_ptr<PseudorangeError<7, 3, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeError<7, 3, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
    }
    else {
      if (!is_state_pose_) {
        const std::shared_ptr<PseudorangeError<3, 1, 1, 1, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeError<3, 1, 1, 1, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
      else {
        const std::shared_ptr<PseudorangeError<7, 3, 1, 1, 1, 1>> pseudorange_error = 
          std::static_pointer_cast<PseudorangeError<7, 3, 1, 1, 1, 1>>(error_interface);
        index = pseudorange_error->getGnssMeasurementIndex();
      }
    }
  }
  if (error_interface->typeInfo() == ErrorType::kPhaserangeError)
  {
    if (!is_state_pose_) {
      const std::shared_ptr<PhaserangeError<3, 1, 1, 1, 1>> phaserange_error = 
        std::static_pointer_cast<PhaserangeError<3, 1, 1, 1, 1>>(error_interface);
      index = phaserange_error->getGnssMeasurementIndex();
    }
    else {
      const std::shared_ptr<PhaserangeError<7, 3, 1, 1, 1, 1>> phaserange_error = 
        std::static_pointer_cast<PhaserangeError<7, 3, 1, 1, 1, 1>>(error_interface);
      index = phaserange_error->getGnssMeasurementIndex();
    }
  }
  if (error_interface->typeInfo() == ErrorType::kDopplerError)
  {
    if (!is_state_pose_) {
      const std::shared_ptr<DopplerError<3, 3, 1>> doppler_error = 
        std::static_pointer_cast<DopplerError<3, 3, 1>>(error_interface);
      index = doppler_error->getGnssMeasurementIndex();
    }
    else {
      const std::shared_ptr<DopplerError<7, 9, 3, 1>> doppler_error = 
        std::static_pointer_cast<DopplerError<7, 9, 3, 1>>(error_interface);
      index = doppler_error->getGnssMeasurementIndex();
    }
  }

  return index;
}

}