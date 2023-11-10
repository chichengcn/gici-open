/**
* @Function: Ambiguity Resolution
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/ambiguity_resolution.h"

#include "gici/gnss/gnss_common.h"
#include "gici/gnss/phaserange_error_sd.h"
#include "gici/gnss/ambiguity_error.h"

namespace gici {

// Solve ambiguity for single differenced ambiguities (with double differenced measurements)
AmbiguityResolution::Result AmbiguityResolution::solveRtk(
             const BackendId& epoch_id, 
             const std::vector<BackendId>& ambiguity_ids,
             const Eigen::MatrixXd& ambiguity_covariance,
             const std::pair<GnssMeasurement, GnssMeasurement>& measurements)
{
  // Prepare data 
  // Shift storage
  ambiguities_.push_back(std::vector<Spec>());
  ambiguity_pairs_.push_back(std::vector<BsdPair>());
  ambiguity_lane_pairs_.push_back(std::vector<LanePair>());
  if (ambiguities_.size() > 2) {
    ambiguities_.pop_front();
    ambiguity_pairs_.pop_front();
    ambiguity_lane_pairs_.pop_front();
  }
  full_parameters_store_.clear();
  lane_groups_.clear();

  // Get parameters
  const GnssMeasurement& measurements_rov = measurements.first;
  const GnssMeasurement& measurements_ref = measurements.second;
  std::unordered_map<size_t, size_t> ambiguity_index_map;
  for (size_t i = 0; i < ambiguity_ids.size(); i++) {
    char system = ambiguity_ids[i].gSystem();
    std::string prn = ambiguity_ids[i].gPrn();
    int phase_id = ambiguity_ids[i].gPhaseId();
    if (!gnss_common::useSystem(options_.system_exclude, system)) continue;
    if (!gnss_common::useSatellite(options_.satellite_exclude, prn)) continue;
    if (!gnss_common::usePhase(options_.phase_exclude, system, phase_id)) continue;

    const Eigen::Vector3d& position = measurements_rov.position;
    const Eigen::Vector3d& sat_position = 
      measurements_rov.satellites.at(prn).sat_position;
    double elevation = gnss_common::satelliteElevation(sat_position, position);
    if (gnss_common::radToDegree(elevation) < options_.min_elevation) continue;

    uint64_t id = ambiguity_ids[i].asInteger();
    Spec ambiguity;
    ambiguity.id = ambiguity_ids[i];
    ambiguity.prn = prn;
    CHECK(graph_->parameterBlockExists(id));

    // parameter and residual block
    ambiguity.parameter_block = graph_->parameterBlockPtr(id);
    ambiguity.value = *ambiguity.parameter_block->parameters();
    Graph::ResidualBlockCollection residuals = graph_->residuals(id);
    CHECK(residuals.size() > 0);
    // Check the number of phaserange errors. Ideally, for a non-reference satellite ambiguity 
    // parameter block, it only conresponds to one phaserange error block.
    int num_phaserange_block = 0;
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeError ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorSD ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorDD) {
        num_phaserange_block++;
      }
    }
    if (num_phaserange_block == 1)
    for (size_t r = 0; r < residuals.size(); ++r) {
      if (residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeError ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorSD ||
          residuals[r].error_interface_ptr->typeInfo() == ErrorType::kPhaserangeErrorDD) {
        ambiguity.residual_block = residuals[r];
        break;
      }
    }

    // wavelength
    const Satellite& satellite = measurements_rov.satellites.at(prn);
    for (auto obs : satellite.observations) {
      auto& observation = obs.second;
      if (gnss_common::getPhaseID(system, obs.first) == phase_id) {
        ambiguity.wavelength = observation.wavelength;
      }
    }

    // elevation angle for reference satellite selection
    ambiguity.elevation = elevation;

    curAmbs().push_back(ambiguity);
    ambiguity_index_map.insert(std::make_pair(curAmbs().size() - 1, i));
  }

  // Get covariance of ambiguity
  CHECK(ambiguity_covariance.cols() == ambiguity_ids.size());
  const size_t ambiguity_size = curAmbs().size();
  ambiguity_covariance_.resize(ambiguity_size, ambiguity_size);
  for (size_t i = 0; i < ambiguity_size; i++) {
    for (size_t j = 0; j < ambiguity_size; j++) {
      const size_t raw_i = ambiguity_index_map.at(i);
      const size_t raw_j = ambiguity_index_map.at(j);
      ambiguity_covariance_(i, j) = ambiguity_covariance(raw_i, raw_j);
    }
    curAmbs()[i].std = sqrt(ambiguity_covariance_(i, i));
  }

  // Collect all parameter blocks
  Graph::ParameterBlockCollection parameters = graph_->parameters();
  for (size_t p = 0; p < parameters.size(); p++) {
    Parameter parameter;
    parameter.id = BackendId(parameters[p].first);
    parameter.size = parameters[p].second->dimension();
    parameter.minimal_size = parameters[p].second->minimalDimension();
    parameter.value = Eigen::Map<Eigen::VectorXd>(
      parameters[p].second->parameters(), parameter.size);
    parameter.handle = parameters[p].second;
    if (full_parameters_store_.size() == 0) {
      parameter.covariance_start_index = 0;
    }
    else {
      Parameter& last_parameter = full_parameters_store_.back();
      parameter.covariance_start_index = 
        last_parameter.covariance_start_index + last_parameter.minimal_size;
    }
    full_parameters_store_.push_back(parameter);
  }

  // Sovle ambiguity
  // Apply Between-Satellite-Difference (BSD) and lane combination
  formSatellitePairRtk();

  // Sort lanes to groups
  groupingSatellitePair();
  if (lane_groups_.size() == 0) return Result::NoFix;

  // Try fix lanes
  int num_success_uwl = 0, num_candidate_uwl = 0; 
  int num_success_wl = 0, num_candidate_wl = 0; 
  int num_success_nl = 0, num_candidate_nl = 0;
  bool has_uwl = false, has_wl = false;
  for (int i = lane_groups_.size() - 1; i >= 0; i--) {
    LaneType type = curAmbLanePairs().at(lane_groups_.at(i).front()).laneType();
    double min_percentage_fixation;
    bool use_rounding = false;
    if (type == LaneType::UWL) {
      min_percentage_fixation = options_.min_percentage_fixation_uwl;
      use_rounding = false;
      has_uwl = true;
    }
    else if (type == LaneType::WL) {
      min_percentage_fixation = options_.min_percentage_fixation_wl;
      use_rounding = false;
      has_wl = true;
    }
    else {
      min_percentage_fixation = options_.min_percentage_fixation_nl;
      use_rounding = false;
    }

    int num_fixed = trySolveLanes(
      lane_groups_.at(i), min_percentage_fixation, use_rounding);

    if (type == LaneType::UWL) {
      num_success_uwl += num_fixed;
      num_candidate_uwl += lane_groups_.at(i).size();
    }
    else if (type == LaneType::WL) {
      num_success_wl += num_fixed;
      num_candidate_wl += lane_groups_.at(i).size();
    }
    else {
      num_success_nl += num_fixed;
      num_candidate_nl += lane_groups_.at(i).size();
    }
  }

  // Check if no valid fixation
  if (num_success_nl == 0 && num_success_wl == 0 && 
      num_success_uwl == 0) return Result::NoFix;

  // Apply in graph and check residual
  double range_cost_before = computeRangeCost(epoch_id);
  graph_->options.max_num_iterations = 1;
  graph_->options.logging_type = ceres::LoggingType::SILENT;
  graph_->options.minimizer_progress_to_stdout = false;
  graph_->solve();
  double range_cost = computeRangeCost(epoch_id);
  const double tolerance = 0.01;
  if (range_cost > range_cost_before + tolerance) {
    LOG(INFO) << "Invalid ambiguity resolution: Total cost changes from " 
      << std::scientific << std::setprecision(3) 
      << range_cost_before << " to " << range_cost << ".";
    setGraphParameters(full_parameters_store_);
    eraseAmbiguityResidualBlocks(curAmbLanePairs());
    return Result::NoFix;
  }

  // Check status
  if (num_success_nl > 0) return Result::NlFix;
  else if (num_success_wl > 0) return Result::WlFix;
  return Result::NoFix;
}

// Apply Between-Satellite-Difference (BSD) and lane combination for RTK
void AmbiguityResolution::formSatellitePairRtk()
{
  // Prepare data
  std::map<char, int> system_to_num_phases;
  std::multimap<char, double> system_to_wavelengths;
  std::map<std::string, int> prn_to_number_phases; 
  std::multimap<std::string, double> prn_to_wavelengths;
  std::multimap<std::string, Spec> prn_to_specs;
  std::map<uint64_t, size_t> id_to_spec_id;
  for (size_t i = 0; i < curAmbs().size(); i++) {
    Spec& ambiguity = curAmbs()[i];
    std::string prn = ambiguity.id.gPrn();

    auto it = prn_to_number_phases.find(prn);
    if (it == prn_to_number_phases.end()) {
      prn_to_number_phases.insert(std::make_pair(prn, 1));
    }
    else it->second++;
    prn_to_specs.insert(std::make_pair(prn, ambiguity));
    prn_to_wavelengths.insert(std::make_pair(prn, ambiguity.wavelength));
    id_to_spec_id.insert(std::make_pair(ambiguity.id.asInteger(), i));
  }
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];

    system_to_num_phases.insert(std::make_pair(system, 0));
    for (auto it : prn_to_number_phases) {
      if (it.first[0] != system) continue;
      if (system_to_num_phases.at(system) < it.second) {
        system_to_num_phases.at(system) = it.second;
      }
    }

    for (auto it : prn_to_wavelengths) {
      if (it.first[0] != system) continue;
      if (system_to_wavelengths.find(system) == system_to_wavelengths.end()) {
        system_to_wavelengths.insert(std::make_pair(system, it.second));
      }
      bool found = false;
      for (auto it_wave = system_to_wavelengths.lower_bound(system); 
          it_wave != system_to_wavelengths.upper_bound(system); it_wave++) {
        if (it_wave->second == it.second) {
          found = true; break;
        }
      }
      if (!found) system_to_wavelengths.insert(std::make_pair(system, it.second));
    }
  }

  // Find reference satellites for each system and frequencies
  std::map<char, std::string> system_to_reference_prn;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];

    // find satellite with maximum elevation angle
    double max_elevation = 0.0;
    for (size_t j = 0; j < curAmbs().size(); j++) {
      Spec& ambiguity = curAmbs()[j];
      if (ambiguity.id.gSystem() != system) continue;

      // we only select satellites with max phase number
      if (prn_to_number_phases.at(ambiguity.id.gPrn()) != 
          system_to_num_phases.at(system)) continue;

      if (max_elevation < ambiguity.elevation) {
        system_to_reference_prn[system] = ambiguity.id.gPrn();
        max_elevation = ambiguity.elevation;
      }
    }
  }

  // Form BSD pair
  for (size_t i = 0; i < curAmbs().size(); i++) {
    Spec& ambiguity = curAmbs()[i];
    char system = ambiguity.id.gSystem();
    std::string prn = ambiguity.id.gPrn();
    if (system_to_reference_prn.find(system) == 
        system_to_reference_prn.end()) continue;
    std::string prn_ref = system_to_reference_prn.at(system);

    if (prn == prn_ref) {
      ambiguity.is_reference = true;
      continue;
    }

    for (auto it = prn_to_specs.lower_bound(prn_ref); 
         it != prn_to_specs.upper_bound(prn_ref); it++) {
      Spec& ambiguity_ref = it->second;
      if (ambiguity_ref.id.gPhaseId() == ambiguity.id.gPhaseId()) {
        BsdPair ambiguity_pair(curAmbs(), i, id_to_spec_id.at(ambiguity_ref.id.asInteger()));
        curAmbPairs().push_back(ambiguity_pair);
        break;
      }
    }
  }

  // Widelane combination 
  std::map<char, double> system_to_base_wavelength;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];
    std::vector<double> wavelengths;
    auto it = system_to_wavelengths.lower_bound(system);
    if (it == system_to_wavelengths.end()) continue;
    for (; it != system_to_wavelengths.upper_bound(system); it++) {
      wavelengths.push_back(it->second);
    }
    if (wavelengths.size() == 0) continue;
    system_to_wavelengths.erase(system);
    std::sort(wavelengths.begin(), wavelengths.end());
    for (std::vector<double>::reverse_iterator wavelength = wavelengths.rbegin(); 
         wavelength != wavelengths.rend(); wavelength++) {
      system_to_wavelengths.insert(std::make_pair(system, *wavelength));
    }
    system_to_base_wavelength.insert(std::make_pair(system, wavelengths.front()));
  }

  // Form combinations
  for (size_t i = 0; i < curAmbPairs().size(); i++) {
    char system = curAmbs()[curAmbPairs()[i].spec_id].id.gSystem();
    std::string prn = curAmbs()[curAmbPairs()[i].spec_id].id.gPrn();

    // base frequency observation for NL AR
    if (curAmbPairs()[i].wavelength == system_to_base_wavelength.at(system)) {
      LanePair nl_pair(curAmbPairs(), i);
      curAmbLanePairs().push_back(nl_pair);
      curAmbPairs()[i].is_base_frequency = true;
    }

    // Widelanes
    double wavelength = curAmbPairs()[i].wavelength;
    // select the nearest frequency to combine
    double desired_wavelength_to_combine = 1.0e6;
    for (auto it = system_to_wavelengths.lower_bound(system); 
         it != system_to_wavelengths.upper_bound(system); it++) {
      if (it->second <= wavelength) continue;
      if (it->second < desired_wavelength_to_combine) {
        desired_wavelength_to_combine = it->second;
      }
    } 
    if (desired_wavelength_to_combine == 1.0e6) continue;
    for (size_t j = 0; j < curAmbPairs().size(); j++) {
      char system_j = curAmbs()[curAmbPairs()[j].spec_id].id.gSystem();
      if (system_j != system) continue;

      std::string prn_j = curAmbs()[curAmbPairs()[j].spec_id].id.gPrn();
      if (prn_j != prn) continue;
      
      double wavelength_j = curAmbPairs()[j].wavelength;
      if (wavelength_j != desired_wavelength_to_combine) continue;

      LanePair wl_pair(curAmbPairs(), i, j);
      curAmbLanePairs().push_back(wl_pair);
    }
  }
}

}