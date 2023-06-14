/**
* @Function: Base class for GNSS estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/gnss_estimator_base.h"

#include "gici/utility/global_variable.h"
#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/code_phase_maps.h"
#include "gici/gnss/pseudorange_error.h"
#include "gici/gnss/phaserange_error.h"

namespace gici {

// Create ambiguity logger
void GnssEstimatorBase::createAmbiguityLogger(const std::string& directory)
{
  double time = vk::Timer::getCurrentTime();
  double ep[6];
  char buf[32];
  time2epoch(gnss_common::doubleToGtime(time), ep);
  sprintf(buf, "%04d%02d%02d-%02d%02d%02d", 
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], (int)ep[5]);
  std::string path = directory + "/ambiguity-" + buf + ".log";
  ambiguity_logger_.open(path, std::ios::out | std::ios::trunc);
}

// Create ionosphere logger
void GnssEstimatorBase::createIonosphereLogger(const std::string& directory)
{
  double time = vk::Timer::getCurrentTime();
  double ep[6];
  char buf[32];
  time2epoch(gnss_common::doubleToGtime(time), ep);
  sprintf(buf, "%04d%02d%02d-%02d%02d%02d", 
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], (int)ep[5]);
  std::string path = directory + "/ionosphere-" + buf + ".log";
  ionosphere_logger_.open(path, std::ios::out | std::ios::trunc);
}

// Create pseudorange residual logger
void GnssEstimatorBase::createPseudorangeResidualLogger(const std::string& directory)
{
  double time = vk::Timer::getCurrentTime();
  double ep[6];
  char buf[32];
  time2epoch(gnss_common::doubleToGtime(time), ep);
  sprintf(buf, "%04d%02d%02d-%02d%02d%02d", 
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], (int)ep[5]);
  std::string path = directory + "/pseudorange_residual-" + buf + ".log";
  pseudorange_residual_logger_.open(path, std::ios::out | std::ios::trunc);
}

// Create phaserange residual logger
void GnssEstimatorBase::createPhaserangeResidualLogger(const std::string& directory)
{
  double time = vk::Timer::getCurrentTime();
  double ep[6];
  char buf[32];
  time2epoch(gnss_common::doubleToGtime(time), ep);
  sprintf(buf, "%04d%02d%02d-%02d%02d%02d", 
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], (int)ep[5]);
  std::string path = directory + "/phaserange_residual-" + buf + ".log";
  phaserange_residual_logger_.open(path, std::ios::out | std::ios::trunc);
}

// Log ambiguity estimate
void GnssEstimatorBase::logAmbiguityEstimate()
{
  // Get ambiguity estimate
  AmbiguityState& ambiguity_state = lastAmbiguityState();
  std::map<std::string, std::map<std::string, double>> ambiguity_estimates;
  for (auto id : ambiguity_state.ids) {
    std::shared_ptr<AmbiguityParameterBlock> block_ptr =
        std::static_pointer_cast<AmbiguityParameterBlock>(
          graph_->parameterBlockPtr(id.asInteger()));
    CHECK(block_ptr != nullptr);
    double value = block_ptr->estimate().x();
    std::string prn = id.gPrn();
    char system = prn[0];
    std::string phase_str;
#define MAP(S, P, PS) \
    if (system == S && id.gPhaseId() == P) { phase_str = PS; }
    PHASE_CHANNEL_TO_STR_MAPS;
#undef MAP
    if (ambiguity_estimates.find(prn) == ambiguity_estimates.end()) {
      ambiguity_estimates.insert(std::make_pair(prn, std::map<std::string, double>()));
    }
    std::map<std::string, double>& phase_to_value = ambiguity_estimates.at(prn);
    phase_to_value.insert(std::make_pair(phase_str, value));
  }

  // Log to file
  double ep[6];
  time2epoch(gnss_common::doubleToGtime(ambiguity_state.timestamp), ep);
  char time_buf[64];
  sprintf(time_buf, "> %04d %02d %02d %02d %02d %10.7lf  %17.7lf",
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5],
    ambiguity_state.timestamp);
  ambiguity_logger_ << time_buf << std::endl;
  for (auto i : ambiguity_estimates) {
    const std::string& prn = i.first;
    ambiguity_logger_ << prn << " ";
    for (auto j : i.second) {
      const std::string& phase_str = j.first;
      const double& value = j.second;
      ambiguity_logger_ << phase_str << " "
        << std::fixed << std::setprecision(4) << value << " ";
    }
    ambiguity_logger_ << std::endl;
  }
} 

// Log ionosphere estimate
void GnssEstimatorBase::logIonosphereEstimate()
{
  // Get ionosphere estimate
  IonosphereState& ionosphere_state = lastIonosphereState();
  std::map<std::string, double> ionosphere_estimates;
  for (auto id : ionosphere_state.ids) {
    std::shared_ptr<IonosphereParameterBlock> block_ptr =
        std::static_pointer_cast<IonosphereParameterBlock>(
          graph_->parameterBlockPtr(id.asInteger()));
    CHECK(block_ptr != nullptr);
    double value = block_ptr->estimate().x();
    std::string prn = id.gPrn();
    ionosphere_estimates.insert(std::make_pair(prn, value));
  }

  // Log to file
  double ep[6];
  time2epoch(gnss_common::doubleToGtime(ionosphere_state.timestamp), ep);
  char time_buf[64];
  sprintf(time_buf, "> %04d %02d %02d %02d %02d %10.7lf  %17.7lf",
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5],
    ionosphere_state.timestamp);
  ionosphere_logger_ << time_buf << std::endl;
  for (auto i : ionosphere_estimates) {
    const std::string& prn = i.first;
    const double& value = i.second;
    ionosphere_logger_ << prn << " " 
      << std::fixed << std::setprecision(4) << value << std::endl;
  }
} 

// Log pseudorange residual
void GnssEstimatorBase::logPseudorangeResidual()
{
  // Get residuals
  State& state = lastState();
  const auto& residuals = graph_->residuals(state.id.asInteger());
  std::map<std::string, std::map<std::string, double>> residual_values;
  for (const auto& residual : residuals) {
    const ceres::ResidualBlockId id = residual.residual_block_id;
    const std::shared_ptr<ErrorInterface>& interface = residual.error_interface_ptr;
    if (interface->typeInfo() != ErrorType::kPseudorangeError) continue;

    GnssMeasurementIndex index = 
      getGnssMeasurementIndexFromErrorInterface(interface);
    std::string prn = index.prn;
    std::string code_str = gnss_common::codeTypeToRinexType(prn[0], index.code_type);
    double residual_evaluate[1];
    graph_->problem_->EvaluateResidualBlock(
        id, false, nullptr, residual_evaluate, nullptr);
    interface->deNormalizeResidual(residual_evaluate);
    if (residual_values.find(prn) == residual_values.end()) {
      residual_values.insert(std::make_pair(prn, std::map<std::string, double>()));
    }
    std::map<std::string, double>& code_to_value = residual_values.at(prn);
    code_to_value.insert(std::make_pair(code_str, residual_evaluate[0]));
  }

  // Log to file
  double ep[6];
  time2epoch(gnss_common::doubleToGtime(state.timestamp), ep);
  char time_buf[64];
  sprintf(time_buf, "> %04d %02d %02d %02d %02d %10.7lf  %17.7lf",
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5],
    state.timestamp);
  pseudorange_residual_logger_ << time_buf << std::endl;
  for (auto i : residual_values) {
    const std::string& prn = i.first;
    pseudorange_residual_logger_ << prn << " ";
    for (auto j : i.second) {
      const std::string& code_str = j.first;
      const double& value = j.second;
      pseudorange_residual_logger_ << code_str << " "
        << std::fixed << std::setprecision(4) << value << " ";
    }
    pseudorange_residual_logger_ << std::endl;
  }
}

// Log phaserange residual
void GnssEstimatorBase::logPhaserangeResidual()
{
  // Get residuals
  State& state = lastState();
  const auto& residuals = graph_->residuals(state.id.asInteger());
  std::map<std::string, std::map<std::string, double>> residual_values;
  for (const auto& residual : residuals) {
    const ceres::ResidualBlockId id = residual.residual_block_id;
    const std::shared_ptr<ErrorInterface>& interface = residual.error_interface_ptr;
    if (interface->typeInfo() != ErrorType::kPhaserangeError) continue;

    GnssMeasurementIndex index = 
      getGnssMeasurementIndexFromErrorInterface(interface);
    std::string prn = index.prn;
    char system = prn[0];
    int phase_id = gnss_common::getPhaseID(system, index.code_type);
    std::string phase_str;
#define MAP(S, P, PS) \
    if (system == S && phase_id == P) { phase_str = PS; }
    PHASE_CHANNEL_TO_STR_MAPS;
#undef MAP
    double residual_evaluate[1];
    graph_->problem_->EvaluateResidualBlock(
        id, false, nullptr, residual_evaluate, nullptr);
    interface->deNormalizeResidual(residual_evaluate);
    if (residual_values.find(prn) == residual_values.end()) {
      residual_values.insert(std::make_pair(prn, std::map<std::string, double>()));
    }
    std::map<std::string, double>& phase_to_value = residual_values.at(prn);
    phase_to_value.insert(std::make_pair(phase_str, residual_evaluate[0]));
  }

  // Log to file
  double ep[6];
  time2epoch(gnss_common::doubleToGtime(state.timestamp), ep);
  char time_buf[64];
  sprintf(time_buf, "> %04d %02d %02d %02d %02d %10.7lf  %17.7lf",
    (int)ep[0], (int)ep[1], (int)ep[2], (int)ep[3], (int)ep[4], ep[5],
    state.timestamp);
  phaserange_residual_logger_ << time_buf << std::endl;
  for (auto i : residual_values) {
    const std::string& prn = i.first;
    phaserange_residual_logger_ << prn << " ";
    for (auto j : i.second) {
      const std::string& phase_str = j.first;
      const double& value = j.second;
      phaserange_residual_logger_ << phase_str << " "
        << std::fixed << std::setprecision(4) << value << " ";
    }
    phaserange_residual_logger_ << std::endl;
  }
}

// Free ambiguity logger
void GnssEstimatorBase::freeAmbiguityLogger()
{
  ambiguity_logger_.close();
}

// Free ionosphere logger
void GnssEstimatorBase::freeIonosphereLogger()
{
  ionosphere_logger_.close();
}

// Free pseudorange residual logger
void GnssEstimatorBase::freePseudorangeResidualLogger()
{
  pseudorange_residual_logger_.close();
}

// Free phaserange residual logger
void GnssEstimatorBase::freePhaserangeResidualLogger()
{
  phaserange_residual_logger_.close();
}

}