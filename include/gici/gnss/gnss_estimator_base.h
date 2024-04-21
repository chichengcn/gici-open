/**
* @Function: Base class for GNSS estimators
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/estimator_base.h"
#include "gici/gnss/gnss_types.h"
#include "gici/gnss/ambiguity_resolution.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// GNSS estimator common options
struct GnssEstimatorBaseOptions {
  // GNSS common options
  GnssCommonOptions common;

  // GNSS error parameter
  GnssErrorParameter error_parameter;

  // Use Fault Detection and Exclusion (FDE)
  bool use_outlier_rejection = true;

  // Reject outlier at a time or reject all 
  bool reject_one_outlier_once = false;

  // Maximum pseudorange error to exclude (m)
  double max_pesudorange_error = 4.0;

  // Maximum phaserange error to exclude (m)
  double max_phaserange_error = 0.03;

  // Maximum doppler error to exclude (m/s)
  double max_doppler_error = 0.5;

  // Minimum number of satellite to be considered as good observation environment
  int good_observation_min_num_satellites = 10;

  // Maximum GDOP value to be considered as good observation environment
  double good_observation_max_gdop = 2.0;

  // Maximim outlier rejection ratio to be considered as good observation environment
  double good_observation_max_reject_ratio = 0.1;

  // Minimum number of continuous unfix under good observation to reset ambiguities
  size_t reset_ambiguity_min_num_continuous_unfix = 10;

  // Maximum outlier rejection ratio to be considered as diverging
  double diverge_max_reject_ratio = 0.5;

  // Minimum number of continuous large amount rejection to be considered as divergence
  size_t diverge_min_num_continuous_reject = 10;
};

// Estimator
class GnssEstimatorBase : public virtual EstimatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssEstimatorBase(const GnssEstimatorBaseOptions& options,
                    const EstimatorBaseOptions& base_options);
  ~GnssEstimatorBase();

  // Add GNSS measurements and state for undifferenced estimators
  virtual bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
  { return false; }

  // Add GNSS measurements and state for differenced estimators
  virtual bool addGnssMeasurementAndState(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref)
  { return false; }

  // Reset ambiguity estimation
  inline void resetAmbiguityEstimation() {
    if (ambiguity_states_.size() == 0) return;
    eraseAmbiguityParameterBlocks(curAmbiguityState(), false);
  }

  // Reset ionosphere estimation
  inline void resetIonosphereEstimation() {
    if (ionosphere_states_.size() == 0) return;
    eraseIonosphereParameterBlocks(curIonosphereState(), false);
  }

  // Get solution status
  inline GnssSolutionStatus getSolutionStatus() { return lastState().status; }

  // Get number of satellite
  inline int getNumberSatellite() { return num_satellites_; }

  // Get differential age
  inline int getDifferentialAge() { return differential_age_; } 

  // Check if we have velocity estimate
  inline bool hasVelocityEstimate() { return has_velocity_estimate_; }

protected: 
  // Add GNSS position block to graph
  BackendId addGnssPositionParameterBlock(
    const int32_t id, 
    const Eigen::Vector3d& prior);

  // Add GNSS velocity block to graph
  BackendId addGnssVelocityParameterBlock(
    const int32_t id, 
    const Eigen::Vector3d& prior = Eigen::Vector3d::Zero());

  // Add GNSS extrinsics block to graph
  BackendId addGnssExtrinsicsParameterBlock(
    const int32_t id, 
    const Eigen::Vector3d& t_SR_S_prior);

  // Add clock blocks to graph
  void addClockParameterBlocks(
    const GnssMeasurement& measurement, 
    const int32_t id, 
    int& num_valid_system,
    const std::map<char, double>& prior = std::map<char, double>(), 
    bool use_single_frequency = false);

  // Add clock blocks to graph for single differenced measurements
  void addSdClockParameterBlocks(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref, 
    const GnssMeasurementSDIndexPairs& index_pairs,
    const int32_t id, 
    int& num_valid_system,
    const std::map<char, double>& prior = std::map<char, double>());

  // Add Inter-Frequency Bias (IFB) to graph. We add for every code type to 
  // support multi-frequency PPP. Ideally, the IFBs should be added for every
  // satellites and codes for GLONASS, but we do not do so because the performance 
  // improvement is not so significant while the computational cost is increased.
  void addIfbParameterBlocks(
    GnssMeasurement& measurement, 
    const int32_t id);

  // Add frequency blocks to graph
  void addFrequencyParameterBlocks(
    const GnssMeasurement& measurement, 
    const int32_t id, 
    int& num_valid_system, 
    const std::map<char, double>& prior = std::map<char, double>(), 
    bool use_single_frequency = false);

  // Add troposphere block to graph
  void addTroposphereParameterBlock(const int32_t id);

  // Add ionosphere blocks to graph
  void addIonosphereParameterBlocks(
    const GnssMeasurement& measurement, 
    const int32_t id,
    IonosphereState& state);

  // Add ambiguity blocks to graph
  void addAmbiguityParameterBlocks(
    const GnssMeasurement& measurement, 
    const int32_t id,
    AmbiguityState& state);

  // Add single-differenced ambiguity blocks to graph
  void addSdAmbiguityParameterBlocks(
    const GnssMeasurement& measurement_rov, 
    const GnssMeasurement& measurement_ref, 
    const GnssMeasurementDDIndexPairs& index_pairs,
    const int32_t id,
    AmbiguityState& state);

  // Correct code biases
  void correctCodeBias(GnssMeasurement& measurement, const bool accept_coarse = true);

  // Correct phase biases
  void correctPhaseBias(GnssMeasurement& measurement);

  // Correct BDS satellite multipath
  void correctBdsSatelliteMultipath(GnssMeasurement& measurement);

  // Compute and set ionosphere delays using dual-frequency or model
  void computeIonosphereDelay(
    GnssMeasurement& measurement, 
    bool use_single_frequency = false);

  // Add position residual block to graph
  void addGnssPositionResidualBlock(
    const State& state, const Eigen::Vector3d& position, const double std);

  // Add velocity residual block to graph
  void addGnssVelocityResidualBlock(
    const State& state, const Eigen::Vector3d& velocity, const double std);

  // Add pseudorange residual blocks to graph
  void addPseudorangeResidualBlocks(
    const GnssMeasurement& measurement,
    const State& state,
    int& num_valid_satellite,
    bool use_single_frequency = false);

  // Add single-differenced pseudorange residual block to graph
  void addSdPseudorangeResidualBlocks(
    const GnssMeasurement& measurement_rov,
    const GnssMeasurement& measurement_ref,
    const GnssMeasurementSDIndexPairs& index_pairs,
    const State& state,
    int& num_valid_satellite,
    bool use_single_frequency = false);

  // Add double-differenced pseudorange residual block to graph
  void addDdPseudorangeResidualBlocks(
    const GnssMeasurement& measurement_rov,
    const GnssMeasurement& measurement_ref,
    const GnssMeasurementDDIndexPairs& index_pairs,
    const State& state,
    int& num_valid_satellite,
    bool use_single_frequency = false);

  // Add phaserange residual blocks to graph
  void addPhaserangeResidualBlocks(
    const GnssMeasurement& measurement,
    const State& state);

  // Add double-differenced phaserange residual blocks to graph
  void addDdPhaserangeResidualBlocks(
    const GnssMeasurement& measurement_rov,
    const GnssMeasurement& measurement_ref,
    const GnssMeasurementDDIndexPairs& index_pairs,
    const State& state);

  // Add doppler residual blocks to graph
  void addDopplerResidualBlocks(
    const GnssMeasurement& measurement,
    const State& state,
    int& num_valid_satellite,
    bool use_single_frequency = false, 
    const Eigen::Vector3d& angular_velocity = Eigen::Vector3d::Zero());

  // Add troposphere residual block to graph
  void addTroposphereResidualBlock(
    const BackendId& tropo_id, 
    const double value, const double std);

  // Add ionosphere residual block to graph
  void addIonosphereResidualBlock(
    const BackendId& iono_id,
    const double value, const double std);

  // Add ambiguity residual block to graph
  void addAmbiguityResidualBlock(
    const BackendId& amb_id,
    const double value, const double std);

  // Add relative position block to graph
  void addRelativePositionResidualBlock(
    const State& last_state, const State& cur_state);

  // Add relative position and velocity block to graph
  void addRelativePositionAndVelocityResidualBlock(
    const State& last_state, const State& cur_state);

  // Add relative ISB block to graph
  void addRelativeIsbResidualBlock(
    const State& last_state, const State& cur_state);

  // Add relative frequency block to graph
  void addRelativeFrequencyResidualBlock(
    const State& last_state, const State& cur_state);

  // Add relative troposphere block to graph
  void addRelativeTroposphereResidualBlock(
    const State& last_state, const State& cur_state);

  // Add relative ionosphere block to graph
  void addRelativeIonosphereResidualBlock(
    const IonosphereState& last_state, const IonosphereState& cur_state);

  // Add relative ambiguity block to graph
  void addRelativeAmbiguityResidualBlock(
    GnssMeasurement& last_measurement,
    GnssMeasurement& cur_measurement,
    const AmbiguityState& last_state, const AmbiguityState& cur_state);

  // Number of pseudorange errors
  size_t numPseudorangeError(const State& state);

  // Number of phaserange errors
  size_t numPhaserangeError(const State& state);

  // Number of doppler errors
  size_t numDopplerError(const State& state);

  // Reject pseudorange outlier
  size_t rejectPseudorangeOutlier(
    const State& state, bool reject_one = false);

  // Reject pseudorange outlier together with corresponding ambiguity parameter
  size_t rejectPseudorangeOutlier(
    const State& state, AmbiguityState& ambiguity_state, 
    bool reject_one = false);

  // Reject phaserange outlier
  size_t rejectPhaserangeOutlier(
    const State& state, AmbiguityState& ambiguity_state, 
    bool reject_one = false);

  // Reject doppler outlier
  size_t rejectDopplerOutlier(
    const State& state, bool reject_one = false);

  // Add GNSS position block to marginalizer
  void addGnssPositionMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add GNSS velocity block to marginalizer
  void addGnssVelocityMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add clock blocks to marginalizer
  void addClockMarginBlocksWithResiduals(const State& state, bool keep = false);

  // Add frequency blocks to marginalizer
  void addFrequencyMarginBlocksWithResiduals(const State& state, bool keep = false);

  // Add troposphere block to marginalizer
  void addTroposphereMarginBlockWithResiduals(const State& state, bool keep = false);

  // Add ionosphere blocks to marginalizer
  void addIonosphereMarginBlocksWithResiduals(const IonosphereState& state, bool keep = false);

  // Add ambiguity blocks and its residuals to marginalizer
  void addAmbiguityMarginBlocksWithResiduals(const AmbiguityState& state, bool keep = false);

  // Add GNSS measurement residual blocks to marginalizer
  void addGnssMeasurementResidualMarginBlocks(const State& state);

  // Add GNSS position and velocity residual blocks to marginalizer
  void addGnssResidualMarginBlocks(const State& state);

  // Add all GNSS loosely coupled residual blocks to marginalier
  void addGnssLooseResidualMarginBlocks(const State& state);

  // Erase GNSS position block
  void eraseGnssPositionParameterBlock(const State& state);

  // Erase GNSS velocity block
  void eraseGnssVelocityParameterBlock(const State& state);

  // Erase GNSS extrinsics
  void eraseGnssExtrinsicsParameterBlock(BackendId& extrinsics_id);

  // Erase clock blocks
  void eraseClockParameterBlocks(const State& state);

  // Erase IFB blocks
  void eraseIfbParameterBlocks(std::vector<std::pair<char, int>>& ifbs);

  // Erase frequency blocks
  void eraseFrequencyParameterBlocks(const State& state, bool reform = true);

  // Erase troposphere blcok
  void eraseTroposphereParameterBlock(const State& state, bool reform = true);

  // Erase ionosphere blocks
  void eraseIonosphereParameterBlocks(IonosphereState& state, bool reform = true);

  // Erase ambiguity blocks
  void eraseAmbiguityParameterBlocks(AmbiguityState& state, bool reform = true);

  // Erase all pseudorange residual blocks
  void erasePseudorangeResidualBlocks(const State& state);

  // Erase GNSS measurement residual blocks
  void eraseGnssMeasurementResidualBlocks(const State& state);

  // Erase GNSS position and velocity residual block
  void eraseGnssLooseResidualBlocks(const State& state);

  // Erase relative position residual blocks
  void eraseRelativePositionResidualBlock(
    const State& last_state, const State& cur_state);

  // Erase relative position and velocity residual blocks
  void eraseRelativePositionAndVelocityBlock(
    const State& last_state, const State& cur_state);

  // Erase relative frequency residual blocks
  void eraseRelativeFrequencyBlock(
    const State& last_state, const State& cur_state);

  // Erase relative troposphere residual blocks
  void eraseRelativeTroposphereResidualBlock(
    const State& last_state, const State& cur_state);

  // Erase relative ionosphere residual blocks
  void eraseRelativeIonosphereResidualBlock(
    const IonosphereState& last_state, const IonosphereState& cur_state);

  // Erase relative ambiguity residual blocks
  void eraseRelativeAmbiguityResidualBlock(
    const AmbiguityState& last_state, const AmbiguityState& cur_state);

  // Convert from estimated states (in ENU) to body states
  void convertStateAndCovarianceToBody(
    Transformation* T_WS, SpeedAndBias* speed_and_bias, 
    Eigen::Matrix<double, 15, 15>* covariance) override;
  
  // Get extrinsics estimate
  Eigen::Vector3d getGnssExtrinsicsEstimate();

  // Get GNSS measurement index from error interface
  GnssMeasurementIndex getGnssMeasurementIndexFromErrorInterface(
    const std::shared_ptr<ErrorInterface>& error_interface);

  // Get GNSS measurement index from error interface
  GnssMeasurementSDIndexPair getGnssMeasurementSDIndexPairFromErrorInterface(
    const std::shared_ptr<ErrorInterface>& error_interface);

  // Get GNSS measurement index from error interface
  GnssMeasurementDDIndexPair getGnssMeasurementDDIndexPairFromErrorInterface(
    const std::shared_ptr<ErrorInterface>& error_interface);

  // Create ambiguity logger
  void createAmbiguityLogger(const std::string& directory);

  // Create ionosphere logger
  void createIonosphereLogger(const std::string& directory);

  // Create pseudorange residual logger
  void createPseudorangeResidualLogger(const std::string& directory);

  // Create DD pseudorange residual logger
  void createDdPseudorangeResidualLogger(const std::string& directory);

  // Create phaserange residual logger
  void createPhaserangeResidualLogger(const std::string& directory);

  // Create DD phaserange residual logger
  void createDdPhaserangeResidualLogger(const std::string& directory);

  // Log ambiguity estimate
  void logAmbiguityEstimate();

  // Log ionosphere estimate
  void logIonosphereEstimate();

  // Log pseudorange residual
  void logPseudorangeResidual();

  // Log DD pseudorange residual
  void logDdPseudorangeResidual();

  // Log phasernage residual
  void logPhaserangeResidual();

  // Log DD phasernage residual
  void logDdPhaserangeResidual();

  // Free ambiguity logger
  void freeAmbiguityLogger();

  // Free ionosphere logger
  void freeIonosphereLogger();

  // Free pseudorange residual logger
  void freePseudorangeResidualLogger();

  // Free DD pseudorange residual logger
  void freeDdPseudorangeResidualLogger();

  // Free phaserange residual logger
  void freePhaserangeResidualLogger();

  // Free DD phaserange residual logger
  void freeDdPhaserangeResidualLogger();

  // Compute and update GDOP
  inline void updateGdop(const GnssMeasurement& measurement) {
    gdop_ = gnss_common::computeDops(measurement, gnss_base_options_.common).x();
  }
  inline void updateGdop(
    const GnssMeasurement& measurement_rov, const GnssMeasurementSDIndexPairs& indexes) {
    gdop_ = gnss_common::computeDops(
      measurement_rov, indexes, gnss_base_options_.common).x();
  }
  inline void updateGdop(
    const GnssMeasurement& measurement_rov, const GnssMeasurementDDIndexPairs& indexes) {
    gdop_ = gnss_common::computeDops(
      measurement_rov, indexes, gnss_base_options_.common).x();
  }

  // Check if we under good observation environment
  inline bool isGnssGoodObservation() {
    if (gdop_ > gnss_base_options_.good_observation_max_gdop) return false;
    if (num_satellites_ < 
        gnss_base_options_.good_observation_min_num_satellites) return false;
    return true;
  }

  // Get current GNSS measurement
  inline GnssMeasurement& curGnss() { 
    return getCurrent(gnss_measurements_); 
  }

  // Get last GNSS measurement
  inline GnssMeasurement& lastGnss() { 
    return getLast(gnss_measurements_); 
  }

  // Get oldest GNSS measurement
  inline GnssMeasurement& oldestGnss() { 
    return getOldest(gnss_measurements_); 
  }

  // Get a GNSS measurement at given timestamp
  inline std::deque<GnssMeasurement>::iterator 
  gnssMeasurementAt(const double timestamp) { 
    auto it = gnss_measurements_.begin();
    for (; it != gnss_measurements_.end(); it++) {
      if (checkEqual(it->timestamp, timestamp)) break;
    }
    return it;
  }

  // Get current GNSS rover measurement
  inline GnssMeasurement& curGnssRov() { 
    return getCurrent(gnss_measurement_pairs_).first; 
  }

  // Get last GNSS rover measurement
  inline GnssMeasurement& lastGnssRov() { 
    return getLast(gnss_measurement_pairs_).first; 
  }

  // Get oldest GNSS rover measurement
  inline GnssMeasurement& oldestGnssRov() { 
    return getOldest(gnss_measurement_pairs_).first; 
  }

  // Get current GNSS reference measurement
  inline GnssMeasurement& curGnssRef() { 
    return getCurrent(gnss_measurement_pairs_).second; 
  }

  // Get last GNSS reference measurement
  inline GnssMeasurement& lastGnssRef() { 
    return getLast(gnss_measurement_pairs_).second; 
  }

  // Get oldest GNSS reference measurement
  inline GnssMeasurement& oldestGnssRef() { 
    return getOldest(gnss_measurement_pairs_).second; 
  }

  // Get a GNSS measurement pair at given timestamp
  inline std::deque<std::pair<GnssMeasurement, GnssMeasurement>>::iterator 
  gnssMeasurementPairAt(const double timestamp) { 
    auto it = gnss_measurement_pairs_.begin();
    for (; it != gnss_measurement_pairs_.end(); it++) {
      if (checkEqual(it->first.timestamp, timestamp)) break;
    }
    return it;
  }

  // Get current Ambiguity state
  inline AmbiguityState& curAmbiguityState() { 
    return getCurrent(ambiguity_states_); 
  }

  // Get last Ambiguity state
  inline AmbiguityState& lastAmbiguityState() { 
    return getLast(ambiguity_states_); 
  }

  // Get oldest Ambiguity state
  inline AmbiguityState& oldestAmbiguityState() { 
    return getOldest(ambiguity_states_); 
  }

  // Get an ambiguity state at given timestamp
  inline std::deque<AmbiguityState>::iterator 
  ambiguityStateAt(const double timestamp) { 
    auto it = ambiguity_states_.begin();
    for (; it != ambiguity_states_.end(); it++) {
      if (checkEqual(it->timestamp, timestamp)) break;
    }
    return it;
  }

  // Get current Ionosphere state
  inline IonosphereState& curIonosphereState() { 
    return getCurrent(ionosphere_states_); 
  }

  // Get last Ionosphere state
  inline IonosphereState& lastIonosphereState() { 
    return getLast(ionosphere_states_); 
  }

  // Get oldest Ionosphere state
  inline IonosphereState& oldestIonosphereState() { 
    return getOldest(ionosphere_states_); 
  }

  // Get an ionosphere state at given timestamp
  inline std::deque<IonosphereState>::iterator 
  ionosphereStateAt(const double timestamp) { 
    auto it = ionosphere_states_.begin();
    for (; it != ionosphere_states_.end(); it++) {
      if (checkEqual(it->timestamp, timestamp)) break;
    }
    return it;
  }

  // Get latest GNSS state
  inline State& latestGnssState() {
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if (state.id.type() == IdType::gPose || 
          state.id.type() == IdType::gPosition) return state;
    }
    return null_state_;
  }

  // Get last GNSS state 
  inline State& lastGnssState() {
    int pass_cnt = 0;
    for (auto it = states_.rbegin(); it != states_.rend(); it++) {
      State& state = *it;
      if (!state.valid()) continue;
      if (state.id.type() == IdType::gPose || 
          state.id.type() == IdType::gPosition) {
        pass_cnt++;
        if (pass_cnt == 2) return state;
      }
    }
    return null_state_;
  }

  // Check if insufficient satellites
  inline bool checkSufficientSatellite(
    const int num_valid_satellite,
    const int num_valid_system,
    bool log = true) {
    int base = is_use_phase_ ? 4 : 3;
    int thr = num_valid_system + base + 
      gnss_base_options_.common.min_num_satellite_redundancy;
    if (num_valid_satellite < thr) {
      if (!log) return false;
      LOG(INFO) << "Insufficient satellites! We need at least " 
                   << thr << " satellites, "
                   << "but we only have " << num_valid_satellite << "!";
      return false;
    }
    return true;
  }

  // Check observation valid for common or precise positioning
  inline bool checkObservationValid(const GnssMeasurement& measurement,
                                    const GnssMeasurementIndex& index,
                                    const bool is_doppler = false) {
    if (is_doppler) {
      if (!gnss_common::checkObservationValid(measurement, index, 
          ObservationType::Doppler, gnss_base_options_.common)) {
        return false;
      }
      else return true;
    }
    if (!gnss_common::checkObservationValid(measurement, index, 
        ObservationType::Pseudorange, gnss_base_options_.common)) {
      return false;
    }
    if (is_use_phase_ && 
        !gnss_common::checkObservationValid(measurement, index, 
        ObservationType::Phaserange, gnss_base_options_.common, is_ppp_)) {
      return false;
    }
    if (is_ppp_ && gnss_common::isBds1(index.prn)) {
      return false;
    }
    return true;
  }

protected:
  // Options
  GnssEstimatorBaseOptions gnss_base_options_;

  // Measurements
  std::deque<GnssMeasurement> gnss_measurements_;
  std::deque<std::pair<GnssMeasurement, GnssMeasurement>> gnss_measurement_pairs_;

  // States
  std::deque<AmbiguityState> ambiguity_states_;
  std::deque<IonosphereState> ionosphere_states_;
  std::vector<std::pair<char, int>> ifbs_;
  int num_satellites_;
  int differential_age_;
  double gdop_;
  static int32_t solution_id;
  BackendId gnss_extrinsics_id_;

  // Ambiguity resolution
  std::unique_ptr<AmbiguityResolution> ambiguity_resolution_;

  // Flags
  bool is_state_pose_ = false;
  bool is_verbose_model_ = false;  // if estimate atmosphere, IFB, etc...
  bool is_ppp_ = false; 
  bool is_use_phase_ = false;
  bool has_velocity_estimate_ = false;

  // Intermediate data loggers
  std::ofstream ambiguity_logger_;
  std::ofstream ionosphere_logger_;
  std::ofstream pseudorange_residual_logger_;
  std::ofstream dd_pseudorange_residual_logger_;
  std::ofstream phaserange_residual_logger_;
  std::ofstream dd_phaserange_residual_logger_;
};

}