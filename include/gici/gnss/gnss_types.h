/**
* @Function: GNSS types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <memory>
#include <Eigen/Core>
#include <glog/logging.h>

#include "gici/gnss/code_bias.h"
#include "gici/gnss/phase_bias.h"
#include "gici/gnss/phase_windup.h"
#include "gici/gnss/phase_center.h"

namespace gici {

// Role of formator
enum class GnssRole {
  None,
  Rover,
  Reference,
  Ephemeris,
  SsrEphemeris,
  IonAndUtc,
  CodeBias,
  PhaseBias,
  Heading,
  PhaseCenter
};

// Satellite ephemeris types
enum class SatEphType {
  None = 0, 
  Broadcast = 1,
  Precise = 2
};

// Ionosphere delay type
enum class IonoType {
  None,
  Broadcast,
  DualFrequency,
  Augmentation
};

// GNSS systems
extern std::vector<char> gnss_systems;

// Get GNSS system list
inline std::vector<char>& getGnssSystemList() {
  return gnss_systems;
}

// One code type measurement
struct Observation {
  double wavelength;
  double pseudorange;
  double phaserange;
  double doppler; // in m/s
  double SNR; // Sigal-to-Noise Ratio
  uint8_t LLI; // Loss of Lock Indicator
  bool slip; // Cycle-slip flag
  int raw_code; // raw code type
};

// One satellite data
struct Satellite {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	std::string prn;
	std::unordered_map<int, Observation> observations;
  SatEphType sat_type = SatEphType::None;
  Eigen::Vector3d sat_position;
  Eigen::Vector3d sat_velocity;
  double sat_clock;
  double sat_frequency;

  double ionosphere;  // In 1575.42 MHz frequency
  IonoType ionosphere_type;
  
  // TODO: GPS L5 IFCB and GLONASS IFB
  // Currently, in PPP, we donot use GLONASS code and GPS L5
  // in RTK, we donot use GLONASS code and disable its AR

  // Get satellite system
  inline char getSystem() const { return prn[0]; }
};

using Satellites = std::map<std::string, Satellite, std::less<std::string>, 
  Eigen::aligned_allocator<std::pair<const std::string, Satellite>>>;

// Index of observation 
struct GnssMeasurementIndex {
  GnssMeasurementIndex() : prn(""), code_type(0) {}
  GnssMeasurementIndex(std::string prn, int code_type) : 
    prn(prn), code_type(code_type) {}
  std::string prn;
  int code_type;

  inline bool operator==(const GnssMeasurementIndex index) {
    if (index.code_type != code_type) return false;
    if (index.prn != prn) return false;
    return true;
  }
};

// Pairs
// single difference pair
struct GnssMeasurementSDIndexPair {
  GnssMeasurementSDIndexPair() : 
    rov(GnssMeasurementIndex()), ref(GnssMeasurementIndex()) {}
  GnssMeasurementSDIndexPair(
    GnssMeasurementIndex rov, GnssMeasurementIndex ref) : 
    rov(rov), ref(ref) { }
  GnssMeasurementIndex rov;
  GnssMeasurementIndex ref;
};
using GnssMeasurementSDIndexPairs = std::vector<GnssMeasurementSDIndexPair>;
// double difference pair
struct GnssMeasurementDDIndexPair {
  GnssMeasurementDDIndexPair() : 
    rov(GnssMeasurementIndex()), ref(GnssMeasurementIndex()),
    rov_base(GnssMeasurementIndex()), ref_base(GnssMeasurementIndex()) {}
  GnssMeasurementDDIndexPair(
    GnssMeasurementIndex rov, GnssMeasurementIndex ref, 
    GnssMeasurementIndex rov_base, GnssMeasurementIndex ref_base) : 
    rov(rov), ref(ref), rov_base(rov_base), ref_base(ref_base) { }
  GnssMeasurementIndex rov;
  GnssMeasurementIndex ref;
  GnssMeasurementIndex rov_base;
  GnssMeasurementIndex ref_base;
};
using GnssMeasurementDDIndexPairs = std::vector<GnssMeasurementDDIndexPair>;

// GNSS epoch data
struct GnssMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GnssMeasurement() : id(++epoch_cnt_) {}

  double timestamp;
  GnssRole role;
  std::string tag;
  int32_t id;  // ID for bundle adjustment
  Satellites satellites;
  Eigen::Vector3d position;  // for reference station
  Eigen::VectorXd ionosphere_parameters;  // GPS broadcast ionosphere parameters
  double troposphere_wet;  // Wet ZTD from augmentation

  // Code bias handle
  CodeBiasPtr code_bias;

  // Phase bias handle
  PhaseBiasPtr phase_bias;

  // Phase center handle (PCVs and PCOs)
  PhaseCenterPtr phase_center;

  // Phase wind-up handle
  PhaseWindupPtr phase_windup;

  inline Satellite& getSat(std::string prn) { 
    auto it = satellites.find(prn);
    CHECK(it != satellites.end());
    return it->second; 
  }
  
  inline Satellite& getSat(GnssMeasurementIndex index) { 
    return getSat(index.prn);
  }

  inline Observation& getObs(GnssMeasurementIndex index) {
    Satellite& satellite = getSat(index);
    auto it = satellite.observations.find(index.code_type);
    CHECK(it != satellite.observations.end());
    return it->second;
  }

  inline const Satellite& getSat(std::string prn) const { 
    const auto it = satellites.find(prn);
    CHECK(it != satellites.end());
    return it->second; 
  }

  inline const Satellite& getSat(GnssMeasurementIndex index) const {
    return getSat(index.prn);
  }

  inline const Observation& getObs(GnssMeasurementIndex index) const {
    const Satellite& satellite = getSat(index);
    const auto it = satellite.observations.find(index.code_type);
    CHECK(it != satellite.observations.end());
    return it->second;
  }

  static int32_t epoch_cnt_;
};

// Observation type
enum class ObservationType {
  Pseudorange,
  Phaserange,
  Doppler
};

// Solution status
enum class GnssSolutionStatus {
  None, 
  Single, 
  DGNSS,
  Float,
  Fixed,
  DeadReckoning
};

// Solutions
struct GnssSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  int32_t id;  // bundle id for integration
  Eigen::Vector3d position; // in ECEF
  Eigen::Vector3d velocity;
  Eigen::Matrix<double, 6, 6> covariance;
  GnssSolutionStatus status;
  int num_satellites;
  int differential_age;
  bool has_velocity = false;
  bool has_position = false;
};

// GNSS common options
struct GnssCommonOptions {
  // Usage of satellite systems
  // In default, we use all systems
  std::vector<char> system_exclude;

  // Usage of specific satellite
  // In default, we use all satellites
  std::vector<std::string> satellite_exclude;

  // Usage of code types
  // In default, we use all code types
  std::vector<std::pair<char, int>> code_exclude;

  // Minimum elevation angle (deg)
  double min_elevation = 12.0;

  // Minimum SNR for frequencies 1575.42 MHz (L1) and 1176.45 MHz (L5).
  // SNR masks for other frequencies will be interpolated by a linear model.
  Eigen::Vector2d min_SNR = Eigen::Vector2d(25.0, 20.0);

  // Minimum number of redundant satellites to apply estimation
  int min_num_satellite_redundancy = 2;

  // Maximum GDOP as valid solution
  double max_gdop = 20.0;

  // Threshold for Melbourne-Wubbena (MW) cycle-slip detection (m)
  double mw_slip_thres = 0.5;

  // Threshold for Geometry-Free (GF) cycle-slip detection (m)
  double gf_slip_thres = 0.05;

  // Threshold for single differenced GF cycle-slip detection (m)
  double gf_sd_slip_thres = 0.05;

  // Receiver Phaes-Center-Offset (PCO)
  Eigen::Vector3d receiver_pco = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Data period
  double period = 1.0;
};

// GNSS error factors
struct GnssErrorParameter {
  // code noise = phase noise * ratio
  double code_to_phase_ratio = 100.0;

  // Error factor a/b/c according to RTKLIB
  std::vector<double> phase_error_factor{0.003, 0.003, 0.0};

  // Doppler error factor (m/s)
  double doppler_error_factor = 0.2;

  // System error ratio
  std::map<char, double> system_error_ratio = 
    {{'G', 1.0}, {'R', 5.0}, {'C', 2.0}, {'E', 1.5}};

  // Ionosphere model error factor
  double ionosphere_broadcast_factor = 0.5;

  // Dual-frequency ionosphere error
  double ionosphere_dual_frequency = 0.2;

  // Augmentation ionosphere error
  double ionosphere_augment = 0.03;

  // Troposphere model error factor
  double troposphere_model_factor = 0.2;

  // Augmentation troposphere error
  double troposphere_augment = 0.01;

  // Broadcast ephemeris error
  double ephemeris_broadcast = 3.0;

  // Precise ephemeris error
  double ephemeris_precise = 0.1;

  // Initial position error
  double initial_position = 100.0;

  // Initial velocity error
  double initial_velocity = 20.0;

  // Initial clock error
  double initial_clock = 5.0;

  // Initial troposphere error
  double initial_troposphere = 0.5;

  // Initial ionosphere error
  double initial_ionosphere = 60.0;
  
  // Initial ambiguity error
  double initial_ambiguity = 60.0;

  // Relative position error in m/sqrt(Hz) in ENU used in GNSS-only positioning
  Eigen::Vector3d relative_position = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Relative velocity error in m/s/sqrt(Hz) in ENU used in GNSS-only positioning 
  // if we estimate receiver velocity, specify this parameter, or, specify the above parameter.
  Eigen::Vector3d relative_velocity = Eigen::Vector3d(10.0, 10.0, 10.0);

  // Relative troposphere delay error in m/sqrt(Hz)
  double relative_troposphere = 3.0e-4;

  // Relative ionosphere delay error in m/sqrt(Hz)
  double relative_ionosphere = 3.0e-2;

  // Relative ambiguity error in m/sqrt(Hz)
  double relative_ambiguity = 1.0e-4;

  // Relative ISB error in m/sqrt(Hz)
  double relative_isb = 1.0e-3;

  // Relative receiver frequency error
  double relative_frequency = 1.0e-2;

  // Relative GPS Inter-Frequency Clock Bias (IFCB) error
  double relative_gps_ifcb = 5.0e-4;

  // Residual amplitude for GPS L5 (influenced by un-calibrated IFCB)
  double residual_gps_ifcb = 0.02;
};

// Ambiguity states
class BackendId;
class AmbiguityState {
public:
  AmbiguityState() { clear(); }

  double timestamp = 0.0;
  std::vector<BackendId> ids;

  inline void clear() {
    ids.clear(); timestamp = 0.0;
  }
};

// Ionosphere states
using IonosphereState = AmbiguityState;

}
