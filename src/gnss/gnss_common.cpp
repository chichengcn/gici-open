/**
* @Function: GNSS common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/gnss_common.h"

#include <glog/logging.h>

#include "gici/utility/common.h"
#include "gici/gnss/code_phase_maps.h"

namespace gici {

namespace gnss_common {

// ----------------------------------------------------------
// Convert char system to int system
int systemConvert(char sys)
{
  switch (sys) {
    case 'G': return SYS_GPS;
    case 'R': return SYS_GLO;
    case 'E': return SYS_GAL;
    case 'C': return SYS_CMP;
    default: return SYS_NONE;
  }
}

// Convert int system to char system
char systemConvert(int sys)
{
  switch (sys) {
    case SYS_GPS: return 'G';
    case SYS_GLO: return 'R';
    case SYS_GAL: return 'E';
    case SYS_CMP: return 'C';
    default: return 0x00;
  }
}

// Convert PRN string to RTKLIB sat
int prnToSat(std::string prn)
{
  int system = systemConvert(prn[0]);
  if (system == SYS_NONE) return 0;
  int prnnum = atoi(prn.substr(1, 2).data());
  return satno(system, prnnum);
}

// Convert RTKLIB sat to PRN string
std::string satToPrn(int sat)
{
  int prnnum;
  char system = systemConvert(satsys(sat, &prnnum));
  if (system == 0x00) return "";
  char prnbuf[10];
  sprintf(prnbuf, "%c%02d", system, prnnum);
  return std::string(prnbuf);
}

// Get frequency from code type, and channel
double codeToFrequency(char system, int code, int channel)
{
  int sys = systemConvert(system);
  return code2freq(sys, (uint8_t)code, channel);
}

// Get frequency from phase type, and channel 
double phaseToFrequency(char system, int phase_id, int channel)
{
  double frequency = 0.0;
#define MAP(S, P, F) \
  if (system == S && phase_id == P) { frequency = F; }
  PHASE_CHANNEL_TO_FREQUENCY_MAPS;
#undef MAP

  if (frequency == 0.0) {
    LOG(FATAL) << "Invalid phase type: " << phase_id << "!";
  }

  if (system == 'R' && phase_id == PHASE_G1) {
    frequency += DFRQ1_GLO * channel;
  }
  if (system == 'R' && phase_id == PHASE_G2) {
    frequency += DFRQ2_GLO * channel;
  }

  return frequency;
}

// Convert gtime to double
double gtimeToDouble(gtime_t time)
{
  return (static_cast<double>(time.time) + time.sec);
}

// Convert GPS time to UTC time
double gpsTimeToUtcTime(double gps_time)
{
  gtime_t gtime = doubleToGtime(gps_time);
  return gtimeToDouble(gpst2utc(gtime));
}

// Convert UTC time to GPS time
double utcTimeToGpsTime(double utc_time)
{
  gtime_t gtime = doubleToGtime(utc_time);
  return gtimeToDouble(utc2gpst(gtime));
}

// Convert double to gtime
gtime_t doubleToGtime(double time)
{
  gtime_t gtime;
  gtime.time = floor(time);
  gtime.sec = time - floor(time);
  return gtime;
}

// Get a phase ID
int getPhaseID(char system, int code_type)
{
#define MAP(S, C, P) \
  if (system == S && code_type == C) { return P; }
  CODE_TO_PHASE_CHANNEL_MAPS;
#undef MAP

  return PHASE_NONE;
}

// Convert rinex type string to code type
int rinexTypeToCodeType(const char system, const std::string str)
{
  CHECK(str.size() == 2);
  
#define MAP(S, R, C) \
  if (system == S && str == R) { return C; }
  RINEX_TO_CODE_MAPS;
#undef MAP

  LOG(FATAL) << "Invalid rinex type for system " << system << ": " << str << "!";

  return CODE_NONE;
}

// Convert code type to rinex type string
std::string codeTypeToRinexType(const char system, const int code_type)
{
#define MAP(S, R, C) \
  if (system == S && code_type == C) { return R; }
  RINEX_TO_CODE_MAPS;
#undef MAP

  LOG(FATAL) << "Invalid code type for system " << system << ": " << code_type << "!";

  return "";
}

// Convert phase string to phase type
int phaseStringToPhaseType(const char system, const std::string str)
{
#define MAP(S, P, PS) \
  if (system == S && str == PS) { return P; }
  PHASE_CHANNEL_TO_STR_MAPS;
#undef MAP

  LOG(FATAL) << "Invalid phase type for system " << system << ": " << str << "!";

  return PHASE_NONE;
}

// Convert phase type to phase string
std::string phaseTypeToPhaseString(const char system, const int phase_type)
{
#define MAP(S, R, PS) \
  if (system == S && phase_type == R) { return PS; }
  PHASE_CHANNEL_TO_STR_MAPS;
#undef MAP

  LOG(FATAL) << "Invalid phase type for system " << system << ": " << phase_type << "!";

  return "";
}

// ----------------------------------------------------------
// Check whether the system is used
bool useSystem(const GnssCommonOptions&  options, const char system)
{
  auto it = std::find(options.system_exclude.begin(), 
    options.system_exclude.end(), system);
  if (it == options.system_exclude.end()) return true;
  else return false;
}
bool useSystem(const std::vector<char>& system_exclude, const char system)
{
  auto it = std::find(system_exclude.begin(), 
    system_exclude.end(), system);
  if (it == system_exclude.end()) return true;
  else return false;
}

// Check whether the satellite is used
bool useSatellite(const GnssCommonOptions&  options, const std::string prn)
{
  auto it = std::find(options.satellite_exclude.begin(), 
    options.satellite_exclude.end(), prn);
  if (it == options.satellite_exclude.end()) return true;
  else return false;
}
bool useSatellite(const std::vector<std::string>& satellite_exclude, 
                  const std::string prn)
{
  auto it = std::find(satellite_exclude.begin(), 
    satellite_exclude.end(), prn);
  if (it == satellite_exclude.end()) return true;
  else return false;
}

// Check whether the code type is used
bool useCode(const GnssCommonOptions&  options, char system, const int code_type)
{
  auto it = std::find(options.code_exclude.begin(), 
    options.code_exclude.end(), std::make_pair(system, code_type));
  if (it == options.code_exclude.end()) return true;
  else return false;
}
bool useCode(const std::vector<std::pair<char, int>>& code_exclude, 
             char system, const int code_type)
{
  auto it = std::find(code_exclude.begin(), 
    code_exclude.end(), std::make_pair(system, code_type));
  if (it == code_exclude.end()) return true;
  else return false;
}

// Check whether the phase type is used
bool usePhase(const std::vector<std::pair<char, int>>& phase_exclude, 
              char system, const int phase_id)
{
  auto it = std::find(phase_exclude.begin(), 
    phase_exclude.end(), std::make_pair(system, phase_id));
  if (it == phase_exclude.end()) return true;
  else return false;
}

// Check elevation threshold
bool checkElevation(const GnssCommonOptions&  options, 
  const GnssMeasurement& measurement, std::string prn)
{
  // no elevation mask
  if (options.min_elevation == 0.0) return true;

  if (checkZero(measurement.position)) {
    // we do not know whether to use this satellite
    return true;
  }

  auto satellite = measurement.satellites.at(prn);
  if (checkZero(satellite.sat_position)) {
    return false;
  }

  double elevation = satelliteElevation(
    satellite.sat_position, measurement.position);
  if (radToDegree(elevation) > options.min_elevation) return true;
  else return false;
}

// Check SNR mask
bool checkSNR(const GnssCommonOptions& options,
  const GnssMeasurement& measurement, const GnssMeasurementIndex& index)
{
  const Observation& obs = measurement.getObs(index);
  const double f = CLIGHT / obs.wavelength;
  const double f1 = FREQ1, f2 = FREQ5;
  const double m1 = options.min_SNR[0], m2 = options.min_SNR[1];
  const double m = (m2 - m1) / (f2 - f1) * (f - f1) + m1;
  if (obs.SNR < m) return false;
  return true;
}

// Erase duplicated phases, arrange to one observation per phase
void rearrangePhasesAndCodes(GnssMeasurement& measurement, bool accept_coarse)
{
  CodeBiasPtr code_bias = measurement.code_bias;
  for (auto& sat : measurement.satellites) {
    std::string prn = sat.first;
    char system = prn[0];
    Satellite& satellite = sat.second;
    std::unordered_map<int, Observation>& observations = satellite.observations;
    std::unordered_map<int, Observation> arranged_observations;
    for (auto it = observations.begin(); it != observations.end(); it++) {
      std::pair<int, Observation> arranged_observation;
      int code = it->first;
      Observation observation = it->second;
      int phase_id = getPhaseID(system, code);
      int default_code = 0;
#define MAP(S, P, C) \
  if (system == S && phase_id == P) { default_code = C; }
  PHASE_CHANNEL_TO_DEFAULT_CODE;
#undef MAP
      bool can_add = true;
      // do not need to arrange
      if (default_code == code) arranged_observation = *it;
      // arrange to default code
      else {
        double bias = code_bias->getCodeBias(prn, code, accept_coarse);
        double default_bias = 
          code_bias->getCodeBias(prn, default_code, accept_coarse);
        // do not have code bias
        if (bias == 0.0 || default_code == 0.0) {
          // consider DCBs in the same frequency are zeros
          if (accept_coarse) {
            arranged_observation = std::make_pair(default_code, observation);
          } 
          // cannot arrange
          else {
            // pass
            can_add = false;
          }
        }
        // use code bias to arrange
        else {
          observation.pseudorange += bias - default_bias;
          arranged_observation = std::make_pair(default_code, observation);
        }
      }
      // add to observations
      if (can_add) {
        if (arranged_observations.find(default_code) == arranged_observations.end()) {
          arranged_observations.insert(arranged_observation);
        }
      }
    }
    satellite.observations = arranged_observations;
  }
}

// Check observation valid
bool checkObservationValid(const GnssMeasurement& measurement,
                           const GnssMeasurementIndex& index,
                           const ObservationType type, 
                           const GnssCommonOptions& options,
                           const bool need_precise_ephemeris)
{
  // Cannot find given satellite
  auto sat = measurement.satellites.find(index.prn);
  if (sat == measurement.satellites.end()) return false;

  auto& satellite = sat->second;

  // System not used 
  if (!gnss_common::useSystem(options, satellite.getSystem())) return false;

  // Satellite not used
  if (!gnss_common::useSatellite(options, satellite.prn)) return false;

  // Ephemeris invalid
  if (satellite.sat_type == SatEphType::None ||
      checkZero(satellite.sat_position) ||
      satellite.sat_clock == 0.0) {
    return false;
  }

  // Precise ephemeris
  if (need_precise_ephemeris && satellite.sat_type != SatEphType::Precise) {
    return false;
  }

  // Elevation mask
  if (!gnss_common::checkElevation(options, measurement, index.prn)) {
    return false;
  }

  auto obs = satellite.observations.find(index.code_type);

  // Cannot find given code type
  if(obs == satellite.observations.end()) return false;

  // SNR mask
  if (!gnss_common::checkSNR(options, measurement, index)) {
    return false;
  }

  // Code type not used
  if (!gnss_common::useCode(options, satellite.getSystem(), obs->second.raw_code)) 
    return false;

  auto& observation = obs->second;
  
  // Observation invalid
  if (type == ObservationType::Pseudorange) {
    if (observation.wavelength == 0.0 || 
        observation.pseudorange == 0.0) {
      return false;
    }
  }
  else if (type == ObservationType::Phaserange) {
    if (observation.wavelength == 0.0 || 
        observation.phaserange == 0.0) {
      return false;
    }
  }
  else if (type == ObservationType::Doppler) {
    if (observation.wavelength == 0.0 || 
        observation.doppler == 0.0) {
      return false;
    }
  }

  return true;
}

// Form single difference pseudorange pair
GnssMeasurementSDIndexPairs formPseudorangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options)
{
  GnssMeasurementSDIndexPairs index_pairs;

  // Find valid observations in measurement_rov
  std::vector<GnssMeasurementIndex> indexes_1;
  for (auto& sat : measurement_rov.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(options, system)) continue;
    for (auto obs : satellite.observations) {
      GnssMeasurementIndex index_rov(satellite.prn, obs.first);
      if (checkObservationValid(measurement_rov, index_rov,
          ObservationType::Pseudorange, options)) {
        indexes_1.push_back(index_rov);
      }
    }
  }

  // Find valid matches in measurement_ref
  for (auto index : indexes_1) 
  {
    auto& satellite = measurement_rov.satellites.at(index.prn);
    auto observation = satellite.observations.at(index.code_type);
    for (auto& sat : measurement_ref.satellites) {
      auto& satellite_2 = sat.second;
      if (satellite_2.prn != satellite.prn) continue;

      if (!checkObservationValid(measurement_ref, 
          GnssMeasurementIndex(satellite.prn, index.code_type), 
          ObservationType::Pseudorange)) {
        continue;
      }

      index_pairs.push_back(GnssMeasurementSDIndexPair(
        index, GnssMeasurementIndex(satellite.prn, index.code_type)));
    }
  }

  return index_pairs;
}                            

// Form single difference phaserange pair
GnssMeasurementSDIndexPairs formPhaserangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options)
{
  GnssMeasurementSDIndexPairs index_pairs;

  // Find valid observations in measurement_rov
  std::vector<GnssMeasurementIndex> indexes_1;
  for (auto& sat : measurement_rov.satellites) 
  {
    auto& satellite = sat.second;
    char system = satellite.getSystem();
    if (!gnss_common::useSystem(options, system)) continue;
    for (auto obs : satellite.observations) {
      GnssMeasurementIndex index_rov(satellite.prn, obs.first);
      if (checkObservationValid(measurement_rov, index_rov,
          ObservationType::Phaserange, options) && 
          checkObservationValid(measurement_rov, index_rov,
          ObservationType::Pseudorange, options)) {
        indexes_1.push_back(index_rov);
      }
    }
  }

  // Find valid matches in measurement_ref
  for (auto index : indexes_1) 
  {
    auto& satellite = measurement_rov.satellites.at(index.prn);
    auto& observation = satellite.observations.at(index.code_type);
    int phase_id = gnss_common::getPhaseID(satellite.getSystem(), index.code_type);

    for (auto& sat : measurement_ref.satellites) {
      auto& satellite_2 = sat.second;
      if (satellite_2.prn != satellite.prn) continue;

      for (auto& obs_2 : satellite_2.observations) {
        if (index.code_type != obs_2.first) continue;
        auto& observation_2 = obs_2.second;
        if (!checkObservationValid(measurement_ref, 
            GnssMeasurementIndex(satellite.prn, index.code_type), 
            ObservationType::Pseudorange)) continue;
        if (!checkObservationValid(measurement_ref, 
            GnssMeasurementIndex(satellite.prn, index.code_type), 
            ObservationType::Phaserange)) continue;

        index_pairs.push_back(GnssMeasurementSDIndexPair(
          index, GnssMeasurementIndex(satellite.prn, index.code_type)));
      }
    }
  }

  return index_pairs;
} 

// Form double difference pseudorange pair
GnssMeasurementDDIndexPairs formPseudorangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options)
{
  // Form SD pair
  GnssMeasurementSDIndexPairs sd_pairs = formPseudorangeSDPair(
    measurement_rov, measurement_ref, options);

  // Prepare data
  std::map<char, int> system_to_num_codes;
  std::multimap<char, double> system_to_codes;
  std::map<std::string, int> prn_to_number_codes; 
  std::multimap<std::string, double> prn_to_codes;
  std::multimap<std::string, int> prn_to_indexes;
  for (size_t i = 0; i < sd_pairs.size(); i++) {
    std::string prn = measurement_rov.getSat(sd_pairs[i].rov).prn;
    auto it = prn_to_number_codes.find(prn);
    if (it == prn_to_number_codes.end()) {
      prn_to_number_codes.insert(std::make_pair(prn, 1));
    }
    else it->second++;
    prn_to_indexes.insert(std::make_pair(prn, i));
    prn_to_codes.insert(std::make_pair(prn, sd_pairs[i].rov.code_type));
  }
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];

    system_to_num_codes.insert(std::make_pair(system, 0));
    for (auto it : prn_to_number_codes) {
      if (it.first[0] != system) continue;
      if (system_to_num_codes.at(system) < it.second) {
        system_to_num_codes.at(system) = it.second;
      }
    }

    for (auto it : prn_to_codes) {
      if (it.first[0] != system) continue;
      if (system_to_num_codes.find(system) == system_to_num_codes.end()) {
        system_to_num_codes.insert(std::make_pair(system, it.second));
      }
      bool found = false;
      for (auto it_wave = system_to_num_codes.lower_bound(system); 
          it_wave != system_to_num_codes.upper_bound(system); it_wave++) {
        if (it_wave->second == it.second) {
          found = true; break;
        }
      }
      if (!found) system_to_num_codes.insert(std::make_pair(system, it.second));
    }
  }

  // Find base satellites for each system and codes
  std::map<char, std::string> system_to_base_prn;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];

    // find satellite with maximum elevation angle
    double max_elevation = 0.0;
    for (size_t j = 0; j < sd_pairs.size(); j++) {
      if (sd_pairs[j].rov.prn[0] != system) continue;

      // we only select satellites with max phase number
      if (prn_to_number_codes.at(sd_pairs[j].rov.prn) != 
          system_to_num_codes.at(system)) continue;

      double elevation = satelliteElevation(
        measurement_ref.getSat(sd_pairs[j].ref).sat_position, 
        measurement_ref.position);
      if (max_elevation < elevation) {
        system_to_base_prn[system] = sd_pairs[j].rov.prn;
        max_elevation = elevation;
      }
    }
  }

  // Form DD pair
  GnssMeasurementDDIndexPairs dd_pairs;
  for (size_t i = 0; i < sd_pairs.size(); i++) {
    char system = sd_pairs[i].rov.prn[0];
    std::string prn = sd_pairs[i].rov.prn;
    std::string prn_base = system_to_base_prn.at(system);

    if (prn == prn_base) continue;

    for (auto it = prn_to_indexes.lower_bound(prn_base); 
         it != prn_to_indexes.upper_bound(prn_base); it++) {
      GnssMeasurementSDIndexPair& sd_pair_base = sd_pairs[it->second];
      if (sd_pair_base.rov.code_type == sd_pairs[i].rov.code_type) {
        dd_pairs.push_back(GnssMeasurementDDIndexPair(
          sd_pairs[i].rov, sd_pairs[i].ref, sd_pair_base.rov, sd_pair_base.ref));
        break;
      }
    }
  }

  return dd_pairs;
}

// Form double difference phaserange pair
GnssMeasurementDDIndexPairs formPhaserangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options)
{
  // Form SD pair
  GnssMeasurementSDIndexPairs sd_pairs = formPhaserangeSDPair(
    measurement_rov, measurement_ref, options);

  // Prepare data
  std::map<char, int> system_to_num_phases;
  std::multimap<char, double> system_to_phases;
  std::map<std::string, int> prn_to_number_phases; 
  std::multimap<std::string, double> prn_to_phases;
  std::multimap<std::string, int> prn_to_indexes;
  for (size_t i = 0; i < sd_pairs.size(); i++) {
    std::string prn = measurement_rov.getSat(sd_pairs[i].rov).prn;

    auto it = prn_to_number_phases.find(prn);
    if (it == prn_to_number_phases.end()) {
      prn_to_number_phases.insert(std::make_pair(prn, 1));
    }
    else it->second++;
    prn_to_indexes.insert(std::make_pair(prn, i));
    int code = sd_pairs[i].rov.code_type;
    double wavelength = measurement_rov.getObs(sd_pairs[i].rov).wavelength;
    int phase_id = getPhaseID(prn[0], code);
    prn_to_phases.insert(std::make_pair(prn, phase_id));
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

    for (auto it : prn_to_phases) {
      if (it.first[0] != system) continue;
      if (system_to_num_phases.find(system) == system_to_num_phases.end()) {
        system_to_num_phases.insert(std::make_pair(system, it.second));
      }
      bool found = false;
      for (auto it_wave = system_to_num_phases.lower_bound(system); 
          it_wave != system_to_num_phases.upper_bound(system); it_wave++) {
        if (it_wave->second == it.second) {
          found = true; break;
        }
      }
      if (!found) system_to_num_phases.insert(std::make_pair(system, it.second));
    }
  }

  // Find base satellites for each system and phases
  std::map<char, std::string> system_to_base_prn;
  for (size_t i = 0; i < getGnssSystemList().size(); i++) {
    char system = getGnssSystemList()[i];

    double max_elevation = 0.0;
    for (size_t j = 0; j < sd_pairs.size(); j++) {
      if (sd_pairs[j].rov.prn[0] != system) continue;

      // we only select satellites with max phase number
      if (prn_to_number_phases.at(sd_pairs[j].rov.prn) != 
          system_to_num_phases.at(system)) continue;

      double elevation = satelliteElevation(
        measurement_ref.getSat(sd_pairs[j].ref).sat_position, 
        measurement_ref.position);
      if (max_elevation < elevation) {
        system_to_base_prn[system] = sd_pairs[j].rov.prn;
        max_elevation = elevation;
      }
    }
  }

  // Form DD pair
  GnssMeasurementDDIndexPairs dd_pairs;
  for (size_t i = 0; i < sd_pairs.size(); i++) {
    char system = sd_pairs[i].rov.prn[0];
    std::string prn = sd_pairs[i].rov.prn;
    std::string prn_base = system_to_base_prn.at(system);

    if (prn == prn_base) continue;

    for (auto it = prn_to_indexes.lower_bound(prn_base); 
         it != prn_to_indexes.upper_bound(prn_base); it++) {
      GnssMeasurementSDIndexPair& sd_pair_base = sd_pairs[it->second];
      int phase_id_base = getPhaseID(system, sd_pair_base.rov.code_type);
      int phase_id = getPhaseID(system, sd_pairs[i].rov.code_type);
      if (phase_id_base == phase_id) {
        dd_pairs.push_back(GnssMeasurementDDIndexPair(
          sd_pairs[i].rov, sd_pairs[i].ref, sd_pair_base.rov, sd_pair_base.ref));
        break;
      }
    }
  }

  return dd_pairs;
}

// ----------------------------------------------------------
// Saastamoinen troposphere delay model
double troposphereSaastamoinen(double time, 
  const Eigen::Vector3d& ecef, double elevation, double humi)
{
  gtime_t gtime = doubleToGtime(time);
  gtime = utc2gpst(gtime);
  double azel[2];
  azel[0] = 0.0; azel[1] = elevation;
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());
  return tropmodel(gtime, lla.data(), azel, humi);
}

// GMF troposphere delay model
void troposphereGMF(double time, 
  const Eigen::Vector3d& ecef, double elevation, 
  double* gmfh, double* gmfw)
{
  double dfac[20], P[10][10], aP[55], bP[55], t;

  int i, n, m, nmax, mmax;
  double doy, phh;
  double ah, bh, ch, aw, bw, cw;
  double ahm, aha, awm, awa;
  double c10h, c11h, c0h;
  double a_ht, b_ht, c_ht;
  double sine, beta, gamma, topcon;
  double hs_km, ht_corr, ht_corr_coef;

  gtime_t gtime = doubleToGtime(time);
  gtime = utc2gpst(gtime);
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());

  //mjulianday mjd;
  //time2mjulianday(gt,&mjd);
  //dmjd=mjd.day+(mjd.ds.sn+mjd.ds.tos)/86400.0;
  const double ep[] = { 2000, 1, 1, 12, 0, 0 };
  double mjd, lat, lon, hgt, zd;

  if (lla[2] < -1000.0 || lla[2] > 20000.0)
  {
    if (gmfw) *gmfw = 0.0;
    return;
  }


  mjd = 51544.5 + (timediff(gtime, epoch2time(ep))) / 86400.0;
  lat = lla[0];
  lon = lla[1];
  hgt = lla[2] - geoidh(lla.data()); /* height in m (mean sea level) */
  zd = PI / 2.0 - elevation;


  double TWOPI = 6.283185307179586476925287;

  //reference day is 28 January
  //this is taken from Niell(1996) to be consistent
  //doy=dmjd-44239.0+1-28;
  doy = mjd - 44239.0 - 27;

  static double ah_mean[55] =
  {
    +1.2517e+02,	+8.503e-01,	+6.936e-02,	-6.760e+00, +1.771e-01,
    +1.130e-02,		+5.963e-01,	+1.808e-02, +2.801e-03, -1.414e-03,
    -1.212e+00,		+9.300e-02,	+3.683e-03, +1.095e-03, +4.671e-05,
    +3.959e-01,		-3.867e-02, +5.413e-03, -5.289e-04, +3.229e-04,
    +2.067e-05,		+3.000e-01, +2.031e-02, +5.900e-03, +4.573e-04,
    -7.619e-05,		+2.327e-06, +3.845e-06, +1.182e-01, +1.158e-02,
    +5.445e-03,		+6.219e-05, +4.204e-06, -2.093e-06, +1.540e-07,
    -4.280e-08,		-4.751e-01, -3.490e-02, +1.758e-03, +4.019e-04,
    -2.799e-06,		-1.287e-06, +5.468e-07, +7.580e-08, -6.300e-09,
    -1.160e-01,		+8.301e-03, +8.771e-04, +9.955e-05, -1.718e-06,
    -2.012e-06,		+1.170e-08, +1.790e-08, -1.300e-09, +1.000e-10
  };

  static double bh_mean[55] =
  {
    +0.000e+00,	+0.000e+00, +3.249e-02, +0.000e+00, +3.324e-02,
    +1.850e-02, +0.000e+00, -1.115e-01, +2.519e-02, +4.923e-03,
    +0.000e+00, +2.737e-02, +1.595e-02, -7.332e-04, +1.933e-04,
    +0.000e+00, -4.796e-02, +6.381e-03, -1.599e-04, -3.685e-04,
    +1.815e-05, +0.000e+00, +7.033e-02, +2.426e-03, -1.111e-03,
    -1.357e-04, -7.828e-06, +2.547e-06, +0.000e+00, +5.779e-03,
    +3.133e-03, -5.312e-04, -2.028e-05, +2.323e-07, -9.100e-08,
    -1.650e-08, +0.000e+00, +3.688e-02, -8.638e-04, -8.514e-05,
    -2.828e-05, +5.403e-07, +4.390e-07, +1.350e-08, +1.800e-09,
    +0.000e+00, -2.736e-02, -2.977e-04, +8.113e-05, +2.329e-07,
    +8.451e-07, +4.490e-08, -8.100e-09, -1.500e-09, +2.000e-10
  };

  static double ah_amp[55] =
  {
    -2.738e-01, -2.837e+00, +1.298e-02, -3.588e-01, +2.413e-02,
    +3.427e-02, -7.624e-01, +7.272e-02, +2.160e-02, -3.385e-03,
    +4.424e-01, +3.722e-02, +2.195e-02, -1.503e-03, +2.426e-04,
    +3.013e-01, +5.762e-02, +1.019e-02, -4.476e-04, +6.790e-05,
    +3.227e-05, +3.123e-01, -3.535e-02, +4.840e-03, +3.025e-06,
    -4.363e-05, +2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
    +8.964e-04, +1.399e-04, -3.990e-06, +7.431e-06, -2.796e-07,
    -1.601e-07, +4.068e-02, -1.352e-02, +7.282e-04, +9.594e-05,
    +2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09,
    +8.625e-02, -5.971e-03, +4.705e-04, +2.335e-05, +4.226e-06,
    +2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, +0.000e+00
  };

  static double bh_amp[55] =
  {
    +0.000e+00, +0.000e+00, -1.136e-01, +0.000e+00, -1.868e-01,
    -1.399e-02, +0.000e+00, -1.043e-01, +1.175e-02, -2.240e-03,
    +0.000e+00, -3.222e-02, +1.333e-02, -2.647e-03, -2.316e-05,
    +0.000e+00, +5.339e-02, +1.107e-02, -3.116e-03, -1.079e-04,
    -1.299e-05, +0.000e+00, +4.861e-03, +8.891e-03, -6.448e-04,
    -1.279e-05, +6.358e-06, -1.417e-07, +0.000e+00, +3.041e-02,
    +1.150e-03, -8.743e-04, -2.781e-05, +6.367e-07, -1.140e-08,
    -4.200e-08, +0.000e+00, -2.982e-02, -3.000e-03, +1.394e-05,
    -3.290e-05, -1.705e-07, +7.440e-08, +2.720e-08, -6.600e-09,
    +0.000e+00, +1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
    +1.162e-06, -1.789e-07, +1.470e-08, -2.400e-09, -4.000e-10
  };

  static double aw_mean[55] =
  {
    +5.640e+01, +1.555e+00, -1.011e+00, -3.975e+00, +3.171e-02,
    +1.065e-01, +6.175e-01, +1.376e-01, +4.229e-02, +3.028e-03,
    +1.688e+00, -1.692e-01, +5.478e-02, +2.473e-02, +6.059e-04,
    +2.278e+00, +6.614e-03, -3.505e-04, -6.697e-03, +8.402e-04,
    +7.033e-04, -3.236e+00, +2.184e-01, -4.611e-02, -1.613e-02,
    -1.604e-03, +5.420e-05, +7.922e-05, -2.711e-01, -4.406e-01,
    -3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, +6.894e-06,
    +2.317e-06, +1.941e+00, -2.562e-01, +1.598e-02, +5.449e-03,
    +3.544e-04, +1.148e-05, +7.503e-06, -5.667e-07, -3.660e-08,
    +8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, +2.029e-04,
    +1.269e-05, +1.629e-06, +9.660e-08, -1.015e-07, -5.000e-10
  };

  static double bw_mean[55] =
  {
    +0.000e+00, +0.000e+00, +2.592e-01, +0.000e+00, +2.974e-02,
    -5.471e-01, +0.000e+00, -5.926e-01, -1.030e-01, -1.567e-02,
    +0.000e+00, +1.710e-01, +9.025e-02, +2.689e-02, +2.243e-03,
    +0.000e+00, +3.439e-01, +2.402e-02, +5.410e-03, +1.601e-03,
    +9.669e-05, +0.000e+00, +9.502e-02, -3.063e-02, -1.055e-03,
    -1.067e-04, -1.130e-04, +2.124e-05, +0.000e+00, -3.129e-01,
    +8.463e-03, +2.253e-04, +7.413e-05, -9.376e-05, -1.606e-06,
    +2.060e-06, +0.000e+00, +2.739e-01, +1.167e-03, -2.246e-05,
    -1.287e-04, -2.438e-05, -7.561e-07, +1.158e-06, +4.950e-08,
    +0.000e+00, -1.344e-01, +5.342e-03, +3.775e-04, -6.756e-05,
    -1.686e-06, -1.184e-06, +2.768e-07, +2.730e-08, +5.700e-09
  };

  static double aw_amp[55] =
  {
    +1.023e-01, -2.695e+00, +3.417e-01, -1.405e-01, +3.175e-01,
    +2.116e-01, +3.536e+00, -1.505e-01, -1.660e-02, +2.967e-02,
    +3.819e-01, -1.695e-01, -7.444e-02, +7.409e-03, -6.262e-03,
    -1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, +7.947e-04,
    +1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03,
    -2.976e-04, +1.857e-05, +3.021e-05, +2.248e+00, -1.178e-01,
    +1.255e-02, +1.134e-03, -2.161e-04, -5.817e-06, +8.836e-07,
    -1.769e-07, +7.313e-01, -1.188e-01, +1.145e-02, +1.011e-03,
    +1.083e-04, +2.570e-06, -2.140e-06, -5.710e-08, +2.000e-08,
    -1.632e+00, -6.948e-03, -3.893e-03, +8.592e-04, +7.577e-05,
    +4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, +5.800e-09
  };

  static double bw_amp[55] =
  {
    +0.000e+00, +0.000e+00, -8.865e-02, +0.000e+00, -4.309e-01,
    +6.340e-02, +0.000e+00, +1.162e-01, +6.176e-02, -4.234e-03,
    +0.000e+00, +2.530e-01, +4.017e-02, -6.204e-03, +4.977e-03,
    +0.000e+00, -1.737e-01, -5.638e-03, +1.488e-04, +4.857e-04,
    -1.809e-04, +0.000e+00, -1.514e-01, -1.685e-02, +5.333e-03,
    -7.611e-05, +2.394e-05, +8.195e-06, +0.000e+00, +9.326e-02,
    -1.275e-02, -3.071e-04, +5.374e-05, -3.391e-05, -7.436e-06,
    +6.747e-07, +0.000e+00, -8.637e-02, -3.807e-03, -6.833e-04,
    -3.861e-05, -2.268e-05, +1.454e-06, +3.860e-07, -1.068e-07,
    +0.000e+00, -2.658e-02, -1.947e-03, +7.131e-04, -3.506e-05,
    +1.885e-07, +5.792e-07, +3.990e-08, +2.000e-08, -5.700e-09
  };

  //parameter t
  t = sin(lat);

  //degree n and order m
  nmax = 9;
  mmax = 9;

  //determine nmax!(faktorielle) moved by 1
  dfac[0] = 1;
  for (i = 1; i <= 2 * nmax + 1; i++)
    dfac[i] = dfac[i - 1] * i;


  int j;
  int ir;
  int k;
  double sum;
  // determine Legendre functions (Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62)
  for (i = 0; i <= nmax; i++)
  {
    for (j = 0; j <= std::min(i, mmax); j++)
    {
      ir = int((i - j) / 2);
      sum = 0.0;

      for (k = 0; k <= ir; k++)
      {
        sum = sum + pow(-1.0, k) * dfac[2 * i - 2 * k] / dfac[k] / 
          dfac[i - k] / dfac[i - j - 2 * k] * pow(t, i - j - 2 * k);
      }

      //Legender functions moved by 1
      P[i][j] = 1.0 / pow(2.0, i) * sqrt(pow(1 - t * t, j)) * sum;
    }
  }

  //spherical harmonics
  i = 0;
  double dt;
  for (n = 0; n <= 9; n++)
  {
    for (m = 0; m <= n; m++)
    {
      i = i + 1;
      dt = m * lon;
      aP[i - 1] = P[n][m] * cos(dt);
      bP[i - 1] = P[n][m] * sin(dt);
    }
  }

  //hydrostatic
  bh = 0.0029;
  c0h = 0.062;

  if (lat < 0)  	//southern hemisphere
  {
    phh = PI;
    c11h = 0.007;
    c10h = 0.002;
  }
  else  	//northern hemisphere
  {
    phh = 0;
    c11h = 0.005;
    c10h = 0.001;
  }

  ch = c0h + ((cos(doy / 365.25 * TWOPI + 
    phh) + 1.0) * c11h / 2.0 + c10h) * (1.0 - cos(lat));

  ahm = 0.0;
  aha = 0.0;
  for (i = 1; i <= 55; i++)
  {
    ahm = ahm + (ah_mean[i - 1] * aP[i - 1] + bh_mean[i - 1] * bP[i - 1]) * 1.0e-5;
    aha = aha + (ah_amp[i - 1] * aP[i - 1] + bh_amp[i - 1] * bP[i - 1]) * 1.0e-5;
  }

  ah = ahm + aha * cos(doy / 365.25 * TWOPI);

  sine = sin(elevation);
  beta = bh / (sine + ch);
  gamma = ah / (sine + beta);
  topcon = (1.0 + ah / (1.0 + bh / (1.0 + ch)));
  if (gmfh) *gmfh = topcon / (sine + gamma);

  a_ht = 2.53e-5;	//2.53 from http://maia.usno.navy.mil/conv2010/chapter9/GMF.F

  b_ht = 5.49e-3;
  c_ht = 1.14e-3;
  hs_km = hgt / 1000.0;

  beta = b_ht / (sine + c_ht);
  gamma = a_ht / (sine + beta);
  topcon = (1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht)));
  ht_corr_coef = 1.0 / sine - topcon / (sine + gamma);
  ht_corr = ht_corr_coef * hs_km;
  if (gmfh) *gmfh = *gmfh + ht_corr;

  //wet
  bw = 0.00146;
  cw = 0.04391;

  awm = 0.0;
  awa = 0.0;
  for (i = 1; i <= 55; i++)
  {
    awm = awm + (aw_mean[i - 1] * aP[i - 1] + bw_mean[i - 1] * bP[i - 1]) * 1e-5;
    awa = awa + (aw_amp[i - 1] * aP[i - 1] + bw_amp[i - 1] * bP[i - 1]) * 1e-5;
  }
  aw = awm + awa * cos(doy / 365.25 * TWOPI);

  beta = bw / (sine + cw);
  gamma = aw / (sine + beta);
  topcon = (1.0 + aw / (1.0 + bw / (1.0 + cw)));
  if (gmfw) *gmfw = topcon / (sine + gamma);
}

// Broadcast ionosphere model
double ionosphereBroadcast(double time, const Eigen::Vector3d& ecef, 
  double azimuth, double elevation, double wavelength, 
  const Eigen::VectorXd& parameters)
{
  gtime_t gtime = doubleToGtime(time);
  gtime = utc2gpst(gtime);
  double azel[2];
  azel[0] = azimuth; azel[1] = elevation;
  Eigen::Vector3d lla;
  ecef2pos(ecef.data(), lla.data());
  Eigen::VectorXd parameters_local = parameters;
  if (parameters.size() < 8) parameters_local = Eigen::VectorXd::Zero(8);
  double ion_l1 = ionmodel(gtime, parameters_local.data(), lla.data(), azel);
  return ion_l1 * square(wavelength / (CLIGHT / FREQ1));
}

// Dual-frequenct ionosphere model
double ionosphereDualFrequency(
  const Observation& obs_1, const Observation& obs_2)
{
  CHECK(obs_1.wavelength < obs_2.wavelength);
  return (obs_1.pseudorange - obs_2.pseudorange) / 
         (1.0 - square(obs_2.wavelength / obs_1.wavelength));
}

// Compute Receiver to satellite distance considering the earth rotation effect
double satelliteToReceiverDistance(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  // Correct the satellite position
  double rho0 = (satellite_ecef - receiver_ecef).norm();
  double dPhi = OMGE * rho0 / CLIGHT;

  Eigen::Vector3d xRec;

  xRec(0) = receiver_ecef(0) * cos(dPhi) - receiver_ecef(1) * sin(dPhi);
  xRec(1) = receiver_ecef(1) * cos(dPhi) + receiver_ecef(0) * sin(dPhi);
  xRec(2) = receiver_ecef(2);

  // Compute the distance
  return (satellite_ecef - xRec).norm();
}

// Satellite elevation angle
double satelliteElevation(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  Eigen::Vector3d rr = satellite_ecef - receiver_ecef;
  double rho0 = rr.norm();

  double enu[3], pos[3];
  ecef2pos(receiver_ecef.data(), pos);
  ecef2enu(pos, rr.data(), enu);

  double el = acos(sqrt(enu[0]*enu[0] + enu[1]*enu[1]) / rho0);
  if (enu[2] < 0) el *= -1.0;
  return el;
}

// Satellite azimuth angle
double satelliteAzimuth(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef)
{
  Eigen::Vector3d rr = satellite_ecef - receiver_ecef;
  double rho0 = rr.norm();

  double enu[3], pos[3];
  ecef2pos(receiver_ecef.data(), pos);
  ecef2enu(pos, rr.data(), enu);

  return atan2(enu[0], enu[1]);
}

// Compute DOPs
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement,
  const GnssCommonOptions& options)
{
  if (checkZero(measurement.position)) {
    LOG(WARNING) << "Cannot compute DOPs: the receiver position should not be zero!";
    return Eigen::Vector4d::Zero();
  }

  int ns = 0;
  double azel[2 * MAXSAT];
  Eigen::Vector4d Dops;
  for (auto sat : measurement.satellites) {
    int num_obs = 0;
    for (auto obs : sat.second.observations) {
      if (num_obs > 0) continue;
      GnssMeasurementIndex index(sat.first, obs.first);
      if (!checkObservationValid(measurement, index, 
          ObservationType::Pseudorange, options)) continue;
      double elevation = satelliteElevation(
        sat.second.sat_position, measurement.position);
      double azimuth = satelliteAzimuth(
        sat.second.sat_position, measurement.position);
      azel[2 * ns] = azimuth;
      azel[2 * ns + 1] = elevation;
      ns++;
      num_obs++;
    }
  }
  dops(ns, azel, degreeToRad(options.min_elevation), Dops.data());

  if (Dops.norm() == 0.0) 
  for (size_t i = 0; i < 4; i++) Dops(i) = 100.0;

  return Dops;
}
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurementSDIndexPairs& indexes,
  const GnssCommonOptions& options)
{
  if (checkZero(measurement_rov.position)) {
    LOG(WARNING) << "Cannot compute DOPs: the receiver position should not be zero!";
    return Eigen::Vector4d::Zero();
  }

  int ns = 0;
  double azel[2 * MAXSAT];
  Eigen::Vector4d Dops;
  std::string current_prn = "";
  for (auto index : indexes) {
    if (index.rov.prn == current_prn) continue;
    current_prn = index.rov.prn;

    const auto& satellite = measurement_rov.getSat(index.rov);
    double elevation = satelliteElevation(
      satellite.sat_position, measurement_rov.position);
    double azimuth = satelliteAzimuth(
      satellite.sat_position, measurement_rov.position);
    azel[2 * ns] = azimuth;
    azel[2 * ns + 1] = elevation;
    ns++;
  }
  dops(ns, azel, degreeToRad(options.min_elevation), Dops.data());

  if (Dops.norm() == 0.0) 
  for (size_t i = 0; i < 4; i++) Dops(i) = 100.0;

  return Dops;
}
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurementDDIndexPairs& indexes,
  const GnssCommonOptions& options)
{
  if (checkZero(measurement_rov.position)) {
    LOG(WARNING) << "Cannot compute DOPs: the receiver position should not be zero!";
    return Eigen::Vector4d::Zero();
  }

  int ns = 0;
  double azel[2 * MAXSAT];
  Eigen::Vector4d Dops;
  std::string current_prn = "";
  for (auto index : indexes) {
    if (index.rov.prn == current_prn) continue;
    current_prn = index.rov.prn;

    const auto& satellite = measurement_rov.getSat(index.rov);
    double elevation = satelliteElevation(
      satellite.sat_position, measurement_rov.position);
    double azimuth = satelliteAzimuth(
      satellite.sat_position, measurement_rov.position);
    azel[2 * ns] = azimuth;
    azel[2 * ns + 1] = elevation;
    ns++;
  }
  // add base satellites
  for (auto index : indexes) {
    if (index.rov_base.prn == current_prn) continue;
    current_prn = index.rov_base.prn;

    const auto& satellite = measurement_rov.getSat(index.rov_base);
    double elevation = satelliteElevation(
      satellite.sat_position, measurement_rov.position);
    double azimuth = satelliteAzimuth(
      satellite.sat_position, measurement_rov.position);
    azel[2 * ns] = azimuth;
    azel[2 * ns + 1] = elevation;
    ns++;
  }
  dops(ns, azel, degreeToRad(options.min_elevation), Dops.data());

  if (Dops.norm() == 0.0) 
  for (size_t i = 0; i < 4; i++) Dops(i) = 100.0;

  return Dops;
}

// Melbourne-Wubbena (MW) combination
double combinationMW(const Observation& observation_1,
                     const Observation& observation_2)
{
  double P1 = observation_1.pseudorange;
  double P2 = observation_2.pseudorange;
  double L1 = observation_1.phaserange;
  double L2 = observation_2.phaserange;
  double lam1 = observation_1.wavelength;
  double lam2 = observation_2.wavelength;

  double mw = (L1 * lam2 - L2 * lam1) / (lam2 - lam1) - 
              (P1 * lam2 + P2 * lam1) / (lam2 + lam1);
  return mw;
}

// Geometry Free (GF) combination
double combinationGF(const Observation& observation_1,
                     const Observation& observation_2)
{
  double L1 = observation_1.phaserange;
  double L2 = observation_2.phaserange;

  double gf = L1 - L2;
  return gf;
}

// BDS satellite multipath correction
double getBdsSatelliteMultipath(const std::string prn, 
  const double elevation, const double code_type)
{
  const static double igso_coef[3][10] = {	
		{-0.55,-0.40,-0.34,-0.23,-0.15,-0.04,0.09,0.19,0.27,0.35},	// B1
		{-0.71,-0.36,-0.33,-0.19,-0.14,-0.03,0.08,0.17,0.24,0.33},	// B2
		{-0.27,-0.23,-0.21,-0.15,-0.11,-0.04,0.05,0.14,0.19,0.32},	// B3
	};
  const static double meo_coef[3][10] = {
		{-0.47,-0.38,-0.32,-0.23,-0.11,0.06,0.34,0.69,0.97,1.05},	  // B1
		{-0.40,-0.31,-0.26,-0.18,-0.06,0.09,0.28,0.48,0.64,0.69},	  // B2
		{-0.22,-0.15,-0.13,-0.10,-0.04,0.05,0.14,0.27,0.36,0.47},	  // B3
	};

  // only for BDS
  if (prn[0] != 'C') return 0.0;

  int icoef = -1;
  int phase_id = getPhaseID(prn[0], code_type);
  if (phase_id == 0) icoef = 0;  // B1
  if (phase_id == 3) icoef = 1;  // B2
  if (phase_id == 5) icoef = 2;  // B3
  // no need to correct the BDS-3 signals
  if (icoef == -1) return 0.0;

  double elevation_deg = elevation * R2D;
  if (elevation_deg <= 0.0) return 0.0;
  double a = elevation_deg * 0.1;
  int b = static_cast<int>(a);
  // IGSO
  if (isBdsIgso(prn)) {
    if (b < 0) return igso_coef[icoef][0];
    else if (b >= 9) return igso_coef[icoef][9];
    else {
      return igso_coef[icoef][b] * (1.0 - a + b) + igso_coef[icoef][b + 1] * (a - b);
    }
  }
  // MEO
  else if (isBdsMeo(prn)) {
    if (b < 0) return meo_coef[icoef][0];
    else if (b >= 9) return meo_coef[icoef][9];
    else {
      return meo_coef[icoef][b] * (1.0 - a + b) + meo_coef[icoef][b + 1] * (a - b);
    }
  }

  return 0.0;
}

// Solid earth tide
Eigen::Vector3d solidEarthTide(
  const double time, const Eigen::Vector3d receiver_ecef)
{
  Eigen::Vector3d tide = Eigen::Vector3d::Zero();
  tidedisp(doubleToGtime(time), receiver_ecef.data(), 1, NULL, NULL, tide.data());
  if (tide.norm() > 10.0 || isnan(tide.norm())) tide = Eigen::Vector3d::Zero();
  return tide;
}

}

}