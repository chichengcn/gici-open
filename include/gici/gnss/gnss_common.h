/**
* @Function: GNSS common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <memory>
#include <map>
#include <Eigen/Core>

#include "gici/utility/rtklib_safe.h"
#include "gici/stream/formator.h"
#include "gici/gnss/gnss_types.h"
#include "gici/utility/common.h"

namespace gici {

namespace gnss_common {

// ----------------------------------------------------------
// Convert char system to int system
int systemConvert(char sys);

// Convert int system to char system
char systemConvert(int sys);

// Convert PRN string to RTKLIB sat (1-MAXSAT)
int prnToSat(std::string prn);

// Convert RTKLIB sat (1-MAXSAT) to PRN string
std::string satToPrn(int sat);

// Get frequency from code type, and channel
double codeToFrequency(char system, int code, int channel = 0);

// Get frequency from phase type, and channel 
double phaseToFrequency(char system, int phase_id, int channel = 0);

// Convert gtime to double
double gtimeToDouble(gtime_t time);

// Convert GPS time to UTC time
double gpsTimeToUtcTime(double gps_time);

// Convert UTC time to GPS time
double utcTimeToGpsTime(double utc_time);

// Convert double to gtime
gtime_t doubleToGtime(double time);

// Degree to Rad
inline double degreeToRad(double degree) {
  return degree * D2R;
}

// Rad to Degree
inline double radToDegree(double rad) {
  return rad * R2D;
}

// Get a phase ID
int getPhaseID(char system, int code_type);

// Check BDS constellation types (http://www.csno-tarc.cn/en/system/constellation)
inline bool checkBdsType(const std::string& prn, const std::vector<int>& list) {
  if (prn[0] != 'C') return false;
  int prn_num = atoi(prn.substr(1, 2).data());
  for (size_t i = 0; i < list.size(); i++) {
    if (prn_num == list[i]) return true;
  }
  return false;
}
inline bool isBds1(const std::string prn) {
  static std::vector<int> list;
  return checkBdsType(prn, list);
}
inline bool isBds2(const std::string prn) {
  static std::vector<int> list;
  if (list.size() == 0) {
    for (size_t i = 1; i <= 16; i++) list.push_back(i);
  }
  return checkBdsType(prn, list);
}
inline bool isBds3(const std::string prn) {
  if (prn[0] != 'C') return false;
  return !isBds1(prn) && !isBds2(prn);
}
inline bool isBdsGeo(const std::string prn) {
  static std::vector<int> list;
  if (list.size() == 0) {
    for (size_t i = 1; i <= 5; i++) list.push_back(i);
    for (size_t i = 59; i <= 60; i++) list.push_back(i);
  }
  return checkBdsType(prn, list);
}
inline bool isBdsIgso(const std::string prn) {
  static std::vector<int> list;
  if (list.size() == 0) {
    for (size_t i = 6; i <= 10; i++) list.push_back(i);
    list.push_back(13);
    list.push_back(16);
    for (size_t i = 38; i <= 40; i++) list.push_back(i);
  }
  return checkBdsType(prn, list);
}
inline bool isBdsMeo(const std::string prn) {
  if (prn[0] != 'C') return false;
  return !isBdsGeo(prn) && !isBdsIgso(prn);
}

// Convert rinex type string to code type
int rinexTypeToCodeType(const char system, const std::string str);

// Convert code type to rinex type string
std::string codeTypeToRinexType(const char system, const int code_type);

// Convert phase string to phase type
int phaseStringToPhaseType(const char system, const std::string str);

// Convert phase type to phase string
std::string phaseTypeToPhaseString(const char system, const int phase_type);

// ----------------------------------------------------------
// Check whether the system is used
bool useSystem(const GnssCommonOptions& options, const char system);
bool useSystem(const std::vector<char>& system_exclude, const char system);

// Check whether the satellite is used
bool useSatellite(const GnssCommonOptions& options, const std::string prn);
bool useSatellite(const std::vector<std::string>& satellite_exclude, 
                  const std::string prn);

// Check whether the code type is used
bool useCode(const GnssCommonOptions& options, char system, const int code_type);
bool useCode(const std::vector<std::pair<char, int>>& code_exclude, 
             char system, const int code_type);

// Check whether the phase type is used
bool usePhase(const std::vector<std::pair<char, int>>& phase_exclude, 
              char system, const int phase_id);

// Check elevation threshold
bool checkElevation(const GnssCommonOptions& options, 
  const GnssMeasurement& measurement, std::string prn);

// Check SNR mask
bool checkSNR(const GnssCommonOptions& options,
  const GnssMeasurement& measurement, const GnssMeasurementIndex& index);

// Erase duplicated phases, arrange to one observation per phase
void rearrangePhasesAndCodes(GnssMeasurement& measurement, bool accept_coarse = true);

// Check observation valid
bool checkObservationValid(const GnssMeasurement& measurement,
                           const GnssMeasurementIndex& index,
                           const ObservationType type, 
                           const GnssCommonOptions& options = GnssCommonOptions(),
                           const bool need_precise_ephemeris = false);

// Form single difference pseudorange pair
GnssMeasurementSDIndexPairs formPseudorangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options = GnssCommonOptions());

// Form single difference phaserange pair
GnssMeasurementSDIndexPairs formPhaserangeSDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssCommonOptions& options = GnssCommonOptions());

// Form double difference pseudorange pair
// we use satellite with highest elevation angle as base satellite
GnssMeasurementDDIndexPairs formPseudorangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            std::map<char, std::string>& system_to_base_prn,
                            const GnssCommonOptions& options = GnssCommonOptions());

// Form double difference phaserange pair
GnssMeasurementDDIndexPairs formPhaserangeDDPair(
                            const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            std::map<char, std::string>& system_to_base_prn,
                            const GnssCommonOptions& options = GnssCommonOptions());

// ----------------------------------------------------------
// Saastamoinen troposphere delay model
double troposphereSaastamoinen(double time, 
  const Eigen::Vector3d& ecef, double elevation, double humi = 0.0);

// GMF troposphere delay model
void troposphereGMF(double time, 
  const Eigen::Vector3d& ecef, double elevation, 
  double* gmfh, double* gmfw);

// Broadcast ionosphere model
double ionosphereBroadcast(double time, const Eigen::Vector3d& ecef, 
  double azimuth, double elevation, double wavelength, 
  const Eigen::VectorXd& parameters = Eigen::VectorXd::Zero(8));

// Dual-frequenct ionosphere model
// the wavelength of obs_1 should be smaller than obs_2
// output ionosphere is in meter at obs_1 frequency
double ionosphereDualFrequency(
  const Observation& obs_1, const Observation& obs_2);

// Convert ionosphere delay to 1575.42 MHz
inline double ionosphereConvertToBase(
  double ionosphere, double wavelength) {
  return ionosphere * square(CLIGHT / FREQ1 / wavelength);
}

// Convert ionosphere delay from 1575.42 MHz to given wavelength
inline double ionosphereConvertFromBase(
  double ionosphere, double wavelength) {
  return ionosphere * square(wavelength / (CLIGHT / FREQ1));
}

// Receiver to satellite distance considering the earth rotation effect
double satelliteToReceiverDistance(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite elevation angle
double satelliteElevation(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Satellite azimuth angle
double satelliteAzimuth(
  const Eigen::Vector3d satellite_ecef, const Eigen::Vector3d receiver_ecef);

// Compute DOPs (GDOP,PDOP,HDOP,VDOP)
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement,
  const GnssCommonOptions& options = GnssCommonOptions());
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurementSDIndexPairs& indexes,
  const GnssCommonOptions& options = GnssCommonOptions());
Eigen::Vector4d computeDops(
  const GnssMeasurement& measurement_rov,
  const GnssMeasurementDDIndexPairs& indexes,
  const GnssCommonOptions& options = GnssCommonOptions());

// Melbourne-Wubbena (MW) combination
double combinationMW(const Observation& observation_1,
                     const Observation& observation_2);

// Geometry Free (GF) combination
double combinationGF(const Observation& observation_1,
                     const Observation& observation_2);

// BDS satellite multipath correction (P_corrected = P + value)
double getBdsSatelliteMultipath(const std::string prn, 
  const double elevation, const double code_type);

// Solid earth tide
Eigen::Vector3d solidEarthTide(
  const double time, const Eigen::Vector3d receiver_ecef);


}

}