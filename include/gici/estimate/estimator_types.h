/**
* @Function: Estimator types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <map>
#include <vector>

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Eigen 3.2.7 uses std::binder1st and std::binder2nd which are deprecated since c++11
// Fix is in 3.3 devel (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=872).
#include <ceres/ceres.h>
#pragma diagnostic pop

#include <glog/logging.h>

#include "gici/utility/svo.h"
#include "gici/utility/rtklib_safe.h"
#include "gici/imu/imu_types.h"
#include "gici/gnss/geodetic_coordinate.h"
#include "gici/gnss/gnss_types.h"
#include "gici/vision/image_types.h"
#include "gici/utility/option.h"
#include "gici/estimate/graph.h"

namespace gici {

// -----------------------------------------------------------------------------
// IDs
enum class IdType : uint8_t
{
  cPose = 0,
  cLandmark = 1,
  ImuStates = 2,
  cExtrinsics = 3,
  gPosition = 4,
  gVelocity = 5,
  gPose = 6, 
  gClock = 7, 
  gFrequency = 8,
  gTroposphere = 9,
  gExtrinsics = 10,
  gAmbiguity = 11,
  gIonosphere = 12,
  gIfb = 13
};

using SensorType = option_tools::SensorType;

// The BackendID for multiple types
// Common bits ----
// bit 0-5:   IdType
#define BITS_IDTYPE 0, 5
// bit 22-49: BundleID 
#define BITS_BUNDLEID 22, 49  // only for time-relevant parameters
#define RANGE_BUNDLE_ID_MIN 0
#define RANGE_BUNDLE_ID_MAX 67108863  // 2^26 - 1
// IMU relevant bits ----
// bit 6-11: Sub-IdType, defining the corresponding pose type.
#define BITS_IMU_SUBID 6, 11
// GNSS relevant bits ----
// bit 6-13: GNSS system
#define BITS_GNSS_SYSTEM 6, 13
// bit 14-21: GNSS PRN number
#define BITS_GNSS_PRN 14, 21
// bit 50-55: GNSS PhaseID
#define BITS_GNSS_PHASEID 50, 55
// bit 50-56: GNSS CodeID
#define BITS_GNSS_CODEID 50, 56
// Visual relevant bits ----
// bit 6-13: CameraIdx
#define BITS_CAMERA_IDX 6, 13
// bit 32-63: Landmark ID
#define BITS_LANDMARKID 32, 63

class BackendId
{
public:
  BackendId() = default;
  explicit BackendId(uint64_t id) : id_(id) {}

  // Get bits
  inline static uint32_t getBits(uint64_t id, int start, int end) {
    int length = end - start + 1;
    uint8_t buffer[8];
    for (int i = 0; i < 8; i++) {
      buffer[i] = (id >> (8 * (7 - i))) & 0xFF;
    }
    return getbitu(buffer, start, length);
  }

  // Set bits
  template<typename T> 
  inline static uint64_t setBits(T data, int start, int end) {
    int length = end - start + 1;
    uint32_t bits = static_cast<uint32_t>(data);
    uint64_t id = 0;
    uint8_t buffer[8];
    memset(buffer, 0, sizeof(uint8_t) * 8);
    setbitu(buffer, start, length, bits);
    for (int i = 0; i < 8; i++) {
      uint64_t byte = static_cast<uint64_t>(buffer[i]) << (8 * (7 - i));
      id += byte;
    }
    return id;
  }

  // Reset some bits
  template<typename T> 
  inline static uint64_t resetBits(
    uint64_t id, T data, int start, int end) {
    int length = end - start + 1;
    uint32_t bits = static_cast<uint32_t>(data);
    uint64_t out_id = 0;
    uint8_t buffer[8];
    for (int i = 0; i < 8; i++) {
      buffer[i] = (id >> (8 * (7 - i))) & 0xFF;
    }
    setbitu(buffer, start, length, bits);
    for (int i = 0; i < 8; i++) {
      uint64_t byte = static_cast<uint64_t>(buffer[i]) << (8 * (7 - i));
      out_id |= byte;
    }
    return out_id;
  }

  // Check sensor type
  inline static SensorType sensorType(IdType type) {
    if (type == IdType::cPose || type == IdType::cExtrinsics || 
        type == IdType::cLandmark) return SensorType::Camera;
    if (type == IdType::ImuStates) return SensorType::IMU;
    if (type == IdType::gAmbiguity || type == IdType::gClock || 
        type == IdType::gExtrinsics || type == IdType::gFrequency ||
        type == IdType::gIonosphere || type == IdType::gPose || 
        type == IdType::gPosition || type == IdType::gTroposphere ||
        type == IdType::gVelocity || type == IdType::gIfb) 
      return SensorType::GNSS;
    return SensorType::None;
  }

  // Adjust bundle_id for sensor types
  inline static int32_t adjustBundleId(int32_t bundle_id, IdType type) {
    int32_t out = bundle_id;
    const int32_t length = RANGE_BUNDLE_ID_MAX - RANGE_BUNDLE_ID_MIN + 1;
    while (out < RANGE_BUNDLE_ID_MIN) out += length;
    while (out > RANGE_BUNDLE_ID_MAX) out -= length;
    return out;
  }

  inline uint64_t asInteger() const {
    return id_;
  }

  inline IdType type() const {
    return static_cast<IdType>(getBits(id_, BITS_IDTYPE));
  }

  inline int32_t bundleId() const {
    CHECK(type() != IdType::cLandmark)
        << "Landmarks do not have a bundle ID.";
    return static_cast<int32_t>(getBits(id_, BITS_BUNDLEID));
  }

  inline uint32_t trackId() const {
    CHECK(type() == IdType::cLandmark);
    return static_cast<uint32_t>(getBits(id_, BITS_LANDMARKID));
  }

  inline uint8_t cameraIndex() const {
    CHECK(type() == IdType::cExtrinsics);
    return static_cast<uint8_t>(getBits(id_, BITS_CAMERA_IDX));
  }

  inline char gSystem() const {
    return static_cast<char>(getBits(id_, BITS_GNSS_SYSTEM));
  }

  inline int gPrnNumber() const {
    return static_cast<int>(getBits(id_, BITS_GNSS_PRN));
  }

  inline std::string gPrn() const {
    char system = gSystem();
    int prn_number = gPrnNumber();
    char prn_buf[4];
    sprintf(prn_buf, "%c%02d", system, prn_number);
    return std::string(prn_buf);
  }

  inline int gPhaseId() const {
    return static_cast<int>(getBits(id_, BITS_GNSS_PHASEID));
  }

  inline int gCodeId() const {
    return static_cast<int>(getBits(id_, BITS_GNSS_CODEID));
  }

  inline bool valid() const {
    return id_ != 0;
  }

private:
  uint64_t id_{0};
};

// Factories
inline BackendId createLandmarkId(int track_id)
{
  return BackendId(
    BackendId::setBits(track_id, BITS_LANDMARKID) |
    BackendId::setBits(IdType::cLandmark, BITS_IDTYPE));
}

inline BackendId createNFrameId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::cPose), BITS_BUNDLEID) |
    BackendId::setBits(IdType::cPose, BITS_IDTYPE));
}

inline BackendId createGnssPositionId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gPosition), BITS_BUNDLEID) |
    BackendId::setBits(IdType::gPosition, BITS_IDTYPE));
}

inline BackendId createGnssVelocityId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gVelocity), BITS_BUNDLEID) |
    BackendId::setBits(IdType::gVelocity, BITS_IDTYPE));
}

inline BackendId createGnssPoseId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gPose), BITS_BUNDLEID) |
    BackendId::setBits(IdType::gPose, BITS_IDTYPE));
}

inline BackendId createGnssClockId(char system,
                                   int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gClock), BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(IdType::gClock, BITS_IDTYPE));
}

inline BackendId createGnssFrequencyId(char system,
                                   int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gFrequency), BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(IdType::gFrequency, BITS_IDTYPE));
}

inline BackendId createGnssAmbiguityId(std::string prn,
                  int phase_id, int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  CHECK_GE(phase_id, 0);
  char system = prn[0];
  int prn_number = atoi(prn.substr(1, 2).data());
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gAmbiguity), BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(prn_number, BITS_GNSS_PRN) |
    BackendId::setBits(phase_id, BITS_GNSS_PHASEID) |
    BackendId::setBits(IdType::gAmbiguity, BITS_IDTYPE));
}

inline BackendId createGnssTroposphereId(int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gTroposphere), BITS_BUNDLEID) |
    BackendId::setBits(IdType::gTroposphere, BITS_IDTYPE));
}

inline BackendId createGnssIonosphereId(std::string prn, int32_t bundle_id)
{
  CHECK_GE(bundle_id, 0);
  char system = prn[0];
  int prn_number = atoi(prn.substr(1, 2).data());
  return BackendId(
    BackendId::setBits(BackendId::adjustBundleId(
      bundle_id, IdType::gIonosphere), BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(prn_number, BITS_GNSS_PRN) |
    BackendId::setBits(IdType::gIonosphere, BITS_IDTYPE));
}

inline BackendId createGnssIfbId(char system,
                  int code_id, std::string prn = "")
{
  CHECK_GE(code_id, 0);
  int prn_number = 0;
  if (prn != "") {
    CHECK(prn.size() == 3);
    prn_number = atoi(prn.substr(1, 2).data());
  }
  return BackendId(
    BackendId::setBits(0, BITS_BUNDLEID) |
    BackendId::setBits(system, BITS_GNSS_SYSTEM) |
    BackendId::setBits(prn_number, BITS_GNSS_PRN) |
    BackendId::setBits(code_id, BITS_GNSS_CODEID) |
    BackendId::setBits(IdType::gIfb, BITS_IDTYPE));
}

inline BackendId changeIdType(BackendId id, IdType type, size_t cam_index = 0)
{
  CHECK(id.type() != IdType::cLandmark);
  CHECK(type != IdType::cLandmark);
  CHECK(cam_index == 0 || type == IdType::cExtrinsics || type == IdType::gVelocity ||
        type == IdType::gExtrinsics || type == IdType::gTroposphere);
  CHECK((BackendId::sensorType(id.type()) == BackendId::sensorType(type)) || 
        (BackendId::sensorType(type) == SensorType::IMU));
  uint64_t out = id.asInteger();
  out = BackendId::resetBits(out, cam_index, BITS_CAMERA_IDX);
  out = BackendId::resetBits(out, type, BITS_IDTYPE);
  if (type == IdType::ImuStates) {
    out = BackendId::resetBits(out, id.type(), BITS_IMU_SUBID);
  }
  else if (id.type() == IdType::ImuStates) {
    out = BackendId::resetBits(out, 0, BITS_IMU_SUBID);
  }
  return BackendId(out);
}

inline BackendId changeIdType(BackendId id, IdType type, const char system)
{
  CHECK(id.type() != IdType::gClock);
  CHECK(type == IdType::gClock || type == IdType::gFrequency);
  CHECK((BackendId::sensorType(id.type()) == BackendId::sensorType(type)) || 
        (BackendId::sensorType(type) == SensorType::IMU));
  uint64_t out = id.asInteger();
  out = BackendId::resetBits(out, system, BITS_GNSS_SYSTEM);
  out = BackendId::resetBits(out, type, BITS_IDTYPE);
  return BackendId(out);
}

inline bool sameAmbiguity(const BackendId& lhs, const BackendId& rhs)
{
  CHECK(BackendId::getBits(lhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gAmbiguity));
  CHECK(BackendId::getBits(rhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gAmbiguity));
  
  uint32_t sys_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_SYSTEM);
  uint32_t sys_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_SYSTEM);
  if (sys_lhs != sys_rhs) return false;

  uint32_t prn_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_PRN);
  uint32_t prn_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_PRN);
  if (prn_lhs != prn_rhs) return false;

  uint32_t phase_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_PHASEID);
  uint32_t phase_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_PHASEID);
  if (phase_lhs != phase_rhs) return false;

  return true;
}

inline bool sameIonosphere(const BackendId& lhs, const BackendId& rhs)
{
  CHECK(BackendId::getBits(lhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gIonosphere));
  CHECK(BackendId::getBits(rhs.asInteger(), BITS_IDTYPE) == 
        static_cast<uint32_t>(IdType::gIonosphere));
  
  uint32_t sys_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_SYSTEM);
  uint32_t sys_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_SYSTEM);
  if (sys_lhs != sys_rhs) return false;

  uint32_t prn_lhs = BackendId::getBits(lhs.asInteger(), BITS_GNSS_PRN);
  uint32_t prn_rhs = BackendId::getBits(rhs.asInteger(), BITS_GNSS_PRN);
  if (prn_lhs != prn_rhs) return false;

  return true;
}

// Comparison operator for use in maps.
inline bool operator<(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() < rhs.asInteger();
}

inline bool operator==(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() == rhs.asInteger();
}

inline bool operator!=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() != rhs.asInteger();
}

inline bool operator>=(const BackendId& lhs, const BackendId& rhs)
{
  return lhs.asInteger() >= rhs.asInteger();
}

inline std::ostream& operator<<(std::ostream& out, const BackendId& id)
{
  out << std::hex << id.asInteger() << std::dec;
  return out;
}

// -----------------------------------------------------------------------------
// Estimators
enum class EstimatorType {
  None,
  Spp,
  Sdgnss,
  Dgnss,
  Rtk, 
  Ppp,
  GnssImuLc,
  SppImuTc,
  DgnssImuTc, 
  RtkImuTc,
  PppImuTc, 
  GnssImuCameraSrr,
  SppImuCameraRrr,
  DgnssImuCameraRrr,
  RtkImuCameraRrr,
  PppImuCameraRrr
};

// Convert from estimator type to string
std::string estimatorTypeToString(const EstimatorType& type);

// States, that handle pose parameters at every timestamp
struct State {
  BackendId id; // pose id
  BackendId id_in_graph;  // real pose id in graph if it is a overlapped state
  double timestamp = 0.0;
  bool is_keyframe = false;
  GnssSolutionStatus status = GnssSolutionStatus::Single;
  ceres::ResidualBlockId imu_residual_to_lhs = nullptr;  // IMU residual from last state to current  
  bool valid() { return (id != BackendId(0) && timestamp != 0.0); }

  // State overlapping handle
  static std::multimap<BackendId, State> overlaps;  // from real pose id to all overlaped states
  // Erase an overlap state
  static inline void eraseOverlap(const State& state) {
    for (auto it = overlaps.lower_bound(state.id_in_graph);
      it != overlaps.upper_bound(state.id_in_graph); it++) {
      if (it->second.id == state.id) { overlaps.erase(it); break; }
    }
    if (overlaps.count(state.id_in_graph) == 1) overlaps.erase(state.id_in_graph);
  }
  // Synchronize overlap states from one modification
  static inline void syncOverlap(const State& state, std::deque<State>& states) {
    if (overlaps.count(state.id_in_graph) > 1) {
      for (auto it = overlaps.lower_bound(state.id_in_graph);
        it != overlaps.upper_bound(state.id_in_graph); it++) {
        for (auto& s : states) {
          if (s.id == it->second.id) {
            s.imu_residual_to_lhs = state.imu_residual_to_lhs;
            break;
          }
        }
        it->second.imu_residual_to_lhs = state.imu_residual_to_lhs;
      }
    }
  }
};

// Estimator status
enum class EstimatorStatus {
  Initializing, 
  Converged,
  InvalidEpoch,
  Diverged
};

// Solution generated by estimators
struct Solution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double timestamp;
  double differential_age;
  int num_satellites;
  GnssSolutionStatus status;
  Transformation pose;
  SpeedAndBias speed_and_bias;
  Eigen::Matrix<double, 15, 15> covariance;
  GeoCoordinatePtr coordinate;
};

// Role of solution when it behaves as measurement (for loosely couple)
enum class SolutionRole {
  None,
  Position,
  Attitude,
  Velocity,
  Pose,
  PoseAndVelocity,
  PositionAndVelocity
};

// All estimator data (measurements and solutions)
// Every instantiation should have only one data type
class EstimatorDataCluster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EstimatorDataCluster(const GnssMeasurement& data) : 
    gnss(std::make_shared<GnssMeasurement>(data)), 
    gnss_role(data.role), tag(data.tag), timestamp(data.timestamp) {}

  EstimatorDataCluster(
    const ImuMeasurement& data, const ImuRole& role, const std::string& tag) :
    imu(std::make_shared<ImuMeasurement>(data)),
    imu_role(role), tag(tag), timestamp(data.timestamp) {}

  EstimatorDataCluster(const std::shared_ptr<cv::Mat>& data, const CameraRole& role, 
                       const std::string& tag, const double time) :
    image(data),
    image_role(role), tag(tag), timestamp(time) {}

  EstimatorDataCluster(
    const FrameBundlePtr& data, const std::string& tag) : 
    frame_bundle(data), timestamp(data->getMinTimestampSeconds()),
    tag(tag) {}

  EstimatorDataCluster(
    const Solution& data, const SolutionRole& role, const std::string& tag) :
    solution(std::make_shared<Solution>(data)),
    solution_role(role), timestamp(data.timestamp), tag(tag) {}

  ~EstimatorDataCluster() {}

  // Raw data
  std::shared_ptr<GnssMeasurement> gnss;
  std::shared_ptr<ImuMeasurement> imu;
  std::shared_ptr<cv::Mat> image;

  // Intermediate processing data, which generated from other estimators
  // or frontend processors
  std::shared_ptr<FrameBundle> frame_bundle;
  std::shared_ptr<Solution> solution;

  // Roles
  GnssRole gnss_role;
  ImuRole imu_role;
  CameraRole image_role;
  SolutionRole solution_role;

  // Common parameters
  std::string tag;
  double timestamp;
};

} // namespace gici
