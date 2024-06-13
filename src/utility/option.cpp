/**
* @Function: Option tools
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/utility/option.h"

#include <glog/logging.h>

#include "gici/stream/streamer.h"
#include "gici/stream/formator.h"
#include "gici/stream/streaming.h"
#include "gici/gnss/gnss_types.h"
#include "gici/imu/imu_types.h"
#include "gici/vision/image_types.h"
#include "gici/gnss/spp_estimator.h"
#include "gici/gnss/ppp_estimator.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/gnss/sdgnss_estimator.h"
#include "gici/gnss/dgnss_estimator.h"
#include "gici/estimate/estimator_types.h"
#include "gici/vision/feature_handler.h"
#include "gici/gnss/code_phase_maps.h"
#include "gici/imu/imu_estimator_base.h"
#include "gici/gnss/gnss_loose_estimator_base.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/gnss_imu_initializer.h"
#include "gici/fusion/spp_imu_tc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/fusion/ppp_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_camera_srr_estimator.h"
#include "gici/fusion/spp_imu_camera_rrr_estimator.h"
#include "gici/fusion/rtk_imu_camera_rrr_estimator.h"

namespace gici {

namespace option_tools {

// Mapping from in to out
#define MAP_IN_OUT(x, y) if (in == x) { out = y; return; }
#define LOG_INVALId LOG(FATAL) << "Option " << in << " invalid!";

// Convert options from yaml type to gici type
template <typename InType, typename OutType>
void convert(const InType& in, OutType& out)
{
  LOG(FATAL) << "Convertion from " << typeid(in).name() 
         << " to " << typeid(out).name() << " not supported!";
}

template <>
void convert<std::string, StreamerType>
  (const std::string& in, StreamerType& out)
{
  MAP_IN_OUT("serial", StreamerType::Serial);
  MAP_IN_OUT("tcp-client", StreamerType::TcpClient);
  MAP_IN_OUT("tcp-server", StreamerType::TcpServer);
  MAP_IN_OUT("file", StreamerType::File);
  MAP_IN_OUT("ntrip-client", StreamerType::NtripClient);
  MAP_IN_OUT("ntrip-server", StreamerType::NtripServer);
  MAP_IN_OUT("v4l2", StreamerType::V4L2);
  MAP_IN_OUT("ros", StreamerType::Ros);
  LOG_INVALId;
}

template <>
void convert<std::string, FormatorType>
  (const std::string& in, FormatorType& out)
{
  MAP_IN_OUT("gnss-rtcm-2", FormatorType::RTCM2);
  MAP_IN_OUT("gnss-rtcm-3", FormatorType::RTCM3);
  MAP_IN_OUT("gnss-raw", FormatorType::GnssRaw);
  MAP_IN_OUT("gnss-rinex", FormatorType::RINEX);
  MAP_IN_OUT("image-v4l2", FormatorType::ImageV4L2);
  MAP_IN_OUT("image-pack", FormatorType::ImagePack);
  MAP_IN_OUT("imu-pack", FormatorType::IMUPack);
  MAP_IN_OUT("option", FormatorType::OptionPack);
  MAP_IN_OUT("nmea", FormatorType::NMEA);
  MAP_IN_OUT("dcb-file", FormatorType::DcbFile);
  MAP_IN_OUT("atx-file", FormatorType::AtxFile);
  LOG_INVALId;
}

template <>
void convert<std::string, GnssRawFormats>
  (const std::string& in, GnssRawFormats& out)
{
  MAP_IN_OUT("ublox", GnssRawFormats::Ublox);
  MAP_IN_OUT("septentrio", GnssRawFormats::Septentrio);
  MAP_IN_OUT("novatel", GnssRawFormats::Novatel);
  MAP_IN_OUT("tersus", GnssRawFormats::Tersus);
  LOG_INVALId;
}

template <>
void convert<std::string, StreamIOType>
  (const std::string& in, StreamIOType& out)
{
  MAP_IN_OUT("input", StreamIOType::Input);
  MAP_IN_OUT("output", StreamIOType::Output);
  MAP_IN_OUT("log", StreamIOType::Log);
  LOG_INVALId;
}

template <>
void convert<std::string, GnssRole>
  (const std::string& in, GnssRole& out)
{
  MAP_IN_OUT("rover", GnssRole::Rover);
  MAP_IN_OUT("reference", GnssRole::Reference);
  MAP_IN_OUT("ephemeris", GnssRole::Ephemeris);
  MAP_IN_OUT("ssr_ephemeris", GnssRole::SsrEphemeris);
  MAP_IN_OUT("ion_utc", GnssRole::IonAndUtc);
  MAP_IN_OUT("code_bias", GnssRole::CodeBias);
  MAP_IN_OUT("phase_bias", GnssRole::PhaseBias);
  MAP_IN_OUT("heading", GnssRole::Heading);
  MAP_IN_OUT("phase_center", GnssRole::PhaseCenter);
  LOG_INVALId;
}

template <>
void convert<std::string, ImuRole>
  (const std::string& in, ImuRole& out)
{
  MAP_IN_OUT("major", ImuRole::Major);
  MAP_IN_OUT("minor", ImuRole::Minor);
  LOG_INVALId;
}

template <>
void convert<std::string, CameraRole>
  (const std::string& in, CameraRole& out)
{
  MAP_IN_OUT("mono", CameraRole::Mono);
  MAP_IN_OUT("stereo_major", CameraRole::StereoMajor);
  MAP_IN_OUT("stereo_minor", CameraRole::StereoMinor);
  MAP_IN_OUT("array", CameraRole::Array);
  LOG_INVALId;
}

template <>
void convert<std::string, EstimatorType>
  (const std::string& in, EstimatorType& out)
{
  MAP_IN_OUT("spp", EstimatorType::Spp);
  MAP_IN_OUT("sdgnss", EstimatorType::Sdgnss);
  MAP_IN_OUT("dgnss", EstimatorType::Dgnss);
  MAP_IN_OUT("rtk", EstimatorType::Rtk);
  MAP_IN_OUT("ppp", EstimatorType::Ppp);
  MAP_IN_OUT("gnss_imu_lc", EstimatorType::GnssImuLc);
  MAP_IN_OUT("spp_imu_tc", EstimatorType::SppImuTc);
  MAP_IN_OUT("dgnss_imu_tc", EstimatorType::DgnssImuTc);
  MAP_IN_OUT("rtk_imu_tc", EstimatorType::RtkImuTc);
  MAP_IN_OUT("ppp_imu_tc", EstimatorType::PppImuTc);
  MAP_IN_OUT("gnss_imu_camera_srr", EstimatorType::GnssImuCameraSrr);
  MAP_IN_OUT("spp_imu_camera_rrr", EstimatorType::SppImuCameraRrr);
  MAP_IN_OUT("dgnss_imu_camera_rrr", EstimatorType::DgnssImuCameraRrr);
  MAP_IN_OUT("rtk_imu_camera_rrr", EstimatorType::RtkImuCameraRrr);
  MAP_IN_OUT("Ppp_imu_camera_rrr", EstimatorType::PppImuCameraRrr);
  LOG_INVALId;
}

template <>
void convert<std::string, SolutionRole>
  (const std::string& in, SolutionRole& out)
{
  MAP_IN_OUT("position", SolutionRole::Position);
  MAP_IN_OUT("attitude", SolutionRole::Attitude);
  MAP_IN_OUT("velocity", SolutionRole::Velocity);
  MAP_IN_OUT("pose", SolutionRole::Pose);
  MAP_IN_OUT("pose_and_velocity", SolutionRole::PoseAndVelocity);
  MAP_IN_OUT("position_and_velocity", SolutionRole::PositionAndVelocity);
  LOG_INVALId;
}

template <>
void convert<std::string, DetectorType>
  (const std::string& in, DetectorType& out)
{
  MAP_IN_OUT("fast", DetectorType::kFast);
  MAP_IN_OUT("grad", DetectorType::kGrad);
  MAP_IN_OUT("fast_and_grad", DetectorType::kFastGrad);
  MAP_IN_OUT("shitomasi", DetectorType::kShiTomasi);
  MAP_IN_OUT("shitomasi_and_grad", DetectorType::kShiTomasiGrad);
  MAP_IN_OUT("grid_grad", DetectorType::kGridGrad);
  MAP_IN_OUT("all", DetectorType::kAll);
  MAP_IN_OUT("mumford_grad", DetectorType::kGradHuangMumford);
  MAP_IN_OUT("canny", DetectorType::kCanny);
  MAP_IN_OUT("sobel", DetectorType::kSobel);
  LOG_INVALId;
}

template <>
void convert<std::string, VisualInitializerType>
  (const std::string& in, VisualInitializerType& out)
{
  MAP_IN_OUT("homography", VisualInitializerType::kHomography);
  MAP_IN_OUT("fundamental", VisualInitializerType::kFundamental);
  LOG_INVALId;
}

template <>
void convert<std::string, ceres::LinearSolverType>
  (const std::string& in, ceres::LinearSolverType& out)
{
  MAP_IN_OUT("dense_normal_cholesky", ceres::DENSE_NORMAL_CHOLESKY);
  MAP_IN_OUT("dense_qr", ceres::DENSE_QR);
  MAP_IN_OUT("sparse_normal_cholesky", ceres::SPARSE_NORMAL_CHOLESKY);
  MAP_IN_OUT("dense_schur", ceres::DENSE_SCHUR);
  MAP_IN_OUT("sparse_schur", ceres::SPARSE_SCHUR);
  MAP_IN_OUT("iterative_schur", ceres::ITERATIVE_SCHUR);
  MAP_IN_OUT("cgnr", ceres::CGNR);
  LOG_INVALId;
}

template <>
void convert<std::string, ceres::TrustRegionStrategyType>
  (const std::string& in, ceres::TrustRegionStrategyType& out)
{
  MAP_IN_OUT("levenberg_marquardt", ceres::LEVENBERG_MARQUARDT);
  MAP_IN_OUT("dogleg", ceres::DOGLEG);
  LOG_INVALId;
}

// Mapping from in to return
#define MAP_IN_RET(x, y) if (in == x) { return y; }

// Get sensor type from options
SensorType sensorType(std::string in)
{
  // From formator role
  MAP_IN_RET("rover", SensorType::GNSS);
  MAP_IN_RET("reference", SensorType::GNSS);
  MAP_IN_RET("ephemeris", SensorType::GNSS);
  MAP_IN_RET("ion_utc", SensorType::GNSS);
  MAP_IN_RET("heading", SensorType::GNSS);
  MAP_IN_RET("code_bias", SensorType::GNSS);
  MAP_IN_RET("phase_bias", SensorType::GNSS);
  MAP_IN_RET("ssr_ephemeris", SensorType::GNSS);
  MAP_IN_RET("phase_center", SensorType::GNSS);
  MAP_IN_RET("major", SensorType::IMU);
  MAP_IN_RET("major", SensorType::IMU);
  MAP_IN_RET("mono", SensorType::Camera);
  MAP_IN_RET("stereo-major", SensorType::Camera);
  MAP_IN_RET("stereo-minor", SensorType::Camera);
  MAP_IN_RET("array", SensorType::Camera);
  MAP_IN_RET("position", SensorType::GeneralSolution);
  MAP_IN_RET("attitude", SensorType::GeneralSolution);
  MAP_IN_RET("velocity", SensorType::GeneralSolution);
  MAP_IN_RET("pose", SensorType::GeneralSolution);
  MAP_IN_RET("pose_and_velocity", SensorType::GeneralSolution);
  MAP_IN_RET("position_and_velocity", SensorType::GeneralSolution);
  MAP_IN_RET("option", SensorType::Option);

  // From formator type
  MAP_IN_RET("gnss-rtcm-2", SensorType::GNSS);
  MAP_IN_RET("gnss-rtcm-3", SensorType::GNSS);
  MAP_IN_RET("gnss-raw", SensorType::GNSS);
  MAP_IN_RET("image-v4l2", SensorType::Camera);
  MAP_IN_RET("image-pack", SensorType::Camera);
  MAP_IN_RET("imu-pack", SensorType::IMU);
  MAP_IN_RET("option", SensorType::Option);

  LOG_INVALId;
  return SensorType::None;
}

// Load option with info
#define LOAD_COMMON(opt) \
  if (!option_tools::safeGet(node, #opt, &options.opt)) { \
  LOG(INFO) << __FUNCTION__ << ": Unable to load " << #opt \
         << ". Using default instead."; }
// Load option with fatal error
#define LOAD_REQUIRED(opt) \
  if (!option_tools::safeGet(node, #opt, &options.opt)) { \
  LOG(FATAL) << __FUNCTION__ << ": Unable to load " << #opt << "!"; }

// Check sub-options exist
inline bool checkSubOption(
  YAML::Node& node, std::string subname, bool fatal = false)
{
  if (!node[subname].IsDefined()) {
    if (fatal) {
      LOG(FATAL) << "Unable to load " << subname << "!";
    }
    else {
      LOG(INFO) << "Unable to load " << subname << ". Using default instead.";
    }
    return false;
  }
  return true;
}

// split line by pattern
inline std::vector<std::string> split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  int size = str.size();
  for (int i = 0; i < size; i++) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

// delete space
inline void delete_space(std::string& strs)
{
  strs.erase(0, strs.find_first_not_of(' '));
  strs.erase(strs.find_last_not_of(' ') + 1, 
    strs.size() - strs.find_last_not_of(' ') - 1);
}

// delete space for each string in vector
inline void delete_spaces(std::vector<std::string>& strs)
{
  for (size_t i = 0; i < strs.size(); i++) {
    delete_space(strs[i]);
  }
}

// Load options
template <typename OptionType>
void loadOptions(YAML::Node& node, OptionType& options)
{
  LOG(FATAL) << "Loading " << typeid(options).name() << " not supported!";
}

template <>
void loadOptions<GnssCommonOptions>(
    YAML::Node& node, GnssCommonOptions& options)
{
  LOAD_COMMON(min_elevation);
  LOAD_COMMON(min_num_satellite_redundancy);
  LOAD_COMMON(max_gdop);
  LOAD_COMMON(mw_slip_thres);
  LOAD_COMMON(gf_slip_thres);
  LOAD_COMMON(gf_sd_slip_thres);
  LOAD_COMMON(period);

  std::vector<std::string> system_excludes;
  if (option_tools::safeGet(node, "system_exclude", &system_excludes)) {
    options.system_exclude.clear();
    for (auto system_exclude : system_excludes) {
      options.system_exclude.push_back(system_exclude[0]);
    }
  }

  std::vector<std::string> satellite_excludes;
  if (option_tools::safeGet(node, "satellite_exclude", &satellite_excludes)) {
    options.satellite_exclude.clear();
    for (auto satellite_exclude : satellite_excludes) {
      options.satellite_exclude.push_back(satellite_exclude);
    }
  }

  std::vector<std::string> code_excludes;
  if (option_tools::safeGet(node, "code_exclude", &code_excludes)) {
    options.code_exclude.clear();
    for (auto code_exclude : code_excludes) {
      char system = code_exclude[0];
      std::string code_str = code_exclude.substr(2, 2);
      int code = gnss_common::rinexTypeToCodeType(system, code_str);
      options.code_exclude.push_back(std::make_pair(system, code));
    }
  }

  std::vector<double> min_SNR;
  if (option_tools::safeGet(node, "min_SNR", 
      &min_SNR) && 
      min_SNR.size() == 2) {
    for (size_t i = 0; i < 2; i++) {
      options.min_SNR[i] = min_SNR[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load min_SNR. Using default instead.";
  }

  std::vector<double> receiver_pco;
  if (option_tools::safeGet(node, "receiver_pco", 
      &receiver_pco) && 
      receiver_pco.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.receiver_pco[i] = receiver_pco[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load receiver_pco. Using default instead.";
  }
}

template <>
void loadOptions<GnssErrorParameter>(
    YAML::Node& node, GnssErrorParameter& options)
{
  LOAD_COMMON(code_to_phase_ratio);
  LOAD_COMMON(phase_error_factor);
  LOAD_COMMON(doppler_error_factor);
  LOAD_COMMON(ionosphere_broadcast_factor);
  LOAD_COMMON(ionosphere_dual_frequency);
  LOAD_COMMON(ionosphere_augment);
  LOAD_COMMON(troposphere_model_factor);
  LOAD_COMMON(troposphere_augment);
  LOAD_COMMON(ephemeris_broadcast);
  LOAD_COMMON(ephemeris_precise);
  LOAD_COMMON(initial_position);
  LOAD_COMMON(initial_velocity);
  LOAD_COMMON(initial_clock);
  LOAD_COMMON(initial_troposphere);
  LOAD_COMMON(initial_ionosphere);
  LOAD_COMMON(initial_ambiguity);
  LOAD_COMMON(relative_troposphere);
  LOAD_COMMON(relative_ionosphere);
  LOAD_COMMON(relative_ambiguity);
  LOAD_COMMON(relative_gps_ifcb);
  LOAD_COMMON(relative_frequency);
  LOAD_COMMON(residual_gps_ifcb);

  std::vector<double> system_error_ratio;
  if (option_tools::safeGet(node, "system_error_ratio", &system_error_ratio)) {
    options.system_error_ratio.at('G') = system_error_ratio[0];
    options.system_error_ratio.at('R') = system_error_ratio[1];
    options.system_error_ratio.at('E') = system_error_ratio[2];
    options.system_error_ratio.at('C') = system_error_ratio[3];
  }
  else {
    LOG(INFO) << "Unable to load system_error_ratio. Using default instead.";
  }

  std::vector<double> relative_position;
  if (option_tools::safeGet(node, "relative_position", 
      &relative_position) && 
      relative_position.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.relative_position[i] = relative_position[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load relative_position. Using default instead.";
  }

  std::vector<double> relative_velocity;
  if (option_tools::safeGet(node, "relative_velocity", 
      &relative_velocity) && 
      relative_velocity.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.relative_velocity[i] = relative_velocity[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load relative_velocity. Using default instead.";
  }
}

template <>
void loadOptions<AmbiguityResolutionOptions>(
    YAML::Node& node, AmbiguityResolutionOptions& options)
{
  LOAD_COMMON(min_elevation);
  LOAD_COMMON(min_percentage_fixation_nl);
  LOAD_COMMON(min_percentage_fixation_wl);
  LOAD_COMMON(min_percentage_fixation_uwl);
  LOAD_COMMON(ratio);

  std::vector<std::string> system_excludes;
  bool has_glonass = false;
  if (option_tools::safeGet(node, "system_exclude", &system_excludes)) {
    options.system_exclude.clear();
    for (auto system_exclude : system_excludes) {
      options.system_exclude.push_back(system_exclude[0]);
      if (system_exclude[0] == 'R') has_glonass = true;
    }
  }
  if (!has_glonass) {
    LOG(WARNING) << "Currently we do not support GLONASS ambiguity resolution!";
    options.system_exclude.push_back('R');
  }

  std::vector<std::string> satellite_excludes;
  if (option_tools::safeGet(node, "satellite_exclude", &satellite_excludes)) {
    options.satellite_exclude.clear();
    for (auto satellite_exclude : satellite_excludes) {
      options.satellite_exclude.push_back(satellite_exclude);
    }
  }

  std::vector<std::string> phase_excludes;
  if (option_tools::safeGet(node, "phase_exclude", &phase_excludes)) {
    options.phase_exclude.clear();
    for (auto phase_exclude : phase_excludes) {
      char system = phase_exclude[0];
      std::string phase_str = phase_exclude.substr(2, phase_exclude.size() - 2);
      int phase = PHASE_NONE;
#define MAP(S, P, PS) \
  if (system == S && phase_str == PS) { phase = P; }
  PHASE_CHANNEL_TO_STR_MAPS;
#undef MAP
      options.phase_exclude.push_back(std::make_pair(system, phase));
    }
  }
}

template <>
void loadOptions<ImuParameters>(
    YAML::Node& node, ImuParameters& options)
{
  LOAD_COMMON(a_max);
  LOAD_COMMON(g_max);
  LOAD_COMMON(sigma_g_c);
  LOAD_COMMON(sigma_bg);
  LOAD_COMMON(sigma_a_c);
  LOAD_COMMON(sigma_ba);
  LOAD_COMMON(sigma_gw_c);
  LOAD_COMMON(sigma_aw_c);
  LOAD_COMMON(rate);
  LOAD_COMMON(delay_imu_cam);
}

template <>
void loadOptions<DetectorOptions>(
    YAML::Node& node, DetectorOptions& options)
{
  LOAD_COMMON(cell_size);
  LOAD_COMMON(max_level);
  LOAD_COMMON(min_level);
  LOAD_COMMON(border);
  LOAD_COMMON(threshold_primary);
  LOAD_COMMON(sampling_level)
  LOAD_COMMON(level);
  LOAD_COMMON(sec_grid_fineness);
  LOAD_COMMON(threshold_shitomasi);

  std::string detector_type;
  if (option_tools::safeGet(node, "detector_type", &detector_type)) {
    delete_space(detector_type);
    convert(detector_type, options.detector_type);
  }
}

template <>
void loadOptions<FeatureTrackerOptions>(
    YAML::Node& node, FeatureTrackerOptions& options)
{
  LOAD_COMMON(window_size);
  LOAD_COMMON(max_level);
  LOAD_COMMON(max_count);
  LOAD_COMMON(epsilon);
  LOAD_COMMON(use_relative_rotation);
  LOAD_COMMON(ransac_threshold)
  LOAD_COMMON(ransac_confidence);
}

template <>
void loadOptions<VisualInitializationOptions>(
    YAML::Node& node, VisualInitializationOptions& options)
{
  LOAD_COMMON(init_min_disparity);
  LOAD_COMMON(init_disparity_pivot_ratio);
  LOAD_COMMON(init_min_features);
  LOAD_COMMON(init_min_inliers);
  LOAD_COMMON(init_map_scale);
  LOAD_COMMON(reproj_error_thresh)

  std::string init_type;
  if (option_tools::safeGet(node, "init_type", &init_type)) {
    delete_space(init_type);
    convert(init_type, options.init_type);
  }
}

template <>
void loadOptions<FeatureHandlerOptions>(
    YAML::Node& node, FeatureHandlerOptions& options)
{
  LOAD_COMMON(max_n_kfs);
  LOAD_COMMON(max_features_per_frame);
  LOAD_COMMON(kfselect_min_numkfs);
  LOAD_COMMON(kfselect_min_disparity);
  LOAD_COMMON(kfselect_min_dist_metric);
  LOAD_COMMON(kfselect_min_angle);
  LOAD_COMMON(kfselect_min_dt);
  LOAD_COMMON(max_pyramid_level);
  LOAD_COMMON(min_disparity_init_landmark);
  LOAD_COMMON(min_translation_init_landmark);
  LOAD_COMMON(min_parallax_angle_init_landmark);

  if (checkSubOption(node, "detector")) {
    YAML::Node subnode = node["detector"];
    loadOptions(subnode, options.detector);
  }

  if (checkSubOption(node, "tracker")) {
    YAML::Node subnode = node["tracker"];
    loadOptions(subnode, options.tracker);
  }

  if (checkSubOption(node, "initialization")) {
    YAML::Node subnode = node["initialization"];
    loadOptions(subnode, options.initialization);
  }

  if (checkSubOption(node, "camera_parameters")) {
    YAML::Node subnode = node["camera_parameters"];
    options.cameras = CameraBundle::loadFromYaml(subnode);
  }
}

template <>
void loadOptions<EstimatorBaseOptions>(
    YAML::Node& node, EstimatorBaseOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(max_solver_time);
  LOAD_COMMON(verbose_output);
  LOAD_COMMON(force_initial_global_position);
  LOAD_COMMON(log_intermediate_data);
  LOAD_COMMON(log_intermediate_data_directory);

  std::string solver_type;
  if (option_tools::safeGet(node, "solver_type", &solver_type)) {
    delete_space(solver_type);
    convert(solver_type, options.solver_type);
  }

  std::string trust_region_strategy_type;
  if (option_tools::safeGet(
      node, "trust_region_strategy_type", &trust_region_strategy_type)) {
    delete_space(trust_region_strategy_type);
    convert(trust_region_strategy_type, options.trust_region_strategy_type);
  }

  if (options.trust_region_strategy_type == ceres::DOGLEG) {
    if (options.solver_type != ceres::SPARSE_SCHUR && 
        options.solver_type != ceres::DENSE_SCHUR &&
        options.solver_type != ceres::DENSE_QR && 
        options.solver_type != ceres::SPARSE_NORMAL_CHOLESKY) {
      LOG(FATAL) << "Ceres solver do not support " << solver_type
                 << " for " << trust_region_strategy_type;
    }
  }

  std::vector<double> initial_global_position;
  if (option_tools::safeGet(node, "initial_global_position", 
      &initial_global_position) && 
      initial_global_position.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.initial_global_position[i] = initial_global_position[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load initial_global_position. Using default instead.";
  }
}

template <>
void loadOptions<GnssEstimatorBaseOptions>(
    YAML::Node& node, GnssEstimatorBaseOptions& options)
{
  LOAD_COMMON(use_outlier_rejection);
  LOAD_COMMON(reject_one_outlier_once);
  LOAD_COMMON(max_pesudorange_error);
  LOAD_COMMON(max_phaserange_error);
  LOAD_COMMON(max_doppler_error);
  LOAD_COMMON(good_observation_min_num_satellites);
  LOAD_COMMON(good_observation_max_gdop);
  LOAD_COMMON(good_observation_max_reject_ratio);
  LOAD_COMMON(reset_ambiguity_min_num_continuous_unfix);
  LOAD_COMMON(diverge_max_reject_ratio);
  LOAD_COMMON(diverge_min_num_continuous_reject);

  if (checkSubOption(node, "gnss_common")) {
    YAML::Node subnode = node["gnss_common"];
    loadOptions(subnode, options.common);
  }

  if (checkSubOption(node, "gnss_error_parameter")) {
    YAML::Node subnode = node["gnss_error_parameter"];
    loadOptions(subnode, options.error_parameter);
  }
}

template <>
void loadOptions<GnssLooseEstimatorBaseOptions>(
    YAML::Node& node, GnssLooseEstimatorBaseOptions& options)
{
  LOAD_COMMON(use_outlier_rejection);
  LOAD_COMMON(max_position_error);
  LOAD_COMMON(max_velocity_error);
  LOAD_COMMON(diverge_min_num_continuous_reject);
}

template <>
void loadOptions<ImuEstimatorBaseOptions>(
    YAML::Node& node, ImuEstimatorBaseOptions& options)
{
  LOAD_COMMON(car_motion);
  LOAD_COMMON(body_to_imu_rotation_std);
  LOAD_COMMON(use_zupt);
  LOAD_COMMON(zupt_duration);
  LOAD_COMMON(zupt_max_acc_std);
  LOAD_COMMON(zupt_max_gyro_std);
  LOAD_COMMON(zupt_max_gyro_median);
  LOAD_COMMON(zupt_sigma_zero_velocity);
  LOAD_COMMON(car_motion_min_velocity);
  LOAD_COMMON(car_motion_max_anguler_velocity);

  if (checkSubOption(node, "imu_parameters")) {
    YAML::Node subnode = node["imu_parameters"];
    loadOptions(subnode, options.imu_parameters);
  }

  std::vector<double> body_to_imu_rotation;
  if (option_tools::safeGet(node, "body_to_imu_rotation", 
      &body_to_imu_rotation) && 
      body_to_imu_rotation.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.body_to_imu_rotation[i] = body_to_imu_rotation[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load body_to_imu_rotation. Using default instead.";
  }
}

template <>
void loadOptions<SppEstimatorOptions>(
    YAML::Node& node, SppEstimatorOptions& options)
{
  LOAD_COMMON(estimate_velocity);
  LOAD_COMMON(use_dual_frequency);
}

template <>
void loadOptions<PppEstimatorOptions>(
    YAML::Node& node, PppEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
  LOAD_COMMON(use_ambiguity_resolution);
  LOAD_COMMON(estimate_velocity);
}

template <>
void loadOptions<RtkEstimatorOptions>(
    YAML::Node& node, RtkEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
  LOAD_COMMON(use_ambiguity_resolution);
  LOAD_COMMON(estimate_velocity);
  LOAD_COMMON(max_age);
}

template <>
void loadOptions<SdgnssEstimatorOptions>(
    YAML::Node& node, SdgnssEstimatorOptions& options)
{
  LOAD_COMMON(estimate_velocity);
  LOAD_COMMON(max_age);
}

template <>
void loadOptions<DgnssEstimatorOptions>(
    YAML::Node& node, DgnssEstimatorOptions& options)
{
  LOAD_COMMON(estimate_velocity);
  LOAD_COMMON(max_age);
}

template <>
void loadOptions<GnssImuLcEstimatorOptions>(
    YAML::Node& node, GnssImuLcEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
}

template <>
void loadOptions<SppImuTcEstimatorOptions>(
    YAML::Node& node, SppImuTcEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
}

template <>
void loadOptions<RtkImuTcEstimatorOptions>(
    YAML::Node& node, RtkImuTcEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
}

template <>
void loadOptions<PppImuTcEstimatorOptions>(
    YAML::Node& node, PppImuTcEstimatorOptions& options)
{
  LOAD_COMMON(max_window_length);
}

template <>
void loadOptions<GnssImuInitializerOptions>(
    YAML::Node& node, GnssImuInitializerOptions& options)
{
  LOAD_COMMON(max_iteration);
  LOAD_COMMON(num_threads);
  LOAD_COMMON(max_solver_time);
  LOAD_COMMON(time_window_length_slow_motion);
  LOAD_COMMON(time_window_length_dynamic_motion);
  LOAD_COMMON(min_acceleration);

  std::vector<double> gnss_extrinsics;
  if (option_tools::safeGet(node, "gnss_extrinsics", 
      &gnss_extrinsics) && 
      gnss_extrinsics.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.gnss_extrinsics[i] = gnss_extrinsics[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load gnss_extrinsics. Using default instead.";
  }

  std::vector<double> gnss_extrinsics_initial_std;
  if (option_tools::safeGet(node, "gnss_extrinsics_initial_std", 
      &gnss_extrinsics_initial_std) && 
      gnss_extrinsics_initial_std.size() == 3) {
    for (size_t i = 0; i < 3; i++) {
      options.gnss_extrinsics_initial_std[i] = gnss_extrinsics_initial_std[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load gnss_extrinsics_initial_std. Using default instead.";
  } 
}

template <>
void loadOptions<GnssImuCameraSrrEstimatorOptions>(
    YAML::Node& node, GnssImuCameraSrrEstimatorOptions& options)
{
  LOAD_COMMON(max_keyframes);
  LOAD_COMMON(max_gnss_window_length_minor);
  LOAD_COMMON(min_yaw_std_init_visual);
}

template <>
void loadOptions<SppImuCameraRrrEstimatorOptions>(
    YAML::Node& node, SppImuCameraRrrEstimatorOptions& options)
{
  LOAD_COMMON(max_keyframes);
  LOAD_COMMON(max_gnss_window_length_minor);
  LOAD_COMMON(min_yaw_std_init_visual);
}

template <>
void loadOptions<RtkImuCameraRrrEstimatorOptions>(
    YAML::Node& node, RtkImuCameraRrrEstimatorOptions& options)
{
  LOAD_COMMON(max_keyframes);
  LOAD_COMMON(max_gnss_window_length_minor);
  LOAD_COMMON(min_yaw_std_init_visual);
}

template <>
void loadOptions<VisualEstimatorBaseOptions>(
    YAML::Node& node, VisualEstimatorBaseOptions& options)
{
  LOAD_COMMON(feature_error_std);
  LOAD_COMMON(stable_feature_error_std);
  LOAD_COMMON(min_observation_stable);
  LOAD_COMMON(landmark_outlier_rejection_threshold);
  LOAD_COMMON(max_frequency);
  LOAD_COMMON(diverge_max_reject_ratio);
  LOAD_COMMON(diverge_min_num_continuous_reject);

  std::vector<double> camera_extrinsics_initial_std;
  if (option_tools::safeGet(node, "camera_extrinsics_initial_std", 
      &camera_extrinsics_initial_std) && 
      camera_extrinsics_initial_std.size() == 6) {
    for (size_t i = 0; i < 6; i++) {
      options.camera_extrinsics_initial_std[i] = camera_extrinsics_initial_std[i];
    }
  }
  else {
    LOG(INFO) << "Unable to load camera_extrinsics_initial_std. Using default instead.";
  } 
}

// Copy the options with the same name from in to out
#define COPY(opt) out.opt = in.opt

// Copy options for similar estimators
template <typename OptionTypeIn, typename OptionTypeOut>
void copyOptions(const OptionTypeIn& in, OptionTypeOut& out)
{
  LOG(FATAL) << "Copying from " << typeid(in).name() << " to " 
             << typeid(out).name() << " is not supported!";
}

template <>
void copyOptions(const PppEstimatorOptions& in, SppEstimatorOptions& out)
{
  COPY(estimate_velocity);
}

}

}