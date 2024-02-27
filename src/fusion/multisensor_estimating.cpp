/**
* @Function: Multisensor estimation thread handle
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/fusion/multisensor_estimating.h"

#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"
#include "gici/utility/spin_control.h"
#include "gici/gnss/ppp_estimator.h"
#include "gici/gnss/sdgnss_estimator.h"
#include "gici/gnss/dgnss_estimator.h"
#include "gici/gnss/rtk_estimator.h"
#include "gici/fusion/gnss_imu_lc_estimator.h"
#include "gici/fusion/rtk_imu_tc_estimator.h"
#include "gici/fusion/gnss_imu_camera_srr_estimator.h"
#include "gici/fusion/rtk_imu_camera_rrr_estimator.h"

namespace gici {

MultiSensorEstimating::MultiSensorEstimating(
  const NodeOptionHandlePtr& nodes, size_t i_estimator) : 
  EstimatingBase(nodes, i_estimator), latest_imu_timestamp_(0.0)
{
  // load base options
  const YAML::Node& node = nodes->estimators[i_estimator]->this_node;
  YAML::Node estimator_base_node = node["estimator_base_options"];
  if (estimator_base_node.IsDefined()) {
    option_tools::loadOptions(estimator_base_node, base_options_);
  }

  force_initial_global_position_ = base_options_.force_initial_global_position;
  initial_global_position_ = base_options_.initial_global_position;
  base_options_.compute_covariance = compute_covariance_;

  // load GNSS base options
  YAML::Node gnss_base_node = node["gnss_estimator_base_options"];
  YAML::Node gnss_loose_base_node = node["gnss_loose_estimator_base_options"];
  if (estimatorTypeContains(SensorType::GNSS, type_) && 
      gnss_base_node.IsDefined()) {
    option_tools::loadOptions(gnss_base_node, gnss_base_options_);
  }
  if (estimatorTypeContains(SensorType::GNSS, type_) && 
      gnss_loose_base_node.IsDefined()) {
     option_tools::loadOptions(gnss_loose_base_node, gnss_loose_base_options_);
  }

  // load IMU base options
  YAML::Node imu_base_node = node["imu_estimator_base_options"];
  if (estimatorTypeContains(SensorType::IMU, type_) && 
      imu_base_node.IsDefined()) {
    option_tools::loadOptions(imu_base_node, imu_base_options_);
  }

  // load camera base options
  YAML::Node visual_estimator_base_node = node["visual_estimator_base_options"];
  YAML::Node feature_handler_node = node["feature_handler_options"];
  if (estimatorTypeContains(SensorType::Camera, type_) && 
      visual_estimator_base_node.IsDefined()) {
    option_tools::loadOptions(visual_estimator_base_node, visual_estimator_base_options_);
  }
  if (estimatorTypeContains(SensorType::Camera, type_) && 
      feature_handler_node.IsDefined()) {
    option_tools::loadOptions(feature_handler_node, feature_handler_options_);
  }

  // Instantiate estimators
  // single point positioning
  if (type_ == EstimatorType::Spp) 
  {
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    estimator_.reset(new SppEstimator(
      spp_options_, gnss_base_options_, base_options_));
  }
  // single-differenced GNSS pseudorange positioning
  else if (type_ == EstimatorType::Sdgnss) 
  {
    YAML::Node sdgnss_node = node["sdgnss_options"];
    if (sdgnss_node.IsDefined()) {
      option_tools::loadOptions(sdgnss_node, sdgnss_options_);
    }
    estimator_.reset(new SdgnssEstimator(
      sdgnss_options_, gnss_base_options_, base_options_));
  }
  // double-differenced GNSS pseudorange positioning (differential GNSS)
  else if (type_ == EstimatorType::Dgnss) 
  {
    YAML::Node dgnss_node = node["dgnss_options"];
    if (dgnss_node.IsDefined()) {
      option_tools::loadOptions(dgnss_node, dgnss_options_);
    }
    estimator_.reset(new DgnssEstimator(
      dgnss_options_, gnss_base_options_, base_options_));
  }
  // real-time kinematic
  else if (type_ == EstimatorType::Rtk) 
  {
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }
    estimator_.reset(new RtkEstimator(
      rtk_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // precise point positioning
  else if (type_ == EstimatorType::Ppp) 
  {
    YAML::Node ppp_node = node["ppp_options"];
    if (ppp_node.IsDefined()) {
      option_tools::loadOptions(ppp_node, ppp_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }
    estimator_.reset(new PppEstimator(
      ppp_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU loosely couple
  else if (type_ == EstimatorType::GnssImuLc)
  {
    YAML::Node gnss_imu_lc_node = node["gnss_imu_lc_options"];
    if (gnss_imu_lc_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_lc_node, gnss_imu_lc_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new GnssImuLcEstimator(
      gnss_imu_lc_options_, gnss_imu_init_options_, gnss_loose_base_options_, 
      imu_base_options_, base_options_));
  }
  // SPP/IMU tightly couple
  else if (type_ == EstimatorType::SppImuTc)
  {
    YAML::Node spp_imu_tc_node = node["spp_imu_tc_options"];
    if (spp_imu_tc_node.IsDefined()) {
      option_tools::loadOptions(spp_imu_tc_node, spp_imu_tc_options_);
    }
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new SppImuTcEstimator(
      spp_imu_tc_options_, gnss_imu_init_options_, spp_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_));
  }
  // RTK/IMU tightly couple
  else if (type_ == EstimatorType::RtkImuTc)
  {
    YAML::Node rtk_imu_tc_node = node["rtk_imu_tc_options"];
    if (rtk_imu_tc_node.IsDefined()) {
      option_tools::loadOptions(rtk_imu_tc_node, rtk_imu_tc_options_);
    }
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new RtkImuTcEstimator(
      rtk_imu_tc_options_, gnss_imu_init_options_, rtk_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_, ambiguity_options_));
  }
  // PPP/IMU tightly couple
  else if (type_ == EstimatorType::PppImuTc)
  {
    YAML::Node ppp_imu_tc_node = node["ppp_imu_tc_options"];
    if (ppp_imu_tc_node.IsDefined()) {
      option_tools::loadOptions(ppp_imu_tc_node, ppp_imu_tc_options_);
    }
    YAML::Node ppp_node = node["ppp_options"];
    if (ppp_node.IsDefined()) {
      option_tools::loadOptions(ppp_node, ppp_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);

    estimator_.reset(new PppImuTcEstimator(
      ppp_imu_tc_options_, gnss_imu_init_options_, ppp_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU/Camera semi-tightly integration
  else if (type_ == EstimatorType::GnssImuCameraSrr)
  {
    YAML::Node gnss_imu_camera_srr_node = node["gnss_imu_camera_srr_options"];
    if (gnss_imu_camera_srr_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_camera_srr_node, gnss_imu_camera_srr_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new GnssImuCameraSrrEstimator(gnss_imu_camera_srr_options_,
      gnss_imu_init_options_, gnss_loose_base_options_, visual_estimator_base_options_, 
      imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // SPP/IMU/Camera tightly integration
  else if (type_ == EstimatorType::SppImuCameraRrr)
  {
    YAML::Node spp_imu_camera_rrr_node = node["spp_imu_camera_rrr_options"];
    if (spp_imu_camera_rrr_node.IsDefined()) {
      option_tools::loadOptions(spp_imu_camera_rrr_node, spp_imu_camera_rrr_options_);
    }
    YAML::Node spp_node = node["spp_options"];
    if (spp_node.IsDefined()) {
      option_tools::loadOptions(spp_node, spp_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new SppImuCameraRrrEstimator(spp_imu_camera_rrr_options_, 
      gnss_imu_init_options_, spp_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // RTK/IMU/Camera tightly integration
  else if (type_ == EstimatorType::RtkImuCameraRrr)
  {
    YAML::Node rtk_imu_camera_rrr_node = node["rtk_imu_camera_rrr_options"];
    if (rtk_imu_camera_rrr_node.IsDefined()) {
      option_tools::loadOptions(rtk_imu_camera_rrr_node, rtk_imu_camera_rrr_options_);
    }
    YAML::Node rtk_node = node["rtk_options"];
    if (rtk_node.IsDefined()) {
      option_tools::loadOptions(rtk_node, rtk_options_);
    }
    YAML::Node gnss_imu_init_node = node["gnss_imu_initializer_options"];
    if (gnss_imu_init_node.IsDefined()) {
      option_tools::loadOptions(gnss_imu_init_node, gnss_imu_init_options_);
    }
    YAML::Node ambiguity_node = node["ambiguity_resolution_options"];
    if (ambiguity_node.IsDefined()) {
      option_tools::loadOptions(ambiguity_node, ambiguity_options_);
    }

    // rotate estrinsics
    gnss_imu_init_options_.gnss_extrinsics = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics, imu_base_options_);
    gnss_imu_init_options_.gnss_extrinsics_initial_std = ImuEstimatorBase::rotateImuToBody(
      gnss_imu_init_options_.gnss_extrinsics_initial_std, imu_base_options_);
    CameraBundlePtr camera_bundle = feature_handler_options_.cameras;
    for (size_t i = 0; i < camera_bundle->numCameras(); i++) {
      camera_bundle->set_T_C_B(i, ImuEstimatorBase::rotateImuToBody(
        camera_bundle->get_T_C_B(i).inverse(), imu_base_options_).inverse());
    }

    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new RtkImuCameraRrrEstimator(rtk_imu_camera_rrr_options_, 
      gnss_imu_init_options_, rtk_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_, ambiguity_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  else {
    LOG(ERROR) << "Invalid estimator type: " << static_cast<int>(type_);
    return;
  }

  // For coordinate initialization
  if (estimatorTypeContains(SensorType::GNSS, type_)) {
    spp_estimator_.reset(new SppEstimator(gnss_base_options_));
  }

  // Initial values
  solution_.timestamp = 0.0;
}

MultiSensorEstimating::~MultiSensorEstimating()
{
  if (image_frontend_thread_) {
    image_frontend_thread_->join(); image_frontend_thread_ = nullptr;
  }
  if (measurement_thread_) {
    measurement_thread_->join(); measurement_thread_ = nullptr;
  }
  if (backend_thread_) {
    backend_thread_->join(); backend_thread_ = nullptr;
  }
}

// Reset estimator
void MultiSensorEstimating::resetProcessors()
{
  backend_firstly_updated_ = false;
  mutex_output_.lock();

  // single point positioning
  if (type_ == EstimatorType::Spp) 
  {
    estimator_.reset(new SppEstimator(
      spp_options_, gnss_base_options_, base_options_));
  }
  // single-differenced GNSS pseudorange positioning
  else if (type_ == EstimatorType::Sdgnss) 
  {
    estimator_.reset(new SdgnssEstimator(
      sdgnss_options_, gnss_base_options_, base_options_));
  }
  // double-differenced GNSS pseudorange positioning (differential GNSS)
  else if (type_ == EstimatorType::Dgnss) 
  {
    estimator_.reset(new DgnssEstimator(
      dgnss_options_, gnss_base_options_, base_options_));
  }
  // real-time kinematic
  else if (type_ == EstimatorType::Rtk) 
  {
    estimator_.reset(new RtkEstimator(
      rtk_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // precise point positioning
  else if (type_ == EstimatorType::Ppp) 
  {
    estimator_.reset(new PppEstimator(
      ppp_options_, gnss_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU loosely couple
  else if (type_ == EstimatorType::GnssImuLc)
  {
    estimator_.reset(new GnssImuLcEstimator(
      gnss_imu_lc_options_, gnss_imu_init_options_, gnss_loose_base_options_, 
      imu_base_options_, base_options_));
  }
  // SPP/IMU tightly couple
  else if (type_ == EstimatorType::SppImuTc) 
  {
    estimator_.reset(new SppImuTcEstimator(
      spp_imu_tc_options_, gnss_imu_init_options_, spp_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_));
  }
  // RTK/IMU tightly couple
  else if (type_ == EstimatorType::RtkImuTc)
  {
    estimator_.reset(new RtkImuTcEstimator(
      rtk_imu_tc_options_, gnss_imu_init_options_, rtk_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_, ambiguity_options_));
  }
  // PPP/IMU tightly couple
  else if (type_ == EstimatorType::PppImuTc)
  {
    estimator_.reset(new PppImuTcEstimator(
      ppp_imu_tc_options_, gnss_imu_init_options_, ppp_options_, gnss_base_options_, 
      gnss_loose_base_options_, imu_base_options_, base_options_, ambiguity_options_));
  }
  // GNSS/IMU/Camera semi-tightly integration
  else if (type_ == EstimatorType::GnssImuCameraSrr)
  {
    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new GnssImuCameraSrrEstimator(gnss_imu_camera_srr_options_,
      gnss_imu_init_options_, gnss_loose_base_options_, visual_estimator_base_options_, 
      imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // SPP/IMU/Camera tightly integration
  else if (type_ == EstimatorType::SppImuCameraRrr)
  {
    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new SppImuCameraRrrEstimator(spp_imu_camera_rrr_options_, 
      gnss_imu_init_options_, spp_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  // RTK/IMU/Camera tightly integration
  else if (type_ == EstimatorType::RtkImuCameraRrr)
  {
    feature_handler_.reset(new FeatureHandler(feature_handler_options_, imu_base_options_));
    estimator_.reset(new RtkImuCameraRrrEstimator(rtk_imu_camera_rrr_options_, 
      gnss_imu_init_options_, rtk_options_, gnss_base_options_, gnss_loose_base_options_, 
      visual_estimator_base_options_, imu_base_options_, base_options_, ambiguity_options_));
    std::shared_ptr<VisualEstimatorBase> visual_estimator = 
      std::dynamic_pointer_cast<VisualEstimatorBase>(estimator_);
    CHECK_NOTNULL(visual_estimator);
    visual_estimator->setFeatureHandler(feature_handler_);
  }
  else {
    LOG(ERROR) << "Invalid estimator type: " << static_cast<int>(type_);
    return;
  }

  // Set coordinate and gravity
  estimator_->setCoordinate(solution_.coordinate);
  if (estimatorTypeContains(SensorType::IMU, type_)) {
    double gravity = earthGravity(solution_.coordinate->getZero(GeoType::LLA));
    std::shared_ptr<ImuEstimatorBase> imu_estimator = 
      std::dynamic_pointer_cast<ImuEstimatorBase>(estimator_);
    CHECK_NOTNULL(imu_estimator);
    imu_estimator->setGravity(gravity);
  }

  // Clear output control
  output_timestamps_.clear();
  mutex_output_.unlock();
}

// Input data callback
void MultiSensorEstimating::estimatorDataCallback(EstimatorDataCluster& data)
{
  if (estimatorDataIllegal(data)) {
    LOG(ERROR) << "Received illegal estimator data cluster!";
    return;
  }

  // temporarily store measurements
  mutex_addin_.lock();
  measurement_addin_buffer_.push_back(data);
  mutex_addin_.unlock();
}

// Process funtion in every loop
void MultiSensorEstimating::process()
{
  // Process frontends in a separated thread
  if (image_frontend_thread_ == nullptr && 
      estimatorTypeContains(SensorType::Camera, type_)) {
    image_frontend_thread_.reset(new std::thread(
      &MultiSensorEstimating::runImageFrontend, this));
  }

  // Put measurements from addin buffer to measurement buffer
  if (measurement_thread_ == nullptr) {
    measurement_thread_.reset(new std::thread(
      &MultiSensorEstimating::runMeasurementAddin, this));
  }

  // Process backend in a separated thread
  if (backend_thread_ == nullptr) {
    backend_thread_.reset(new std::thread(
      &MultiSensorEstimating::runBackend, this)); 
  }

  // kill threads
  if (quit_thread_) {
    if (image_frontend_thread_) {
      image_frontend_thread_->join(); image_frontend_thread_ = nullptr;
    }
    if (measurement_thread_) {
      measurement_thread_->join(); measurement_thread_ = nullptr;
    }
    if (backend_thread_) {
      backend_thread_->join(); backend_thread_ = nullptr;
    }
  }
}

// Update latest solution
bool MultiSensorEstimating::updateSolution()
{
  // Backend not working yet
  if (!backend_firstly_updated_) return false;

  mutex_output_.lock();

  // No need to update 
  if (output_timestamps_.size() == 0) {
    mutex_output_.unlock(); return false;
  }

  // Erase timestamps in front of estimator window
  while (output_timestamps_.front() < estimator_->getOldestTimestamp()) {
    LOG(WARNING) << "Erasing output timestamp " << std::fixed 
      << output_timestamps_.front() << " because it is too old!";
    output_timestamps_.pop_front();
  }

  // Check pending
  if (output_timestamps_.back() - output_timestamps_.front() > 1.0) {
    LOG(WARNING) << "Large pending in output control!";
  }

  // Get solution
  const double timestamp = output_timestamps_.front();
  mutex_output_.unlock();
  solution_.timestamp = timestamp;
  solution_.covariance.setZero();
  if (!estimator_->getPoseEstimateAt(timestamp, solution_.pose) || 
      !estimator_->getSpeedAndBiasEstimateAt(timestamp, solution_.speed_and_bias) || 
      (compute_covariance_ && 
      !estimator_->getCovarianceAt(timestamp, solution_.covariance))) {
    return false;
  }

  // if we have GNSS, get GNSS variables
  if (estimatorTypeContains(SensorType::GNSS, type_)) {
    if (std::shared_ptr<GnssEstimatorBase> gnss_estimator = 
      std::dynamic_pointer_cast<GnssEstimatorBase>(estimator_)) {
      solution_.status = gnss_estimator->getSolutionStatus();
      solution_.num_satellites = gnss_estimator->getNumberSatellite();
      solution_.differential_age = gnss_estimator->getDifferentialAge();
    }
    else if (std::shared_ptr<GnssLooseEstimatorBase> gnss_estimator = 
      std::dynamic_pointer_cast<GnssLooseEstimatorBase>(estimator_)) {
      solution_.status = gnss_estimator->getSolutionStatus();
      solution_.num_satellites = gnss_estimator->getNumberSatellite();
      solution_.differential_age = gnss_estimator->getDifferentialAge();
    }
    else {
      LOG(FATAL) << "Unable to cast estimaotr to GNSS estimator!";
    }
  }

  mutex_output_.lock();
  output_timestamps_.pop_front();
  mutex_output_.unlock();

  return true;
}

// Handle time-propagation sensors
void MultiSensorEstimating::handleTimePropagationSensors(EstimatorDataCluster& data)
{
  // only support IMU
  CHECK(estimatorDataIsImu(data));
  if (estimator_) estimator_->addMeasurement(data);
  mutex_input_.lock();
  latest_imu_timestamp_ = data.timestamp;
  mutex_input_.unlock();
  // align timeline for output control
  if (backend_firstly_updated_ && output_align_tag_ == data.tag) {
    if (checkDownsampling(data.tag)) {
      mutex_output_.lock();
      output_timestamps_.push_back(data.timestamp);
      mutex_output_.unlock();
    }
  }
}

// Handle non-time-propagation sensors
void MultiSensorEstimating::handleNonTimePropagationSensors(EstimatorDataCluster& data)
{
  // Input align mode
  if (enable_input_align_) {
    // Insert a measurement to addin buffer realigning timestamps
    const double buffer_time = 2.0 * input_align_latency_;
    if (!needTimeAlign(type_)) {
      measurement_align_buffer_.push_back(data);
    }
    else if (measurement_align_buffer_.size() == 0 || 
        data.timestamp >= measurement_align_buffer_.back().timestamp) {
      measurement_align_buffer_.push_back(data);
    }
    else if (data.timestamp <= measurement_align_buffer_.front().timestamp) {
      if (data.timestamp < measurement_align_buffer_.back().timestamp - buffer_time) {
        LOG(WARNING) << "Throughing data at timestamp " << std::fixed << data.timestamp 
          << " because its latency is too large!";
      }
      else {
        measurement_align_buffer_.push_front(data);
      }
    }
    else
    for (auto it = measurement_align_buffer_.begin(); 
          it != measurement_align_buffer_.end(); it++) {
      if (data.timestamp >= it->timestamp) continue;
      measurement_align_buffer_.insert(it, data);
      break;
    }
  }
  // Non-align mode
  else {
    measurement_align_buffer_.push_back(data);
  }

  // Check if we can add to measurement buffer
  for (auto it = measurement_align_buffer_.begin(); it != measurement_align_buffer_.end();)
  {
    // we delay the data for input_align_latency_ to wait incoming data for realigning.
    if (measurement_align_buffer_.back().timestamp - 
        measurement_align_buffer_.front().timestamp < input_align_latency_) break;

    // we always add IMU measurement to estimator at a given timestamp before we 
    // add other sensor measurements.
    EstimatorDataCluster& measurement = *it;
    if (estimatorTypeContains(SensorType::IMU, type_) && 
        measurement.timestamp > latest_imu_timestamp_) {
      it++; continue;
    }

    mutex_input_.lock();

    // add measurements
    measurements_.push_back(measurement);

    // check pending, sparcify if needed
    if (enable_backend_data_sparsify_)
    {
      if (measurements_.size() > pending_num_threshold_) {
        pending_sparsify_num_++;
      }
      else if (pending_sparsify_num_ > 0) pending_sparsify_num_--;
      if (pending_sparsify_num_) {
        LOG(WARNING) << "Backend pending! Sparsifying measurements with counter " 
                    << pending_sparsify_num_ << ".";
        for (int i = 0; i < pending_sparsify_num_; i++) {
          // some measurements we cannot erase
          if (measurements_.front().frame_bundle && 
              measurements_.front().frame_bundle->isKeyframe()) break;
          // erase front measurement
          if (measurements_.size() > 1) measurements_.pop_front();
        }
      }
    }

    mutex_input_.unlock();
    it = measurement_align_buffer_.erase(it);
  }
}

// Handle sensors that need frontends
void MultiSensorEstimating::handleFrontendSensors(EstimatorDataCluster& data)
{
  CHECK(estimatorDataNeedFrontend(data));
  if (data.image) {
    mutex_image_input_.lock();
    image_frontend_measurements_.push_back(data);
    mutex_image_input_.unlock();
  }
}

// Put data from addin buffer to measurement buffer
void MultiSensorEstimating::putMeasurements()
{
  // get data
  mutex_addin_.lock();
  if (measurement_addin_buffer_.size() == 0) {
    mutex_addin_.unlock(); return;
  }
  EstimatorDataCluster data = measurement_addin_buffer_.front();
  measurement_addin_buffer_.pop_front();
  mutex_addin_.unlock();

  // time-propagation sensors
  if (estimatorDataIsImu(data)) {
    handleTimePropagationSensors(data);
  } 
  // sensors that needs frontends
  else if (estimatorDataNeedFrontend(data)) {
    handleFrontendSensors(data);
  }
  // GNSS reference station data (no need to align time)
  else if (data.gnss && data.gnss_role == GnssRole::Reference) {
    mutex_input_.lock();
    measurements_.push_back(data);
    mutex_input_.unlock();
  }
  // other sensors
  else {
    handleNonTimePropagationSensors(data);
  }
}

// Process estimator
bool MultiSensorEstimating::processEstimator()
{
  // Check if we have data to process
  mutex_input_.lock();
  if (measurements_.size() == 0) {
    mutex_input_.unlock(); return false;
  }
  EstimatorDataCluster measurement = measurements_.front();
  measurements_.pop_front();
  mutex_input_.unlock();

  // Check pending
  if (measurements_.size() > 5) {
    if (last_backend_pending_num_ != measurements_.size()) {
      LOG(WARNING) << "Large backend pending: " << measurements_.size()
                  << " measurements are waiting!";
    }
    last_backend_pending_num_ = measurements_.size();
  }

  // Set coordinate and gravity
  if (solution_.coordinate == nullptr) 
  {
    Eigen::Vector3d position_ecef;

    // Force set coordinate zero
    if (force_initial_global_position_) {
      GeoCoordinate coordinate;
      position_ecef = coordinate.convert(
        GeoCoordinate::degToRad(initial_global_position_), 
        GeoType::LLA, GeoType::ECEF);
    }
    // get coordinate zero from GNSS raw
    else if (measurement.gnss && 
             estimatorTypeContains(SensorType::GNSS, type_)) {
      if (!spp_estimator_->addMeasurement(measurement)) {
        return false;
      }
      if (!spp_estimator_->estimate()) {
        return false;
      }
      position_ecef = spp_estimator_->getPositionEstimate();
      measurement.gnss->position = position_ecef;
    }
    // get coordinate zero from solution
    else if (measurement.solution) {
      position_ecef = measurement.solution->coordinate->convert(
        measurement.solution->pose.getPosition(), 
        GeoType::ENU, GeoType::ECEF);
    }
    // no where to get coordinate zero
    else {
      return false;
    }

    // set coordinate
    solution_.coordinate = std::make_shared<GeoCoordinate>(
      position_ecef, GeoType::ECEF);
    Eigen::Vector3d lla = solution_.coordinate->convert(
      position_ecef, GeoType::ECEF, GeoType::LLA);
    estimator_->setCoordinate(solution_.coordinate);
    
    // Set gravity if needed
    if (estimatorTypeContains(SensorType::IMU, type_)) {
      double gravity = earthGravity(lla);
      std::shared_ptr<ImuEstimatorBase> imu_estimator = 
        std::dynamic_pointer_cast<ImuEstimatorBase>(estimator_);
      CHECK_NOTNULL(imu_estimator);
      imu_estimator->setGravity(gravity);
    }
  }

  // Process estimator
  bool is_updated = false;

  // add measurement
  if (estimator_->addMeasurement(measurement)) {
    // solve
    if (estimator_->estimate()) is_updated = true;
    // check if estimator valid
    if (estimator_->getStatus() == EstimatorStatus::Diverged) {
      // reset estimator
      LOG(WARNING) << "Reset estimator because it is diverge!";
      resetProcessors();
      is_updated = false;
    }
    // log intermediate data
    estimator_->logIntermediateData();
  }

  // Backend updated
  if (is_updated)
  {
    // update flag
    backend_firstly_updated_ = true;
    // align timeline for output control
    if (output_align_tag_ == measurement.tag) {
      mutex_output_.lock();
      if (checkDownsampling(measurement.tag)) {
        output_timestamps_.push_back(measurement.timestamp);
      }
      mutex_output_.unlock();
    }
    // check pendding
    if (measurements_.size() > 1) {
      double pending_period = measurements_.back().timestamp - 
                        solution_.timestamp;
      // add feedbacks here
    }
  }

  return true;
}

// Camera frontend processing
void MultiSensorEstimating::runImageFrontend()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    // Check if we have new image data
    mutex_image_input_.lock();
    if (image_frontend_measurements_.size() == 0) {
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }
    EstimatorDataCluster& front_measurement = image_frontend_measurements_.front();
    if (front_measurement.image_role != CameraRole::Mono) {
      image_frontend_measurements_.pop_front();
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }

    // Check if timestamp is valid
    if (!feature_handler_->isFirstFrame() && 
        feature_handler_->getFrameBundle()->getMinTimestampSeconds() >= 
        front_measurement.timestamp) {
      LOG(WARNING) << "Image timestamp descending detected! ("
        << std::fixed << front_measurement.timestamp << " vs " 
        << feature_handler_->getFrameBundle()->getMinTimestampSeconds() << ")";
      image_frontend_measurements_.pop_front();
      mutex_image_input_.unlock(); 
      spin.sleep(); continue;
    }

    // Check pending
    if (image_frontend_measurements_.size() > 5) {
      if (last_image_pending_num_ != image_frontend_measurements_.size()) {
        LOG(WARNING) << "Large image frontend pending: " 
                     << image_frontend_measurements_.size()
                     << " frames are waiting!";
      }
      last_image_pending_num_ = image_frontend_measurements_.size();
    }

    // Process feature detecting and tracking
    std::shared_ptr<cv::Mat>& image = front_measurement.image;
    double timestamp = front_measurement.timestamp;
    std::string tag = front_measurement.tag;
    bool ret = false;
    Transformation T_WS;
    if (!estimator_->getPoseEstimateAt(timestamp, T_WS)) {
      ret = feature_handler_->addImageBundle({image}, timestamp);
    }
    else {
      ret = feature_handler_->addImageBundle({image}, timestamp, {T_WS});
    }
    image_frontend_measurements_.pop_front();
    mutex_image_input_.unlock();
    if (ret) {
      ret = feature_handler_->processImageBundle();
    }
    if (ret) {
      FrameBundlePtr frame_bundle = feature_handler_->getFrameBundle();
      EstimatorDataCluster measurement(frame_bundle, tag);
      estimatorDataCallback(measurement);

      // call featured image output and map point output (for ROS)
      const FramePtr& frame = frame_bundle->at(0);
      const MapPtr& map = feature_handler_->getMap();
      std::shared_ptr<DataCluster> frame_data = std::make_shared<DataCluster>(frame);
      std::shared_ptr<DataCluster> map_data = std::make_shared<DataCluster>(map);
      for (auto& callback : output_data_callbacks_) {
        callback(tag_, frame_data);
        callback(tag_, map_data);
      }
    }

    spin.sleep();
  }
}

// Measurement addin thread
void MultiSensorEstimating::runMeasurementAddin()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    putMeasurements();
    spin.sleep();
  }
}

// Backend processing
void MultiSensorEstimating::runBackend()
{
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    processEstimator();
    spin.sleep();
  }
}

}
