/**
* @Function: Integrate formator-decoded data to estimator data
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/stream/streaming.h"
#include "gici/stream/files_reading.h"
#include "gici/estimate/estimating.h"

namespace gici {

// Base class
class DataIntegrationBase {
public:
  using EstimatorDataCallback = std::function<void(EstimatorDataCluster&)>;
  using EstimatorDataCallbacks = std::vector<EstimatorDataCallback>;

  DataIntegrationBase(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::vector<std::shared_ptr<Streaming>>& streamings,
    const std::vector<std::string>& formator_tags,
    const std::vector<std::vector<std::string>>& roles);
  DataIntegrationBase(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<FilesReading>& files_reading,
    const std::vector<std::string>& streamer_tags,
    const std::vector<std::vector<std::string>>& roles);
  DataIntegrationBase() { }
  ~DataIntegrationBase() { }

  // Data callback
  virtual void dataCallback(
    const std::string& input_tag, const std::shared_ptr<DataCluster>& data) = 0;

protected:
  // Outside callbacks to handle epoch data
  EstimatorDataCallbacks estimator_callbacks_;

  // Mutex that blocks pending data when procssing current
  std::mutex mutex_;
  bool valid_ = false;

  // Behavior of corresponding formator
  using Behaviors = std::vector<std::string>;
  std::unordered_map<std::string, Behaviors> behaviors_;
};

// GNSS data integration
// Combines ephemeris, SSR messages, DCB file, and etc.
class GnssDataIntegration : public DataIntegrationBase {
public:
  GnssDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::vector<std::shared_ptr<Streaming>>& streamings,
    const std::vector<std::string>& formator_tags,
    const std::vector<std::vector<std::string>>& roles) : 
    DataIntegrationBase(estimating, streamings, formator_tags, roles) {
    init();
  }
  GnssDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<FilesReading>& files_reading,
    const std::vector<std::string>& streamer_tags,
    const std::vector<std::vector<std::string>>& roles) : 
    DataIntegrationBase(estimating, files_reading, streamer_tags, roles) {
    init();
  }
  ~GnssDataIntegration() {
    free();
  }

  // Data callback
  void dataCallback(
    const std::string& input_tag, const std::shared_ptr<DataCluster>& data) override;

protected:
  // Initialize
  void init();

  // Free
  void free();

  // Handle GNSS data
  void handleGNSS(const std::string& formator_tag, 
                  const std::shared_ptr<DataCluster::GNSS>& gnss);

  // Update GNSS ephemerides to local
  void updateEphemerides(const nav_t *nav);
  
  // Update GNSS code bias to local
  void updateCodeBias();

  // Update GNSS TGD to local
  void updateTgd();

  // Update GNSS phase bias to local
  void updatePhaseBias();

protected:
  // Local GNSS data to select ephemeris and SSR corrections
  std::shared_ptr<DataCluster::GNSS> gnss_local_;
  CodeBiasPtr code_bias_local_;
  PhaseBiasPtr phase_bias_local_;
  PhaseCenterPtr phase_center_local_;
};
  
// IMU data integration
class ImuDataIntegration : public DataIntegrationBase {
public:
  ImuDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::vector<std::shared_ptr<Streaming>>& streamings,
    const std::vector<std::string>& formator_tags,
    const std::vector<std::vector<std::string>>& roles) :
    DataIntegrationBase(estimating, streamings, formator_tags, roles) {
    valid_ = true;
  }
  ImuDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<FilesReading>& files_reading,
    const std::vector<std::string>& streamer_tags,
    const std::vector<std::vector<std::string>>& roles) : 
    DataIntegrationBase(estimating, files_reading, streamer_tags, roles) {
    valid_ = true;
  }

  // Data callback
  void dataCallback(
    const std::string& input_tag, const std::shared_ptr<DataCluster>& data) override;

protected:
  // Handle IMU data
  void handleIMU(const std::string& formator_tag, 
                 const std::shared_ptr<DataCluster::IMU>& imu);
};

// Image data integration
class ImageDataIntegration : public DataIntegrationBase {
public:
  ImageDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::vector<std::shared_ptr<Streaming>>& streamings,
    const std::vector<std::string>& formator_tags,
    const std::vector<std::vector<std::string>>& roles) :
    DataIntegrationBase(estimating, streamings, formator_tags, roles) {
    valid_ = true;
  }
  ImageDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<FilesReading>& files_reading,
    const std::vector<std::string>& streamer_tags,
    const std::vector<std::vector<std::string>>& roles) : 
    DataIntegrationBase(estimating, files_reading, streamer_tags, roles) {
    valid_ = true;
  }

  // Data callback
  void dataCallback(
    const std::string& input_tag, const std::shared_ptr<DataCluster>& data) override;

protected:
  // Handle Image data
  void handleImage(const std::string& formator_tag, 
                   const std::shared_ptr<DataCluster::Image>& image);
};

// Solution data integration
class SolutionDataIntegration : public DataIntegrationBase {
public:
  // The tag can be formator tag, streamer tag (only for ROS), or estimator tag.
  // input from estimator
  SolutionDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<EstimatingBase>& input_estimating,
    const std::string& input_tag,
    const std::vector<std::string>& roles);
  // input from streamer
  SolutionDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::vector<std::shared_ptr<Streaming>>& streamings,
    const std::vector<std::string>& formator_tags,
    const std::vector<std::vector<std::string>>& roles) :
    DataIntegrationBase(estimating, streamings, formator_tags, roles) {
    is_from_estimator_ = false;
    valid_ = true;
  }
  SolutionDataIntegration(
    const std::shared_ptr<EstimatingBase>& estimating, 
    const std::shared_ptr<FilesReading>& files_reading,
    const std::vector<std::string>& streamer_tags,
    const std::vector<std::vector<std::string>>& roles) : 
    DataIntegrationBase(estimating, files_reading, streamer_tags, roles) {
    is_from_estimator_ = false;
    valid_ = true;
  }

  // Data callback
  void dataCallback(
    const std::string& input_tag, const std::shared_ptr<DataCluster>& data) override;

  // If the input data is from estimator
  const bool isFromEstimator() const { return is_from_estimator_; }

protected:
  // Handle Solution data
  void handleSolution(const std::string& input_tag, 
                      const std::shared_ptr<Solution>& solution);

  // Check if a solution parameter valid
  inline bool checkCovarianceAt(const Solution& solution, int i)
  {
    const Eigen::Matrix3d& cov = solution.covariance.block<3, 3>(i, i);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (cov(i, j) != 0.0) return true;
      }
    }
    return false;
  }
  inline bool checkPosition(const Solution& solution) {
    return checkCovarianceAt(solution, 0);
  }
  inline bool checkVelocity(const Solution& solution) {
    return checkCovarianceAt(solution, 6);
  }
  inline bool checkAttitude(const Solution& solution) {
    return checkCovarianceAt(solution, 3);
  }

  // Check and adjust solution role
  SolutionRole adjustSolutionRole(
    const Solution& solution, const SolutionRole& role);

private:
  // If the input data is from estimator
  bool is_from_estimator_;
};

}
