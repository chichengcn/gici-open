/**
* @Function: Estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <functional>
#include <glog/logging.h>

#include "gici/utility/option.h"
#include "gici/estimate/estimator_types.h"
#include "gici/stream/formator.h"
#include "gici/estimate/estimator_base.h"
#include "gici/utility/node_option_handle.h"

namespace gici {

class EstimatingBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The solution data cluster contains the following structures:
  // Solution: position, velocity and attitude solutions. The SolutionRole need to be 
  // specified for this data type, it tells other estimators how it will be used to 
  // implement multi-sensor fusion algorithm.
  // Frame: Features outputs to ROS
  // Map: Landmarks outputs to ROS
  using OutputDataCallback = std::function<
    void(const std::string&, const std::shared_ptr<DataCluster>&)>;
  // We do not need to store tags here because if any client registered this callback, 
  // we will send solution to it without tag check. The tag check is defined to distingush
  // input streams, see EstimateHandle::bindWithStreams.
  using OutputDataCallbacks = std::vector<OutputDataCallback>;

  EstimatingBase(const NodeOptionHandlePtr& nodes, size_t i_estimator);
  ~EstimatingBase();

  // Start thread
  void start();

  // Stop thread
  void stop();

  // Estimator data callback
  virtual void estimatorDataCallback(EstimatorDataCluster& data) {
    return;
  }

  // Set solution callback
  void setOutputDataCallback(OutputDataCallback output_data_callback) {
    output_data_callbacks_.push_back(output_data_callback);
  } 

  // Get tag
  std::string getTag() { return tag_; }

  // Process funtion in every loop
  virtual void process() = 0;

  // Update latest solution
  virtual bool updateSolution() = 0;

  // Check if we can continue under downsampling
  inline bool checkDownsampling(const std::string& tag) {
    if (++output_downsample_cnt_ < output_downsample_rate_) {
      return false;
    }
    else {
      output_downsample_cnt_ = 0; 
      return true;
    }
    return true;
  }

private:
	// Loop processing
	void run();

protected:
	// Thread handles
	std::unique_ptr<std::thread> thread_;
	bool quit_thread_ = false;
  // input data alignment
  bool enable_input_align_ = false;
  double input_align_latency_ = 0.0;
  // through some data when backend is pending
  bool enable_backend_data_sparsify_ = false;
  int pending_num_threshold_ = 0;
  int pending_sparsify_num_ = 0;
  // if user setted "loop_duration" as zero, we align the output rate to this input stream
  std::string output_align_tag_;
  // Output downsampling
  int output_downsample_rate_;
  int output_downsample_cnt_;
  // Pending output timestamps
  std::deque<double> output_timestamps_;

  // Between-estimator data pipeline control
  std::map<std::string, SolutionRole> estimator_tag_to_role_;

  // Estimator control
  std::string tag_;  // estimator tag
  EstimatorType type_;
  std::shared_ptr<EstimatorBase> estimator_;
  bool compute_covariance_;

  // Solutions
  Solution solution_;
  OutputDataCallbacks output_data_callbacks_;
};

}
