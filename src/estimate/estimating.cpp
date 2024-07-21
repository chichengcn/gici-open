/**
* @Function: Estimator thread
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/estimate/estimating.h"

#include "gici/utility/spin_control.h"
#include "gici/imu/imu_common.h"
#include "gici/imu/imu_error.h"

namespace gici {

EstimatingBase::EstimatingBase(
  const NodeOptionHandlePtr& nodes, size_t i_estimator) : 
  compute_covariance_(true)
{
  // Get options
  const auto& estimator_node = nodes->estimators[i_estimator];
  const YAML::Node& node = estimator_node->this_node;
  tag_ = estimator_node->tag;
  std::string type_str = estimator_node->type;
  option_tools::convert(type_str, type_);

  if (!option_tools::safeGet(
      node, "output_align_tag", &output_align_tag_)) {
    LOG(FATAL) << "Unable to load output align tag!";
    return;
  }
  if (output_align_tag_.substr(0, 4) == "fmt_" || 
      output_align_tag_.substr(0, 4) == "str_" || 
      output_align_tag_.substr(0, 4) == "est_") {
    output_align_tag_ = 
      output_align_tag_.substr(4, output_align_tag_.size() - 4);
  }

  if (!option_tools::safeGet(node, "compute_covariance", &compute_covariance_)) {
    LOG(INFO) << "Unable to load compute_covariance!";
  }
  // input data roles
  for (size_t i = 0; i < estimator_node->input_tags.size(); i++) {
    const std::string& tag = estimator_node->input_tags[i];
    const std::vector<std::string>& roles = estimator_node->input_tag_roles[i];
    if (tag.substr(0, 4) != "est_") continue;
    CHECK(roles.size() == 1);
    SolutionRole solution_role;
    option_tools::convert(roles[0], solution_role);
    estimator_tag_to_role_.insert(std::make_pair(tag, solution_role));
  }

  // input data alignment
  if (option_tools::safeGet(node, "enable_input_align", &enable_input_align_)) {
    if (enable_input_align_)
    if (!option_tools::safeGet(node, "input_align_latency", &input_align_latency_)) {
      LOG(FATAL) << "Unable to load input_align_latency while enable_input_align is setted!";
      return;
    }
  }

  // backend pending check and measurement data sparsifying
  if (option_tools::safeGet(node, 
    "enable_backend_data_sparsify", &enable_backend_data_sparsify_)) {
    if (enable_backend_data_sparsify_)
    if (!option_tools::safeGet(node, 
      "pending_num_threshold", &pending_num_threshold_)) {
      LOG(FATAL) << "Unable to load pending_num_threshold "
                 << "while enable_backend_data_sparsify is setted!";
      return;
    }
  }

  // output data downsampling rate
  if (!option_tools::safeGet(node, "output_downsample_rate", &output_downsample_rate_)) {
    output_downsample_rate_ = 1;
  }
  output_downsample_cnt_ = 0;

  solution_.timestamp = 0.0;
}

EstimatingBase::~EstimatingBase()
{}

// Start thread
void EstimatingBase::start()
{
  // Create thread
  quit_thread_ = false;
  thread_.reset(new std::thread(&EstimatingBase::run, this));
}

// Stop thread
void EstimatingBase::stop()
{
  // Kill thread
  if(thread_ != nullptr) {
    quit_thread_ = true;
    thread_->join();
    thread_.reset();
  }
}

// Loop processing
void EstimatingBase::run()
{
  // Spin until quit command or global shutdown called 
  SpinControl spin(1.0e-4);
  while (!quit_thread_ && SpinControl::ok()) {
    // Process funtion in every loop
    process();

    // Publish solution
    if (updateSolution()) {
      std::shared_ptr<DataCluster> out_data = std::make_shared<DataCluster>(solution_);
      for (auto& out_callback : output_data_callbacks_) {
        out_callback(tag_, out_data);
      }
    }

    spin.sleep();
  } 
}

}
