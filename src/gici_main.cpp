/**
* @Function: main function of gici library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/node_handle.h"
#include "gici/utility/signal_handle.h"
#include "gici/utility/spin_control.h"
#include "gici/utility/node_option_handle.h"

using namespace gici;

// Process streamers and estimators which defined in option.yaml file.
// Usage: <path>/gici_main <path-to-option>. 
// For more details on how to configure your option.yaml file, see doc/configuration_instructions.md
int main(int argc, char** argv)
{
  // Get option file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-option>" << std::endl;
    return -1;
  }
  std::string config_file_path = argv[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch (YAML::BadFile &e) {
    std::cerr << "Unable to load option file!" << std::endl;
    return -1;
  }

  // Initialize glog for logging
  bool enable_logging = false;
  if (yaml_node["logging"].IsDefined() && 
      option_tools::safeGet(yaml_node["logging"], "enable", &enable_logging) && 
      enable_logging == true) {
    YAML::Node logging_node = yaml_node["logging"];
    google::InitGoogleLogging("gici");
    int min_log_level = 0;
    if (option_tools::safeGet(
        logging_node, "min_log_level", &min_log_level)) {
      FLAGS_minloglevel = min_log_level;
    }
    option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
    option_tools::safeGet(logging_node, "file_directory", &FLAGS_log_dir);
    if (FLAGS_logtostderr) FLAGS_stderrthreshold = min_log_level;
    else FLAGS_stderrthreshold = 5;
  }

  // Initialize signal handles to catch faults
  initializeSignalHandles();

  // Organize nodes
  NodeOptionHandlePtr node_option_handle = 
    std::make_shared<NodeOptionHandle>(yaml_node);
  if (!node_option_handle->valid) {
    std::cerr << "Invalid configurations!" << std::endl;
    return -1;
  }

  // Initialize nodes
  std::unique_ptr<NodeHandle> node_handle = 
    std::make_unique<NodeHandle>(node_option_handle);

  // Show information
  const std::vector<size_t> sizes = {
    node_option_handle->streamers.size(),
    node_option_handle->formators.size(), 
    node_option_handle->estimators.size()};
  std::cout << "Initialized " 
    << sizes[0] << " streamer" << (sizes[0] > 1 ? "s" : "") << ", "
    << sizes[1] << " formater" << (sizes[1] > 1 ? "s" : "") << ", and "
    << sizes[2] << " estimator" << (sizes[2] > 1 ? "s" : "") << ". "
    << "Running..." << std::endl;

  // Start running all threads
  SpinControl::run();

  // Loop
  SpinControl spin(1e-1);
  while (SpinControl::ok()) {
    spin.sleep();
  }

  return 0;
}