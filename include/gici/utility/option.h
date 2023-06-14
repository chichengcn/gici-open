/**
* @Function: Option tools
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace gici {

namespace option_tools {

enum class SensorType {
  None,
  GNSS,
  IMU,
  Camera,
  GeneralSolution,
  Option
};

// Convert options from yaml type to gici type
template <typename InType, typename OutType>
void convert(const InType& in, OutType& out);

// Get sensor type from options
SensorType sensorType(std::string in);

// Load options
template <typename OptionType>
void loadOptions(YAML::Node& node, OptionType& options);

// Copy options for similar estimators
template <typename OptionTypeIn, typename OptionTypeOut>
void copyOptions(const OptionTypeIn& in, OptionTypeOut& out);

/// \brief A function to get a value from a YAML node with non-exception error handling.
/// \param[in] node The YAML node.
/// \param[in] key The key used to dereference the node (node[key]).
/// \param[out] value The return value.
/// \returns True if the value was filled in successfully. False otherwise.
template<typename ValueType>
bool safeGet(const YAML::Node& node, const std::string& key, ValueType* value) {
  CHECK_NOTNULL(value);
  bool success = false;
  if(!node.IsMap()) {
    // LOG(INFO) << "Unable to get Node[\"" << key << "\"] because the node is not a map";
  } else {
    const YAML::Node sub_node = node[key];
    if(sub_node) {
      try {
        *value = sub_node.as<ValueType>();
        success = true;
      } catch(const YAML::Exception& e) {
        // LOG(INFO) << "Error getting key \"" << key << "\" as type "
        //     << typeid(ValueType).name() << ": " << e.what();
      }
    } else {
      // LOG(INFO) << "Key \"" << key << "\" does not exist";
    }
  }
  return success;
}

}

}
