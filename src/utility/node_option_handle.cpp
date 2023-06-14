/**
* @Function: Handles the configurations of nodes
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/utility/node_option_handle.h"

#include "gici/utility/option.h"

namespace gici {

NodeOptionHandle::NodeOptionHandle(const YAML::Node& yaml_node) :
  valid(true)
{
  // Load streamers and formators
  if (yaml_node["stream"].IsDefined()) 
  {
    const YAML::Node& stream_node = yaml_node["stream"];

    // Load streamers
    if (stream_node["streamers"].IsDefined()) {
      const YAML::Node& streamer_nodes = stream_node["streamers"];
      for (size_t i = 0; i < streamer_nodes.size(); i++) {
        const YAML::Node& streamer_node = streamer_nodes[i]["streamer"];
        StreamerNodeBasePtr streamer = 
          std::make_shared<StreamerNodeBase>(streamer_node);
        streamers.push_back(streamer);
        nodes.push_back(std::static_pointer_cast<NodeBase>(streamer));
        tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
      }
    }

    // Load formators
    if (stream_node["formators"].IsDefined()) {
      const YAML::Node& formator_nodes = stream_node["formators"];
      for (size_t i = 0; i < formator_nodes.size(); i++) {
        const YAML::Node& formator_node = formator_nodes[i]["formator"];
        FormatorNodeBasePtr formator = 
          std::make_shared<FormatorNodeBase>(formator_node);
        formators.push_back(formator);
        nodes.push_back(std::static_pointer_cast<NodeBase>(formator));
        tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
      }
    }

    // Relay options
    if (stream_node["replay"].IsDefined()) {
      replay_options = stream_node["replay"];
    }
  }

  // Load estimators
  if (yaml_node["estimate"].IsDefined()) {
    const YAML::Node& estimator_nodes = yaml_node["estimate"];
    for (size_t i = 0; i < estimator_nodes.size(); i++) {
      const YAML::Node& estimator_node = estimator_nodes[i]["estimator"];
      EstimatorNodeBasePtr estimator = 
        std::make_shared<EstimatorNodeBase>(estimator_node);
      estimators.push_back(estimator);
      nodes.push_back(std::static_pointer_cast<NodeBase>(estimator));
      tag_to_node.insert(std::make_pair(nodes.back()->tag, nodes.back()));
    }
  }

  // Check all nodes
  if (!checkAllNodeOptions()) { valid = false; return; }

  // Organize connections
  for (size_t i = 0; i < nodes.size(); i++) {
    if (nodes[i]->input_tags.size() > 0) 
    for (auto& input_tag : nodes[i]->input_tags) {
      for (size_t j = 0; j < nodes.size(); j++) {
        if (nodes[j]->tag != input_tag) continue;
        if (!tagExists(nodes[j]->output_tags, nodes[i]->tag)) {
          nodes[j]->output_tags.push_back(nodes[i]->tag);
        }
      }
    }
    for (auto& output_tag : nodes[i]->output_tags) {
      for (size_t j = 0; j < nodes.size(); j++) {
        if (nodes[j]->tag != output_tag) continue;
        if (!tagExists(nodes[j]->input_tags, nodes[i]->tag)) {
          nodes[j]->input_tags.push_back(nodes[i]->tag);
        }
      }
    }
  }

  // Check if connections valid
  for (size_t i = 0; i < nodes.size(); i++) {
    if (nodes[i]->input_tags.size() == 0 && nodes[i]->output_tags.size() == 0) {
      LOG(ERROR) << nodes[i]->tag << ": " 
        << "At least one input tag or output tag should be specified!";
      valid = false; return;
    }

    int n_out_fmts = 0, n_out_strs = 0, n_out_ests = 0;
    int n_in_fmts = 0, n_in_strs = 0, n_in_ests = 0;
    for (auto& output_tag : nodes[i]->output_tags) {
      if (output_tag.substr(0, 4) == "fmt_") n_out_fmts++;
      if (output_tag.substr(0, 4) == "str_") n_out_strs++;
      if (output_tag.substr(0, 4) == "est_") n_out_ests++;
      if (tag_to_node.find(output_tag) == tag_to_node.end()) {
        LOG(ERROR) << nodes[i]->tag << ": " 
          << "Cannot find the node with tag " << output_tag << "!";
        valid = false; return;
      }
    }
    for (auto& input_tag : nodes[i]->input_tags) {
      if (input_tag.substr(0, 4) == "fmt_") n_in_fmts++;
      if (input_tag.substr(0, 4) == "str_") n_in_strs++;
      if (input_tag.substr(0, 4) == "est_") n_in_ests++;
      if (tag_to_node.find(input_tag) == tag_to_node.end()) {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Cannot find the node with tag " << input_tag << "!";
        valid = false; return;
      }
    }
    if (nodes[i]->node_type == NodeType::Streamer) {
      if (n_out_ests > 0 && nodes[i]->type != "ros") {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Only ROS streamer is allowed to output to estimator!";
        valid = false; return;
      }
      if (n_in_strs > 1) {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "A streamer can connect to only one streamer input!";
        valid = false; return;
      }
      if (n_in_ests > 0 && nodes[i]->type != "ros") {
        LOG(ERROR) << nodes[i]->tag << ": "
          << "Only ROS streamer is allowed to input from estimator!";
        valid = false; return;
      }
    }
    if (nodes[i]->node_type == NodeType::Formator) {
      FormatorNodeBasePtr formator = 
        std::static_pointer_cast<FormatorNodeBase>(nodes[i]);
      if (formator->io == "input") {
        /* Allowed for ROS streamer.
        if (n_out_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to streamers is not allowed by formator in input mode!";
          valid = false; return;
        } */
        if (n_in_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from formators is not allowed by formator in input mode!";
          valid = false; return;
        }
        if (n_in_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in input mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_in_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from estimators is not allowed by formator in input mode!";
          valid = false; return;
        }
      }
      else if (formator->io == "log") {
        if (n_out_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to formators is not allowed by formator in log mode!";
          valid = false; return;
        }
        if (n_out_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_out_strs == 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode should connect to one stream!";
          valid = false; return;
        }
        if (n_out_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to estimators is not allowed by formator in log mode!";
          valid = false; return;
        }
        if (n_in_fmts > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one formator!";
          valid = false; return;
        }
        if (n_in_fmts == 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode should connect to one formator!";
          valid = false; return;
        }
        if (n_in_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from streamers is not allowed by formator in input mode!";
          valid = false; return;
        }
        if (n_in_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from estimators is not allowed by formator in input mode!";
          valid = false; return;
        }
      }
      else if (formator->io == "output") {
        if (n_out_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to formators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_out_strs > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in output mode can only connect to one stream!";
          valid = false; return;
        }
        if (n_out_ests > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Sending data to estimators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_fmts > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from formators is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_strs > 0) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "Getting data from streamers is not allowed by formator in output mode!";
          valid = false; return;
        }
        if (n_in_ests > 1) {
          LOG(ERROR) << nodes[i]->tag << ": "
            << "A formator in log mode can only connect to one estimator!";
          valid = false; return;
        }
      }
    }
    if (nodes[i]->node_type == NodeType::Estimator) {
      // already checked by above
    }
  }

  // Check estimator to estimator connection


  // We need at least one streamer
  if (streamers.size() == 0) {
    LOG(ERROR) << "At least one streamer should be specified!";
    valid = false; return;
  }
}

// Constructor for NodeBase
NodeOptionHandle::NodeBase::NodeBase(const YAML::Node& yaml_node) :
  valid(true)
{
  if (!option_tools::safeGet(yaml_node, "tag", &tag)) {
    LOG(ERROR) << "Unable to load tag!";
    valid = false; return;
  }
  if (!option_tools::safeGet(yaml_node, "type", &type)) {
    LOG(ERROR) << "Unable to load type!";
    valid = false; return;
  }
  option_tools::safeGet(yaml_node, "input_tags", &input_tags);
  option_tools::safeGet(yaml_node, "output_tags", &output_tags);
  // store other options
  this_node = yaml_node;
}

// Constructor for StreamerNodeBase
NodeOptionHandle::StreamerNodeBase::StreamerNodeBase(const YAML::Node& yaml_node) :
  NodeBase(yaml_node)
{
  if (!valid) return;
  node_type = NodeType::Streamer;

  if (tag.substr(0, 4) != "str_") {
    LOG(ERROR) << "Invalid tag name for streamer: " << tag << "!";
    valid = false; return;
  }
}

// Constructor for FormatorNodeBase
NodeOptionHandle::FormatorNodeBase::FormatorNodeBase(const YAML::Node& yaml_node) :
  NodeBase(yaml_node)
{
  if (!valid) return;
  node_type = NodeType::Formator;

  if (tag.substr(0, 4) != "fmt_") {
    LOG(ERROR) << "Invalid tag name for formator: " << tag << "!";
    valid = false; return;
  }
  if (!option_tools::safeGet(yaml_node, "io", &io)) {
    LOG(ERROR) << "Unable to load io type!";
    valid = false; return;
  }
}

// Constructor for FormatorNodeBase
NodeOptionHandle::EstimatorNodeBase::EstimatorNodeBase(const YAML::Node& yaml_node) :
  NodeBase(yaml_node)
{
  if (!valid) return;
  node_type = NodeType::Estimator;

  if (tag.substr(0, 4) != "est_") {
    LOG(ERROR) << "Invalid tag name for estimator: " << tag << "!";
    valid = false; return;
  }
  for (auto& input_tag : input_tags) {
    std::string option_name = input_tag + "_roles";
    std::vector<std::string> roles;
    if (!option_tools::safeGet(yaml_node, option_name, &roles) || roles.empty()) {
      LOG(ERROR) << "Unable to load input tag roles for " << input_tag << "!";
      valid = false; return;
    }
    input_tag_roles.push_back(roles);
  }
}

// Check if all nodes valid
bool NodeOptionHandle::checkAllNodeOptions()
{
  for (auto& streamer : streamers) {
    if (!streamer->valid) {
      LOG(ERROR) << "Node " << streamer->tag << " invalid!";
      return false;
    }
  }
  for (auto& formator : formators) {
    if (!formator->valid) {
      LOG(ERROR) << "Node " << formator->tag << " invalid!";
      return false;
    }
  }
  for (auto& estimator : estimators) {
    if (!estimator->valid) {
      LOG(ERROR) << "Node " << estimator->tag << " invalid!";
      return false;
    }
  }
  return true;
}

// Find if a tag exists in list
bool NodeOptionHandle::tagExists(
  const std::vector<std::string>& list, const std::string& tag)
{
  for (auto t : list) {
    if (t == tag) return true;
  }
  return false;
}

}
