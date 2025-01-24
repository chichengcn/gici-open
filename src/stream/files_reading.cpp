/**
* @Function: Thread to read all post files
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/files_reading.h"

#include <glog/logging.h>

#include "gici/utility/spin_control.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

FilesReading::FilesReading(const NodeOptionHandlePtr& nodes)
{
  // Get streamer options and initialize
  for (size_t i = 0; i < nodes->streamers.size(); i++)
  {
    const auto& node = nodes->streamers[i];
    if (node->type != "post-file") continue;
    YAML::Node streamer_node = node->this_node;

    // initialize file reader
    std::shared_ptr<FileReaderBase> file_reader = makeFileReader(streamer_node);
    if (!file_reader->valid()) continue;
    tags_.push_back(node->tag);
    file_readers_.push_back(file_reader);
  }
}

FilesReading::~FilesReading()
{ }

// Start thread
void FilesReading::start()
{
  if (!valid()) return;

  // Create thread
  quit_thread_ = false;
  thread_.reset(new std::thread(&FilesReading::run, this));
}

// Stop thread
void FilesReading::stop()
{
  if (!valid()) return;

  // Kill thread
  if(thread_ != nullptr) {
    quit_thread_ = true;
    thread_->join();
    thread_.reset();
  }
}

// Loop processing
void FilesReading::run()
{
  // Get initial timestamp
  double initial_timestamp = INFINITY;
  double load_timestamp;
  for (size_t i = 0; i < tags_.size(); i++) {
    std::shared_ptr<FileReaderBase>& reader = file_readers_[i];
    double timestamp = reader->getInitialTimestamp();
    if (timestamp == 0.0) continue;  // burst-load data, such as ephemeris
    if (timestamp < initial_timestamp) initial_timestamp = timestamp;
  }
  load_timestamp = initial_timestamp;

  // Spin until all file read done, quit command or global shutdown called 
  std::pair<double, std::shared_ptr<DataCluster>> data;
  while (!quit_thread_ && SpinControl::ok()) {
    // Load data
    for (size_t i = 0; i < tags_.size(); i++) {
      std::string& tag = tags_[i];
      std::shared_ptr<FileReaderBase>& reader = file_readers_[i];

      // load until invalid
      while (reader->get(data, load_timestamp)) {
        // callback
        for (auto& callback : data_callbacks_) {
          // skip start offset
          const double timestamp = data.first;
          if (timestamp != 0.0 &&  // not for burst-load data
              (timestamp - initial_timestamp < start_offset_)) continue;
          // call
          callback(tag, data.second);
        }
      }
    }
    load_timestamp += step_;  // shift desired timestamp

    // Check if all load done
    quit_thread_ = true;
    for (size_t i = 0; i < tags_.size(); i++) {
      std::shared_ptr<FileReaderBase>& reader = file_readers_[i];
      if (!reader->done()) quit_thread_ = false;
    }
  }
}

}
