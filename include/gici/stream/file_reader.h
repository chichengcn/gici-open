/**
* @Function: File reader for post-processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/stream/formator.h"
#include "gici/estimate/estimator_types.h"
#include "gici/utility/node_option_handle.h"

namespace gici {

// Base class for file reader
class FileReaderBase {
public:
  // Types
  using DataClusterPair = std::pair<double, std::shared_ptr<DataCluster>>;

  // Node should be a streamer node with type "post-file"
  FileReaderBase(const YAML::Node& node);
  FileReaderBase() { }
  ~FileReaderBase();

  // Get a data cluster within the given timestamp
  // Return false if no valid data within given timestamp
  // The output data should be used before the next time calling this function because of shallow copy 
  bool get(DataClusterPair& data, 
    const double timestamp = INFINITY);

  // Get tag
  std::string getTag() { return tag_; }

  // Get initial timestamp
  double getInitialTimestamp();

  // Get valid status
  bool valid() { return (fd_ != NULL); }

  // Check if reached the end of file
  bool done() { return fd_ == NULL || feof(fd_); }

protected:
  // Load at least one data from file
  virtual bool load();

  // Get timestamp for a DataCluster (latency added)
  double getTimestamp(const std::shared_ptr<DataCluster>& data);

  // Check if format type valid
  bool formatTypeValid(const std::string type);

protected:
  // Load control
  std::string tag_;
  FormatorType type_;
  std::shared_ptr<FormatorBase> formator_;
  std::string path_;
  FILE *fd_;
  size_t max_buf_size_ = 32768;
  uint8_t *buf_;

  // Store loaded data
  std::deque<DataClusterPair> data_buf_;  

  // Timestamp control
  bool burst_load_;  // load all data at initial
  double latency_;  // to simulate sensor timestamp latency
  double initial_timestamp_;  // first package
  double current_timestamp_;  // make sure ascending
};

// Readers that has formators defined
using RTCM2Reader = FileReaderBase;
using RTCM3Reader = FileReaderBase;
using GnssRawReader = FileReaderBase;
using RINEXReader = FileReaderBase;
using ImagePackReader = FileReaderBase;
using IMUPackReader = FileReaderBase;
using NmeaReader = FileReaderBase;
using DcbFileReader = FileReaderBase;
using AtxFileReader = FileReaderBase;

// Readers that has no formators defined (not suitable for real-time stream)

// IMU text reader
class ImuTextReader : public FileReaderBase {
public:
  ImuTextReader(const YAML::Node& node) : 
    FileReaderBase(node) { }
  ~ImuTextReader() { }

protected:
  bool load() override;
};

// SP3 file reader
class Sp3FileReader : public FileReaderBase {
public:
  Sp3FileReader(const YAML::Node& node) : 
    FileReaderBase(node) { }
  ~Sp3FileReader() { }

protected:
  bool load() override;

  bool loaded_ = false;
};

// CLK file reader
class ClkFileReader : public FileReaderBase {
public:
  ClkFileReader(const YAML::Node& node) : 
    FileReaderBase(node) { }
  ~ClkFileReader() { }

protected:
  bool load() override;

  bool loaded_ = false;
};

// Get file reader handle from yaml
std::shared_ptr<FileReaderBase> makeFileReader(const YAML::Node& node);

}
