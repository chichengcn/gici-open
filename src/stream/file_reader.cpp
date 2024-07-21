/**
* @Function: File reader for post-processing
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/file_reader.h"

#include "gici/stream/streamer.h"
#include "gici/gnss/gnss_common.h"

namespace gici {

// Base class for file reader
FileReaderBase::FileReaderBase(const YAML::Node& node) : 
  fd_(NULL), current_timestamp_(0.0), initial_timestamp_(0.0)
{
  // Type check
  if (!node["type"].IsDefined()) {
    LOG(FATAL) << "Unable to load streamer type!";
  }
  std::string type_str = node["type"].as<std::string>();
  CHECK(type_str == "post-file");

  // Load options
  option_tools::safeGet(node, "tag", &tag_);
  std::string path;
  if (!option_tools::safeGet(node, "path", &path)) {
    LOG(ERROR) << "Unable to load post-file path!";
    return;
  }
  // burst read mode
  if (!option_tools::safeGet(node, "burst_load", &burst_load_)) {
    burst_load_ = false;
  }
  // timestamp latency simulation
  if (!option_tools::safeGet(node, "latency", &latency_)) latency_ = 0.0;
  // format control
  CHECK(node["format"].IsDefined()) << "Unable to load post-file format!";
  const YAML::Node& format_node = node["format"];
  std::string format_type_str;
  if (!option_tools::safeGet(format_node, "type", &format_type_str)) {
    LOG(ERROR) << "Unable to load format type!";
    return;
  }
  if (formatTypeValid(format_type_str)) formator_ = makeFormator(format_node);
  else formator_ = nullptr;

  // Open file
  if (formatTypeValid(format_type_str)) {
    fd_ = fopen(path.data(), "rb");
    if (fd_ == NULL) {
      LOG(ERROR) << "Unable to open file " << path << "!";
      return;
    }
  }
  else fd_ = NULL;

  // Initialize buffer
  if (!(buf_ = (uint8_t *)malloc(sizeof(uint8_t) * max_buf_size))) {
    free(buf_); 
    fclose(fd_); fd_ = NULL;
    LOG(FATAL) << __FUNCTION__ << ": Buffer malloc error!";
  }

  // Load first package to get initial timestamp 
  if (!load()) {
    fclose(fd_); fd_ = NULL;
    LOG(ERROR) << "Unable to load first package for file " << tag_;
    return;
  }
  CHECK(data_buf_.size() > 0);
  initial_timestamp_ = data_buf_.front().first;
}

FileReaderBase::~FileReaderBase()
{
  // Close file
  if (fd_) fclose(fd_);

  // Free buffer
  if (buf_) free(buf_);
}

// Get a data cluster within the given timestamp
// we cannot directly get a batch of data till the timestamp here, because new data loading
// will modify the values in DataCluster, since we use a fix-length buffer in formator
bool FileReaderBase::get(
  DataClusterPair& data, const double timestamp)
{
  // Load from file if data used-up
  if (data_buf_.size() == 0) load();

  // Get data from data buffer
  if (data_buf_.size() > 0 && data_buf_.front().first <= timestamp) {
    data = data_buf_.front();
    data_buf_.pop_front();
  }
  else return false;

  return true;
}

// Load at least one data from file
// As default, we use the logic of the streaming class to read and decode data
// For other logic, users should override this function in subclasss
bool FileReaderBase::load()
{
  bool has_new_data = false;
  while (!feof(fd_))
  {
    size_t nread = fread(buf_, 1, max_buf_size, fd_);

    // Decode data
    std::vector<std::shared_ptr<DataCluster>> dataset;
    int nobs = formator_->decode(buf_, nread, dataset);
    for (int iobs = 0; iobs < nobs; iobs++) {
      std::shared_ptr<DataCluster>& data = dataset.at(iobs);
      double timestamp = burst_load_ ? 0.0 : getTimestamp(data);
      data_buf_.push_back(std::make_pair(timestamp, data));
      has_new_data = true;
    }

    if (has_new_data) break;
  }

  return !feof(fd_);
}

// Get timestamp for a DataCluster (latency added)
double FileReaderBase::getTimestamp(const std::shared_ptr<DataCluster>& data)
{
  #define SET_TIMESTAMP(t) \
    current_timestamp_ = current_timestamp_ < t ? t : current_timestamp_;

  if (data->gnss) {
    std::shared_ptr<DataCluster::GNSS> gnss = data->gnss;
    for (auto type : gnss->types) {
      if (type == GnssDataType::Observation) {
        gtime_t t_gps = gnss->observation->data[0].time;
        gtime_t t_utc = gpst2utc(t_gps);
        SET_TIMESTAMP(gnss_common::gtimeToDouble(t_utc));
      }
      if (type == GnssDataType::AntePos || type == GnssDataType::IonAndUtcPara || 
          type == GnssDataType::PhaseCenter || type == GnssDataType::Ephemeris) {
        // output current timestamp
      }
      if (type == GnssDataType::SSR) {
        for (int i = 0; i < MAXSAT; i++) {
          gtime_t t_gps = {0};
          for (int j = 0; j < 6; j++) {
            if (timediff(t_gps, gnss->ephemeris->ssr[i].t0[j]) < 0.0) {
              t_gps = gnss->ephemeris->ssr[i].t0[j];
            }
          }
          gtime_t t_utc = gpst2utc(t_gps);
          SET_TIMESTAMP(gnss_common::gtimeToDouble(t_utc));
        }
      }
    }
  }
  if (data->image) {
    SET_TIMESTAMP(data->image->time);
  }
  if (data->imu) {
    SET_TIMESTAMP(data->imu->time);
  }
  if (data->solution) {
    SET_TIMESTAMP(data->solution->timestamp);
  }

  return current_timestamp_ - latency_;
}

// Check if format type valid
bool FileReaderBase::formatTypeValid(const std::string type)
{
  return (type == "gnss-rtcm-2" || type == "gnss-rtcm-3" || 
          type == "gnss-raw" || type == "gnss-rinex" || 
          type == "image-v4l2" || type == "image-pack" || 
          type == "imu-pack" || type == "option" || 
          type == "nmea" || type == "dcb-file" || 
          type == "atx-file");
}

// -------------------------------------------------------------
// Get file reader handle from yaml
#define MAP_FILE_READER(Type, Reader) \
  if (type == Type) { return std::make_shared<Reader>(node); }
#define LOG_UNSUPPORT LOG(FATAL) << "FileReader type not supported!";
inline static FileReaderType loadType(const YAML::Node& node)
{
  if (!node["type"].IsDefined()) {
    LOG(FATAL) << "Unable to load formator type!";
  }
  YAML::Node format_node = node["format"];
  if (!format_node.IsDefined()) {
    LOG(FATAL) << "Format for post-file " << node["tag"] << " undefined!";
  }
  std::string type_str = format_node["type"].as<std::string>();
  FileReaderType type;
  option_tools::convert(type_str, type);
  return type;
}
std::shared_ptr<FileReaderBase> makeFileReader(const YAML::Node& node)
{
  FileReaderType type = loadType(node);
  MAP_FILE_READER(FileReaderType::RTCM2, RTCM2Reader);
  MAP_FILE_READER(FileReaderType::RTCM3, RTCM3Reader);
  MAP_FILE_READER(FileReaderType::GnssRaw, GnssRawReader);
  MAP_FILE_READER(FileReaderType::RINEX, RINEXReader);
  MAP_FILE_READER(FileReaderType::ImagePack, ImagePackReader);
  MAP_FILE_READER(FileReaderType::IMUPack, IMUPackReader);
  MAP_FILE_READER(FileReaderType::NMEA, NmeaReader);
  MAP_FILE_READER(FileReaderType::DcbFile, DcbFileReader);
  MAP_FILE_READER(FileReaderType::AtxFile, AtxFileReader);
  LOG_UNSUPPORT;
  return nullptr;
}

}
