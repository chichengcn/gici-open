/**
* @Function: Streamer functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/streamer.h"

#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/stat.h>
#include <unistd.h>

namespace gici {

// Static variables
std::vector<StreamerBase *> StreamerBase::static_this_;

// Load option with info
#define LOAD_COMMON(opt) \
  if (!option_tools::safeGet(node, #opt, &option_.opt)) { \
  LOG(INFO) << __FUNCTION__ << ": Unable to load " << #opt \
         << ". Using default instead."; }
// Load option with fatal error
#define LOAD_REQUIRED(opt) \
  if (!option_tools::safeGet(node, #opt, &option_.opt)) { \
  LOG(FATAL) << __FUNCTION__ << ": Unable to load " << #opt << "!"; }

// Enable replay mode
void StreamerBase::enableReplay(StreamerReplayOptions option)
{
  // Only file streams are enabled
  for (auto it : static_this_) {
    if (it->getType() != StreamerType::File) {
      // it->disable();
    }
    else {
      it->replay_options_ = option;
    }
  }
}

// Synchronize streams for replay
void StreamerBase::syncStreams()
{
  // Declare here becasue it was a local structure
  // we only read tag here, so we neglect the followings
  typedef struct {      /* file control type */
    FILE *fp;               /* file pointer */
    FILE *fp_tag;           /* file pointer of tag file */
    FILE *fp_tmp;           /* temporary file pointer for swap */
    FILE *fp_tag_tmp;       /* temporary file pointer of tag file for swap */
    char path[MAXSTRPATH];  /* file path */
    char openpath[MAXSTRPATH]; /* open file path */
  } file_t;

  // Find file with maximum size to set as master
  // because we assume this file has the highest frequency
  int master_index = -1, max_size = 0;
  for (size_t i = 0; i < static_this_.size(); i++) {
    if (static_this_[i]->stream_.type != STR_FILE) continue;
    if (static_this_[i]->stream_.port == NULL) continue;
    file_t *file = static_cast<file_t *>(static_this_[i]->stream_.port);
    if (file->fp_tag == NULL) continue;
    std::string tag_path = file->openpath + std::string(".tag");
    struct stat statbuf;  
    stat(tag_path.data(), &statbuf);  
    int size = statbuf.st_size; 
    if (size > max_size) {
      max_size = size; master_index = i;
    }
  } 
  if (master_index == -1) {
    LOG(ERROR) << "File synchronization error!";
    return;
  }

  // Set file synchronization
  for (size_t i = 0; i < static_this_.size(); i++) {
    if (static_this_[i]->stream_.type != STR_FILE) continue;
    if (i == master_index) continue;
    strsync(&static_this_[master_index]->stream_, &static_this_[i]->stream_);
  } 
}

// Serial stream control
SerialStreamer::SerialStreamer(YAML::Node& node)
{
  type_ = StreamerType::Serial;
  LOAD_REQUIRED(port); 
  LOAD_REQUIRED(baudrate);
  LOAD_COMMON(bit_size);
  LOAD_COMMON(parity);
  LOAD_COMMON(stop_bit);
  LOAD_COMMON(flow_control);
}

// Open stream
int SerialStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << option_.port << ":" << option_.baudrate << ":" 
     << option_.parity << ":" << option_.stop_bit << ":" 
     << option_.flow_control;
     
  return stropen(&stream_, STR_SERIAL, static_cast<int>(type), path.str().data());
}

// File stream control
FileStreamer::FileStreamer(YAML::Node& node)
{
  type_ = StreamerType::File;
  LOAD_REQUIRED(path);
  LOAD_COMMON(enable_time_tag);
  LOAD_COMMON(swap_interval);
}

// Open stream
int FileStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << option_.path;
  if (option_.enable_time_tag) {
#if 1
    path << "::T::+" << replay_options_.start_offset << "::x" 
       << replay_options_.speed << "::S=" << option_.swap_interval << "::P=4"; 
#else
    path << "::T::+" << replay_options_.start_offset << "::x" 
       << replay_options_.speed << "::S=" << option_.swap_interval << "::P=8"; 
#endif
  }
  
  // R&W mode may fail if file does not exist
  if (!stropen(&stream_, STR_FILE, static_cast<int>(type), path.str().data())) {
    return 0;
  }

  // Pre-shift file pointer to start_offset
  if (option_.enable_time_tag && replay_options_.start_offset > 0.0) {
    fileshift(&stream_);
  }

  return 1;
}

// TCP server stream control
TcpServerStreamer::TcpServerStreamer(YAML::Node& node)
{
  type_ = StreamerType::TcpServer;
  LOAD_REQUIRED(port);
}

// Open stream
int TcpServerStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << ":" << option_.port;
        
  return stropen(&stream_, STR_TCPSVR, static_cast<int>(type), path.str().data());
}

// TCP client stream control
TcpClientStreamer::TcpClientStreamer(YAML::Node& node)
{
  type_ = StreamerType::TcpClient;
  LOAD_REQUIRED(ip);
  LOAD_REQUIRED(port);
}

// Open stream
int TcpClientStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << option_.ip << ":" << option_.port;
        
  return stropen(&stream_, STR_TCPCLI, static_cast<int>(type), path.str().data());
}

// Ntrip server stream control
NtripServerStreamer::NtripServerStreamer(YAML::Node& node)
{
  type_ = StreamerType::NtripServer;
  LOAD_REQUIRED(ip);
  LOAD_REQUIRED(port);
  LOAD_REQUIRED(passward);
  LOAD_REQUIRED(mountpoint);
}

// Open stream
int NtripServerStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << ":" << option_.passward << "@" << option_.ip << ":" 
     << option_.port << "/" << option_.mountpoint << ":";
        
  return stropen(&stream_, STR_NTRIPSVR, static_cast<int>(type), path.str().data());
}

// Ntrip client stream control
NtripClientStreamer::NtripClientStreamer(YAML::Node& node)
{
  type_ = StreamerType::NtripClient;
  LOAD_REQUIRED(ip);
  LOAD_REQUIRED(port);
  LOAD_REQUIRED(username);
  LOAD_REQUIRED(passward);
  LOAD_REQUIRED(mountpoint);
}

// Open stream
int NtripClientStreamer::open(StreamerRWType type)
{
  std::stringstream path;
  path << option_.username << ":" << option_.passward << "@"
     << option_.ip << ":" << option_.port << "/" << option_.mountpoint;
        
  return stropen(&stream_, STR_NTRIPCLI, static_cast<int>(type), path.str().data());
}

// V4L2 stream control
V4l2Streamer::V4l2Streamer(YAML::Node& node)
{
  type_ = StreamerType::V4L2;
  LOAD_REQUIRED(dev);
  LOAD_REQUIRED(height);
  LOAD_REQUIRED(width);
  LOAD_COMMON(buffer_count);
}

// Open stream
int V4l2Streamer::open(StreamerRWType type)
{
  v4l2_format format;
  v4l2_requestbuffers req;
  v4l2_buffer buffer;

  // Open video device
  option_.dev = "/dev/" + option_.dev;
  if ((dev_ = ::open(option_.dev.data(), O_RDWR)) < 0) {
    LOG(ERROR) << "V4L2 device open failed!"; return 0;
  }

  // Initialize video device 
  memset(&format, 0, sizeof(format));
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
  format.fmt.pix.width = option_.width;
  format.fmt.pix.height = option_.height;
  if (ioctl(dev_, VIDIOC_TRY_FMT, &format) != 0) {
    LOG(ERROR) << "V4L2 VIDIOC_TRY_FMT failed!"; return 0;
  }
 
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(ioctl(dev_, VIDIOC_S_FMT, &format) != 0) {
    LOG(ERROR) << "V4L2 VIDIOC_S_FMT failed!"; return 0;
  }
 
  req.count = option_.buffer_count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(dev_, VIDIOC_REQBUFS, &req) != 0 || 
    req.count < option_.buffer_count) {
    LOG(ERROR) << "V4L2 VIDIOC_REQBUFS failed!"; return 0;
  }
    
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = req.type;
  buffer.memory = V4L2_MEMORY_MMAP;
  v4l2_buf = (uint8_t **)malloc(sizeof(uint8_t *)*option_.buffer_count);
  for (int i = 0; i < static_cast<int>(req.count); i++) {
    buffer.index = i;
    if(ioctl(dev_, VIDIOC_QUERYBUF, &buffer) != 0) {
      LOG(ERROR) << "V4L2 VIDIOC_QUERYBUF failed!"; return 0;
    }
    v4l2_buf[i] = (uint8_t*)mmap(NULL, buffer.length,
      PROT_READ|PROT_WRITE, MAP_SHARED, dev_, buffer.m.offset);
    if (v4l2_buf[i] == MAP_FAILED) {
      LOG(ERROR) << "V4L2 mmap failed!"; return 0;
    }
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;
    if (ioctl(dev_, VIDIOC_QBUF, &buffer) != 0) {
      LOG(ERROR) << "V4L2 VIDIOC_QBUF failed!"; return 0;
    }
  }
 
  // Start stream
  int buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(dev_, VIDIOC_STREAMON, &buf_type)!=0) {
    LOG(ERROR) << "V4L2 VIDIOC_STREAMON failed!"; return 0;
  }
        
  return 1;
}

// Close stream
void V4l2Streamer::close()
{
  if (dev_ < 0) return;

  ::close(dev_);
  free(v4l2_buf);
}

// Read data from stream
int V4l2Streamer::read(uint8_t *buf, int max_size)
{
  if (disable_) return 0;

  int nr, ret;
  v4l2_buffer buffer;
  
  if (dev_ < 0) return 0;

  memset(&buffer, 0, sizeof(buffer));
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = option_.buffer_count;
  ret = ioctl(dev_, VIDIOC_DQBUF, &buffer);
  if (ret != 0) {
    LOG(ERROR) << "V4L2 VIDIOC_DQBUF failed!"; return 0;
  }
  if (buffer.index<0 || buffer.index >= option_.buffer_count) {
    LOG(ERROR) << "V4L2: Invalid buffer index " << buffer.index << "!"; 
    return 0;
  }
 
  int length = option_.width * option_.height;
  memcpy(buf, v4l2_buf[buffer.index], length);
 
  ret = ioctl(dev_, VIDIOC_QBUF, &buffer);
  if (ret != 0) {
    LOG(ERROR) << "V4L2 VIDIOC_QBUF failed!"; return 0;
  }
 
  return length;
}

// Write data to stream
int V4l2Streamer::write(uint8_t *buf, int size)
{
  if (disable_) return 0;

  v4l2_control ctrl;
  char *p, cmd[32], val[16], *buff_c;

  buff_c = (char*)buf;
  if ((p = strchr(buff_c, ':'))) {
    strncpy(cmd, buff_c, p-buff_c); cmd[p-buff_c] = '\0';
    sscanf(p, ":%d", &ctrl.value);
  }
  // Auto gain control
  if (strcmp(cmd,"agc") == 0) ctrl.id=V4L2_CID_AUTOGAIN;
  // Auto exposure control
  else if (strcmp(cmd,"aec") == 0) {
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = !ctrl.value;
  }
  // Desired brightness in AGC and AEC mode
  else if (strcmp(cmd,"bri") == 0) ctrl.id = V4L2_CID_USER_BASE | 0x1002;
  // Gain
  else if (strcmp(cmd,"gain") == 0) ctrl.id = V4L2_CID_GAIN;
  // Exposure time
  else if (strcmp(cmd,"exp") == 0) ctrl.id = V4L2_CID_EXPOSURE;
  if (ioctl(dev_, VIDIOC_S_CTRL, &ctrl) != 0) {
    LOG(ERROR) << "V4L2 VIDIOC_S_CTRL failed!"; return 0;
  }

  return 1;
}

// Get stream handle from yaml
#define MAP_STREAMER(Type, Streamer) \
  if (type == Type) { return std::make_shared<Streamer>(node); }
std::shared_ptr<StreamerBase> makeStreamer(YAML::Node& node) {
  if (!node["type"].IsDefined()) {
    LOG(FATAL) << "Unable to load streamer type!";
  }
  std::string type_str = node["type"].as<std::string>();
  StreamerType type;
  option_tools::convert(type_str, type);
  MAP_STREAMER(StreamerType::Serial, SerialStreamer);
  MAP_STREAMER(StreamerType::File, FileStreamer);
  MAP_STREAMER(StreamerType::TcpServer, TcpServerStreamer);
  MAP_STREAMER(StreamerType::TcpClient, TcpClientStreamer);
  MAP_STREAMER(StreamerType::NtripServer, NtripServerStreamer);
  MAP_STREAMER(StreamerType::NtripClient, NtripClientStreamer);
  MAP_STREAMER(StreamerType::V4L2, V4l2Streamer);
  if (type == StreamerType::Ros) return nullptr;

  LOG(FATAL) << "Streamer type not supported!";
}

}