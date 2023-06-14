/**
* @Function: Streamer functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <memory>
#include <glog/logging.h>

#include "gici/utility/transform.h"
#include "gici/utility/option.h"
#include "gici/utility/rtklib_safe.h"

namespace gici {

enum class StreamerType {
  Serial = STR_SERIAL,
  TcpClient = STR_TCPCLI,
  TcpServer = STR_TCPSVR,
  File = STR_FILE,
  NtripClient = STR_NTRIPCLI,
  NtripServer = STR_NTRIPSVR,
  V4L2 = 10,
  Ros
};

enum class StreamerRWType {
  Read = STR_MODE_R,
  Write = STR_MODE_W,
  ReadAndWrite = STR_MODE_RW
};

struct StreamerReplayOptions {
  double start_offset = 0;
  double speed = 1;
};

// Streamer control
class StreamerBase {
public:
  StreamerBase() { 
    memset(&stream_, 0, sizeof(stream_t));
    static_this_.push_back(this);
  }
  ~StreamerBase() { }

  // Open stream
  virtual int open(StreamerRWType type) = 0;

  // Close stream
  virtual void close() {
    strclose(&stream_);
  }

  // Read data from stream
  virtual int read(uint8_t *buf, int max_size) {
    if (disable_) return 0;
    return strread(&stream_, buf, max_size);
  }

  // Write data to stream
  virtual int write(uint8_t *buf, int size) {
    if (disable_) return 0;
    return strwrite(&stream_, buf, size);
  }

  // Get type
  StreamerType getType() { return type_; }

  // Get Read/Write mode
  StreamerRWType getRwType() 
  { return static_cast<StreamerRWType>(stream_.mode); }

  // Pause stream
  void disable() { disable_ = true; }

  // Resume stream
  void enable() { disable_ = false; }

  // Get stream message
  const std::string getMessage() const { return stream_.msg; }

  // Enable replay mode
  // This function should be called after all the streamers have been initialized
  static void enableReplay(StreamerReplayOptions option);

  // Synchronize streams for replay
  static void syncStreams();

private:

protected:
  stream_t stream_;
  StreamerType type_;
  bool disable_ = false;
  StreamerReplayOptions replay_options_;
  static std::vector<StreamerBase *> static_this_;
};

// Serial stream control
class SerialStreamer : public StreamerBase {
public:
  struct Option {
    std::string port;
    int baudrate;
    int bit_size = 8;
    std::string parity = "n";  // n|o|e
    int stop_bit = 1;
    std::string flow_control = "off"; // off|rts
  };

  SerialStreamer(Option& option) :
    option_(option) { 
    type_ = StreamerType::Serial;
  }
  SerialStreamer(YAML::Node& node);
  ~SerialStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// File stream control
class FileStreamer : public StreamerBase {
public:
  struct Option {
    std::string path;
    bool enable_time_tag = true;
    int swap_interval = 0;  // output swap interval (hr) (0: no swap)
  };

  FileStreamer(Option& option) :
    option_(option) { 
    type_ = StreamerType::File;
  }
  FileStreamer(YAML::Node& node);
  ~FileStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// TCP server stream control
class TcpServerStreamer : public StreamerBase {
public:
  struct Option {
    std::string port;
  };

  TcpServerStreamer(Option& option) :
    option_(option) {
    type_ = StreamerType::TcpServer;
  }
  TcpServerStreamer(YAML::Node& node);
  ~TcpServerStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// TCP client stream control
class TcpClientStreamer : public StreamerBase {
public:
  struct Option {
    std::string ip;
    std::string port;
  };

  TcpClientStreamer(Option& option) :
    option_(option) {
    type_ = StreamerType::TcpClient;
  }
  TcpClientStreamer(YAML::Node& node);
  ~TcpClientStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// Ntrip server stream control
class NtripServerStreamer : public StreamerBase {
public:
  struct Option {
    std::string ip;
    std::string port;
    std::string passward;
    std::string mountpoint;
  };

  NtripServerStreamer(Option& option) :
    option_(option) { 
    type_ = StreamerType::NtripServer;
  }
  NtripServerStreamer(YAML::Node& node);
  ~NtripServerStreamer() { }
  
  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// Ntrip client stream control
class NtripClientStreamer : public StreamerBase {
public:
  struct Option {
    std::string ip;
    std::string port;
    std::string username;
    std::string passward;
    std::string mountpoint;
  };

  NtripClientStreamer(Option& option) :
    option_(option) { 
    type_ = StreamerType::NtripClient;
  }
  NtripClientStreamer(YAML::Node& node);
  ~NtripClientStreamer() { }

  // Open stream
  int open(StreamerRWType type) override;

protected:
  Option option_;
};

// V4L2 stream control
class V4l2Streamer : public StreamerBase {
public:
  struct Option {
    std::string dev;
    int height;
    int width;
    int buffer_count = 1;
  };

  V4l2Streamer(Option& option) :
    option_(option) { 
    type_ = StreamerType::V4L2;
  }
  V4l2Streamer(YAML::Node& node);
  ~V4l2Streamer() { }

  // Open stream
  int open(StreamerRWType type) override;

  // Close stream
  void close() override;

  // Read data from stream
  int read(uint8_t *buf, int max_size) override;

  // Write data to stream
  int write(uint8_t *buf, int size) override;

protected:
  Option option_;
  dev_t dev_;
  uint8_t **v4l2_buf;
};

// Get stream handle from configure
#define MAKE_STREAMER(Streamer) \
  inline std::shared_ptr<StreamerBase> makeStreamer(Streamer::Option& option) { \
    return std::make_shared<Streamer>(option); \
  }
MAKE_STREAMER(SerialStreamer);
MAKE_STREAMER(FileStreamer);
MAKE_STREAMER(TcpServerStreamer);
MAKE_STREAMER(TcpClientStreamer);
MAKE_STREAMER(NtripServerStreamer);
MAKE_STREAMER(NtripClientStreamer);
MAKE_STREAMER(V4l2Streamer);

// Get stream handle from yaml
std::shared_ptr<StreamerBase> makeStreamer(YAML::Node& node);

}
