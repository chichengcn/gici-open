/**
* @Function: Decoding and encoding stream
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

#include "gici/stream/format_image.h"
#include "gici/stream/format_imu.h"
#include "gici/utility/option.h"
#include "gici/utility/rtklib_safe.h"
#include "gici/estimate/estimator_types.h"
#include "gici/gnss/code_bias.h"

namespace gici {

// Formator types
enum class FormatorType {
  RTCM2, 
  RTCM3,
  GnssRaw, 
  RINEX, 
  ImageV4L2,
  ImagePack,  
  IMUPack,
  OptionPack, 
  NMEA,
  DcbFile,
  AtxFile
};

// GNSS data types
enum class GnssDataType {
  None = 0,
  Ephemeris = 2,
  Observation = 1,
  AntePos = 5,  // Antenna position
  IonAndUtcPara = 9,  // Ionosphere and UTC parameters
  SSR = 10,
  PhaseCenter   // PCVs and PCOs
};

// Data 
class DataCluster {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DataCluster() {}

  DataCluster(FormatorType type);

  DataCluster(FormatorType type, int _width, int _height, int _step);

  DataCluster(const Solution& data) : 
    solution(std::make_shared<Solution>(data)) {}

  DataCluster(const FramePtr& data) : frame(data) {}

  DataCluster(const MapPtr& data) : map(data) {}

  ~DataCluster();

  // GNSS data format
  struct GNSS {
    void init();
    void free();

    std::vector<GnssDataType> types;
    obs_t *observation;
    nav_t *ephemeris;
    sta_t *antenna;
  };

  // Image data format
  struct Image {
    void init(int _width, int _height, int _step);
    void free();

    double time;
    int width;
    int height;
    int step;
    uint8_t *image;
  };

  // IMU data format
  struct IMU {
    double time;
    double acceleration[3];
    double angular_velocity[3];
  };

  // Option data format
  struct Option {

  };

  // Input data types
  std::shared_ptr<GNSS> gnss;
  std::shared_ptr<Image> image;
  std::shared_ptr<IMU> imu;
  std::shared_ptr<Option> option;

  // Output data types
  std::shared_ptr<Solution> solution;
  std::shared_ptr<Frame> frame;
  std::shared_ptr<Map> map;
};

// Formats of FormatorType::GNSS_Raw
enum class GnssRawFormats {
  Ublox = STRFMT_UBX,
  Septentrio = STRFMT_SEPT,
  Novatel = STRFMT_OEM4,
  Tersus = STRFMT_OEM4
};

// Max number of output data buffers for decoders
struct MaxDataSize {
  static const int RTCM2 = 30;
  static const int RTCM3 = RTCM2;
  static const int GnssRaw = RTCM2;
  static const int RINEX = RTCM2;
  static const int ImagePack = 2;
  static const int IMUPack = 500;
};

// Tools for RTKLIB types
namespace gnss_common {

// Update observation data
extern void updateObservation(
  obs_t *obs, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ephemeris
extern void updateEphemeris(
  nav_t *nav, int sat, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ion/utc parameters
extern void updateIonAndUTC(
  nav_t *nav, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update antenna position
extern void updateAntennaPosition(
  sta_t *sta, std::shared_ptr<DataCluster::GNSS>& gnss_data);

// Update ssr corrections
enum class UpdateSsrType {
  Ephemeris,
  CodeBias,
  PhaseBias
};
extern void updateSsr(
  ssr_t *ssr, std::shared_ptr<DataCluster::GNSS>& gnss_data,
  std::vector<UpdateSsrType> type = {UpdateSsrType::Ephemeris, 
  UpdateSsrType::CodeBias, UpdateSsrType::PhaseBias},
  bool reset_ssr_status = true);

// Select data from GNSS stream
// Note that data except for observation are 
// putted in the first place of the vector
extern void updateStreamData(int ret, obs_t *obs, nav_t *nav, 
  sta_t *sta, ssr_t *ssr, int iobs, int sat, 
  std::vector<std::shared_ptr<DataCluster::GNSS>>& gnss_data);

}

// Base class
class FormatorBase {
public:
  FormatorBase() { }
  ~FormatorBase() { }

  // Decode stream to data
  virtual int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) = 0;

  // Encode data to stream
  virtual int encode(
    const std::shared_ptr<DataCluster>& data, uint8_t *buf) = 0;

  // Get formator type
  FormatorType getType() { return type_; }

  // Get data handle
  std::vector<std::shared_ptr<DataCluster>>& getDataHandle() { return data_; }

protected:
  std::vector<std::shared_ptr<DataCluster>> data_;
  FormatorType type_;
};

// RTCM 2
class RTCM2Formator : public FormatorBase {
public:
  struct Option {
    double start_time; 
  };

  RTCM2Formator(Option& option);
  RTCM2Formator(YAML::Node& node);
  ~RTCM2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// RTCM 3
class RTCM3Formator : public FormatorBase {
public:
  struct Option {
    double start_time; 
  };

  RTCM3Formator(Option& option);
  RTCM3Formator(YAML::Node& node);
  ~RTCM3Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rtcm_t rtcm_;
};

// GNSS raw
class GnssRawFormator : public FormatorBase {
public:
  struct Option {
    double start_time; 
    std::string sub_type;
  };

  GnssRawFormator(Option& option);
  GnssRawFormator(YAML::Node& node);
  ~GnssRawFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  raw_t raw_;
  GnssRawFormats format_;
};

// GNSS Rinex
class RINEXFormator : public FormatorBase {
public:
  struct Option {
    int buffer_length = 32768;
  };

  RINEXFormator(Option& option);
  RINEXFormator(YAML::Node& node);
  ~RINEXFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  rnxctr_t rnx_;
  bool header_decoded_ = false;
  std::string line_;
  char *buf_memory_, *p_memory_;
  FILE *fp_memory_;
};

// Image V4L2
class ImageV4L2Formator : public FormatorBase {
public:
  struct Option {
    int width;
    int height;
    int step = 1;
  };

  ImageV4L2Formator(Option& option);
  ImageV4L2Formator(YAML::Node& node);
  ~ImageV4L2Formator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// Image pack
class ImagePackFormator : public FormatorBase {
public:
  struct Option {
    int width;
    int height;
    int step = 1;
  };

  ImagePackFormator(Option& option);
  ImagePackFormator(YAML::Node& node);
  ~ImagePackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  img_t image_;
};

// IMU pack
class IMUPackFormator : public FormatorBase {
public:
  struct Option {
    
  };

  IMUPackFormator(Option& option);
  IMUPackFormator(YAML::Node& node);
  ~IMUPackFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  imu_t imu_;
};

// Option
class OptionFormator : public FormatorBase {
public:
  struct Option {
    
  };

  OptionFormator(Option& option);
  OptionFormator(YAML::Node& node);
  ~OptionFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:

};

// NMEA (for solution)
class NmeaFormator : public FormatorBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Option {
    bool use_gga = true;  // Use GxGGA message
    bool use_rmc = true;  // Use GxRMC message
    bool use_esa = false; // Use GxESA message (see encodeESA)
    bool use_esd = false; // Use GxESD message (see encodeESD)
    std::string talker_id = "GN";
  };

  NmeaFormator(Option& option);
  NmeaFormator(YAML::Node& node);
  ~NmeaFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  // Encode GNGGA message
  int encodeGGA(const Solution& solution, uint8_t* buf);

  // Encode GNRMC message
  int encodeRMC(const Solution& solution, uint8_t* buf);

  // Encode GNESA (self-defined Extended Speed and Attitude) message
  // Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
  int encodeESA(const Solution& solution, uint8_t* buf);

  // Encode GNESD (self-defined Extended STD) message
  // Format: $GNESD,tod,STD_Pe,STD_Pn,STD_Pu,STD_Ve,STD_Vn,STD_Vu,
  //         STD_Ar,STD_Ap,STD_Py*checksum
  int encodeESD(const Solution& solution, uint8_t* buf);

  // Convert Solution to sol_t
  void convertSolution(const Solution& solution, sol_t& sol);

  // Configure
  Option option_;
};

// Read DCB file in CAS format (https://cddis.nasa.gov/archive/gnss/products/bias/)
class DcbFileFormator : public FormatorBase {
public:
  struct Option {
    
  };

  DcbFileFormator(Option& option);
  DcbFileFormator(YAML::Node& node);
  ~DcbFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

protected:
  const int max_line_length_ = 128;
  std::string line_;
  bool passed_header_ = false;
  bool finished_reading_ = false;

  // map from PRN string to DCB storage
  using Dcb = CodeBias::Dcb;
  std::multimap<std::string, Dcb> dcbs_;
};

// Read IGS ATX file
class AtxFileFormator : public FormatorBase {
public:
  struct Option {
    
  };

  AtxFileFormator(Option& option);
  AtxFileFormator(YAML::Node& node);
  ~AtxFileFormator();

  // Decode stream to data
  int decode(const uint8_t *buf, int size, 
    std::vector<std::shared_ptr<DataCluster>>& data) override;

  // Encode data to stream
  int encode(const std::shared_ptr<DataCluster>& data, uint8_t *buf) override;

private:
  // Add antenna parameter
  void addpcv(const pcv_t *pcv, pcvs_t *pcvs);

  // Decode antenna parameter field
  int decodef(char *p, int n, double *v);

protected:
  const int max_line_length_ = 256;
  std::string line_;
  pcvs_t *pcvs_;
  pcv_t pcv_;
  int state_ = 0;
  int last_size_ = 0;
};

// Get formator handle from configure
#define MAKE_FORMATOR(Formator) \
inline std::shared_ptr<FormatorBase> makeFormator( \
  Formator::Option& option) { \
   return std::make_shared<Formator>(option); \
}
MAKE_FORMATOR(RTCM2Formator);
MAKE_FORMATOR(RTCM3Formator);
MAKE_FORMATOR(GnssRawFormator);
MAKE_FORMATOR(RINEXFormator);
MAKE_FORMATOR(ImageV4L2Formator);
MAKE_FORMATOR(ImagePackFormator);
MAKE_FORMATOR(IMUPackFormator);
MAKE_FORMATOR(OptionFormator);
MAKE_FORMATOR(NmeaFormator);
MAKE_FORMATOR(DcbFileFormator);
MAKE_FORMATOR(AtxFileFormator);

// Get formator handle from yaml
std::shared_ptr<FormatorBase> makeFormator(YAML::Node& node);


}
