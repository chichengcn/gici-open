/**
* @Function: Convert file from GICI formats to rosbag
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <gici_ros/GlonassEphemeris.h>
#include <gici_ros/GnssAntennaPosition.h>
#include <gici_ros/GnssEphemerides.h>
#include <gici_ros/GnssIonosphereParameter.h>
#include <gici_ros/GnssObservations.h>
#include <gici_ros/GnssSsrCodeBiases.h>
#include <gici_ros/GnssSsrPhaseBiases.h>
#include <gici_ros/GnssSsrEphemerides.h>
#include "gici/utility/node_option_handle.h"
#include "gici/stream/streamer.h"
#include "gici/gnss/gnss_common.h"
#include "gici/ros_utility/edit_timestamp_utility.h"

using namespace gici;

// Rosbag options
struct RosbagOption {
  std::string path;
  std::string topic_name;
  std::string format;

  // for gnss_raw
  bool enable_observation = false;
  bool enable_ephemeris = false;
  bool enable_antenna_position = false;
  bool enable_ionosphere_parameter = false;
  bool enable_ssr_code_bias = false;
  bool enable_ssr_phase_bias = false;
  bool enable_ssr_ephemeris = false;
};

// Write DataCluster to rosbag
void writeRosbag(const std::shared_ptr<DataCluster> data_cluster,
  rosbag::Bag& bag, RosbagOption& option, int sequence, double time_tag)
{
  if (option.format == "image") 
  {
    CHECK(data_cluster->image);
    auto& img = *data_cluster->image;
    cv::Mat image_mat(img.height, img.width, CV_8UC(img.step), img.image);
    sensor_msgs::ImagePtr img_msg = 
      cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::
      image_encodings::MONO8, image_mat).toImageMsg();
    img_msg->header.seq = sequence;
    img_msg->header.stamp = ros::Time(img.time);
    bag.write(option.topic_name, ros::Time(time_tag), *img_msg);
  }
  if (option.format == "imu")
  {
    CHECK(data_cluster->imu);
    auto& imu = *data_cluster->imu;
    sensor_msgs::Imu msg;
    msg.header.seq = sequence;
    msg.header.stamp = ros::Time(imu.time);
    msg.angular_velocity.x = imu.angular_velocity[0];
    msg.angular_velocity.y = imu.angular_velocity[1];
    msg.angular_velocity.z = imu.angular_velocity[2];
    msg.linear_acceleration.x = imu.acceleration[0];
    msg.linear_acceleration.y = imu.acceleration[1];
    msg.linear_acceleration.z = imu.acceleration[2];
    bag.write(option.topic_name, ros::Time(time_tag), msg);
  }
  if (option.format == "gnss_raw")
  {
    CHECK(data_cluster->gnss);
    auto& gnss = *data_cluster->gnss;
#define HAS(t) (std::find(gnss.types.begin(), gnss.types.end(), t) != gnss.types.end())
    if (option.enable_observation && HAS(GnssDataType::Observation))
    {
      gici_ros::GnssObservations msg;
      for (int i = 0; i < gnss.observation->n; i++) {
        gici_ros::GnssObservation o;
        obsd_t *obs = gnss.observation->data + i;
        int week = 0;
        o.tow = time2gpst(obs->time, &week);
        o.week = week;
        char prn_buf[5];
        satno2id(obs->sat, prn_buf);
        o.prn = prn_buf;
        const char system = o.prn[0];
        if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
        for (int j = 0; j < NFREQ+NEXOBS; j++) {
          if (obs->code[j] == CODE_NONE) continue;
          o.SNR.push_back(obs->SNR[j]);
          o.LLI.push_back(obs->LLI[j]);
          o.code.push_back(gnss_common::codeTypeToRinexType(o.prn[0], obs->code[j]));
          o.L.push_back(obs->L[j]);
          o.P.push_back(obs->P[j]);
          o.D.push_back(obs->D[j]);
        }
        msg.observations.push_back(o);
      }
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/observations";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_ephemeris && HAS(GnssDataType::Ephemeris))
    {
      gici_ros::GnssEphemerides msg;
      nav_t *nav = gnss.ephemeris;
      for (int i = 0; i < MAXSAT; i++) {
        eph_t *eph = nav->eph + i;
        if (eph->sat == 0) continue;
        gici_ros::GnssEphemeris e;
        char prn_buf[5];
        satno2id(eph->sat, prn_buf);
        e.prn = prn_buf;
        e.week = eph->week;
        if (e.prn[0] == 'C') {
          e.toes = time2bdt(gpst2bdt(eph->toe), NULL);
          e.toc = time2bdt(gpst2bdt(eph->toc), NULL);
        }
        else {
          e.toes = time2gpst(eph->toe, NULL);
          e.toc = time2gpst(eph->toc, NULL);
        }
        e.A = eph->A;
        e.sva = eph->sva;
        e.code = eph->code;
        e.idot = eph->idot;
        e.iode = eph->iode;
        e.f2 = eph->f2;
        e.f1 = eph->f1;
        e.f0 = eph->f0;
        e.iodc = eph->iodc;
        e.crs = eph->crs;
        e.deln = eph->deln;
        e.M0 = eph->M0;
        e.cuc = eph->cuc;
        e.e = eph->e;
        e.cus = eph->cus;
        e.toes = eph->toes;
        e.cic = eph->cic;
        e.OMG0 = eph->OMG0;
        e.cis = eph->cis;
        e.i0 = eph->i0;
        e.crc = eph->crc;
        e.omg = eph->omg;
        e.OMGd = eph->OMGd;
        for (int j = 0; j < 6; j++) {
          if (eph->tgd[j] != 0.0) {
            e.tgd.push_back(eph->tgd[j]);
          }
        }
        e.svh = eph->svh;
        msg.ephemerides.push_back(e);
      }
      for (int i = 0; i < MAXPRNGLO; i++) {
        geph_t *geph = nav->geph + i;
        if (geph->sat == 0) continue;
        gici_ros::GlonassEphemeris e;
        char prn_buf[5];
        satno2id(geph->sat, prn_buf);
        e.prn = prn_buf;
        e.svh = geph->svh;
        e.iode = geph->iode;
        int week = 0;
        e.tof = time2gpst(geph->tof, &week);
        e.toe = time2gpst(geph->toe, &week);
        e.week = week;
        e.frq = geph->frq;
        for (int j = 0; j < 3; j++) {
          e.vel.push_back(geph->vel[j]);
          e.pos.push_back(geph->pos[j]);
          e.acc.push_back(geph->acc[j]);
        }
        e.gamn = geph->gamn;
        e.taun = geph->taun;
        e.dtaun = geph->dtaun;
        e.age = geph->age;
        msg.glonass_ephemerides.push_back(e);
      }
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/ephemerides";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_antenna_position && HAS(GnssDataType::AntePos))
    {
      gici_ros::GnssAntennaPosition msg;
      for (size_t i = 0; i < 3; i++) {
        msg.pos.push_back(gnss.antenna->pos[i]);
      }
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/antenna_position";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_ionosphere_parameter && HAS(GnssDataType::IonAndUtcPara))
    {
      gici_ros::GnssIonosphereParameter msg;
      // use GPS parameters
      msg.type = 0;
      for (int i = 0; i < 8; i++) {
        msg.parameters.push_back(gnss.ephemeris->ion_gps[i]);
      }
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/ionosphere_parameter";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_ssr_code_bias && HAS(GnssDataType::SSR))
    {
      gici_ros::GnssSsrCodeBiases msg;
      for (int i = 0; i < MAXSAT; i++) {
        gici_ros::GnssSsrCodeBias b;
        ssr_t *ssr = gnss.ephemeris->ssr + i;
        char prn_buf[5];
        satno2id(i + 1, prn_buf);
        b.prn = prn_buf;
        int week;
        b.tow = time2gpst(ssr->t0[4], &week);
        b.week = week;
        b.udi = ssr->udi[4];
        b.isdcb = ssr->isdcb;
        const char system = b.prn[0];
        if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
        for (int j = 0; j < MAXCODE; j++) {
          if (ssr->cbias[j] == 0.0) continue;
          b.code.push_back(gnss_common::codeTypeToRinexType(b.prn[0], j + 1));
          b.bias.push_back(ssr->cbias[j]);
        }
        if (b.code.size() == 0) continue;
        msg.biases.push_back(b);
      }
      if (msg.biases.size() == 0) return;
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/code_bias";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_ssr_phase_bias && HAS(GnssDataType::SSR))
    {
      gici_ros::GnssSsrPhaseBiases msg;
      for (int i = 0; i < MAXSAT; i++) {
        gici_ros::GnssSsrPhaseBias b;
        ssr_t *ssr = gnss.ephemeris->ssr + i;
        char prn_buf[5];
        satno2id(i + 1, prn_buf);
        b.prn = prn_buf;
        int week;
        b.tow = time2gpst(ssr->t0[4], &week);
        b.week = week;
        b.udi = ssr->udi[4];
        b.isdpb = ssr->isdpb;
        const char system = b.prn[0];
        if (system != 'G' && system != 'R' && system != 'E' && system != 'C') continue;
        for (int j = 0; j < MAXCODE; j++) {
          if (ssr->pbias[j] == 0.0) continue;
          int phase = gnss_common::getPhaseID(b.prn[0], j + 1);
          b.phase.push_back(gnss_common::phaseTypeToPhaseString(b.prn[0], phase));
          b.bias.push_back(ssr->pbias[j]);
        }
        if (b.phase.size() == 0) continue;
        msg.biases.push_back(b);
      }
      if (msg.biases.size() == 0) return;
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/phase_bias";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
    if (option.enable_ssr_ephemeris && HAS(GnssDataType::SSR))
    {
      gici_ros::GnssSsrEphemerides msg;
      for (int i = 0; i < MAXSAT; i++) {
        gici_ros::GnssSsrEphemeris c;
        ssr_t *ssr = gnss.ephemeris->ssr + i;
        if (ssr->deph[0] == 0.0 || ssr->dclk[0] == 0.0) continue;
        char prn_buf[5];
        satno2id(i + 1, prn_buf);
        c.prn = prn_buf;
        int week;
        c.tow = time2gpst(ssr->t0[0], &week);
        c.week = week;
        c.udi = ssr->udi[0];
        c.iod = ssr->iod[0];
        c.iode = ssr->iode;
        c.iodcrc = ssr->iodcrc;
        c.refd = ssr->refd;
        for (int j = 0; j < 3; j++) {
          c.deph.push_back(ssr->deph[j]);
          c.ddeph.push_back(ssr->ddeph[j]);
          c.dclk.push_back(ssr->dclk[j]);
        }
        msg.corrections.push_back(c);
      }
      if (msg.corrections.size() == 0) return;
      msg.header.stamp = ros::Time(time_tag);
      std::string topic_name = option.topic_name + "/ephemerides_correction";
      bag.write(topic_name, ros::Time(time_tag), msg);
    }
  }
}

int main(int argc, char** argv)
{
  // Get file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-config>" << std::endl;
    return -1;
  }
  std::string config_file_path = argv[1];
  YAML::Node yaml_node;
  try {
     yaml_node = YAML::LoadFile(config_file_path);
  } catch (YAML::BadFile &e) {
    std::cerr << "Unable to load config file!" << std::endl;
    return -1;
  }

  // Initialize logging
  google::InitGoogleLogging("gici_tools");
  FLAGS_logtostderr = true;

  // Load options
  NodeOptionHandlePtr nodes = 
    std::make_shared<NodeOptionHandle>(yaml_node);
  if (!nodes->valid) {
    std::cerr << "Invalid base configurations!" << std::endl;
    return -1;
  }

  // Get rosbag options
  std::vector<NodeOptionHandle::NodeBasePtr> rosbags;
  if (yaml_node["rosbags"].IsDefined()) {
    const YAML::Node& rosbag_nodes = yaml_node["rosbags"];
    for (size_t i = 0; i < rosbag_nodes.size(); i++) {
      const YAML::Node& rosbag_node = rosbag_nodes[i]["rosbag"];
      NodeOptionHandle::NodeBasePtr rosbag = 
        std::make_shared<NodeOptionHandle::NodeBase>(rosbag_node);
      rosbags.push_back(rosbag);
    }
  }

  // Initialize files
  std::vector<file_t *> in_files;
  std::vector<std::shared_ptr<FormatorBase>> in_formators;
  std::vector<std::shared_ptr<rosbag::Bag>> out_files;
  std::vector<int> out_sequences;
  std::vector<RosbagOption> out_options;
  std::map<size_t, size_t> in_to_out_map;
  // initialize input files
  for (size_t i = 0; i < nodes->streamers.size(); i++) {
    const auto& node = nodes->streamers[i];
    YAML::Node streamer_node = node->this_node;
    std::string type_str = streamer_node["type"].as<std::string>();
    StreamerType type;
    option_tools::convert(type_str, type);
    CHECK(type == StreamerType::File);
    FileStreamer::Option option;
    option_tools::safeGet(streamer_node, "path", &option.path);
    // open input file
    in_files.push_back(openfile(option.path.data(), FILE_MODE_INPUT));
    // initialize formator
    std::vector<std::string> formator_tags;
    const std::vector<std::string>& output_tags = node->output_tags;
    for (const auto& tag : output_tags) {
      if (tag.substr(0, 4) == "fmt_") formator_tags.push_back(tag);
    }
    CHECK(formator_tags.size() == 1);
    NodeOptionHandle::FormatorNodeBasePtr formator_node = 
      std::static_pointer_cast<NodeOptionHandle::FormatorNodeBase>
      (nodes->tag_to_node.at(formator_tags[0]));
    in_formators.push_back(makeFormator(formator_node->this_node));
    // find options
    for (size_t j = 0; j < rosbags.size(); j++) {
      for (auto input_tag : rosbags[j]->input_tags) {
        if (input_tag == formator_tags[0]) {
          in_to_out_map.insert(std::make_pair(i, j));
        }
      }
    }
  }
  // initialize output files
  for (size_t i = 0; i < rosbags.size(); i++) {
    out_files.push_back(std::make_shared<rosbag::Bag>());
    YAML::Node rosbag_node = rosbags[i]->this_node;
    RosbagOption option;
    option_tools::safeGet(rosbag_node, "path", &option.path);
    option_tools::safeGet(rosbag_node, "topic_name", &option.topic_name);
    option_tools::safeGet(rosbag_node, "format", &option.format);
    if (option.format == "gnss_raw") {
      option_tools::safeGet(
        rosbag_node, "enable_observation", &option.enable_observation);
      option_tools::safeGet(
        rosbag_node, "enable_ephemeris", &option.enable_ephemeris);
      option_tools::safeGet(
        rosbag_node, "enable_antenna_position", &option.enable_antenna_position);
      option_tools::safeGet(
        rosbag_node, "enable_ionosphere_parameter", &option.enable_ionosphere_parameter);
      option_tools::safeGet(
        rosbag_node, "enable_ssr_code_bias", &option.enable_ssr_code_bias);
      option_tools::safeGet(
        rosbag_node, "enable_ssr_phase_bias", &option.enable_ssr_phase_bias);
      option_tools::safeGet(
        rosbag_node, "enable_ssr_ephemeris", &option.enable_ssr_ephemeris);
    }
    out_options.push_back(option);
    out_files[i]->open(option.path, rosbag::bagmode::Write);
    out_sequences.push_back(0);
  }

  // Read and write files
  for (size_t i = 0; i < in_files.size(); i++) {
    file_t *in_file = in_files[i];
    std::shared_ptr<FormatorBase> in_formator = in_formators[i];
    if (in_to_out_map.find(i) == in_to_out_map.end()) continue;
    std::cout << "Converting " << nodes->streamers[i]->tag << "..." << std::endl;
    size_t out_index = in_to_out_map.at(i);
    rosbag::Bag& out_file = *out_files[out_index];
    RosbagOption& out_option = out_options[out_index];
    int& out_sequence = out_sequences[out_index];
    uint8_t *buf;
    int buffer_length;
    if (!option_tools::safeGet(
      nodes->streamers[i]->this_node, "buffer_length", &buffer_length)) {
      LOG(INFO) << nodes->streamers[i]->tag 
        << ": Unable to load buffer length! Using default instead.";
      buffer_length = 32768;
    }
    buf = (uint8_t *)malloc(sizeof(uint8_t) * buffer_length);

    std::vector<std::shared_ptr<DataCluster>> dataset;
    uint32_t fpos_4B;
    uint64_t fpos_8B;
    double in_file_start_time = gnss_common::gtimeToDouble(gpst2utc(in_file->time));
    while (fread(&in_file->tick_n, sizeof(uint32_t), 1, in_file->fp_tag) && 
          fread((in_file->size_fpos==4)?(void *)&fpos_4B:(void *)&fpos_8B, 
          in_file->size_fpos, 1, in_file->fp_tag)) 
    {
      in_file->fpos_n = (long)((in_file->size_fpos==4)?fpos_4B:fpos_8B);
      int n_read = in_file->fpos_n - ftell(in_file->fp);
      if (n_read > buffer_length) {
        std::cerr << "WARN: Max buffer length exceeded!" << std::endl;
        n_read = buffer_length;
      }
      int n = (int)fread(buf, 1, n_read, in_file->fp);
      int nobs = in_formator->decode(buf, n, dataset);
      for (int k = 0; k < nobs; k++) {
        double time_tag = in_file_start_time + (double)in_file->tick_n * 1.0e-3;
        writeRosbag(dataset[k], out_file, out_option, ++out_sequence, time_tag);
      }
    }

    free(buf);
  }

  for (size_t i = 0; i < in_files.size(); i++) closefile(in_files[i]);
  for (size_t i = 0; i < out_files.size(); i++) out_files[i]->close();

  return 0;
}