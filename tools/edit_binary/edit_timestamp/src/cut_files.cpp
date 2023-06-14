/**
* @Function: Cut files according to timestamps
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "edit_timestamp_utility.h"
#include "gici/utility/node_option_handle.h"
#include "gici/stream/streamer.h"

using namespace gici;

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

  // get edit options
  if (!yaml_node["edit"].IsDefined()) {
    std::cerr << "Cannot load edit options!" << std::endl;
    return -1;
  }
  YAML::Node edit_node = yaml_node["edit"];
  std::string master_file_tag;
  if (!option_tools::safeGet(edit_node, "master_file_tag", &master_file_tag)) {
    std::cerr << "Cannot load master_file_tag!" << std::endl;
    return -1;
  }
  double start_timestamp = 0.0, end_timestamp = 0.0;
  if (!option_tools::safeGet(edit_node, "start_timestamp", &start_timestamp)) {
    std::cerr << "Cannot load start_timestamp!" << std::endl;
    return -1;
  }
  if (!option_tools::safeGet(edit_node, "end_timestamp", &end_timestamp)) {
    std::cerr << "Cannot load end_timestamp!" << std::endl;
    return -1;
  }
  std::string out_folder;
  if (!option_tools::safeGet(edit_node, "output_folder", &out_folder)) {
    std::cerr << "Cannot load output_folder!" << std::endl;
    return -1;
  }
  std::string mkdir_cmd = "mkdir " + out_folder;
  system(mkdir_cmd.data());
  CHECK(end_timestamp > start_timestamp);
  start_timestamp = gnss_common::utcTimeToGpsTime(start_timestamp);
  end_timestamp = gnss_common::utcTimeToGpsTime(end_timestamp);

  // Initialize files
  std::vector<file_t *> in_files, out_files;
  std::shared_ptr<FormatorBase> master_formator;
  size_t master_file_index = 0;
  int master_file_buf_length = 0;
  std::string master_file_path;
  uint8_t *master_file_buf;
  for (size_t i = 0; i < nodes->streamers.size(); i++) {
    const auto& node = nodes->streamers[i];
    YAML::Node streamer_node = node->this_node;
    std::string type_str = streamer_node["type"].as<std::string>();
    StreamerType type;
    option_tools::convert(type_str, type);
    CHECK(type == StreamerType::File);
    FileStreamer::Option option;
    option_tools::safeGet(streamer_node, "path", &option.path);
    in_files.push_back(openfile(option.path.data(), FILE_MODE_INPUT));
    std::string out_path;
    for (auto it = option.path.rbegin(); it != option.path.rend(); it++) {
      if (*it != '/') out_path.push_back(*it);
      else break;
    }
    std::reverse(out_path.begin(), out_path.end());
    out_path = out_folder + "/" + out_path;
    out_files.push_back(openfile(out_path.data(), FILE_MODE_OUTPUT));
    // init formator for master file
    if (node->tag == master_file_tag) {
      master_file_index = i;
      std::vector<std::string> formator_tags;
      const std::vector<std::string>& output_tags = node->output_tags;
      for (const auto& tag : output_tags) {
        if (tag.substr(0, 4) == "fmt_") formator_tags.push_back(tag);
      }
      CHECK(formator_tags.size() == 1);
      NodeOptionHandle::FormatorNodeBasePtr formator_node = 
        std::static_pointer_cast<NodeOptionHandle::FormatorNodeBase>
        (nodes->tag_to_node.at(formator_tags[0]));
      master_formator = makeFormator(formator_node->this_node);
      if (!option_tools::safeGet(streamer_node, "buffer_length", &master_file_buf_length)) {
        LOG(INFO) << "Master file: Unable to load buffer length! Using default instead.";
        master_file_buf_length = 32768;
      }
      master_file_buf = (uint8_t *)malloc(sizeof(uint8_t) * master_file_buf_length);
      master_file_path = option.path;
    }
  }

  // Handle master file first, get start and end ticks
  file_t *in_master_file = in_files[master_file_index];
  uint32_t start_tick = 0, end_tick = 0;
  uint32_t fpos_4B;
  uint64_t fpos_8B;
  std::vector<std::shared_ptr<DataCluster>> dataset;
  while (fread(&in_master_file->tick_n, sizeof(uint32_t), 1, in_master_file->fp_tag) && 
         fread((in_master_file->size_fpos==4)?(void *)&fpos_4B:(void *)&fpos_8B, 
         in_master_file->size_fpos, 1, in_master_file->fp_tag)) 
  {
    in_master_file->fpos_n = (long)((in_master_file->size_fpos==4)?fpos_4B:fpos_8B);
    int n_read = in_master_file->fpos_n - ftell(in_master_file->fp);
    if (n_read > master_file_buf_length) {
      std::cerr << "WARN: Max buffer length exceeded!" << std::endl;
      n_read = master_file_buf_length;
    }
    if (n_read <= 0) continue;
    int n = (int)fread(master_file_buf, 1, n_read, in_master_file->fp);
    int nobs = master_formator->decode(master_file_buf, n, dataset);
    if (nobs == 0) continue;
    double timestamp = getTimestamp(dataset[nobs - 1]);
    // update ticks
    if (start_tick == 0) start_tick = in_master_file->tick_n;
    if (timestamp <= start_timestamp) start_tick = in_master_file->tick_n;
    else if (timestamp <= end_timestamp) end_tick = in_master_file->tick_n;
    else break;
  }
  free(master_file_buf);
  closefile(in_files[master_file_index]);
  in_files[master_file_index] = openfile(master_file_path.data(), FILE_MODE_INPUT);

  // Cut all files
  for (size_t i = 0; i < in_files.size(); i++) {
    file_t *in_file = in_files[i];
    file_t *out_file = out_files[i];
    uint32_t start_tick_o = in_file->tick_f + start_tick - in_files[master_file_index]->tick_f;
    uint32_t end_tick_o = in_file->tick_f + end_tick - in_files[master_file_index]->tick_f;

    // write output tag header
    char tagh[TIMETAGH_LEN+1]="";
    sprintf(tagh,"TIMETAG RTKLIB %s",VER_RTKLIB);
    out_file->tick=out_file->tick_f=in_file->tick_f + start_tick_o;
    memcpy(tagh+TIMETAGH_LEN-4,&out_file->tick_f,sizeof(out_file->tick_f));
    out_file->time=timeadd(in_file->time, (double)start_tick / 1.0e3);
    uint32_t time_time=(uint32_t)out_file->time.time;
    double time_sec=out_file->time.sec;
    fwrite(&tagh,1,TIMETAGH_LEN,out_file->fp_tag);
    fwrite(&time_time,1,sizeof(time_time),out_file->fp_tag);
    fwrite(&time_sec ,1,sizeof(time_sec ),out_file->fp_tag);
    fflush(out_file->fp_tag);

    // body
    const auto& node = nodes->streamers[i];
    YAML::Node streamer_node = node->this_node;
    int buffer_length;
    uint8_t *buf;
    if (!option_tools::safeGet(streamer_node, "buffer_length", &buffer_length)) {
      LOG(INFO) << node->tag << ": Unable to load buffer length! Using default instead.";
      buffer_length = 32768;
    }
    buf = (uint8_t *)malloc(sizeof(uint8_t) * buffer_length);
    while (fread(&in_file->tick_n, sizeof(uint32_t), 1, in_file->fp_tag) && 
          fread((in_file->size_fpos==4)?(void *)&fpos_4B:(void *)&fpos_8B, 
          in_file->size_fpos, 1, in_file->fp_tag)) 
    {
      in_file->fpos_n = (long)((in_file->size_fpos==4)?fpos_4B:fpos_8B);
      if (in_file->tick_n < start_tick_o) {
        fseek(in_file->fp,in_file->fpos_n,SEEK_SET);
        continue;
      }
      if (in_file->tick_n > end_tick_o) break;
      
      int n_read = in_file->fpos_n - ftell(in_file->fp);
      if (n_read > buffer_length) {
        std::cerr << "WARN: Max buffer length exceeded!" << std::endl;
        n_read = buffer_length;
      }
      int n = (int)fread(buf, 1, n_read, in_file->fp);
      // write bin
      fwrite(buf, 1, n, out_file->fp);
      long out_fpos = ftell(out_file->fp);
      fflush(out_file->fp);
      // write tag
      out_file->tick_n = in_file->tick_n - start_tick_o;
      fwrite(&out_file->tick_n, 1, sizeof(uint32_t), out_file->fp_tag);
      if (out_file->size_fpos==4) {
          fpos_4B=(uint32_t)out_fpos;
          fwrite(&fpos_4B,1,sizeof(fpos_4B),out_file->fp_tag);
      }
      else {
          fpos_8B=(uint64_t)out_fpos;
          fwrite(&fpos_8B,1,sizeof(fpos_8B),out_file->fp_tag);
      }
      fflush(out_file->fp_tag);
    }
    free(buf);
  }

  for (size_t i = 0; i < in_files.size(); i++) {
    closefile(in_files[i]);
    closefile(out_files[i]);
  }

  return 0;
}
