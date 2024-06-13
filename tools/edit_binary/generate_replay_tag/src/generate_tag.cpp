/**
* @Function: Generate replay tag
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "utility.h"

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

  // Load client config
  YAML::Node client_node;
  std::string client_file_path;
  if (!yaml_node["client"].IsDefined()) {
    std::cerr << "Unable to load client node!" << std::endl;
    return -1;
  }
  client_node = yaml_node["client"];
  if (!option_tools::safeGet(client_node, "path", &client_file_path)) {
    std::cerr << "Unable to load client file path!" << std::endl;
    return -1;
  }

  // Initialize logging
  google::InitGoogleLogging("gici_tools");
  FLAGS_logtostderr = true;

  // Load master config
  YAML::Node master_node;
  std::string master_file_path;
  if (yaml_node["master"].IsDefined()) {
    master_node = yaml_node["master"];
    if (!option_tools::safeGet(master_node, "path", &master_file_path)) {
      std::cerr << "Unable to load master file path!" << std::endl;
      return -1;
    }
  }
  
  // Open files
  file_t *file_client = NULL, *file_master = NULL;
  file_client = openfile(client_file_path.data(), FILE_MODE_CLIENT);
  if (!master_file_path.empty()) {
    file_master = openfile(master_file_path.data(), FILE_MODE_MASTER);
  }
  
  // Generate tag
  /* time tag file structure   */
  /*   HEADER(60)+TICK(4)+TIME(4+8)+ */
  /*   TICK0(4)+FPOS0(4/8)+    */
  /*   TICK1(4)+FPOS1(4/8)+... */
  std::shared_ptr<FormatorBase> master_formator, client_formator;
  client_formator = makeFormator(client_node);
  if (!master_file_path.empty()) master_formator = makeFormator(master_node);
  const int buf_length = getProperBufLength(client_node);
  uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * buf_length);
  std::vector<std::shared_ptr<DataCluster>> dataset;

  // single file
  if (master_formator == nullptr)
  {
    bool printed_header = false;
    double start_timestamp = 0.0;
    while (fread(buf, 1, buf_length, file_client->fp)) 
    {
      // get timestamp
      int nobs = client_formator->decode(buf, buf_length, dataset);
      if (nobs == 0) continue;
      if (nobs > 1) {
        LOG(WARNING) << "More than one message was decoded, this may cause latency in replay!"
          << " (" << nobs << ")";
      }
      double timestamp = getTimestamp(dataset[nobs - 1], client_node);
      if (timestamp == 0.0) continue;
      
      // header
      if (!printed_header) {
        char tagh[TIMETAGH_LEN+1]="";
        sprintf(tagh,"TIMETAG RTKLIB %s",VER_RTKLIB);
        file_client->tick=file_client->tick_f=0;
        memcpy(tagh+TIMETAGH_LEN-4,&file_client->tick_f,sizeof(file_client->tick_f));
        file_client->time=gnss_common::doubleToGtime(timestamp);
        start_timestamp = timestamp;
        uint32_t time_time=(uint32_t)file_client->time.time;
        double time_sec=file_client->time.sec;
        fwrite(&tagh,1,TIMETAGH_LEN,file_client->fp_tag);
        fwrite(&time_time,1,sizeof(time_time),file_client->fp_tag);
        fwrite(&time_sec ,1,sizeof(time_sec ),file_client->fp_tag);
        fflush(file_client->fp_tag);
        printed_header = true;
        continue;
      }

      // body
      uint32_t tick = (uint32_t)((timestamp - start_timestamp) * 1.0e3);
      fwrite(&tick,1,sizeof(tick),file_client->fp_tag);
      long fpos=ftell(file_client->fp);
      if (file_client->size_fpos==4) {
        uint32_t fpos_4B=(uint32_t)fpos;
        fwrite(&fpos_4B,1,sizeof(fpos_4B),file_client->fp_tag);
      }
      else {
        uint64_t fpos_8B=(uint64_t)fpos;
        fwrite(&fpos_8B,1,sizeof(fpos_8B),file_client->fp_tag);
      }
      fflush(file_client->fp_tag);
    }
  }
  // align slave to master
  else 
  {
    // write slave header
    double start_timestamp;
    char tagh[TIMETAGH_LEN+1]="";
    sprintf(tagh,"TIMETAG RTKLIB %s",VER_RTKLIB);
    file_client->tick=file_client->tick_f=file_master->tick_f;
    memcpy(tagh+TIMETAGH_LEN-4,&file_client->tick_f,sizeof(file_client->tick_f));
    file_client->time=file_master->time;
    start_timestamp = gnss_common::gtimeToDouble(file_client->time);
    uint32_t time_time=(uint32_t)file_client->time.time;
    double time_sec=file_client->time.sec;
    fwrite(&tagh,1,TIMETAGH_LEN,file_client->fp_tag);
    fwrite(&time_time,1,sizeof(time_time),file_client->fp_tag);
    fwrite(&time_sec ,1,sizeof(time_sec ),file_client->fp_tag);
    fflush(file_client->fp_tag);

    // write slave body
    while (fread(buf, 1, buf_length, file_client->fp)) 
    {
      // get timestamp
      int nobs = client_formator->decode(buf, buf_length, dataset);
      if (nobs == 0) continue;
      if (nobs > 1) {
        LOG(WARNING) << "More than one message was decoded, this may cause latency in replay!"
          << " (" << nobs << ")";
      }
      double timestamp = getTimestamp(dataset[nobs - 1], client_node);
      if (timestamp == 0.0) continue;

      double dt = timestamp - start_timestamp;
      if (dt < 0.0) dt = 0.0;
      uint32_t tick = (uint32_t)(dt * 1.0e3);
      fwrite(&tick,1,sizeof(tick),file_client->fp_tag);
      long fpos=ftell(file_client->fp);
      if (file_client->size_fpos==4) {
        uint32_t fpos_4B=(uint32_t)fpos;
        fwrite(&fpos_4B,1,sizeof(fpos_4B),file_client->fp_tag);
      }
      else {
        uint64_t fpos_8B=(uint64_t)fpos;
        fwrite(&fpos_8B,1,sizeof(fpos_8B),file_client->fp_tag);
      }
      fflush(file_client->fp_tag);

      static uint32_t last_tick = 1;
      if (tick != last_tick) std::cout << "Generated tick at " << tick << std::endl;
      last_tick = tick;
    }
  }

  closefile(file_client);
  closefile(file_master);
  free(buf);

  return 0;
}
