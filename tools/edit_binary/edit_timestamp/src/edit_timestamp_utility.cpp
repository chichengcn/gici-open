/**
* @Function: Utilities 
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "edit_timestamp_utility.h"

namespace gici {

file_t *openfile(const char *path, int mode)
{
  file_t *file;
  gtime_t time0={0};

  if (!(file=(file_t *)malloc(sizeof(file_t)))) return NULL;
  
  file->fp=file->fp_tag=file->fp_tmp=file->fp_tag_tmp=NULL;
  strcpy(file->path,path);
  file->openpath[0]='\0';
  file->mode=mode;
  file->timetag=0;
  file->repmode=0;
  file->offset=0;
  file->size_fpos=USE_4BIT_TAG ? 4 : 8;
  file->time=file->wtime=time0;
  file->tick=file->tick_f=file->tick_n=file->fpos_n=0;
  file->start=0.0;
  file->speed=1.0;
  file->swapintv=0.0;

  file->time=utc2gpst(timeget());
  file->tick=file->tick_f=tickget();
  file->fpos_n=0;
  file->tick_n=0;

  char tagpath[MAXSTRPATH+4]="";
  reppath(file->path,file->openpath,timeget(),"","");

  sprintf(tagpath,"%s.tag",file->openpath);
  if (file->mode == FILE_MODE_INPUT) {
    if (!(file->fp_tag=fopen(tagpath,"rb"))) {
      std::cerr << "Cannot open tag file " << tagpath << "!" << std::endl;
      fclose(file->fp);
      free(file);
      return NULL;
    }
    
    if (!(file->fp=fopen(file->openpath,"rb"))) {
      std::cerr << "Cannot open file " << file->openpath << "!" << std::endl;
      free(file);
      return NULL;
    }

    char tagh[TIMETAGH_LEN+1]="";
    double time_sec;
    uint32_t time_time;
    if (fread(&tagh,TIMETAGH_LEN,1,file->fp_tag)==1&&
        fread(&time_time,sizeof(time_time),1,file->fp_tag)==1&&
        fread(&time_sec ,sizeof(time_sec ),1,file->fp_tag)==1) {
        memcpy(&file->tick_f,tagh+TIMETAGH_LEN-4,sizeof(file->tick_f));
        file->time.time=(time_t)time_time;
        file->time.sec =time_sec;
        file->wtime=file->time;
    }
    else {
      fclose(file->fp);
      fclose(file->fp_tag);
      std::cerr << "Bad tag file: " << tagpath << "." << std::endl;
      free(file);
      return NULL;
    }
  }
  else {
    if (!(file->fp_tag=fopen(tagpath,"wb"))) {
      std::cerr << "Cannot create tag file " << tagpath << "!" << std::endl;
      fclose(file->fp);
      free(file);
      return NULL;
    }

    if (!(file->fp=fopen(file->openpath,"wb"))) {
      std::cerr << "Cannot open file " << file->openpath << "!" << std::endl;
      free(file);
      return NULL;
    }
  }

  return file;
}

void closefile(file_t *file)
{
  if (!file) return;
  if (file->fp) fclose(file->fp);
  if (file->fp_tag) fclose(file->fp_tag);
  if (file->fp_tmp) fclose(file->fp_tmp);
  if (file->fp_tag_tmp) fclose(file->fp_tag_tmp);
  file->fp=file->fp_tag=file->fp_tmp=file->fp_tag_tmp=NULL;
  free(file);
}

int getProperBufLength(const YAML::Node& node)
{
  if (!node["type"].IsDefined()) {
    LOG(FATAL) << "Unable to load formator type!";
  }
  std::string type_str = node["type"].as<std::string>();
  FormatorType type;
  option_tools::convert(type_str, type);
  
  // large value may cause latency, small value may cause jamming.
  if (type == FormatorType::RTCM2 || type == FormatorType::RTCM3 
   || type == FormatorType::GnssRaw) {
    return 32;
  }
  else if (type == FormatorType::ImagePack) {
    return 262144;
  }
  else if (type == FormatorType::IMUPack) return 16;
  else return 0;
}

double getTimestamp(const std::shared_ptr<DataCluster>& data)
{
  static double timestamp = -1.0;
#define SET_TIMESTAMP(t) timestamp = timestamp < t ? t : timestamp;

  if (data->gnss) {
    std::shared_ptr<DataCluster::GNSS> gnss = data->gnss;
    for (auto type : gnss->types) {
      if (type == GnssDataType::Observation) {
        gtime_t t_gps = gnss->observation->data[0].time;
        gtime_t t_utc = gpst2utc(t_gps);
        SET_TIMESTAMP(gnss_common::gtimeToDouble(t_utc));
      }
    }
  }
  if (data->image) {
    SET_TIMESTAMP(data->image->time);
  }
  if (data->imu) {
    SET_TIMESTAMP(data->imu->time);
  }

  // we need gps time
  return gnss_common::gtimeToDouble(utc2gpst(gnss_common::doubleToGtime(timestamp)));
}

}
