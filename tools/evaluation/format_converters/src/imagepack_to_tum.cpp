/**
* @Function: Convert file from image pack to TUM image format
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/stream/format_image.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

const int image_width = 752;
const int image_height = 480;
const int image_step = 1;

int main(int argc, char ** argv)
{
  char imagepack_path[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(imagepack_path, argv[1]);
	}

  if (image_step != 1) {
    std::cerr << "We only support image input with step size of 1!" << std::endl;
    return -1;
  }

  FILE *fp_imagepack = fopen(imagepack_path, "r");
  char buf[1050];
  sprintf(buf, "%s.timestamp.txt", imagepack_path);
  FILE *fp_image_timestamp = fopen(buf, "w");
  std::string folder;
  int pos_to_cut = 0;
  for (int i = 0; i < strlen(imagepack_path); i++) {
    if (imagepack_path[i] == '/') pos_to_cut = i;
    folder.push_back(imagepack_path[i]);
  }
  folder = folder.substr(0, pos_to_cut);
  std::string cmd = "mkdir " + folder + "/images/";
  system(cmd.data());
  
  fprintf(fp_image_timestamp, "# timestamp filename\r\n");

  int n = 0;
  img_t img;
  init_img(&img, image_width, image_height, image_step);
  while ((n = fread(buf, sizeof(char), 1050, fp_imagepack)) != 0)
  {
    for (int i = 0; i < n; i++) {
      if (!input_image(&img, buf[i])) continue;
      char timestamp_str[32], filename_buf[64], filepath_buf[128];
      sprintf(timestamp_str, "%17.6lf", (double)img.time.time + img.time.sec);
      sprintf(filename_buf, "images/%s.png", timestamp_str);
      sprintf(filepath_buf, "%s/%s", folder.data(), filename_buf);
      fprintf(fp_image_timestamp, "%s %s\r\n", timestamp_str, filename_buf);
      cv::Mat image_mat(img.height, img.width, CV_8UC(img.step), img.image);
      cv::imwrite(filepath_buf, image_mat);
    }
  }

  fclose(fp_image_timestamp);
  fclose(fp_imagepack);
  free_img(&img);

  return 0;
}