/**
* @Function: Check consistancy of image timestamp
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include <iostream>
#include "gici/stream/formator.h"
#include "gici/gnss/gnss_common.h"

using namespace gici;

const int image_width = 752;
const int image_height = 480;
const int image_step = 1;
const double dt_tolerant = 0.4;

int main(int argc, char** argv)
{
  // Get file
  if (argc != 2) {
    std::cerr << "Invalid input variables! Supported variables are: "
              << "<path-to-executable> <path-to-bin-file>" << std::endl;
    return -1;
  }
  std::string file_path = argv[1];

  if (image_step != 1) {
    std::cerr << "We only support image input with step size of 1!" << std::endl;
    return -1;
  }

  // Open file
  FILE *fp_imagepack = fopen(file_path.data(), "r");

  char buf[1024];
  int n = 0;
  img_t img;
  init_img(&img, image_width, image_height, image_step);
  double last_timestamp = 0.0;
  while ((n = fread(buf, sizeof(char), 1024, fp_imagepack)) != 0)
  {
    for (int i = 0; i < n; i++) {
      if (!input_image(&img, buf[i])) continue;
      double timestamp = gnss_common::gtimeToDouble(img.time);
      if (last_timestamp != 0.0 && timestamp - last_timestamp > dt_tolerant) {
        std::cerr << "Detected timestamp jump at " << std::fixed << timestamp
          << "! Last timestamp is " << last_timestamp << "." << std::endl;
      }
      if (last_timestamp >= timestamp) {
        std::cerr << "Detected timestamp descending at " << std::fixed << timestamp
          << "! Last timestamp is " << last_timestamp << "." << std::endl;
      }
      last_timestamp = timestamp;
    }
  }

  free_img(&img);
  fclose(fp_imagepack);

  return 0;
}