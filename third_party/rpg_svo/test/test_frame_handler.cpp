#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/common/camera.h>
#include <svo/common/conversions.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>
#include <svo/svo.h>
#include <svo/common/imu_calibration.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>

#include <list>
#include <vector>
#include <string>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <vikit/math_utils.h>
#include <vikit/blender_utils.h>
#include <vikit/timer.h>
#include <iostream>
#include <svo/common/camera.h>
#include <svo/common/frame.h>
#include <svo/common/seed.h>
#include <svo/common/transformation.h>
#include <svo/common/types.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/common/frame.h>
#include <vikit/cameras.h>
#include <vikit/cameras/camera_geometry.h>
#include <vikit/cameras/no_distortion.h>
#include <vikit/cameras/pinhole_projection.h>
#include "vikit/cameras/camera_geometry_base.h"
#include "vikit/cameras/atan_distortion.h"
#include "vikit/cameras/camera_geometry.h"
#include "vikit/cameras/equidistant_distortion.h"
#include "vikit/cameras/no_distortion.h"
#include "vikit/cameras/pinhole_projection.h"
#include "vikit/cameras/radial_tangential_distortion.h"
#include "vikit/cameras/omni_geometry.h"
#include <opencv2/opencv.hpp>
#include <svo/frame_handler_base.h>
#include "svo/initialization.h"
#include "svo/tracker/feature_tracker.h"

#include <dirent.h>

using namespace std;
using namespace svo;

// const int image_width = 640;  	
// const int image_height = 480;  
// const double fx = 481.2f;	
// const double fy = 480.0f;
// const double cx = 319.5f;
// const double cy = 239.5f;
// const double k1 = 0.0;
// const double k2 = 0.0;
// const double p1 = 0.0;
// const double p2 = 0.0;

const int image_width = 752;  	
const int image_height = 480;  
const double fx = 315.54083037273074;
const double fy = 315.1858821111163;
const double cx = 360.57955594734096;
const double cy = 251.26381960919133;
const double k1 = -0.01288326670663061;
const double k2 = -0.00021668779553504528;
const double p1 = 0.0014331179307111846;
const double p2 = -0.0006177362267997965;

vector< string > color_image_files;
vector<vk::cameras::Transformation> poses;
vector<svo::FramePtr> frames;
svo::FramePtr key_frame;
svo::FramePtr last_frame;
cv::Mat img_rgb;
cv::Mat img;

std::shared_ptr<FrameHandlerBase> svo_;

int main(int argc, char** argv)
{
  // std::string dataset_dir("/home/cc/linux/softwares/feature/feature_estimator/third_party/rpg_svo/test/data");
  // std::string dataset_dir("/home/cc/linux/softwares/feature/feature_estimator/third_party/rpg_svo/test/data2");
  std::string dataset_dir("/home/cc/linux/softwares/feature/feature_estimator/third_party/rpg_svo/test/data2/indoor_2");
  std::string experiment_name("frame_handler");

  BaseOptions base_options;
  DepthFilterOptions depth_filter_options;
  depth_filter_options.use_threaded_depthfilter = false;
  DetectorOptions feature_detector_options;
  InitializationOptions init_options;
  ReprojectorOptions reprojector_options;
  FeatureTrackerOptions tracker_options;
  Eigen::Vector4d intrinsics(fx, fy, cx, cy);
  Eigen::Vector4d distortion_parameters(k1, k2, p1, p2);
  svo::CameraPtr camera;
  camera.reset(new vk::cameras::PinholeRadTanGeometry(
                  image_width, image_height,
                  vk::cameras::PinholeProjection<vk::cameras::RadialTangentialDistortion>(
                    intrinsics,
                    vk::cameras::RadialTangentialDistortion(distortion_parameters))));
  std::vector<svo::CameraPtr> cameras;
  cameras.push_back(camera);
  vk::cameras::Quaternion q_B_C = vk::cameras::Quaternion(
      static_cast<Eigen::Matrix3d>(Eigen::Matrix3d::Identity()));
  vk::cameras::Transformation T_B_C(q_B_C, Eigen::Vector3d::Zero());
  vk::cameras::TransformationVector T_Ci_B;
  T_Ci_B.push_back(T_B_C.inverse());
  CameraBundle::Ptr cams;
  cams.reset(new vk::cameras::NCamera(T_Ci_B, cameras, "label"));
  svo_ = std::make_shared<FrameHandlerMono>(base_options,
    depth_filter_options, feature_detector_options, init_options, 
    reprojector_options, tracker_options, cams);
  svo_->start();

  // Read files
  // ifstream fin(dataset_dir+"/first_200_frames_traj_over_table_input_sequence.txt");
  // if ( !fin ) return 0;
  // while ( !fin.eof() )
  // {
  // // 数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ，注意是 TWC 而非 TCW
  //     string image; 
  //     fin>>image; 
  //     double data[7];
  //     for ( double& d:data ) fin>>d;
      
  //     color_image_files.push_back( dataset_dir+string("/images/")+image );
  //     vk::cameras::Quaternion q_0 = vk::cameras::Quaternion(
  //         Eigen::Quaterniond(data[6], data[3], data[4], data[5]));
  //     vk::cameras::Transformation T_0(q_0, Eigen::Vector3d(data[0], data[1], data[2]));
  //     poses.push_back(T_0);
  //     if ( !fin.good() ) break;
  // } 
  // SVO_INFO_STREAM("read total "<<color_image_files.size()<<" files.");

  DIR *directory;
  struct dirent *files_ptr;
  vector<int> file_indexes;
  int file_cnt;
  // string image_path = dataset_dir + "/Image_Group_001";
  string image_path = dataset_dir;
  directory = opendir(image_path.data());
  if (directory == NULL) {
      printf("No such directory: %s\r\n", image_path.data());
      return NULL;
  }
  while((files_ptr = readdir(directory)) != NULL) {
      string file_name = files_ptr->d_name;
      if (file_name.find(".jpg") == string::npos) continue;
      file_indexes.push_back(atoi(file_name.substr(4, 3).data()));
  }
  closedir(directory);
  sort(file_indexes.begin(), file_indexes.end());
  if (file_indexes.size() == 0) {
      printf("No image files in the folder!\r\n");
      return NULL;
  }
  file_cnt = 0;
  while (file_cnt < file_indexes.size()) {
    char path[128];
    sprintf(path, "%s/IMG_%03d.jpg", 
        image_path.data(), file_indexes[file_cnt]);
    color_image_files.push_back(path);
    file_cnt++;
  }
  SVO_INFO_STREAM("read total "<<color_image_files.size()<<" files.");

  // Process
  uint64_t current_time = 0;
  for (size_t i = 0; i < color_image_files.size(); i++) {
    img = cv::imread(color_image_files[i], 0);
    if (img.empty()) continue;
    // if (i % 10 != 0) continue;
    // frames.push_back(std::make_shared<svo::Frame>
    //         (cams->getCameraShared(0), img.clone(), 
    //         current_time, base_options.img_align_max_level + 1));
    // frames.back()->T_f_w_ = poses[i];

    svo_->addImageBundle({img}, current_time);

    // FrameBundlePtr frame_bundle = svo_->getLastFrames();
    // svo::feature_detection_utils::drawFeatures(*frame_bundle->at(0), 0u, false,
    //                                       &img_rgb);
    // string filename = dataset_dir + "/features_" + to_string(i) + ".png";
    // cv::imwrite(filename, img_rgb);

    FramePtr key_frame = svo_->map()->getLastKeyframe();
    int cnt = 0;
    if (key_frame != nullptr && svo_->last_frames_ != nullptr) {
      FramePtr cur_frame = svo_->last_frames_->frames_[0];
      Matcher matcher = svo_->depth_filter_->getMatcher();
      cv::cvtColor(cur_frame->img(), img_rgb, cv::COLOR_GRAY2RGB);
      for (size_t k = 0; k < key_frame->num_features_; k++) {
        // if (!isSeed(key_frame->type_vec_[k])) continue;
        FeatureWrapper ref_ftr = key_frame->getFeatureWrapper(k);
        const FloatType ref_depth = key_frame->getSeedDepth(k);
        Keypoint cur_px;
        Position xyz_world = key_frame->T_world_cam() * key_frame->getSeedPosInFrame(k);
        if (!reprojector_utils::projectPointAndCheckVisibility(cur_frame, xyz_world, &cur_px))
          continue;
        matcher.options_.align_max_iter = 30;
        Matcher::MatchResult res =
            matcher.findMatchDirect(*key_frame, *cur_frame, ref_ftr, ref_depth,
                                    cur_px);
        if (res != Matcher::MatchResult::kSuccess) 
          continue;
        cv::circle(img_rgb, cv::Point2f(cur_px(0), cur_px(1)),
                    1, cv::Scalar(0, 0, 255), 3);
        cnt++;
      }
      string filename = dataset_dir + "/features" "/features_" + to_string(i) + ".png";
      cv::imwrite(filename, img_rgb);
      SVO_WARN_STREAM("nfeatures = " << cnt);
    }

    current_time += 100e6;
    SVO_WARN_STREAM("Processed number " << i << " image. " << svo_->stageStr());
  }
  
  return 0;
}