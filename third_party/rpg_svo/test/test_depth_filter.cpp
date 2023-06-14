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

namespace {

using namespace std;

const int image_width = 640;  	
const int image_height = 480;  
const double fx = 481.2f;	
const double fy = 480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
const double k1 = 0.0;
const double k2 = 0.0;
const double p1 = 0.0;
const double p2 = 0.0;

struct ConvergedSeed {
  int x_, y_;
  double depth_, error_;
  ConvergedSeed(int x, int y, double depth, double error) :
    x_(x), y_(y), depth_(depth), error_(error)
  {}
};

/// DepthFilter Test-Fixture
class DepthFilterTest {
 public:
  DepthFilterTest();
  virtual ~DepthFilterTest();
  svo::UpdateResult processFirstFrame();
  void testReconstruction(std::string dataset_dir, std::string experiment_name);

  std::list<ConvergedSeed> results_;
  std::vector<double> errors_;
  size_t n_converged_seeds_;
  svo::CameraBundle::Ptr cams_;
  svo::InitializerPtr initializer_;
  svo::DepthFilterPtr depth_filter_;
  svo::FramePtr frame_ref_;
  svo::FramePtr frame_cur_;
  cv::Mat depth_ref_;
  svo::BaseOptions options_;
  bool have_rotation_prior_;
  bool initialized_;
  double depth_median_, depth_min_, depth_max_;

  vector< string > color_image_files;
  vector<vk::cameras::Transformation> poses;
  vector<svo::FramePtr> frames;
  svo::FramePtr key_frame;
  svo::FramePtr last_frame;
  cv::Mat img_rgb;
  cv::Mat img;
};

DepthFilterTest::DepthFilterTest() :
    n_converged_seeds_(0)
{
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
  cams_.reset(new vk::cameras::NCamera(T_Ci_B, cameras, "label"));

  depth_filter_ = NULL;

  have_rotation_prior_ = true;
  initialized_ = false;

  errors_.reserve(1000);
}

DepthFilterTest::~DepthFilterTest()
{

}

svo::UpdateResult DepthFilterTest::processFirstFrame()
{
  if(!initializer_->have_depth_prior_)
  {
    initializer_->setDepthPrior(options_.init_map_scale);
  }
  if(have_rotation_prior_)
  {
    VLOG(2) << "Setting absolute orientation prior";
    initializer_->setAbsoluteOrientationPrior(
          frames.back()->T_cam_imu().getRotation());
  }
  svo::FrameBundlePtr framebundle(new svo::FrameBundle({frames.back()}));
  const auto res = initializer_->addFrameBundle(framebundle);

  if(res == svo::InitResult::kTracking)
    return svo::UpdateResult::kDefault;

  // make old frame keyframe
  initializer_->frames_ref_->setKeyframe();
  initializer_->frames_ref_->at(0)->setKeyframe();

  // make new frame keyframe
  frames.back()->setKeyframe();
  svo::frame_utils::getSceneDepth(frames.back(), depth_median_, depth_min_, depth_max_);
  VLOG(40) << "Current Frame Depth: " << "min: " << depth_min_
          << ", max: " << depth_max_ << ", median: " << depth_median_;
  frames.back()->clearFeatureStorage();
  depth_filter_->addKeyframe(
              frames.back(), depth_median_, 0.5*depth_min_, depth_median_*1.5);
  frames.back()->track_id_vec_.conservativeResize(frames.back()->numFeatures());
  VLOG(40) << "Updating seeds in second frame using last frame...";
  depth_filter_->updateSeeds({ frames.back() }, last_frame);
  key_frame = frames.back();

  // add frame to map
  initializer_->reset();
  VLOG(1) << "Init: Selected second frame, triangulated initial map.";
  return svo::UpdateResult::kKeyframe;
}

void DepthFilterTest::testReconstruction(
    std::string dataset_dir,
    std::string experiment_name)
{
  // Read files
  ifstream fin(dataset_dir+"/first_200_frames_traj_over_table_input_sequence.txt");
  if ( !fin ) return;
  
  while ( !fin.eof() )
  {
  // 数据格式：图像文件名 tx, ty, tz, qx, qy, qz, qw ，注意是 TWC 而非 TCW
      string image; 
      fin>>image; 
      double data[7];
      for ( double& d:data ) fin>>d;
      
      color_image_files.push_back( dataset_dir+string("/images/")+image );
      vk::cameras::Quaternion q_0 = vk::cameras::Quaternion(
          Eigen::Quaterniond(data[6], data[3], data[4], data[5]));
      vk::cameras::Transformation T_0(q_0, Eigen::Vector3d(data[0], data[1], data[2]));
      poses.push_back(T_0);
      if ( !fin.good() ) break;
  } 
  cout<<"read total "<<color_image_files.size()<<" files."<<endl;

  // Depth filter
  svo::DepthFilterOptions depthfilter_options;
  svo::DetectorOptions detector_options;
  depthfilter_options.use_threaded_depthfilter = false;
  depthfilter_options.max_n_seeds_per_frame = 10;
  depth_filter_.reset(new svo::DepthFilter(depthfilter_options, detector_options, cams_));
  
  // Initializer
  svo::InitializationOptions initialization_options;
  initialization_options.init_min_disparity = 10.0;
  initializer_ = svo::initialization_utils::makeInitializer(
    initialization_options, svo::FeatureTrackerOptions(), detector_options, cams_);

  // Process
  uint64_t current_time = 0;
  for (size_t i = 0; i < color_image_files.size(); i++) {
    img = cv::imread(color_image_files[i], 0);
    if (img.empty()) continue;
    frames.push_back(std::make_shared<svo::Frame>
            (cams_->getCameraShared(0), img.clone(), 
            current_time, options_.img_align_max_level + 1));
    frames.back()->T_f_w_ = poses[i];

    // First frame
    if (!initialized_) {
      // key_frame = frames.back();
      // key_frame->setKeyframe();
      // // extract features
      // svo::DetectorOptions fast_options;
      // svo::AbstractDetector::Ptr detector =
      //     svo::feature_detection_utils::makeDetector(fast_options, key_frame->cam());
      // detector->detect(key_frame);
      // svo::feature_detection_utils::drawFeatures(*key_frame, 0u, false, &img_rgb);
      // string filename = dataset_dir + "/features.png";
      // cv::imwrite(filename, img_rgb);
      // svo::frame_utils::getSceneDepth(key_frame, depth_median, depth_min, depth_max);
      // cout << "Current Frame Depth: " << "min: " << depth_min
      //         << ", max: " << depth_max << ", median: " << depth_median << endl;
      // key_frame->clearFeatureStorage();
      // depth_filter_->addKeyframe(
      //             key_frame, depth_median, 0.5*depth_min, depth_median*1.5);
      // initialized_ = true;

      if (processFirstFrame() == svo::UpdateResult::kKeyframe)
        initialized_ = true;
    } 
    else {
      cout << "Updating seeds in overlapping keyframes..." << endl;
      // depth_filter_->updateSeeds({key_frame}, frames.back());

      // Put seeds on current frame
      frames.back()->copyFeaturesFrom(*key_frame);
      // for(size_t k = 0; k < key_frame->num_features_; ++k)
      // {
      //   const svo::FeatureType& type = key_frame->type_vec_[k];
      //   if(isSeed(type))
      //   {
      //     svo::Keypoint px;
      //     svo::FeatureWrapper ref_ftr = key_frame->getFeatureWrapper(k);
      //     Eigen::Ref<svo::SeedState> state = key_frame->invmu_sigma2_a_b_vec_.col(k);
      //     svo::Matcher::MatchResult res = 
      //     depth_filter_->getMatcher().findEpipolarMatchDirect(
      //         *key_frame, *frames.back(), ref_ftr, svo::seed::getInvDepth(state),
      //         svo::seed::getInvMinDepth(state), svo::seed::getInvMaxDepth(state), px);
      //     frames.back()->px_vec_.col(k) = px;
      //   }
      // }
    }

    if (initialized_) {
      svo::feature_detection_utils::drawFeatures(*frames.back(), 0u, false,
                                            &img_rgb);
      string filename = dataset_dir + "/features_" + to_string(i) + ".png";
      cv::imwrite(filename, img_rgb);
    }

    current_time += 100e6;
    last_frame = frames.back();
    cout << "Processed number " << i << " image." << endl;
  }

  // Output
  std::ofstream ofs;
  // write ply file for pointcloud visualization in Meshlab
  string trace_name = dataset_dir + "/depth_filter_" + experiment_name + ".ply";
  ofs.open(trace_name.c_str());
  ofs << "ply" << std::endl
      << "format ascii 1.0" << std::endl
	  << "element vertex " << key_frame->num_features_ << std::endl
	  << "property float x" << std::endl
	  << "property float y" << std::endl
	  << "property float z" << std::endl
	  << "property uchar blue" << std::endl
	  << "property uchar green" << std::endl
	  << "property uchar red" << std::endl
	  << "end_header" << std::endl;
  for (size_t i = 0; i < key_frame->num_features_; i++) {
    Eigen::Vector3d p = key_frame->T_world_cam() * key_frame->getSeedPosInFrame(i);
    ofs << p[0] << " " << p[1] << " " << p[2] << " "
        << (int) 0 << " " << (int) 0 << " " << (int) 0 << std::endl;
  }
}

}  // namespace

int main(int argc, char** argv)
{
  DepthFilterTest test;
  std::string dataset_dir("/home/cc/linux/softwares/feature/feature_estimator/third_party/rpg_svo/test/data");
  std::string experiment_name("depth_filter");
  test.testReconstruction(dataset_dir, experiment_name);
  return 0;
}
