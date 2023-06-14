/**
* @Function: SVO library
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <svo/svo.h>
#include <svo/tracker/feature_tracking_utils.h>
#include <svo/img_align/sparse_img_align.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/direct/feature_alignment.h>
#include <svo/direct/patch_warp.h>
#include <svo/direct/patch_score.h>
#include <svo/direct/patch_utils.h>

// We do not directly apply (using namespace gici) here to 
// avoid some naming conflit when reforming some features
// of svo.
namespace gici {

// Common
namespace frame_utils = svo::frame_utils;
namespace warp = svo::warp;
namespace patch_utils = svo::patch_utils;
namespace feature_detection_utils = svo::feature_detection_utils;
namespace reprojector_utils = svo::reprojector_utils;
namespace feature_alignment = svo::feature_alignment;

using Frame = svo::Frame;
using FramePtr = svo::FramePtr;
using Keypoint = svo::Keypoint;
using DetectorOptions = svo::DetectorOptions;
using ReprojectorOptions = svo::ReprojectorOptions;
using FrameBundle = svo::FrameBundle;
using FrameBundlePtr = svo::FrameBundlePtr;
using Camera = svo::Camera;
using CameraPtr = svo::CameraPtr;
using CameraBundle = svo::CameraBundle;
using CameraBundlePtr = svo::CameraBundlePtr;
using Transformation = svo::Transformation;
using Bearings = svo::Bearings;
using Positions = svo::Positions;
using Point = svo::Point;
using PointPtr = svo::PointPtr;
using FeatureType = svo::FeatureType;
using BearingVector = svo::BearingVector;
using FeatureWrapper = svo::FeatureWrapper;
using FloatType = svo::FloatType;
using GradientVector = svo::GradientVector;
using SeedState = svo::SeedState;
using DetectorPtr = svo::DetectorPtr;
using DetectorType = svo::DetectorType;
using AbstractDetector = svo::AbstractDetector;
using Keypoints = svo::Keypoints;
using Scores = svo::Scores;
using Levels = svo::Levels;
using Gradients = svo::Gradients;
using FeatureTypes = svo::FeatureTypes;
using OccupandyGrid2D = svo::OccupandyGrid2D;
using Point = svo::Point;
using PointPtr = svo::PointPtr;
using BundleId = svo::BundleId;
using Position = svo::Position;
using KeypointIdentifier = svo::KeypointIdentifier;
using PerformanceMonitorPtr = svo::PerformanceMonitorPtr;
using EnumClassHash = svo::EnumClassHash;
using Quaternion = svo::Quaternion;
using ReprojectorPtr = svo::ReprojectorPtr;
using Reprojector = svo::Reprojector;
using DetectorPtr = svo::DetectorPtr;
using SeedRef = svo::SeedRef;
using GradientVector = svo::GradientVector;
using CameraConstPtr = svo::CameraConstPtr;
using Map = svo::Map;
using MapPtr = svo::MapPtr;

inline void changeFeatureTypeToSeed(FeatureType& t) 
{
  if(t == FeatureType::kCorner)
    t = FeatureType::kCornerSeed;
  else if(t == FeatureType::kEdgelet)
    t = FeatureType::kEdgeletSeed;
  else if(t == FeatureType::kMapPoint)
    t = FeatureType::kMapPointSeed;
  else
    LOG(ERROR) << "Unknown feature types: " << static_cast<int>(t);
}

inline void changeFeatureTypeFromSeed(FeatureType& t) 
{
  if(t == FeatureType::kCornerSeed)
    t = FeatureType::kCorner;
  else if(t == FeatureType::kEdgeletSeed)
    t = FeatureType::kEdgelet;
  else if(t == FeatureType::kMapPointSeed)
    t = FeatureType::kMapPoint;
  else
    LOG(ERROR) << "Unknown feature types: " << static_cast<int>(t);
}

}
