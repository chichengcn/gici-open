/**
* @Function: Image types
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <iostream>
#include <vector>

#include "gici/utility/svo.h"

namespace gici {

// Role of formator
enum class CameraRole {
  None,
  Mono, 
  StereoMajor,
  StereoMinor,
  Array
};

/// \brief Struct to define the behavior of the camera extrinsics.
struct ExtrinsicsEstimationParameters
{
  // set to 0 in order to turn off
  /// \brief Default Constructor -- fixed camera extrinsics.
  ExtrinsicsEstimationParameters()
      : sigma_absolute_translation(0.0),
        sigma_absolute_orientation(0.0),
        sigma_c_relative_translation(0.0),
        sigma_c_relative_orientation(0.0)
  {
  }

  /**
   * @brief Constructor.
   * @param sigma_absolute_translation Absolute translation stdev. [m]
   * @param sigma_absolute_orientation Absolute orientation stdev. [rad]
   * @param sigma_c_relative_translation Relative translation noise density. [m/sqrt(Hz)]
   * @param sigma_c_relative_orientation Relative orientation noise density. [rad/sqrt(Hz)]
   */
  ExtrinsicsEstimationParameters(double sigma_absolute_translation,
                                 double sigma_absolute_orientation,
                                 double sigma_c_relative_translation,
                                 double sigma_c_relative_orientation)
      : sigma_absolute_translation(sigma_absolute_translation),
        sigma_absolute_orientation(sigma_absolute_orientation),
        sigma_c_relative_translation(sigma_c_relative_translation),
        sigma_c_relative_orientation(sigma_c_relative_orientation)
  {
  }

  inline bool isExtrinsicsFixed() const
  {
    return absoluteTranslationVar() < 1.0e-16 &&
        absoluteRotationVar() < 1.0e-16;
  }

  inline double absoluteTranslationVar() const
  {
    return sigma_absolute_translation * sigma_absolute_translation;
  }

  inline double absoluteRotationVar() const
  {
    return sigma_absolute_orientation * sigma_absolute_orientation;
  }

  // absolute (prior) w.r.t frame S
  double sigma_absolute_translation; ///< Absolute translation stdev. [m]
  double sigma_absolute_orientation; ///< Absolute orientation stdev. [rad]

  // relative (temporal)
  double sigma_c_relative_translation;
  ///< Relative translation noise density. [m/sqrt(Hz)]
  double sigma_c_relative_orientation;
  ///< Relative orientation noise density. [rad/sqrt(Hz)]
};

typedef std::vector<ExtrinsicsEstimationParameters,
                    Eigen::aligned_allocator<ExtrinsicsEstimationParameters> >
        ExtrinsicsEstimationParametersVec;

/**
 * @brief A type to store information about a point in the world map.
 */
struct MapPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor. Point is nullptr.
  MapPoint()
      : point(nullptr), fixed_position(false)
  {}
  /**
   * @brief Constructor.
   * @param point     Pointer to underlying svo::Point
   */
  MapPoint(const PointPtr& point)
    : point(point), fixed_position(false)
  {
    hom_coordinates << point->pos(), 1;
  }

  Eigen::Vector4d hom_coordinates; ///< Continuosly updates position of point


  //! Pointer to the point. The position is not updated inside backend
  //! because of possible multithreading conflicts.
  PointPtr point;

  //! Observations of this point. The uint64_t's are the casted
  //! ceres::ResidualBlockId values of the reprojection error residual block.
  std::map<KeypointIdentifier, uint64_t> observations;

  // Observation counter since the landmark has been initialized
  size_t num_observations_historical = 0;

  //! Is the point position fixed by a loop closure?
  bool fixed_position;
};
typedef std::vector<MapPoint, Eigen::aligned_allocator<MapPoint> > MapPointVector;
typedef std::map<BackendId, MapPoint, std::less<BackendId>,
  Eigen::aligned_allocator<std::pair<const BackendId, MapPoint>> > PointMap;

}
