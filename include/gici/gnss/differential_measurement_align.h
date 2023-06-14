/**
* @Function: Align differential measurements
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include <deque>

#include "gici/gnss/gnss_types.h"
#include "gici/estimate/estimator_types.h"
#include "gici/utility/common.h"

namespace gici {

// Estimator
class DifferentialMeasurementsAlign {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The default constructor
  DifferentialMeasurementsAlign() {}
  ~DifferentialMeasurementsAlign() {}

  // Set measurements
  inline void add(const EstimatorDataCluster& measurement) {
    if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
      measurement_rov_.push_back(*measurement.gnss);
    }
    if (measurement.gnss && measurement.gnss_role == GnssRole::Reference) {
      measurement_ref_.push_back(*measurement.gnss);
    }
  }

  // Get aligned
  inline bool get(const double max_age, 
    GnssMeasurement& rov, GnssMeasurement& ref) {

    const double offset = 0.0;  // shift a time offset for test

    if (measurement_rov_.size() == 0) return false;
    if (measurement_ref_.size() == 0) {
      measurement_rov_.clear(); return false;
    }

    rov = measurement_rov_.front();
    measurement_rov_.pop_front();

    // get the nearest timestamp
    double min_dt = 1.0e6;
    int index = -1;
    for (int i = measurement_ref_.size() - 1; i >= 0; i--) {
      double dt = fabs(rov.timestamp - measurement_ref_.at(i).timestamp - offset);
      if (min_dt > dt) {
        min_dt = dt; index = i;
      }
    }
    CHECK(index != -1);
    ref = measurement_ref_.at(index);
    // pop all in front of this
    double cut_timestamp = ref.timestamp;
    while (!checkEqual(cut_timestamp, measurement_ref_.front().timestamp)) {
      measurement_ref_.pop_front();
    }

    // Check timestamps
    if (!checkEqual(rov.timestamp, ref.timestamp, max_age)) {
      LOG(WARNING) << "Max age between two measurements exceeded! "
        << "age = " << fabs(rov.timestamp - ref.timestamp)
        << ", max_age = " << max_age;
      return false;
    }

    return true;
  }

protected:
  // Measurement storage for aligning
  std::deque<GnssMeasurement> measurement_rov_;
  std::deque<GnssMeasurement> measurement_ref_;
};

}