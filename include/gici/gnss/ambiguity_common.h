/**
* @Function: Ambiguity common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/gnss/gnss_types.h"
#include "gici/estimate/estimator_types.h"
#include "gici/estimate/graph.h"

namespace gici {

// Cycle slip detection
// We apply Loss of Lock Indicator (LLI) detection, Geometry-Free (GF) detection 
// and Melbourne-Wubbena (MW) detection in default. If relative position is given, 
// we apply relative position assisted single frequency cycle slip detection.
void cycleSlipDetection(GnssMeasurement& measurement_pre, 
                        GnssMeasurement& measurement_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d position_cur = Eigen::Vector3d::Zero());

// Cycle slip detection after single difference
void cycleSlipDetectionSD(GnssMeasurement& measurement_rov_pre, 
                        GnssMeasurement& measurement_ref_pre, 
                        GnssMeasurement& measurement_rov_cur,
                        GnssMeasurement& measurement_ref_cur,
                        const GnssCommonOptions& options,
                        const Eigen::Vector3d position_pre = Eigen::Vector3d::Zero(),
                        const Eigen::Vector3d position_cur = Eigen::Vector3d::Zero());

// Cycle slip detection by Loss of Lock Indicator (LLI)
void cycleSlipDetectionLLI(GnssMeasurement& measurement_pre, 
                           GnssMeasurement& measurement_cur);

// Cycle slip detection by Melbourne-Wubbena (MW) combination
void cycleSlipDetectionMW(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold);
                        
// Cycle slip detection by Geometry-Free (GF) combination
void cycleSlipDetectionGF(GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double threshold);

// Cycle slip detection by relative position
void cycleSlipDetectionPosition(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          const Eigen::Vector3d position_pre,
                          const Eigen::Vector3d position_cur,
                          double threshold);

// Cycle slip detection by time gap for single frequency receiver
void cycleSlipDetectionTimeGap(
                          GnssMeasurement& measurement_pre, 
                          GnssMeasurement& measurement_cur,
                          double max_time_gap);

// Compute initial ambiguity for single differenced measurements
double getInitialAmbiguitySD(const GnssMeasurement& measurement_rov, 
                            const GnssMeasurement& measurement_ref,
                            const GnssMeasurementIndex& index_rov,
                            const GnssMeasurementIndex& index_ref);

// Solve integer ambiguity by LAMBDA
bool solveAmbiguityLambda(const Eigen::VectorXd& float_ambiguities,
                          const Eigen::MatrixXd& covariance, 
                          const double ratio_threshold, 
                          Eigen::VectorXd& fixed_ambiguities,
                          double& ratio);

}