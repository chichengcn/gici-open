/**
* @Function: Phase-range residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/phaserange_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/global_variable.h"
#include "gici/gnss/code_phase_maps.h"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PhaserangeError<Ns ...>::PhaserangeError(
                       const GnssMeasurement& measurement,
                       const GnssMeasurementIndex index,
                       const GnssErrorParameter& error_parameter)
{
  setMeasurement(measurement);
  satellite_ = measurement_.getSat(index);
  observation_ = measurement_.getObs(index);

  // Check parameter block types
  // Group 1
  if (dims_.kNumParameterBlocks == 5 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1) {
    is_estimate_body_ = false;
    parameter_block_group_ = 1;
  }
  // Group 2
  else if (dims_.kNumParameterBlocks == 6 && 
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1 && dims_.GetDim(5) == 1) {
    is_estimate_body_ = true;
    parameter_block_group_ = 2;
  }
  else {
    LOG(FATAL) << "PhaserangeError parameter blocks setup invalid!";
  }

  setInformation(error_parameter);
}

// Set the information.
template<int... Ns>
void PhaserangeError<Ns ...>::setInformation(const GnssErrorParameter& error_parameter)
{
  // compute variance
  error_parameter_ = error_parameter;
  Eigen::Vector3d factor;
  for (size_t i = 0; i < 3; i++) factor(i) = error_parameter_.phase_error_factor[i];
  double elevation = gnss_common::satelliteElevation(
    satellite_.sat_position, measurement_.position);
  double covariance = square(factor(0)) + square(factor(1) / sin(elevation));
  char system = satellite_.getSystem();
  covariance *= square(error_parameter_.system_error_ratio.at(system));
  // check precise ephemeris
  if (satellite_.sat_type != SatEphType::Precise) {
    covariance += square(error_parameter_.ephemeris_broadcast);
  }
  // add IFCB residual error for GPS L5
  if (satellite_.getSystem() == 'G' && checkEqual(observation_.wavelength, 
      CLIGHT / gnss_common::phaseToFrequency('G', PHASE_L5))) {
    covariance += square(error_parameter_.residual_gps_ifcb);
  }
  covariance_ = covariance_t(covariance);

  information_ = covariance_.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
  square_root_information_inverse_ = square_root_information_.inverse();
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool PhaserangeError<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PhaserangeError<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  double clock;
  double troposphere_delay = 0.0, ionosphere_delay = 0.0;
  double ambiguity;
  double gmf_wet, gmf_hydro;
  double phase_windup;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
    clock = parameters[1][0];
    ambiguity = parameters[2][0];
  }
  else 
  {
    // pose in ENU frame
    t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
    q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

    // relative position
    t_SR_S = Eigen::Map<const Eigen::Vector3d>(parameters[1]);

    // clock
    clock = parameters[2][0];

    // ambiguity
    ambiguity = parameters[3][0];

    // receiver position
    Eigen::Vector3d t_WR_W = t_WS_W + q_WS * t_SR_S;

    if (!coordinate_) {
      LOG(FATAL) << "Coordinate not set!";
    }
    if (!coordinate_->isZeroSetted()) {
      LOG(FATAL) << "Coordinate zero not set!";
    }
    t_WR_ECEF = coordinate_->convert(t_WR_W, GeoType::ENU, GeoType::ECEF);
  }

  // Earth tide
  double timestamp = measurement_.timestamp;
  Eigen::Vector3d tide = gnss_common::solidEarthTide(timestamp, t_WR_ECEF);
  t_WR_ECEF += tide;

  double rho = gnss_common::satelliteToReceiverDistance(
    satellite_.sat_position, t_WR_ECEF);
  double elevation = gnss_common::satelliteElevation(
    satellite_.sat_position, t_WR_ECEF);
  double azimuth = gnss_common::satelliteAzimuth(
    satellite_.sat_position, t_WR_ECEF);

  // Atmosphere
  double zwd = 0.0;
  if (!is_estimate_body_) {
    zwd = parameters[3][0];
    ionosphere_delay = parameters[4][0];
  }
  else {
    zwd = parameters[4][0];
    ionosphere_delay = parameters[5][0];
  }

  // troposphere hydro-static delay
  double zhd = gnss_common::troposphereSaastamoinen(timestamp, t_WR_ECEF, PI / 2.0);
  // mapping
  gnss_common::troposphereGMF(timestamp, t_WR_ECEF, elevation, &gmf_hydro, &gmf_wet);
  troposphere_delay = zhd * gmf_hydro + zwd * gmf_wet;

  // ionosphere delay to current frequency
  ionosphere_delay = gnss_common::ionosphereConvertFromBase(
    ionosphere_delay, observation_.wavelength);

  // phase wind-up
  phase_windup = measurement_.phase_windup->get(
    timestamp, satellite_.prn, satellite_.sat_position, t_WR_ECEF);
  phase_windup *= observation_.wavelength;

  // Get estimate derivated measurement
  double phaserange_estimate = rho + clock - satellite_.sat_clock
   + troposphere_delay - ionosphere_delay + ambiguity + phase_windup;

  // Compute error
  double phaserange = observation_.phaserange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(phaserange - phaserange_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Receiver position in ECEF
    Eigen::Matrix<double, 1, 3> J_t_ECEF = 
      -((t_WR_ECEF - satellite_.sat_position) / rho).transpose();
    
    // Poses
    Eigen::Matrix<double, 1, 6> J_T_WS;
    Eigen::Matrix<double, 1, 3> J_t_SR_S;
    if (is_estimate_body_) {
      // Body position in ENU
      Eigen::Matrix<double, 1, 3> J_t_W = J_t_ECEF * 
        coordinate_->rotationMatrix(GeoType::ENU, GeoType::ECEF);

      // Body rotation in ENU
      Eigen::Matrix<double, 1, 3> J_q_WS = J_t_W * 
        -skewSymmetric(q_WS.toRotationMatrix() * t_SR_S);

      // Body pose in ENU
      J_T_WS.setZero();
      J_T_WS.topLeftCorner(1, 3) = J_t_W;
      J_T_WS.topRightCorner(1, 3) = J_q_WS;

      // Relative position 
      J_t_SR_S = J_t_W * q_WS.toRotationMatrix();
    }

    // Clock
    Eigen::Matrix<double, 1, 1> J_clock = -Eigen::MatrixXd::Identity(1, 1);

    // Ambiguity
    Eigen::Matrix<double, 1, 1> J_amb = -Eigen::MatrixXd::Identity(1, 1);

    // Troposphere
    Eigen::Matrix<double, 1, 1> J_trop = 
      -gmf_wet * Eigen::MatrixXd::Identity(1, 1);

    // Ionosphere
    Eigen::Matrix<double, 1, 1> J_iono = Eigen::MatrixXd::Identity(1, 1) * 
      gnss_common::ionosphereConvertFromBase(1.0, observation_.wavelength);

    // Group 1
    if (parameter_block_group_ == 1) 
    {
      // Position
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J0(jacobians[0]);
        J0 = square_root_information_ * J_t_ECEF;
        
        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0;
        }
      }
      // Clock
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = square_root_information_ * J_clock;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      // Ambiguity
      if (jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = square_root_information_ * J_amb;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Troposphere
      if (jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Ionosphere
      if (jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information_ * J_iono;

        if (jacobians_minimal != nullptr && jacobians_minimal[4] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J4_minimal_mapped(jacobians_minimal[4]);
          J4_minimal_mapped = J4;
        }
      }
    }
    // Group 2
    if (parameter_block_group_ == 2)
    {
      // Pose
      if (jacobians[0] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J0(jacobians[0]);
        Eigen::Matrix<double, 1, 6, Eigen::RowMajor> J0_minimal;
        J0_minimal = square_root_information_ * J_T_WS;

        // pseudo inverse of the local parametrization Jacobian:
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        J0 = J0_minimal * J_lift;

        if (jacobians_minimal != nullptr && jacobians_minimal[0] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor> >
              J0_minimal_mapped(jacobians_minimal[0]);
          J0_minimal_mapped = J0_minimal;
        }
      }
      // Relative position
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = square_root_information_ * J_t_SR_S;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      // Clock
      if (jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = square_root_information_ * J_clock;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Ambiguity
      if (jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_amb;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Troposphere
      if (jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information_ * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[4] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J4_minimal_mapped(jacobians_minimal[4]);
          J4_minimal_mapped = J4;
        }
      }
      // Ionosphere
      if (jacobians[5] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J5(jacobians[5]);
        J5 = square_root_information_ * J_iono;

        if (jacobians_minimal != nullptr && jacobians_minimal[5] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J5_minimal_mapped(jacobians_minimal[5]);
          J5_minimal_mapped = J5;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
