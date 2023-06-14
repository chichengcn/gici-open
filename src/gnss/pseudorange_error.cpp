/**
* @Function: Pseudorange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/pseudorange_error.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"
#include "gici/utility/global_variable.h"
#include "gici/gnss/code_phase_maps.h"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PseudorangeError<Ns ...>::PseudorangeError(
                       const GnssMeasurement& measurement,
                       const GnssMeasurementIndex index,
                       const GnssErrorParameter& error_parameter)
{
  setMeasurement(measurement);
  satellite_ = measurement_.getSat(index);
  observation_ = measurement_.getObs(index);

  // Check parameter block types
  // Group 1
  if (dims_.kNumParameterBlocks == 2 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 1;
  }
  // Group 2
  else if (dims_.kNumParameterBlocks == 3 &&
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 2;
  }
  // Group 3
  else if (dims_.kNumParameterBlocks == 5 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 3;
  }
  // Group 4
  else if (dims_.kNumParameterBlocks == 6 && 
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1 && dims_.GetDim(5) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 4;
  }
  else {
    LOG(FATAL) << "PseudorangeError parameter blocks setup invalid!";
  }

  setInformation(error_parameter);
}

// Set the information.
template<int... Ns>
void PseudorangeError<Ns ...>::setInformation(const GnssErrorParameter& error_parameter)
{
  // compute variance
  error_parameter_ = error_parameter;
  Eigen::Vector3d factor;
  for (size_t i = 0; i < 3; i++) factor(i) = error_parameter_.phase_error_factor[i];
  double ratio = square(error_parameter_.code_to_phase_ratio);
  double elevation = gnss_common::satelliteElevation(
    satellite_.sat_position, measurement_.position);
  double azimuth = gnss_common::satelliteAzimuth(
    satellite_.sat_position, measurement_.position);
  double timestamp = measurement_.timestamp;

  double ephemeris_var, ionosphere_var, troposphere_var;
  if (!is_estimate_atmosphere_) {
    // ephemeris error
    if (satellite_.sat_type == SatEphType::Broadcast) {
      ephemeris_var = square(error_parameter_.ephemeris_broadcast);
    }
    else if (satellite_.sat_type == SatEphType::Precise) {
      ephemeris_var = square(error_parameter_.ephemeris_precise);
    }
    // ionosphere error
    if (satellite_.ionosphere != 0.0 && 
        satellite_.ionosphere_type == IonoType::Augmentation) {
      ionosphere_var = square(error_parameter_.ionosphere_augment);
    }
    else if (satellite_.ionosphere != 0.0 && 
             satellite_.ionosphere_type == IonoType::DualFrequency) {
      ionosphere_var = square(error_parameter_.ionosphere_dual_frequency);
    }
    else {
      double ionosphere_delay = gnss_common::ionosphereBroadcast(
        timestamp, measurement_.position, azimuth, elevation, 
        observation_.wavelength, measurement_.ionosphere_parameters);
      ionosphere_var = square(
        error_parameter_.ionosphere_broadcast_factor * ionosphere_delay);
    }
    // troposphere error
    double troposphere_delay = gnss_common::troposphereSaastamoinen(
      timestamp, measurement_.position, elevation);
    troposphere_var = square(
      error_parameter_.troposphere_model_factor * troposphere_delay);
    // troposphere wet delay
    if (measurement_.troposphere_wet != 0.0) {
      troposphere_var = square(error_parameter_.troposphere_augment);
    }
  }
  else {
    // we do not add ephemeris error in precise positioning case
    // because the precise ephemeris is highly time-correlated, which
    // cannot be treated as white noise.
    if (satellite_.sat_type == SatEphType::Broadcast) {
      ephemeris_var = square(error_parameter_.ephemeris_broadcast);
    }
    else if (satellite_.sat_type == SatEphType::Precise) {
      ephemeris_var = 0.0;
    }
    // ionosphere error
    ionosphere_var = 0.0;
    // troposphere error
    troposphere_var = 0.0; 
  }

  double covariance = (square(factor(0)) + square(factor(1) / sin(elevation))) * ratio + 
    ephemeris_var + ionosphere_var + troposphere_var;
  char system = satellite_.getSystem();
  covariance *= square(error_parameter_.system_error_ratio.at(system));
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
bool PseudorangeError<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PseudorangeError<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  double clock, ifb;
  double troposphere_delay, ionosphere_delay;
  double gmf_wet, gmf_hydro;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
    clock = parameters[1][0];
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
  if (!is_estimate_atmosphere_) 
  {
    // Inter-Frequency Bias (IFB)
    ifb = 0.0;

    // troposphere hydro-static delay
    troposphere_delay = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, elevation);
    // troposphere wet delay
    if (measurement_.troposphere_wet != 0.0) {
      gnss_common::troposphereGMF(timestamp, t_WR_ECEF, elevation, nullptr, &gmf_wet);
      troposphere_delay += measurement_.troposphere_wet * gmf_wet;
    }

    // ionosphere
    if (satellite_.ionosphere != 0.0 && 
        satellite_.ionosphere_type == IonoType::Augmentation) {
      ionosphere_delay = satellite_.ionosphere;
      ionosphere_delay = gnss_common::ionosphereConvertFromBase(
        ionosphere_delay, observation_.wavelength);
    }
    else if (satellite_.ionosphere != 0.0 && 
             satellite_.ionosphere_type == IonoType::DualFrequency) {
      ionosphere_delay = satellite_.ionosphere;
      ionosphere_delay = gnss_common::ionosphereConvertFromBase(
        ionosphere_delay, observation_.wavelength);
    }
    else {
      ionosphere_delay = gnss_common::ionosphereBroadcast(timestamp, t_WR_ECEF, 
          azimuth, elevation, observation_.wavelength, measurement_.ionosphere_parameters);
    }
  }
  // use estimated atomspheric delays
  else
  { 
    double zwd = 0.0;
    if (!is_estimate_body_) {
      ifb = parameters[2][0];
      zwd = parameters[3][0];
      ionosphere_delay = parameters[4][0];
    }
    else {
      ifb = parameters[3][0];
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
  }

  // Get estimate derivated measurement
  double pseudorange_estimate = rho + clock - 
    satellite_.sat_clock + ifb + troposphere_delay + ionosphere_delay;

  // Compute error
  double pseudorange = observation_.pseudorange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(pseudorange - pseudorange_estimate);

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

    // IFB
    Eigen::Matrix<double, 1, 1> J_ifb = -Eigen::MatrixXd::Identity(1, 1);

    // Troposphere
    Eigen::Matrix<double, 1, 1> J_trop = -gmf_wet * Eigen::MatrixXd::Identity(1, 1);

    // Ionosphere
    Eigen::Matrix<double, 1, 1> J_iono = -Eigen::MatrixXd::Identity(1, 1) * 
      gnss_common::ionosphereConvertFromBase(1.0, observation_.wavelength);

    // Group 1
    if (parameter_block_group_ == 1 || parameter_block_group_ == 3) 
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
      // IFB
      if (is_estimate_atmosphere_ && jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = square_root_information_ * J_ifb;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Troposphere
      if (is_estimate_atmosphere_ && jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Ionosphere
      if (is_estimate_atmosphere_ && jacobians[4] != nullptr) {
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
    if (parameter_block_group_ == 2 || parameter_block_group_ == 4)
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
      // IFB
      if (is_estimate_atmosphere_ && jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_ifb;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Troposphere
      if (is_estimate_atmosphere_ && jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information_ * J_trop;

        if (jacobians_minimal != nullptr && jacobians_minimal[4] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J4_minimal_mapped(jacobians_minimal[4]);
          J4_minimal_mapped = J4;
        }
      }
      // Ionosphere
      if (is_estimate_atmosphere_ && jacobians[5] != nullptr) {
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
