/**
* @Function: Single-differenced (between stations) phaserange residual block for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/phaserange_error_dd.h"

#include "gici/gnss/gnss_common.h"
#include "gici/utility/transform.h"
#include "gici/estimate/pose_local_parameterization.h"

namespace gici {

// Construct with measurement and information matrix
template<int... Ns>
PhaserangeErrorDD<Ns ...>::PhaserangeErrorDD(
                    const GnssMeasurement& measurement_rov,
                    const GnssMeasurement& measurement_ref,
                    const GnssMeasurementIndex index_rov,
                    const GnssMeasurementIndex index_ref,
                    const GnssMeasurementIndex index_rov_base,
                    const GnssMeasurementIndex index_ref_base,
                    const GnssErrorParameter& error_parameter)
{
  CHECK(!checkZero(measurement_ref.position)) << 
    "The position of reference station is not setted!";

  setMeasurement(measurement_rov, measurement_ref);

  satellite_rov_ = measurement_rov_.getSat(index_rov);
  satellite_ref_ = measurement_ref_.getSat(index_ref);
  satellite_rov_base_ = measurement_rov_.getSat(index_rov_base);
  satellite_ref_base_ = measurement_ref_.getSat(index_ref_base);

  observation_rov_ = measurement_rov_.getObs(index_rov);
  observation_ref_ = measurement_ref_.getObs(index_ref);
  observation_rov_base_ = measurement_rov_.getObs(index_rov_base);
  observation_ref_base_ = measurement_ref_.getObs(index_ref_base);

  // Check parameter block types
  // Group 1
  if (dims_.kNumParameterBlocks == 3 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1 &&
      dims_.GetDim(2) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 1;
  }
  // Group 2
  else if (dims_.kNumParameterBlocks == 4 &&
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = false;
    parameter_block_group_ = 2;
  }
  // Group 3
  else if (dims_.kNumParameterBlocks == 7 && 
      dims_.GetDim(0) == 3 && dims_.GetDim(1) == 1 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 && 
      dims_.GetDim(4) == 1 && dims_.GetDim(5) == 1 &&
      dims_.GetDim(6) == 1) {
    is_estimate_body_ = false;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 3;
  }
  // Group 4
  else if (dims_.kNumParameterBlocks == 8 && 
      dims_.GetDim(0) == 7 && dims_.GetDim(1) == 3 &&
      dims_.GetDim(2) == 1 && dims_.GetDim(3) == 1 &&
      dims_.GetDim(4) == 1 && dims_.GetDim(5) == 1 &&
      dims_.GetDim(6) == 1 && dims_.GetDim(7) == 1) {
    is_estimate_body_ = true;
    is_estimate_atmosphere_ = true;
    parameter_block_group_ = 4;
  }
  else {
    LOG(FATAL) << "PhaserangeErrorDD parameter blocks setup invalid!";
  }

  setInformation(error_parameter);
}

// Set the information.
template<int... Ns>
void PhaserangeErrorDD<Ns ...>::setInformation(const GnssErrorParameter& error_parameter)
{
  // compute variance
  error_parameter_ = error_parameter;
  Eigen::Vector3d factor;
  for (size_t i = 0; i < 3; i++) factor(i) = error_parameter_.phase_error_factor[i];
  double elevation_rov = gnss_common::satelliteElevation(
    satellite_rov_.sat_position, measurement_rov_.position);
  covariance_ = covariance_t(
    (square(factor(0)) + square(factor(1) / sin(elevation_rov))) * 4.0);
  char system = satellite_rov_.getSystem();
  covariance_ *= square(error_parameter_.system_error_ratio.at(system));

  information_ = covariance_.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
  square_root_information_inverse_ = square_root_information_.inverse();
}

// This evaluates the error term and additionally computes the Jacobians.
template<int... Ns>
bool PhaserangeErrorDD<Ns ...>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const
{
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template<int... Ns>
bool PhaserangeErrorDD<Ns ...>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  Eigen::Vector3d t_WR_ECEF, t_WS_W, t_SR_S;
  Eigen::Quaterniond q_WS;
  double dtroposphere_delay = 0.0, dionosphere_delay = 0.0;
  double gmf_wet_rov, gmf_wet_ref, gmf_hydro_rov, gmf_hydro_ref;
  double gmf_wet_rov_base, gmf_wet_ref_base, gmf_hydro_rov_base, gmf_hydro_ref_base;
  double dambiguity, dambiguity_base;
  
  // Position and clock
  if (!is_estimate_body_) 
  {
    t_WR_ECEF = Eigen::Map<const Eigen::Vector3d>(parameters[0]);
    dambiguity = parameters[1][0];
    dambiguity_base = parameters[2][0];
  }
  else 
  {
    // pose in ENU frame
    t_WS_W = Eigen::Map<const Eigen::Vector3d>(&parameters[0][0]);
    q_WS = Eigen::Map<const Eigen::Quaterniond>(&parameters[0][3]);

    // relative position
    t_SR_S = Eigen::Map<const Eigen::Vector3d>(parameters[1]);

    // receiver position
    Eigen::Vector3d t_WR_W = t_WS_W + q_WS * t_SR_S;

    // ambiguity
    dambiguity = parameters[2][0];
    dambiguity_base = parameters[3][0];

    if (!coordinate_) {
      LOG(FATAL) << "Coordinate not set!";
    }
    if (!coordinate_->isZeroSetted()) {
      LOG(FATAL) << "Coordinate zero not set!";
    }
    t_WR_ECEF = coordinate_->convert(t_WR_W, GeoType::ENU, GeoType::ECEF);
  }

  double timestamp = measurement_rov_.timestamp;

  double rho_rov = gnss_common::satelliteToReceiverDistance(
    satellite_rov_.sat_position, t_WR_ECEF);
  double rho_ref = gnss_common::satelliteToReceiverDistance(
    satellite_ref_.sat_position, measurement_ref_.position);
  double rho_rov_base = gnss_common::satelliteToReceiverDistance(
    satellite_rov_base_.sat_position, t_WR_ECEF);
  double rho_ref_base = gnss_common::satelliteToReceiverDistance(
    satellite_ref_base_.sat_position, measurement_ref_.position);

  double elevation_rov = gnss_common::satelliteElevation(
    satellite_rov_.sat_position, t_WR_ECEF);
  double elevation_ref = gnss_common::satelliteElevation(
    satellite_ref_.sat_position, measurement_ref_.position);
  double elevation_rov_base = gnss_common::satelliteElevation(
    satellite_rov_base_.sat_position, t_WR_ECEF);
  double elevation_ref_base = gnss_common::satelliteElevation(
    satellite_ref_base_.sat_position, measurement_ref_.position);

  // Atmosphere
  if (!is_estimate_atmosphere_) 
  {
    // We think all atmosphere delays are eliminated by double-difference
  }
  else
  { 
    // use estimated atomspheric delays
    double zwd_rov = 0.0, zwd_ref = 0.0;
    double dionosphere_delay_cur = 0.0, dionosphere_delay_base = 0.0;
    if (!is_estimate_body_) {
      zwd_rov = parameters[3][0];
      zwd_ref = parameters[4][0];
      dionosphere_delay_cur = parameters[5][0];
      dionosphere_delay_base = parameters[6][0];
    }
    else {
      zwd_rov = parameters[4][0];
      zwd_ref = parameters[5][0];
      dionosphere_delay_cur = parameters[6][0];
      dionosphere_delay_base = parameters[7][0];
    }

    // troposphere hydro-static delay
    double zhd_rov = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, PI / 2.0);
    double zhd_ref = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, PI / 2.0);
    double zhd_rov_base = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, PI / 2.0);
    double zhd_ref_base = gnss_common::troposphereSaastamoinen(
      timestamp, t_WR_ECEF, PI / 2.0);
    // mapping
    gnss_common::troposphereGMF(
      timestamp, t_WR_ECEF, elevation_rov, &gmf_hydro_rov, &gmf_wet_rov);
    gnss_common::troposphereGMF(
      timestamp, t_WR_ECEF, elevation_ref, &gmf_hydro_ref, &gmf_wet_ref);
    gnss_common::troposphereGMF(
      timestamp, t_WR_ECEF, elevation_rov_base, &gmf_hydro_rov_base, &gmf_wet_rov_base);
    gnss_common::troposphereGMF(
      timestamp, t_WR_ECEF, elevation_ref_base, &gmf_hydro_ref_base, &gmf_wet_ref_base);
    dtroposphere_delay = zhd_rov * gmf_hydro_rov - zhd_ref * gmf_hydro_ref - 
      zhd_rov_base * gmf_hydro_rov_base + zhd_ref_base * gmf_hydro_ref_base + 
      zwd_rov * gmf_wet_rov - zwd_ref * gmf_wet_ref - 
      zwd_rov * gmf_wet_rov_base + zwd_ref * gmf_wet_ref_base;

    // ionosphere
    dionosphere_delay_cur = gnss_common::ionosphereConvertFromBase(
      dionosphere_delay_cur, observation_rov_.wavelength);
    dionosphere_delay_base = gnss_common::ionosphereConvertFromBase(
      dionosphere_delay_base, observation_rov_base_.wavelength);
    dionosphere_delay = dionosphere_delay_cur - dionosphere_delay_base;
  }

  // Get estimate derivated measurement
  double dphaserange_estimate = rho_rov - rho_ref - rho_rov_base + rho_ref_base
   + dtroposphere_delay - dionosphere_delay + dambiguity - dambiguity_base;

  // Compute error
  double dphaserange = observation_rov_.phaserange - observation_ref_.phaserange -
    observation_rov_base_.phaserange + observation_ref_base_.phaserange;
  Eigen::Matrix<double, 1, 1> error = 
    Eigen::Matrix<double, 1, 1>(dphaserange - dphaserange_estimate);

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 1, 1> > weighted_error(residuals);
  weighted_error = square_root_information_ * error;

  // compute Jacobian
  if (jacobians != nullptr)
  {
    // Receiver position in ECEF
    Eigen::Matrix<double, 1, 3> J_t_ECEF = 
      -((t_WR_ECEF - satellite_rov_.sat_position) / rho_rov).transpose() + 
       ((t_WR_ECEF - satellite_rov_base_.sat_position) / rho_rov_base).transpose();
    
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

    // Ambiguity
    Eigen::Matrix<double, 1, 1> J_amb = -Eigen::MatrixXd::Identity(1, 1);
    Eigen::Matrix<double, 1, 1> J_amb_base = Eigen::MatrixXd::Identity(1, 1);

    // Troposphere
    Eigen::Matrix<double, 1, 1> J_trop_rov = 
      -(gmf_wet_rov - gmf_wet_rov_base) * Eigen::MatrixXd::Identity(1, 1);
    Eigen::Matrix<double, 1, 1> J_trop_ref = 
      (gmf_wet_ref - gmf_wet_ref_base) * Eigen::MatrixXd::Identity(1, 1);

    // Ionosphere
    Eigen::Matrix<double, 1, 1> J_iono = Eigen::MatrixXd::Identity(1, 1) * 
      gnss_common::ionosphereConvertFromBase(1.0, observation_rov_.wavelength);
    Eigen::Matrix<double, 1, 1> J_iono_base = -Eigen::MatrixXd::Identity(1, 1) * 
      gnss_common::ionosphereConvertFromBase(1.0, observation_rov_base_.wavelength);

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
      // Ambiguity
      if (jacobians[1] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J1(jacobians[1]);
        J1 = square_root_information_ * J_amb;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J1_minimal_mapped(jacobians_minimal[1]);
          J1_minimal_mapped = J1;
        }
      }
      // Ambiguity for base satellite
      if (jacobians[2] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J2(jacobians[2]);
        J2 = square_root_information_ * J_amb_base;

        if (jacobians_minimal != nullptr && jacobians_minimal[2] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J2_minimal_mapped(jacobians_minimal[2]);
          J2_minimal_mapped = J2;
        }
      }
      // Troposphere at rov
      if (is_estimate_atmosphere_ && jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_trop_rov;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Troposphere at ref
      if (is_estimate_atmosphere_ && jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information_ * J_trop_ref;

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
      // Ionosphere at base satellite
      if (is_estimate_atmosphere_ && jacobians[6] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J6(jacobians[6]);
        J6 = square_root_information_ * J_iono_base;

        if (jacobians_minimal != nullptr && jacobians_minimal[6] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J6_minimal_mapped(jacobians_minimal[6]);
          J6_minimal_mapped = J6;
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
        J1 = square_root_information_* J_t_SR_S;

        if (jacobians_minimal != nullptr && jacobians_minimal[1] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor> >
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
      // Ambiguity of base satellite
      if (jacobians[3] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J3(jacobians[3]);
        J3 = square_root_information_ * J_amb_base;

        if (jacobians_minimal != nullptr && jacobians_minimal[3] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J3_minimal_mapped(jacobians_minimal[3]);
          J3_minimal_mapped = J3;
        }
      }
      // Troposphere at rov
      if (is_estimate_atmosphere_ && jacobians[4] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J4(jacobians[4]);
        J4 = square_root_information_ * J_trop_rov;

        if (jacobians_minimal != nullptr && jacobians_minimal[4] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J4_minimal_mapped(jacobians_minimal[4]);
          J4_minimal_mapped = J4;
        }
      }
      // Troposphere at ref
      if (is_estimate_atmosphere_ && jacobians[5] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J5(jacobians[5]);
        J5 = square_root_information_ * J_trop_ref;

        if (jacobians_minimal != nullptr && jacobians_minimal[5] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J5_minimal_mapped(jacobians_minimal[5]);
          J5_minimal_mapped = J5;
        }
      }
      // Ionosphere
      if (is_estimate_atmosphere_ && jacobians[6] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J6(jacobians[6]);
        J6 = square_root_information_ * J_iono;

        if (jacobians_minimal != nullptr && jacobians_minimal[6] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J6_minimal_mapped(jacobians_minimal[6]);
          J6_minimal_mapped = J6;
        }
      }
      // Ionosphere at base satellite
      if (is_estimate_atmosphere_ && jacobians[7] != nullptr) {
        Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J7(jacobians[7]);
        J7 = square_root_information_ * J_iono_base;

        if (jacobians_minimal != nullptr && jacobians_minimal[7] != nullptr) {
          Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor> >
              J7_minimal_mapped(jacobians_minimal[7]);
          J7_minimal_mapped = J7;
        }
      }
    }
  }

  return true;
}

}  // namespace gici
