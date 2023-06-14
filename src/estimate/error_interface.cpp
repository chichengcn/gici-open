#include "gici/estimate/error_interface.h"

namespace gici
{

const std::map<ErrorType, std::string> kErrorToStr
{
  {ErrorType::kHomogeneousPointError, std::string("HomogeneousPointError") },
  {ErrorType::kReprojectionError, std::string("ReprojectionError") },
  {ErrorType::kSpeedAndBiasError, std::string("SpeedAndBiasError") },
  {ErrorType::kMarginalizationError, std::string("MarginalizationError") },
  {ErrorType::kPoseError, std::string("PoseError") },
  {ErrorType::kRelativePoseError, std::string("kRelativePoseError") },
  {ErrorType::kIMUError, std::string("IMUError") },
  {ErrorType::kHMCError, std::string("HMCError") },
  {ErrorType::kNHCError, std::string("NHCError") },
  {ErrorType::kRelativePoseError, std::string("RelativePoseError") },
  {ErrorType::kPseudorangeError, std::string("PseudorangeError") },
  {ErrorType::kPseudorangeErrorSD, std::string("PseudorangeErrorSD") },
  {ErrorType::kPseudorangeErrorDD, std::string("PseudorangeErrorDD") },
  {ErrorType::kPhaserangeError, std::string("PhaserangeError") },
  {ErrorType::kPhaserangeErrorSD, std::string("PhaserangeErrorSD") },
  {ErrorType::kPhaserangeErrorDD, std::string("PhaserangeErrorDD") },
  {ErrorType::kDopplerError, std::string("DopplerError") },
  {ErrorType::kAmbiguityError, std::string("AmbiguityError") },
  {ErrorType::kPositionError, std::string("PositionError") },
  {ErrorType::kVelocityError, std::string("VelocityError") },
  {ErrorType::kClockError, std::string("ClockError") },
  {ErrorType::kFrequencyError, std::string("FrequencyError") },
  {ErrorType::kTroposphereError, std::string("TroposphereError") },
  {ErrorType::kIonosphereError, std::string("IonosphereError") },
  {ErrorType::kRelativePositionError, std::string("RelativePositionError") },
  {ErrorType::kRelativePositionAndVelocityError, std::string("RelativePositionAndVelocityError") },
  {ErrorType::kRelativeClockError, std::string("RelativeClockError") },
  {ErrorType::kRelativeFrequencyError, std::string("RelativeFrequencyError") },
  {ErrorType::kRelativeAmbiguityError, std::string("RelativeAmbiguityError") },
  {ErrorType::kRelativeTroposphereError, std::string("RelativeTroposphereError") },
  {ErrorType::kRelativeIonosphereError, std::string("RelativeIonosphereError") },
};

}
