/**
* @Function: GNSS relative constant errors for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/relative_const_error.h"
#include "gici/estimate/relative_integration_error.h"

namespace gici {

using RelativePositionError = RelativeConstError<3, ErrorType::kRelativePositionError>;
using RelativeClockError = RelativeConstError<1, ErrorType::kRelativeClockError>;
using RelativeFrequencyError = RelativeConstError<1, ErrorType::kRelativeFrequencyError>;
using RelativeAmbiguityError = RelativeConstError<1, ErrorType::kRelativeAmbiguityError>;
using RelativePositionAndVelocityError = 
  RelativeIntegrationError<3, ErrorType::kRelativePositionAndVelocityError>;
using RelativeTroposphereError = RelativeConstError<1, ErrorType::kRelativeTroposphereError>;
using RelativeIonosphereError = RelativeConstError<1, ErrorType::kRelativeIonosphereError>;

} 
