/**
* @Function: GNSS parameter blocks for ceres backend
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#pragma once

#include "gici/estimate/common_parameter_block.h"

namespace gici {

// All the parameters used by GNSS positioning are common blocks
using PositionParameterBlock = CommonParameterBlock<3, CommonParameterBlockType::Position>;
using VelocityParameterBlock = CommonParameterBlock<3, CommonParameterBlockType::Velocity>;
using ClockParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Clock>;
using IfbParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Ifb>;
using FrequencyParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Frequency>;
using AmbiguityParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Ambiguity>;
using IonosphereParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Ionosphere>;
using TroposphereParameterBlock = CommonParameterBlock<1, CommonParameterBlockType::Troposphere>;

} 
