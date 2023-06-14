/**
* @Function: Un-define RTKLIB types to avoid conflicts
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#ifndef RTKLIB_SAFE
#define RTKLIB_SAFE

#include <rtklib.h>

#ifdef __cplusplus
extern "C" {
#endif

#undef lock
#undef unlock

#ifdef __cplusplus
}
#endif

#endif
