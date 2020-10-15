////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __MW_VESION_H__
#define __MW_VESION_H__

#define GETSTR(s) #s
#define STR(s)  GETSTR(s)

#define VER_MAJOR 1
#define VER_MINOR 4
#define VER_BUILD 47
#define VERSION_STRING  STR(VER_MAJOR)"."STR(VER_MINOR)"."STR(VER_BUILD)

#endif /* __MW_VERSION_H__ */
