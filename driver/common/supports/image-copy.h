////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "win-types.h"

void memlinecpy(void* dst, const void* src, int dst_stride, int src_stride, int line_size, int lines, bool flip);

void ImageCopy(
    DWORD dwFOURCC,
    int cx,
    int cy,
    const void * pvSrc,
    int cbSrcStride,
    void * pvDest,
    int cbDestStride,
    bool bVertFlip
    );
