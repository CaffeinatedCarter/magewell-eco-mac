////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __IMAGE_BUFFER_H__
#define __IMAGE_BUFFER_H__

#include "win-types.h"

struct mw_heap;
struct mw_mem_desc;

struct image_buffer {
	struct mw_heap     *m_pHeap;
	u32					m_cx;
	u32					m_cy;
	DWORD				m_cbStride;
    struct mw_mem_desc *m_pMemDesc;
};

bool image_buffer_create(struct image_buffer *imgbuf, struct mw_heap *heap,
        u32 cx, u32 cy, u32 stride);

void image_buffer_destroy(struct image_buffer *imgbuf);

static inline bool image_buffer_isvalid(struct image_buffer *imgbuf)
{
    return (imgbuf->m_pMemDesc != NULL);
}

u32 image_buffer_get_address(struct image_buffer *imgbuf);

static inline u32 image_buffer_get_width(struct image_buffer *imgbuf)
{
    return imgbuf->m_cx;
}

static inline u32 image_buffer_get_height(struct image_buffer *imgbuf)
{
    return imgbuf->m_cy;
}

static inline u32 image_buffer_get_stride(struct image_buffer *imgbuf)
{
    return imgbuf->m_cbStride;
}

#endif /* __IMAGE_BUFFER_H__ */
