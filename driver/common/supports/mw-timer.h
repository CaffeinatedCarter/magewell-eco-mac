////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __TIMER_EVENT_H__
#define __TIMER_EVENT_H__

#include "ospi/ospi.h"
#if defined(__linux__)
#include <linux/eventfd.h>
#endif

typedef struct _timer_event_t {
    struct list_head    user_node;

    int                 m_nIndex;
    long long           m_llExpireTime;

    os_event_t          event;

#if defined(__linux__)
    struct eventfd_ctx  *eventfd;
#endif
} timer_event_t;

#endif /* __TIMER_EVENT_H__ */
