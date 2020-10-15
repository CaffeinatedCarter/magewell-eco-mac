////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __NOTIFY_EVENT_H__
#define __NOTIFY_EVENT_H__

#include "ospi/ospi.h"
#if defined(__linux__)
#include <linux/eventfd.h>
#endif

typedef struct _notify_event_t {
    struct list_head    user_node;
    struct list_head    list_node;

    u64                 enable_bits;
    u64                 status_bits;
    os_event_t          event;

#if defined(__linux__)
    struct eventfd_ctx  *eventfd;
#endif
} notify_event_t;

#endif /* __NOTIFY_EVENT_H__ */
