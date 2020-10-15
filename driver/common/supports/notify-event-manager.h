////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __NOTIFY_EVENT_MANAGER_H__
#define __NOTIFY_EVENT_MANAGER_H__

#include "ospi/ospi.h"

#include "mw-notify.h"

typedef struct _notify_event_manager notify_event_manager;

notify_event_t* notify_event_mgr_add(notify_event_manager* nm,
                                    u64 enable_bits,
                                    os_event_t event);

#if defined(__linux__)
notify_event_t*
notify_event_mgr_add_eventfd(notify_event_manager* nm, u64 enable_bits, int fd);
#endif

void notify_event_mgr_remove(notify_event_manager *nm, notify_event_t *event);

void notify_event_mgr_set_enable_bits(notify_event_manager *nm,
                                      notify_event_t *event, u64 enable_bits);

u64 notify_event_mgr_get_status_bits(notify_event_manager *nm, notify_event_t *event);

#endif /* __NOTIFY_EVENT_MANAGER_H__ */
