////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __TIMER_EVENT_MANAGER_H__
#define __TIMER_EVENT_MANAGER_H__

#include "ospi/ospi.h"
#include "win-types.h"

#include "mw-timer.h"

typedef struct _timer_event_manager timer_event_manager;

timer_event_t *timer_event_mgr_NewTimer(timer_event_manager *tm, os_event_t event);

#if defined(__linux__)
timer_event_t* timer_event_mgr_NewTimerEventFd(timer_event_manager* tm, int fd);
#endif

void timer_event_mgr_DeleteTimer(timer_event_manager *tm, timer_event_t *pTimer);

BOOLEAN timer_event_mgr_ScheduleTimer(timer_event_manager *tm, LONGLONG llTime, timer_event_t *pTimer);

BOOLEAN timer_event_mgr_CancelTimer(timer_event_manager *tm, timer_event_t *pTimer);

#endif /* __TIMER_EVENT_MANAGER_H__ */
