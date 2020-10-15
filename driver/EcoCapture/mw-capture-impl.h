////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2017 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __MW_CAPTURE_IMPL_H__
#define __MW_CAPTURE_IMPL_H__

#include "ospi/ospi.h"

#if defined(__linux__)
#include <linux/eventfd.h>
#endif

#include "supports/mw-notify.h"
#include "supports/notify-event-manager.h"
#include "supports/timer-event-manager.h"

struct mw_stream_cap {
    void                        *driver;
    os_dma_par_t                dma_priv;

    os_spinlock_t               timer_lock;
    struct list_head            timer_list;
    os_spinlock_t               notify_lock;
    struct list_head            notify_list;

    /* video capture */
    os_mutex_t                  video_mutex;
    unsigned long               generating;
    os_spinlock_t               req_lock;
    os_thread_t                 kthread;
    struct list_head            req_list;
    os_event_t                  req_event;
    struct list_head            completed_list;
#if defined(__linux__)
    struct eventfd_ctx          *video_done;
#else
    os_event_t                  video_done; // from user
#endif
    bool                        exit_capture_thread;

    os_event_t                  cap_event;
    notify_event_t              *cap_notify;
    timer_event_t               *cap_timer;

    int                         width;
    int                         height;
    __u32                       fourcc;
    __u32                       capstride;

    LONGLONG                    m_llStartTime;
    LONGLONG                    m_llFrameDuration;

    /* audio capture */
    os_mutex_t                  audio_mutex;
    unsigned long               audio_open;
    int                         prev_audio_block_id;
    notify_event_t              *audio_notify;
    bool                        m_bAudioAcquired;

    notify_event_manager        *m_pNotifyEventManager;
    timer_event_manager         *m_pTimerEventManager;
};

int mw_capture_init(struct mw_stream_cap *mwcap, void *driver, os_dma_par_t dma_priv);

void mw_capture_deinit(struct mw_stream_cap *mwcap);

bool mw_capture_ioctl(struct mw_stream_cap *mwcap, os_iocmd_t cmd,
                      void *arg, long *ret_val);


static inline bool mw_capture_is_generating(struct mw_stream_cap *mwcap)
{
    return os_test_bit(0, &mwcap->generating);
}

#endif /* __MW_CAPTURE_IMPL_H__ */
