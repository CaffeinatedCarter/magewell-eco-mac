////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __CAPTURE_CHANNEL_H__
#define __CAPTURE_CHANNEL_H__

#include "mw-procapture-extension.h"
#include "ospi/ospi.h"

#include "supports/mw-timer.h"
#include "supports/mw-notify.h"
#include "supports/notify-event-manager.h"
#include "supports/timer-event-manager.h"

enum AUDIO_SIZE {
    AUDIO_FRAME_COUNT			= 32
};

struct capture_device;
struct capture_channel;

/* base info */
BYTE capture_channel_GetChannelIndex(struct capture_channel *pdev);
BYTE capture_channel_GetAdapterChannelIndex(struct capture_channel *pdev);
int capture_channel_GetChannelName(struct capture_channel* pdev, char *name, int size);
BOOLEAN capture_channel_IsSupportedMultiStream(struct capture_channel* pdev);

/* supports */
notify_event_manager *capture_channel_GetNotifyEventManager(struct capture_channel *pdev);
timer_event_manager *capture_channel_GetTimerEventManager(struct capture_channel *pdev);

// Input source management
void capture_channel_SetInputSourceScan(struct capture_channel* pdev, BOOLEAN bEnableScan);
BOOLEAN capture_channel_GetInputSourceScan(struct capture_channel* pdev);
BOOLEAN capture_channel_GetInputSourceScanState(struct capture_channel* pdev);
void capture_channel_SetAVInputSourceLink(struct capture_channel* pdev, BOOLEAN bLink);
BOOLEAN capture_channel_GetAVInputSourceLink(struct capture_channel* pdev);
void capture_channel_SetVideoInputSource(struct capture_channel* pdev, DWORD dwInputSource);
DWORD capture_channel_GetVideoInputSource(struct capture_channel* pdev);
void capture_channel_SetAudioInputSource(struct capture_channel* pdev, DWORD dwInputSource);
DWORD capture_channel_GetAudioInputSource(struct capture_channel* pdev);
const DWORD* capture_channel_GetSupportedVideoInputSources(
        struct capture_channel* pdev,
        int * pcInputSources
        );
const DWORD* capture_channel_GetSupportedAudioInputSources(
        struct capture_channel* pdev,
        int* pcInputSources
        );

// A/V Signal status
void capture_channel_GetInputSpecificStatus(
        struct capture_channel* pdev,
        MWCAP_INPUT_SPECIFIC_STATUS* pStatus
        );
void capture_channel_GetVideoSignalStatus(
        struct capture_channel* pdev,
        MWCAP_VIDEO_SIGNAL_STATUS* pStatus
        );
void capture_channel_GetAudioSignalStatus(
        struct capture_channel* pdev,
        MWCAP_AUDIO_SIGNAL_STATUS* pStatus
        );

// Audio capture
int capture_channel_AcquireAudioCapture(struct capture_channel*pdev);
VOID capture_channel_ReleaseAudioCapture(struct capture_channel*pdev);
LONG capture_channel_GetCompletedAudioBlockId(struct capture_channel*pdev);
void capture_channel_GetNormalizedAudioCaptureFrame(
        struct capture_channel *pdev,
        LONG lBlockId,
        MWCAP_AUDIO_CAPTURE_FRAME * pAudioFrameDest);

// Video capture
int capture_channel_AcquireVideoCapture(
        struct capture_channel* pdev,
        int cx,
        int cy,
        DWORD dwFOURCC,
        DWORD* pcbStride
        );
VOID capture_channel_ReleaseVideoCapture(struct capture_channel* pdev);
void capture_channel_SetVideoSettings(
        struct capture_channel* pdev,
        MWCAP_VIDEO_ECO_CAPTURE_SETTINGS * pVideoSettings
        );
void capture_channel_GetVideoSettings(
        struct capture_channel* pdev,
        MWCAP_VIDEO_ECO_CAPTURE_SETTINGS * pVideoSettings
        );
VOID* capture_channel_GetVideoCaptureFrame(struct capture_channel* pdev,
                                           LONGLONG *pllTimestamp,
                                           MWCAP_SDI_ANC_PACKET *pPackets,
                                           int *iCount
                                           );
void capture_channel_ReleaseVideoCaptureFrame(struct capture_channel* pdev);


/* Magewell Capture Extensions */
void capture_channel_GetVideoCaps(struct capture_channel* pdev, MWCAP_VIDEO_CAPS *pInfo);
void capture_channel_GetAudioCaps(struct capture_channel* pdev, MWCAP_AUDIO_CAPS *pInfo);
LONGLONG capture_channel_GetTime(struct capture_channel* pdev);
void capture_channel_SetTime(struct capture_channel* pdev, LONGLONG llTime);
void capture_channel_Regulation(struct capture_channel* pdev, LONGLONG llTime);

#endif /* __CAPTURE_CHANNEL_H__ */
