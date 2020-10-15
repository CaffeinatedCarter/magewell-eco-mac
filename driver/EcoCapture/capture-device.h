////////////////////////////////////////////////////////////////////////////////
// CONFIDENTIAL and PROPRIETARY software of Magewell Electronics Co., Ltd.
// Copyright (c) 2011-2014 Magewell Electronics Co., Ltd. (Nanjing)
// All rights reserved.
// This copyright notice MUST be reproduced on all authorized copies.
////////////////////////////////////////////////////////////////////////////////
#ifndef __CAPTURE_DEVICE_H__
#define __CAPTURE_DEVICE_H__

#include "mw-procapture-extension.h"

#include "picopng/picopng.h"

typedef enum _CAPTURE_STATUS_IMAGE {
    CAPTURE_STATUS_IMAGE_NONE,
    CAPTURE_STATUS_IMAGE_NO_SIGNAL,
    CAPTURE_STATUS_IMAGE_LOCKING,
    CAPTURE_STATUS_IMAGE_UNSUPPORTED,
    CAPTURE_STATUS_IMAGE_LIMITED_BANDWIDTH
} CAPTURE_STATUS_IMAGE;

struct capture_device {
    void                       *priv; // kmalloc, if is NULL capture_device not init;
};

bool capture_device_is_support_msi(volatile void __iomem *reg_base);

int capture_device_init(struct capture_device *pdev,
                        volatile void __iomem *reg_base,
                        os_pci_dev_t pci_dev,
                        os_dma_par_t dma_par,
                        const char* nosignal_png_file,
                        const char* unsupported_png_file,
                        const char* locking_png_file,
                        const char* limited_bandwidth_png_file
                        );

void capture_device_deinit(struct capture_device *pdev);

int capture_device_resume(struct capture_device *pdev);

int capture_device_sleep(struct capture_device *pdev);

int capture_device_irq_process_top(struct capture_device *pdev);

void capture_device_irq_process_bottom(struct capture_device *pdev);

DWORD capture_device_GetRefClockFreq(struct capture_device *pdev);

BYTE *capture_device_GetProductName(struct capture_device *pdev);

BYTE capture_device_GetBoardIndex(struct capture_device *pdev);

BYTE capture_device_GetInstanceIndex(struct capture_device *pdev);

BOOLEAN capture_device_IsSupportedMultiStream(struct capture_device *pdev);

int capture_device_GetChannelCount(struct capture_device *pdev);

struct capture_channel* capture_device_GetChannel(struct capture_device *pdev, int iChannel);

png_pix_info_t* capture_device_GetStatusImage(struct capture_device *pdev, CAPTURE_STATUS_IMAGE statusImage);


int capture_device_GetChannelInfo(struct capture_device *pdev, MWCAP_CHANNEL_INFO *pInfo);

void capture_device_GetFamilyInfo(struct capture_device *pdev, MWCAP_PRO_CAPTURE_INFO * pInfo);

int capture_device_FirmwareGetStorageInfo(struct capture_device *pdev, MWCAP_FIRMWARE_STORAGE *pStorage);

int capture_device_FirmwareErase(struct capture_device *pdev, MWCAP_FIRMWARE_ERASE *pErase);

int capture_device_FirmwareWrite(struct capture_device *pdev,
                                 DWORD cbOffset,
                                 const BYTE * pbyData,
                                 DWORD cbData);

int capture_device_FirmwareRead(struct capture_device *pdev,
                                DWORD cbOffset,
                                BYTE * pbyData,
                                DWORD cbData);

void capture_device_SetLEDMode(struct capture_device *pdev, DWORD dwLEDMode);

void capture_device_SetPostReconfigDelay(struct capture_device *pdev, DWORD dwPostReconfigDelay);

long capture_device_GetCoreTemperature(struct capture_device *pdev);

#endif /* __CAPTURE_DEVICE_H__ */
