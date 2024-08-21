/******************************************************************************\
|* Copyright (c) 2020 by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")       *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/return_codes.h>
#include <common/misc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "vvsensor.h"

CREATE_TRACER( IMX676_INFO , "IMX676: ", INFO,    1);
CREATE_TRACER( IMX676_WARN , "IMX676: ", WARNING, 1);
CREATE_TRACER( IMX676_ERROR, "IMX676: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
#endif

static const char SensorName[16] = "imx676";

typedef struct IMX676_Context_s
{
    IsiSensorContext_t  IsiCtx;
    struct vvcam_mode_info_s CurMode;
    IsiSensorAeInfo_t AeInfo;
    IsiSensorIntTime_t IntTime;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    IsiSensorGain_t SensorGain;
    uint32_t minAfps;
    uint64_t AEStartExposure;
} IMX676_Context_t;

static RESULT IMX676_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( IMX676_INFO, "%s: (enter)\n", __func__);
    TRACE( IMX676_INFO, "%s: set power %d\n", __func__,on);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(IMX676_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( IMX676_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX676_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(IMX676_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(IMX676_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    IsiSensorAeInfo_t *pAeInfo = &pIMX676Ctx->AeInfo;
    pAeInfo->oneLineExpTime = pIMX676Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;

    if (pIMX676Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pIMX676Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime * 1000;
        pAeInfo->minIntTime.linearInt =
            pIMX676Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime * 1000;
        pAeInfo->maxAGain.linearGainParas = pIMX676Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pIMX676Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pIMX676Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pIMX676Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pIMX676Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pIMX676Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pIMX676Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pIMX676Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pIMX676Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pIMX676Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pIMX676Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pIMX676Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pIMX676Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pIMX676Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pIMX676Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pIMX676Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pIMX676Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pIMX676Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pIMX676Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pIMX676Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pIMX676Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pIMX676Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pIMX676Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pIMX676Ctx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pIMX676Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pIMX676Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pIMX676Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pIMX676Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pIMX676Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pIMX676Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX676Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX676Ctx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pIMX676Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pIMX676Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pIMX676Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pIMX676Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pIMX676Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pIMX676Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pIMX676Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pIMX676Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pIMX676Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pIMX676Ctx->minAfps != 0) {
        pAeInfo->minAfps = pIMX676Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pIMX676Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(IMX676_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX676Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    IMX676_UpdateIsiAEInfo(handle);

    TRACE(IMX676_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(IMX676_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(IMX676_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    IMX676_Context_t *pIMX676Ctx;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pIMX676Ctx = (IMX676_Context_t *) malloc(sizeof(IMX676_Context_t));
    if (!pIMX676Ctx)
        return RET_OUTOFMEM;

    memset(pIMX676Ctx, 0, sizeof(IMX676_Context_t));
    pIMX676Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX676Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pIMX676Ctx;

    result = IMX676_IsiSensorSetPowerIss(pIMX676Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = IMX676_IsiSensorGetClkIss(pIMX676Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = IMX676_IsiSensorSetClkIss(pIMX676Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = IMX676_IsiResetSensorIss(pIMX676Ctx);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = IMX676_IsiSetSensorModeIss(pIMX676Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX676_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(IMX676_INFO, "%s (enter) \n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    if (pIMX676Ctx == NULL)
        return (RET_WRONG_HANDLE);

    IMX676_IsiSensorSetStreamingIss(pIMX676Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    IMX676_IsiSensorGetClkIss(pIMX676Ctx, &clk);
    clk.status = 0;
    IMX676_IsiSensorSetClkIss(pIMX676Ctx, &clk);
    IMX676_IsiSensorSetPowerIss(pIMX676Ctx, BOOL_FALSE);
    free(pIMX676Ctx);
    pIMX676Ctx = NULL;

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX676_INFO, "%s (enter) \n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    result = IMX676_IsiHalQuerySensorIss(pIMX676Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(IMX676_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX676_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX676_INFO, "%s (enter) \n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = IMX676_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pIMX676Ctx->CurMode.index;

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX676_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pIMX676Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = IMX676_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(IMX676_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pIMX676Ctx->CurMode.size.bounds_width;
    format.format.height = pIMX676Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(IMX676_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(IMX676_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = IMX676_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(IMX676_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 676) {
        TRACE(IMX676_ERROR,
            "%s:ChipID=676,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pIMX676Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pIMX676Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pIMX676Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(IMX676_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX676_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pIMX676Ctx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    IMX676_UpdateIsiAEInfo(handle);

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pIMX676Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT IMX676_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;
    printf("\n\nAAAAAAAAAAAAAAA %u\n\n", pIntegrationTime->IntegrationTime.linearInt);
    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pIMX676Ctx->AeInfo.oneLineExpTime;
    pIMX676Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = pIntegrationTime->IntegrationTime.linearInt;
            if (IntLine != pIMX676Ctx->IntLine) {
                printf("\n\ngoing into S_EXP ioctl %u\n\n", IntLine);
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pIMX676Ctx->IntLine = IntLine;
            }
            TRACE(IMX676_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pIMX676Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = pIntegrationTime->IntegrationTime.dualInt.dualIntTime;
            if (IntLine != pIMX676Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX676Ctx->IntLine = IntLine;
            }

            if (pIMX676Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = pIntegrationTime->IntegrationTime.dualInt.dualSIntTime;
                if (ShortIntLine != pIMX676Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(IMX676_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX676Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pIMX676Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX676_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pIMX676Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine + oneLineTime;
            pIMX676Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine + oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pIMX676Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triLIntTime;
                if (LongIntLine != pIMX676Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(IMX676_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX676Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
                pIMX676Ctx->LongIntLine = LongIntLine;
            }

            IntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
            if (IntLine != pIMX676Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX676Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = pIntegrationTime->IntegrationTime.triInt.triSIntTime;
            if (ShortIntLine != pIMX676Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX676Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX676_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pIMX676Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine + oneLineTime;
            pIMX676Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine + oneLineTime;
            pIMX676Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine + oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pIMX676Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pIMX676Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pIMX676Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pIMX676Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(IMX676_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pIMX676Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pIMX676Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pIMX676Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pIMX676Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX676_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pIMX676Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pIMX676Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pIMX676Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pIMX676Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pIMX676Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(IMX676_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX676_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pIMX676Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pIMX676Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pIMX676Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(IMX676_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT IMX676_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pIMX676Ctx->CurMode.ae_info.cur_fps;

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(IMX676_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(IMX676_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX676Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    IMX676_UpdateIsiAEInfo(handle);

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT IMX676_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pIMX676Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pIMX676Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pIMX676Ctx->minAfps = minAfps;
    pIMX676Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    if (pIMX676Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
#ifndef ISI_LITE
static RESULT IMX676_IsiSensorSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t * pBlc)
{
    int32_t ret = 0;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pBlc == NULL)
        return RET_NULL_POINTER;

    struct sensor_blc_s SensorBlc;
    SensorBlc.red = pBlc->red;
    SensorBlc.gb = pBlc->gb;
    SensorBlc.gr = pBlc->gr;
    SensorBlc.blue = pBlc->blue;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_BLC, &SensorBlc);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSensorGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorExpandCurve_t *pExpandCurve)
{
    int32_t ret = 0;

    TRACE(IMX676_INFO, "%s: (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    if (pExpandCurve == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_EXPAND_CURVE, pExpandCurve);
    if (ret != 0) {
        TRACE(IMX676_ERROR, "%s: get  expand cure error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( IMX676_INFO, "%s (enter)\n", __func__);

    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX676Ctx->IsiCtx.HalHandle;

    struct sensor_test_pattern_s TestPattern;
    if (tpgMode == ISI_TPG_DISABLE) {
        TestPattern.enable = 0;
        TestPattern.pattern = 0;
    } else {
        TestPattern.enable = 1;
        TestPattern.pattern = (uint32_t)tpgMode - 1;
    }

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_TEST_PATTERN, &TestPattern);
    if (ret != 0)
    {
        TRACE(IMX676_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(IMX676_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX676_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    if (pIMX676Ctx->AEStartExposure == 0) {
        pIMX676Ctx->AEStartExposure =
            (uint64_t)pIMX676Ctx->CurMode.ae_info.start_exposure *
            pIMX676Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pIMX676Ctx->AEStartExposure;
    TRACE(IMX676_INFO, "%s:get start exposure %d\n", __func__, pIMX676Ctx->AEStartExposure);

    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX676_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);
    IMX676_Context_t *pIMX676Ctx = (IMX676_Context_t *) handle;

    pIMX676Ctx->AEStartExposure = exposure;
    TRACE(IMX676_INFO, "set start exposure %d\n", __func__,pIMX676Ctx->AEStartExposure);
    TRACE(IMX676_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT IMX676_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( IMX676_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = IMX676_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = IMX676_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = IMX676_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = IMX676_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = IMX676_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = IMX676_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = IMX676_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = IMX676_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = IMX676_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = IMX676_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = IMX676_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = IMX676_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = IMX676_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = IMX676_IsiGetAeInfoIss;
     pIsiSensor->pIsiSetHdrRatioIss              = IMX676_IsiSetHdrRatioIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = IMX676_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = IMX676_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = IMX676_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = IMX676_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = IMX676_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = IMX676_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = IMX676_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = IMX676_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetBlcIss              = IMX676_IsiSensorSetBlcIss;
    pIsiSensor->pIsiSensorSetWBIss               = IMX676_IsiSensorSetWBIss;
    pIsiSensor->pIsiSensorGetExpandCurveIss      = IMX676_IsiSensorGetExpandCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = IMX676_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = IMX676_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = IMX676_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = IMX676_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = IMX676_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = IMX676_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = IMX676_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = IMX676_IsiGetAeStartExposureIs;
#endif
    TRACE( IMX676_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x0676,
    .pIsiHalQuerySensor = IMX676_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = IMX676_IsiGetSensorIss,
};
