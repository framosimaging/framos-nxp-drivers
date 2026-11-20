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

CREATE_TRACER( VB1943_INFO , "VB1943: ", INFO,    1);
CREATE_TRACER( VB1943_WARN , "VB1943: ", WARNING, 1);
CREATE_TRACER( VB1943_ERROR, "VB1943: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
#endif

static const char SensorName[16] = "vb1943";

typedef struct VB1943_Context_s
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
} VB1943_Context_t;

static RESULT VB1943_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( VB1943_INFO, "%s: (enter)\n", __func__);
    TRACE( VB1943_INFO, "%s: set power %d\n", __func__,on);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(VB1943_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( VB1943_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( VB1943_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(VB1943_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(VB1943_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    IsiSensorAeInfo_t *pAeInfo = &pVB1943Ctx->AeInfo;
    pAeInfo->oneLineExpTime = pVB1943Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;

    if (pVB1943Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pVB1943Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime * 1000;
        pAeInfo->minIntTime.linearInt =
            pVB1943Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime * 1000;
        pAeInfo->maxAGain.linearGainParas = pVB1943Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pVB1943Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pVB1943Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pVB1943Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pVB1943Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pVB1943Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pVB1943Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pVB1943Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pVB1943Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pVB1943Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pVB1943Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pVB1943Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pVB1943Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pVB1943Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pVB1943Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pVB1943Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pVB1943Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pVB1943Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pVB1943Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pVB1943Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pVB1943Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pVB1943Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pVB1943Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pVB1943Ctx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pVB1943Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pVB1943Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pVB1943Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pVB1943Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pVB1943Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pVB1943Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pVB1943Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pVB1943Ctx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pVB1943Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pVB1943Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pVB1943Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pVB1943Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pVB1943Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pVB1943Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pVB1943Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pVB1943Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pVB1943Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pVB1943Ctx->minAfps != 0) {
        pAeInfo->minAfps = pVB1943Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pVB1943Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(VB1943_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pVB1943Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    VB1943_UpdateIsiAEInfo(handle);

    TRACE(VB1943_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(VB1943_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(VB1943_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    VB1943_Context_t *pVB1943Ctx;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pVB1943Ctx = (VB1943_Context_t *) malloc(sizeof(VB1943_Context_t));
    if (!pVB1943Ctx)
        return RET_OUTOFMEM;

    memset(pVB1943Ctx, 0, sizeof(VB1943_Context_t));
    pVB1943Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pVB1943Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pVB1943Ctx;

    result = VB1943_IsiSensorSetPowerIss(pVB1943Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = VB1943_IsiSensorGetClkIss(pVB1943Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = VB1943_IsiSensorSetClkIss(pVB1943Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = VB1943_IsiResetSensorIss(pVB1943Ctx);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = VB1943_IsiSetSensorModeIss(pVB1943Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT VB1943_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(VB1943_INFO, "%s (enter) \n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    if (pVB1943Ctx == NULL)
        return (RET_WRONG_HANDLE);

    VB1943_IsiSensorSetStreamingIss(pVB1943Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    VB1943_IsiSensorGetClkIss(pVB1943Ctx, &clk);
    clk.status = 0;
    VB1943_IsiSensorSetClkIss(pVB1943Ctx, &clk);
    VB1943_IsiSensorSetPowerIss(pVB1943Ctx, BOOL_FALSE);
    free(pVB1943Ctx);
    pVB1943Ctx = NULL;

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(VB1943_INFO, "%s (enter) \n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    result = VB1943_IsiHalQuerySensorIss(pVB1943Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(VB1943_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT VB1943_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(VB1943_INFO, "%s (enter) \n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = VB1943_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pVB1943Ctx->CurMode.index;

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT VB1943_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pVB1943Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = VB1943_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(VB1943_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pVB1943Ctx->CurMode.size.bounds_width;
    format.format.height = pVB1943Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(VB1943_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(VB1943_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = VB1943_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(VB1943_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 1943) {
        TRACE(VB1943_ERROR,
            "%s:ChipID=1943,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pVB1943Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pVB1943Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pVB1943Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(VB1943_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(VB1943_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pVB1943Ctx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    VB1943_UpdateIsiAEInfo(handle);

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pVB1943Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT VB1943_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;
    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pVB1943Ctx->AeInfo.oneLineExpTime;
    pVB1943Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = pIntegrationTime->IntegrationTime.linearInt;
            if (IntLine != pVB1943Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pVB1943Ctx->IntLine = IntLine;
            }
            TRACE(VB1943_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pVB1943Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = pIntegrationTime->IntegrationTime.dualInt.dualIntTime;
            if (IntLine != pVB1943Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pVB1943Ctx->IntLine = IntLine;
            }

            if (pVB1943Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = pIntegrationTime->IntegrationTime.dualInt.dualSIntTime;
                if (ShortIntLine != pVB1943Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(VB1943_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pVB1943Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pVB1943Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(VB1943_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pVB1943Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine + oneLineTime;
            pVB1943Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine + oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pVB1943Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triLIntTime;
                if (LongIntLine != pVB1943Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(VB1943_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pVB1943Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
                pVB1943Ctx->LongIntLine = LongIntLine;
            }

            IntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
            if (IntLine != pVB1943Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pVB1943Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = pIntegrationTime->IntegrationTime.triInt.triSIntTime;
            if (ShortIntLine != pVB1943Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pVB1943Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(VB1943_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pVB1943Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine + oneLineTime;
            pVB1943Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine + oneLineTime;
            pVB1943Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine + oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pVB1943Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pVB1943Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pVB1943Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pVB1943Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(VB1943_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pVB1943Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pVB1943Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pVB1943Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pVB1943Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(VB1943_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pVB1943Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pVB1943Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pVB1943Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pVB1943Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pVB1943Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(VB1943_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(VB1943_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pVB1943Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pVB1943Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pVB1943Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(VB1943_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT VB1943_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pVB1943Ctx->CurMode.ae_info.cur_fps;

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(VB1943_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(VB1943_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pVB1943Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    VB1943_UpdateIsiAEInfo(handle);

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT VB1943_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pVB1943Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pVB1943Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pVB1943Ctx->minAfps = minAfps;
    pVB1943Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    if (pVB1943Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
#ifndef ISI_LITE
static RESULT VB1943_IsiSensorSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t * pBlc)
{
    int32_t ret = 0;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pBlc == NULL)
        return RET_NULL_POINTER;

    struct sensor_blc_s SensorBlc;
    SensorBlc.red = pBlc->red;
    SensorBlc.gb = pBlc->gb;
    SensorBlc.gr = pBlc->gr;
    SensorBlc.blue = pBlc->blue;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_BLC, &SensorBlc);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSensorGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorExpandCurve_t *pExpandCurve)
{
    int32_t ret = 0;

    TRACE(VB1943_INFO, "%s: (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

    if (pExpandCurve == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_EXPAND_CURVE, pExpandCurve);
    if (ret != 0) {
        TRACE(VB1943_ERROR, "%s: get  expand cure error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( VB1943_INFO, "%s (enter)\n", __func__);

    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pVB1943Ctx->IsiCtx.HalHandle;

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
        TRACE(VB1943_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(VB1943_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT VB1943_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    if (pVB1943Ctx->AEStartExposure == 0) {
        pVB1943Ctx->AEStartExposure =
            (uint64_t)pVB1943Ctx->CurMode.ae_info.start_exposure *
            pVB1943Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pVB1943Ctx->AEStartExposure;
    TRACE(VB1943_INFO, "%s:get start exposure %d\n", __func__, pVB1943Ctx->AEStartExposure);

    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT VB1943_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);
    VB1943_Context_t *pVB1943Ctx = (VB1943_Context_t *) handle;

    pVB1943Ctx->AEStartExposure = exposure;
    TRACE(VB1943_INFO, "set start exposure %d\n", __func__,pVB1943Ctx->AEStartExposure);
    TRACE(VB1943_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT VB1943_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( VB1943_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = VB1943_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = VB1943_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = VB1943_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = VB1943_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = VB1943_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = VB1943_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = VB1943_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = VB1943_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = VB1943_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = VB1943_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = VB1943_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = VB1943_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = VB1943_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = VB1943_IsiGetAeInfoIss;
     pIsiSensor->pIsiSetHdrRatioIss              = VB1943_IsiSetHdrRatioIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = VB1943_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = VB1943_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = VB1943_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = VB1943_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = VB1943_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = VB1943_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = VB1943_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = VB1943_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetBlcIss              = VB1943_IsiSensorSetBlcIss;
    pIsiSensor->pIsiSensorSetWBIss               = VB1943_IsiSensorSetWBIss;
    pIsiSensor->pIsiSensorGetExpandCurveIss      = VB1943_IsiSensorGetExpandCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = VB1943_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = VB1943_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = VB1943_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = VB1943_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = VB1943_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = VB1943_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = VB1943_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = VB1943_IsiGetAeStartExposureIs;
#endif
    TRACE( VB1943_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x1943,
    .pIsiHalQuerySensor = VB1943_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = VB1943_IsiGetSensorIss,
};
