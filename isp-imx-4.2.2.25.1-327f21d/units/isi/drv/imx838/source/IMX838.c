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

CREATE_TRACER( IMX838_INFO , "IMX838: ", INFO,    1);
CREATE_TRACER( IMX838_WARN , "IMX838: ", WARNING, 1);
CREATE_TRACER( IMX838_ERROR, "IMX838: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
#endif

static const char SensorName[16] = "imx838";

typedef struct IMX838_Context_s
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
} IMX838_Context_t;

static RESULT IMX838_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( IMX838_INFO, "%s: (enter)\n", __func__);
    TRACE( IMX838_INFO, "%s: set power %d\n", __func__,on);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(IMX838_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( IMX838_INFO, "%s: status:%d sensor_mclk:%ld csi_max_pixel_clk:%ld\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX838_INFO, "%s: status:%d sensor_mclk:%ld csi_max_pixel_clk:%ld\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(IMX838_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(IMX838_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    const int32_t NANO_MICRO_COEFF = 1000;

    IsiSensorAeInfo_t *pAeInfo = &pIMX838Ctx->AeInfo;
    pAeInfo->oneLineExpTime = pIMX838Ctx->CurMode.ae_info.one_line_exp_time_ns / NANO_MICRO_COEFF;

    if (pIMX838Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pIMX838Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime * NANO_MICRO_COEFF;
        pAeInfo->minIntTime.linearInt =
            pIMX838Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime * NANO_MICRO_COEFF;
        pAeInfo->maxAGain.linearGainParas = pIMX838Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pIMX838Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pIMX838Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pIMX838Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pIMX838Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pIMX838Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pIMX838Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pIMX838Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pIMX838Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pIMX838Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pIMX838Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pIMX838Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pIMX838Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pIMX838Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pIMX838Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pIMX838Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pIMX838Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pIMX838Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pIMX838Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pIMX838Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pIMX838Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pIMX838Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pIMX838Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pIMX838Ctx->CurMode.ae_info.min_long_dgain;

                pAeInfo->hdrRatio[0] = pIMX838Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
                pAeInfo->hdrRatio[1] = pIMX838Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pIMX838Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime
                        * NANO_MICRO_COEFF;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pIMX838Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime
                        * NANO_MICRO_COEFF;

                if (pIMX838Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pAeInfo->maxIntTime.dualInt.dualIntTime * NANO_MICRO_COEFF;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime
                        * NANO_MICRO_COEFF;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pIMX838Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime
                            * NANO_MICRO_COEFF;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pIMX838Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime
                            * NANO_MICRO_COEFF;
                }

                if (pIMX838Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pIMX838Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pIMX838Ctx->CurMode.ae_info.min_dgain;
                }
                pAeInfo->hdrRatio[0] = pIMX838Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pIMX838Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pIMX838Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pIMX838Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pIMX838Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pIMX838Ctx->CurMode.ae_info.min_afps;

    pAeInfo->intUpdateDlyFrm = pIMX838Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pIMX838Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pIMX838Ctx->minAfps != 0) {
        pAeInfo->minAfps = pIMX838Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pIMX838Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(IMX838_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX838Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    IMX838_UpdateIsiAEInfo(handle);

    TRACE(IMX838_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(IMX838_ERROR, "%s set sensor stream error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(IMX838_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    IMX838_Context_t *pIMX838Ctx;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pIMX838Ctx = (IMX838_Context_t *) malloc(sizeof(IMX838_Context_t));
    if (!pIMX838Ctx)
        return RET_OUTOFMEM;

    memset(pIMX838Ctx, 0, sizeof(IMX838_Context_t));
    pIMX838Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pIMX838Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pIMX838Ctx;

    result = IMX838_IsiSensorSetPowerIss(pIMX838Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = IMX838_IsiSensorGetClkIss(pIMX838Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = IMX838_IsiSensorSetClkIss(pIMX838Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = IMX838_IsiResetSensorIss(pIMX838Ctx);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = IMX838_IsiSetSensorModeIss(pIMX838Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX838_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(IMX838_INFO, "%s (enter) \n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    if (pIMX838Ctx == NULL)
        return (RET_WRONG_HANDLE);

    IMX838_IsiSensorSetStreamingIss(pIMX838Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    IMX838_IsiSensorGetClkIss(pIMX838Ctx, &clk);
    clk.status = 0;
    IMX838_IsiSensorSetClkIss(pIMX838Ctx, &clk);
    IMX838_IsiSensorSetPowerIss(pIMX838Ctx, BOOL_FALSE);
    free(pIMX838Ctx);
    pIMX838Ctx = NULL;

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX838_INFO, "%s (enter) \n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    result = IMX838_IsiHalQuerySensorIss(pIMX838Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(IMX838_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX838_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX838_INFO, "%s (enter) \n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = IMX838_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pIMX838Ctx->CurMode.index;

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT IMX838_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pIMX838Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = IMX838_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(IMX838_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pIMX838Ctx->CurMode.size.bounds_width;
    format.format.height = pIMX838Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(IMX838_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(IMX838_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = IMX838_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(IMX838_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 838) {
        TRACE(IMX838_ERROR,
            "%s:ChipID=838,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pIMX838Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;

    // Could be refactored with the new NXP version:
    // set by exposure.ratio in Calibration file, even in Dol2 anc ClearHDR both ratios
    // are set to the same number but this does not effect streaming.
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pIMX838Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pIMX838Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(IMX838_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(IMX838_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pIMX838Ctx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    IMX838_UpdateIsiAEInfo(handle);

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pIMX838Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT IMX838_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;
    TRACE(IMX838_INFO,"%s: Integration time: %u\n", __func__, pIntegrationTime->IntegrationTime.linearInt);
    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime = pIMX838Ctx->AeInfo.oneLineExpTime;
    pIMX838Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = pIntegrationTime->IntegrationTime.linearInt;
            if (IntLine != pIMX838Ctx->IntLine) {
                printf("\n\ngoing into S_EXP ioctl %u\n\n", IntLine);
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pIMX838Ctx->IntLine = IntLine;
            }
            TRACE(IMX838_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pIMX838Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = pIntegrationTime->IntegrationTime.dualInt.dualIntTime;
            if (IntLine != pIMX838Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX838Ctx->IntLine = IntLine;
            }

            if (pIMX838Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = pIntegrationTime->IntegrationTime.dualInt.dualSIntTime;
                if (ShortIntLine != pIMX838Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(IMX838_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX838Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pIMX838Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX838_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pIMX838Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine + oneLineTime;
            pIMX838Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine + oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pIMX838Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triLIntTime;
                if (LongIntLine != pIMX838Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(IMX838_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pIMX838Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
                pIMX838Ctx->LongIntLine = LongIntLine;
            }

            IntLine = pIntegrationTime->IntegrationTime.triInt.triIntTime;
            if (IntLine != pIMX838Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX838Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = pIntegrationTime->IntegrationTime.triInt.triSIntTime;
            if (ShortIntLine != pIMX838Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pIMX838Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(IMX838_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pIMX838Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine + oneLineTime;
            pIMX838Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine + oneLineTime;
            pIMX838Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine + oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pIMX838Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pIMX838Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pIMX838Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pIMX838Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(IMX838_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pIMX838Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pIMX838Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pIMX838Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pIMX838Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX838_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pIMX838Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pIMX838Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pIMX838Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pIMX838Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pIMX838Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(IMX838_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(IMX838_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pIMX838Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pIMX838Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pIMX838Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(IMX838_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT IMX838_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pIMX838Ctx->CurMode.ae_info.cur_fps;

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(IMX838_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(IMX838_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pIMX838Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    IMX838_UpdateIsiAEInfo(handle);

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT IMX838_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pIMX838Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pIMX838Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pIMX838Ctx->minAfps = minAfps;
    pIMX838Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    if (pIMX838Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
#ifndef ISI_LITE
static RESULT IMX838_IsiSensorSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t * pBlc)
{
    int32_t ret = 0;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pBlc == NULL)
        return RET_NULL_POINTER;

    struct sensor_blc_s SensorBlc;
    SensorBlc.red = pBlc->red;
    SensorBlc.gb = pBlc->gb;
    SensorBlc.gr = pBlc->gr;
    SensorBlc.blue = pBlc->blue;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_BLC, &SensorBlc);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSensorGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorExpandCurve_t *pExpandCurve)
{
    int32_t ret = 0;

    TRACE(IMX838_INFO, "%s: (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

    if (pExpandCurve == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_EXPAND_CURVE, pExpandCurve);
    if (ret != 0) {
        TRACE(IMX838_ERROR, "%s: get  expand cure error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( IMX838_INFO, "%s (enter)\n", __func__);

    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pIMX838Ctx->IsiCtx.HalHandle;

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
        TRACE(IMX838_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(IMX838_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT IMX838_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    if (pIMX838Ctx->AEStartExposure == 0) {
        pIMX838Ctx->AEStartExposure =
            (uint64_t)pIMX838Ctx->CurMode.ae_info.start_exposure *
            pIMX838Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pIMX838Ctx->AEStartExposure;
    TRACE(IMX838_INFO, "%s:get start exposure %ld\n", __func__, pIMX838Ctx->AEStartExposure);

    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT IMX838_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);
    IMX838_Context_t *pIMX838Ctx = (IMX838_Context_t *) handle;

    pIMX838Ctx->AEStartExposure = exposure;
    TRACE(IMX838_INFO, "set start exposure %ld\n", pIMX838Ctx->AEStartExposure);
    TRACE(IMX838_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT IMX838_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( IMX838_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;

     memset(pIsiSensor, 0, sizeof(IsiSensor_t));
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = IMX838_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = IMX838_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = IMX838_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = IMX838_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = IMX838_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = IMX838_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = IMX838_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = IMX838_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = IMX838_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = IMX838_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = IMX838_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = IMX838_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = IMX838_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = IMX838_IsiGetAeInfoIss;
     pIsiSensor->pIsiSetHdrRatioIss              = IMX838_IsiSetHdrRatioIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = IMX838_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = IMX838_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = IMX838_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = IMX838_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = IMX838_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = IMX838_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = IMX838_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = IMX838_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetBlcIss              = IMX838_IsiSensorSetBlcIss;
    pIsiSensor->pIsiSensorSetWBIss               = IMX838_IsiSensorSetWBIss;
    pIsiSensor->pIsiSensorGetExpandCurveIss      = IMX838_IsiSensorGetExpandCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = IMX838_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = IMX838_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = IMX838_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = IMX838_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = IMX838_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = IMX838_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = IMX838_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = IMX838_IsiGetAeStartExposureIs;
#endif
    TRACE( IMX838_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x0838,
    .pIsiHalQuerySensor = IMX838_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = IMX838_IsiGetSensorIss,
};
