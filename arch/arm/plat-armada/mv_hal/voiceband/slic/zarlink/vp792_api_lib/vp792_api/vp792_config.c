/** \file vp792_config.c
 * vp792_config.c
 *
 *  This file contains the implementation of the VP-API 792 Series
 *  Configuration Functions.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 7142 $
 * $LastChangedDate: 2010-05-12 13:09:30 -0500 (Wed, 12 May 2010) $
 */

#include "vp_api.h"

#if defined (VP_CC_792_SERIES) /* Compile only if required */

#include "vp_api_int.h"
#include "vp792_api_int.h"
#include "vp_debug.h"


/* ================================
    SYSTEM CONFIGURATION FUNCTIONS
   ================================ */

VpStatusType
Vp792MakeDeviceCtx(
    VpDevCtxType *pDevCtx,
    Vp792DeviceObjectType *pDevObj)
{
    uint8 channelCount;

    /* redundant checking of the object and context */
    if((pDevCtx == VP_NULL) || (pDevObj == VP_NULL)) {
        VP_ERROR(None, VP_NULL, ("VpMakeVp792DeviceCtx(): Device context and device object cannot be NULL"));
        return VP_STATUS_INVALID_ARG;
    }

    /* Initialize the members of device context */
    pDevCtx->pDevObj = pDevObj;
    pDevCtx->deviceType = VP_DEV_792_SERIES;

    /* Initialize all of the line context pointers to null */
    for (channelCount = 0; channelCount < pDevObj->maxChannels; channelCount++) {
        pDevCtx->pLineCtx[channelCount] = VP_NULL;
    }

    /* System Configuration function pointers */
    pDevCtx->funPtrsToApiFuncs.MakeLineObject = Vp792MakeLineObject;

    /* Initialization function pointers */
    pDevCtx->funPtrsToApiFuncs.InitDevice = Vp792InitDevice;
    pDevCtx->funPtrsToApiFuncs.InitLine = Vp792InitLine;
    pDevCtx->funPtrsToApiFuncs.ConfigLine = Vp792ConfigLine;
    pDevCtx->funPtrsToApiFuncs.CalCodec = Vp792CalCodec;
    pDevCtx->funPtrsToApiFuncs.CalLine = Vp792CalLine;
    pDevCtx->funPtrsToApiFuncs.InitRing = Vp792InitRing;
    pDevCtx->funPtrsToApiFuncs.InitCid = Vp792InitCid;
    pDevCtx->funPtrsToApiFuncs.InitMeter = Vp792InitMeter;
    pDevCtx->funPtrsToApiFuncs.InitProfile = Vp792InitProfile;
    pDevCtx->funPtrsToApiFuncs.SetBatteries = Vp792SetBatteries;

    /* Control function pointers */
    pDevCtx->funPtrsToApiFuncs.SetLineState = Vp792SetLineState;
    pDevCtx->funPtrsToApiFuncs.SetLineTone = Vp792SetLineTone;
    pDevCtx->funPtrsToApiFuncs.SetRelayState = Vp792SetRelayState;
    pDevCtx->funPtrsToApiFuncs.SetRelGain = Vp792SetRelGain;
    pDevCtx->funPtrsToApiFuncs.SendSignal = Vp792SendSignal;
    pDevCtx->funPtrsToApiFuncs.SendCid = Vp792SendCid;
    pDevCtx->funPtrsToApiFuncs.ContinueCid = Vp792ContinueCid;
    pDevCtx->funPtrsToApiFuncs.SetOption = Vp792SetOption;
    pDevCtx->funPtrsToApiFuncs.DeviceIoAccessExt = Vp792DeviceIoAccessExt;
    pDevCtx->funPtrsToApiFuncs.LineIoAccess = Vp792LineIoAccess;
    pDevCtx->funPtrsToApiFuncs.SetBFilter = Vp792SetBFilter;
    pDevCtx->funPtrsToApiFuncs.LowLevelCmd16 = Vp792LowLevelCmd16;
    pDevCtx->funPtrsToApiFuncs.StartMeter32Q = Vp792StartMeter32Q;

#ifdef VP792_INCLUDE_TESTLINE_CODE
    /* An incomplete imlementation of VpGenTimerCtrl() is used by the VoicePath
       Test Library.  VpGenTimerCtrl() is undocumented and not supported for
       VE792 devices. */
    pDevCtx->funPtrsToApiFuncs.GenTimerCtrl = Vp792GenTimerCtrl;
#endif

    /* Status and Query function pointers */
    pDevCtx->funPtrsToApiFuncs.GetEvent = Vp792GetEvent;
    pDevCtx->funPtrsToApiFuncs.GetLineStatus = Vp792GetLineStatus;
    pDevCtx->funPtrsToApiFuncs.GetDeviceStatusExt = Vp792GetDeviceStatusExt;
    pDevCtx->funPtrsToApiFuncs.GetLoopCond = Vp792GetLoopCond;
    pDevCtx->funPtrsToApiFuncs.GetOption = Vp792GetOption;
    pDevCtx->funPtrsToApiFuncs.FlushEvents = Vp792FlushEvents;
    pDevCtx->funPtrsToApiFuncs.GetResults = Vp792GetResults;
    pDevCtx->funPtrsToApiFuncs.ClearResults = Vp792ClearResults;
#if (VP_CC_DEBUG_SELECT && VP_DBG_INFO)
    pDevCtx->funPtrsToApiFuncs.ObjectDump = Vp792ObjectDump;
#endif

#ifdef VP792_INCLUDE_TESTLINE_CODE
    pDevCtx->funPtrsToApiFuncs.TestLine = Vp792TestLine;
#endif

    VP_API_INT_EXIT(None, VP_NULL, "Vp792MakeDeviceCtx", VP_STATUS_SUCCESS);
    return VP_STATUS_SUCCESS;
} /* VpMakeVp792DeviceCtx() */

VpStatusType
Vp792MakeDeviceObject(
    VpDevCtxType *pDevCtx,
    Vp792DeviceObjectType *pDevObj)
{

    /* Start with a clean slate. */
    VpMemSet(pDevObj, 0, sizeof(Vp792DeviceObjectType));

    pDevObj->maxChannels = VP792_MAX_NUM_CHANNELS;
    pDevObj->slacId = VP792_MIN_SLAC_ID;
    pDevObj->options.debugSelect = VP_OPTION_DEFAULT_DEBUG_SELECT;

    if (pDevCtx == VP_NULL) {
        return VP_STATUS_SUCCESS;
    }

    return Vp792MakeDeviceCtx(pDevCtx, pDevObj);
} /* VpMakeVp792DeviceObject() */

VpStatusType
Vp792MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pVoidLineObj,
    VpDevCtxType *pDevCtx)
{
    Vp792LineObjectType *pLineObj = pVoidLineObj;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    /* Basic error checking */
    if (channelId >= pDevObj->maxChannels) {
        return VP_STATUS_INVALID_ARG;
    }

    switch (termType) {
        /* Internal ringing line termination types: */
        case VP_TERM_FXS_GENERIC:
        case VP_TERM_FXS_TI:
            break;

#if 0
        /* External ringing line termination types: */
        case VP_TERM_FXS_75282:
        case VP_TERM_FXS_RR:
        case VP_TERM_FXS_LCAS:
        case VP_TERM_FXS_RR_TI:
            break;
#endif

        default:
            return VP_STATUS_ERR_VTD_CODE;
    }

    /* Start with a clean slate. */
    VpMemSet(pLineObj, 0, sizeof(Vp792LineObjectType));

    /* Initialize line context */
    pLineCtx->pLineObj = pLineObj;

    /* Establish the links between device context and line context */
    pLineCtx->pDevCtx = pDevCtx;
    pDevCtx->pLineCtx[channelId] = pLineCtx;

    /* Initialize line object */
    pLineObj->termType = termType;
    pLineObj->channelId = channelId;
    pLineObj->options.debugSelect = VP_OPTION_DEFAULT_DEBUG_SELECT;
    pLineObj->metering.active = FALSE;
    pLineObj->metering.eventRate = 1;

    /* Note that we are not initializing the 'lineId' member because we do not
       know what the value should be; that information is not provided through
       the VpMakeLineObj() function).  Furthermore, VpLineIdType is defined by
       the customer, so it is impossible for us to know what a logical default
       would be.  VpMapLineId() should be called after this function. */

    return VP_STATUS_SUCCESS;
} /* VpMakeVp792LineObject() */

#endif /* VP_CC_792_SERIES */
