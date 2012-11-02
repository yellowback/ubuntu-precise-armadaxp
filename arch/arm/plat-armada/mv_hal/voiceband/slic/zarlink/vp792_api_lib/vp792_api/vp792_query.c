/** \file vp792_query.c
 * vp792_query.c
 *
 *  This file contains the implementation of the VP-API 792 Series
 *  Status and Query Functions.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 7146 $
 * $LastChangedDate: 2010-05-12 13:50:09 -0500 (Wed, 12 May 2010) $
 */

#include "vp_api.h"

#if defined (VP_CC_792_SERIES)  /* Compile only if required */

#define VP_HAL_DEVICE_TYPE VP_DEV_792_SERIES

#include "vp_debug.h"
#include "vp_api_int.h"
#include "vp792_api_int.h"
#include "sys_service.h"

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
/* #include <stdio.h> */ /* sprintf() */
#endif


/* ==============
    Local Macros
   ============== */

#define CopyIntInd(intInd, eventIdWord, eventParamWord) \
    (intInd)[VP792_REG_INTIND_EVID_WORD] = (eventIdWord); \
    (intInd)[VP792_REG_INTIND_PARM_WORD] = (eventParamWord);

#define TS_DIVISOR 4 /* API timestamp granularity is 1/4 that of the SLAC */

#if (TS_DIVISOR > 0)
#define CALCULATE_TIMESTAMP(pDevObj, loBits) (((pDevObj)->timeStampHiBits) + (loBits) / TS_DIVISOR)
#else
#define CALCULATE_TIMESTAMP(pDevObj, loBits) (loBits)
#endif


/* =================================
    Prototypes for Static Functions
   ================================= */

static bool
CheckEventMask(
    VpEventType *pEvent);

static bool
CidAckDetectInterval(
    VpEventType *pEvent,
    int toneIndex1,
    int toneIndex2,
    VpProfilePtrType pCidProfile);

static VpDigitType
ConvertCharToDigitType(
    char digit);

static void
DeferEvent(
    VpDevCtxType *pDevCtx,
    uint16 eventIdWord,
    uint16 eventParamWord);

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
static void
DisplayVp792Event(
    VpDeviceIdType deviceId,
    uint16 eventIdWord,
    uint16 eventParamWord);

static void
DisplayVp792Profile(
    VpProfilePtrType pProfile,
    uint8 maxLen);
#endif

static bool
EhCal(
    VpEventType *pEvent,
    uint16 eventIdWord);

static bool
EhInitDevice(
    VpEventType *pEvent,
    uint16 eventIdWord);

static bool
EhInitLine(
    VpEventType *pEvent,
    uint16 eventIdWord);

static bool
EhSendCid(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static bool
EhSendSignal(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static VpStatusType
EhSetLineTone(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static void
GetCidAckToneIndices(
    uint16 userEventId,
    int *pIndex1,
    int *pIndex2);

static bool
GetEventInternal(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static uint8
GetFskDataLength(
    Vp792LineObjectType *pLineObj);

static bool
HandleFirmwareEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static bool
HandleSequenceIntervalEvent(
    VpEventType *pEvent,
    uint16 userEventId);

static bool
HandleSystemInt(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static bool
HandleUserEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);

static uint16
Hypotenuse(
    uint16 a,
    uint16 b);

static VpStatusType
InitAllLines(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    uint16p pBuf,
    uint16p *ppBufEnd);

static VpStatusType
LineIoAccessIntRead(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 *pData);

static VpStatusType
MergeSequences(
    uint8p pRingSeq,
    uint8p pCidSeq,
    uint16p pMergedSeq);

static VpStatusType
ParkMode(
    VpLineCtxType *pLineCtx);

static VpStatusType
ReadResponse(
    VpEventType *pEvent,
    uint16 requestIdx,
    void *pResults);

static VpStatusType
ReadFirmwareRevCode(
    VpDevCtxType *pDevCtx);

static VpStatusType
ReadLineIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpOptionLineIoConfigType *pLineIoCfg);

static void
RelocateSeqCmds(
    uint8p pSrcSeq,
    uint8 srcIndex,
    uint16p pDestSeq,
    uint8 destIndex,
    uint8 numCmds);

static bool
ReportDtmfEvent(
    VpEventType *pEvent,
    VpDigitSenseType sense,
    bool needTimeStamp);

static VpStatusType
RestoreDriveState(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static VpStatusType
RingEnter(
    VpEventType *pEvent,
    bool polrev);

static VpStatusType
RingExit(
    VpLineCtxType *pLineCtx);

static VpStatusType
SendDtmfData(
    VpLineCtxType *pLineCtx);

static VpStatusType
SendDtmfProgSigGen(
    VpLineCtxType *pLineCtx,
    VpDigitType digit);

static VpStatusType
SendSigMsgWaitReadDcParams(
    VpLineCtxType *pLineCtx);

static VpStatusType
SendSigMsgWaitWriteDcParams(
    VpLineCtxType *pLineCtx,
    uint16 v1,
    uint16 vas,
    uint16 ila);

static VpStatusType
SendSigMsgWaitSaveDcParams(
    VpEventType *pEvent);

static VpStatusType
SequencerControl(
    VpLineCtxType *pLineCtx,
    uint16 cmd);

static VpStatusType
StartSequence(
    VpLineCtxType *pLineCtx,
    uint8p pSequence,
    bool endian,
    uint16 seqType);

static bool
UnexpectedEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord);


/* ============================
    STATUS AND QUERY FUNCTIONS
   ============================ */

VpStatusType
Vp792ClearResults(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792ClearResultsInt(pDevCtx);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792ClearResults() */

VpStatusType
Vp792ClearResultsInt(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint16 requestIdx = pDevObj->requestIdx;
    if (requestIdx < VP792_MAX_OUTSTANDING_RESPONSES) {
        pDevObj->responseRequest[requestIdx].outstanding = FALSE;
    }
    pDevObj->requestIdx = VP792_MAX_OUTSTANDING_RESPONSES;

    /* There may be garbage in the SLAC's response mailbox that VpGetEvent()
       couldn't read.  Clear it. */
    return Vp792RspMailboxRelease(pDevCtx);
} /* Vp792ClearResultsInt() */

VpStatusType
Vp792FlushEvents(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 sanity = 255;
    bool gotEvent;
    VpStatusType status;

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    do {
        VpEventType event;
        gotEvent = Vp792GetEvent(pDevCtx, &event);
        status = event.status;
        if (!sanity--) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792FlushEvents(): Event register is stuck!"));
            status = VP_STATUS_FAILURE;
        }
    } while ((status == VP_STATUS_SUCCESS) && gotEvent);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792FlushEvents() */

VpStatusType
Vp792GetDeviceStatusExt(
    VpDevCtxType *pDevCtx,
    VpDeviceStatusType *pDeviceStatus)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpInputType input = pDeviceStatus->input;
    VpStatusType status;
    uint8 offset, result;
    bool readChanReg = FALSE, readBothHookRegs = FALSE;
    uint16 globalReg, rawHookReg, chanReg[VP792_MAX_NUM_CHANNELS];
    int chan;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Choose which status register to read based on input type. */
    switch (input) {
        case VP_INPUT_HOOK:
            offset = VP792_REG_GHOOK_OFFSET;

            /* For channels with pulse detection disabled, we need to report the
               raw hook status.  So we read that register also.*/
            readBothHookRegs = TRUE;
            break;
        case VP_INPUT_RAW_HOOK:
            offset = VP792_REG_GRHOOK_OFFSET;
            break;
        case VP_INPUT_GKEY:
            offset = VP792_REG_GGKEY_OFFSET;
            break;
        case VP_INPUT_AC_FLT:
        case VP_INPUT_DC_FLT:
        case VP_INPUT_THERM_FLT:
            offset = VP792_REG_GFLT_OFFSET;
            readChanReg = TRUE;
            break;
        case VP_INPUT_CLK_FLT:
            offset = VP792_REG_CLK_STATUS_OFFSET;
            break;
        case VP_INPUT_BAT1_FLT:
        case VP_INPUT_BAT2_FLT:
        case VP_INPUT_BAT3_FLT:
            offset = VP792_REG_SYS_STAT_OFFSET;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Read the appropriate global status register(s). */
    status = Vp792HbiDirectPageRead(pDevCtx, offset, 1, &globalReg);
    if ((readBothHookRegs) && (status == VP_STATUS_SUCCESS)) {
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_GRHOOK_OFFSET, 1,
            &rawHookReg);
    }

    /* If necessary, read channel supervision register(s). */
    if (readChanReg) {
        for (chan = 0; chan < pDevObj->maxChannels; chan++) {

            /* Only read the channel supervision register if the
               fault bit is set in the global status reg. */
            uint8 chanBit = (1 << chan);
            if ((globalReg & chanBit) && (status == VP_STATUS_SUCCESS)) {
                status = Vp792HbiPagedRead(pDevCtx, chan,
                    VP792_REG_CH_SUP_OFFSET, VP792_REG_CH_SUP_LEN,
                    &chanReg[chan]);
            }
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Handle the register value(s) according to input. */
    result = 0;
    switch (input) {
        case VP_INPUT_GKEY:
        case VP_INPUT_RAW_HOOK:
            result = (uint8)globalReg;
            break;
        case VP_INPUT_HOOK:
            /* For lines with pulse detection enabled, use the hook bit; for other
               lines, use the raw hook bit. */
            globalReg &= pDevObj->options.pulseEnabled;
            globalReg |= (rawHookReg & ~pDevObj->options.pulseEnabled);
            result = (uint8)globalReg;
            break;
        case VP_INPUT_CLK_FLT:
            if (globalReg & (VP792_REG_CLK_STATUS_PCLK_FAIL_MASK |
                VP792_REG_CLK_STATUS_FS_FAIL_MASK |
                VP792_REG_CLK_STATUS_CFAIL_MASK)) {
                result = 0xFF;
            }
            break;
        case VP_INPUT_BAT1_FLT: /* VBL */
            if (globalReg & VP792_REG_SYS_STAT_LBF_MASK) {
                result = 0xFF;
            }
            break;
        case VP_INPUT_BAT2_FLT:
            if (globalReg & VP792_REG_SYS_STAT_HBF_MASK) {
                result = 0xFF;
            }
            break;
        case VP_INPUT_BAT3_FLT:
            if (globalReg & VP792_REG_SYS_STAT_PBF_MASK) {
                result = 0xFF;
            }
            break;
        case VP_INPUT_AC_FLT:
        case VP_INPUT_DC_FLT:
        case VP_INPUT_THERM_FLT: {

            /* Choose the relevant bit in the channel supervision
               register. */
            uint16 flagMask;
            if (input == VP_INPUT_AC_FLT) {
                flagMask = VP792_REG_CH_SUP_ACFT_ST_MASK;
            } else if (input == VP_INPUT_DC_FLT) {
                flagMask = VP792_REG_CH_SUP_DCFT_ST_MASK;
            } else {
                flagMask = VP792_REG_CH_SUP_TFT_ST_MASK ;
            }

            /* Set the result bit for each channel. */
            for (chan = 0; chan < pDevObj->maxChannels; chan++) {
                uint8 chanBit = (1 << chan);
                if ((globalReg & chanBit) && (chanReg[chan] & flagMask)) {
                    result |= chanBit;
                }
            }
            break;
        }
        default: /* can't happen */
            break;
    }

    pDeviceStatus->status[0] = result;
    return status;
} /* Vp792GetDeviceStatusExt() */


bool
Vp792GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 intInd[VP792_REG_INTIND_LEN];
    uint16 eventId;
    uint16 eventParam;
    bool gotEvent = FALSE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792GetEvent()+"));

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    do {

        /* Check to see if there is a response in the cache.  If so, handle
           the cached response as if a MB_RSP event had been received. */
        if (pDevObj->respMboxCache.count > 0) {
            eventId = VP792_EVID_MB_RSP;
            eventParam = 0; /* not used */
        }

        /* Check to see if we have already read the event ID and parameter in
           a previous call to VpGetEvent(). */
        else if (pDevObj->intInd[VP792_REG_INTIND_EVID_WORD] != VP792_REG_INTIND_NO_EVENT) {

            /* Get previous ID and param from device object. */
            eventId = pDevObj->intInd[VP792_REG_INTIND_EVID_WORD];
            eventParam = pDevObj->intInd[VP792_REG_INTIND_PARM_WORD];
            CopyIntInd(pDevObj->intInd, 0, 0);
            pEvent->status = VP_STATUS_SUCCESS;
        } else {

            /* Otherwise, read event ID and parameter from the device. */
            pEvent->status = Vp792HbiDirectPageRead(pDevCtx,
                VP792_REG_INTIND_OFFSET, VP792_REG_INTIND_LEN, intInd);
            if (pEvent->status != VP_STATUS_SUCCESS) {
                eventId = VP792_REG_INTIND_NO_EVENT;
                break;
            }
            eventId = intInd[VP792_REG_INTIND_EVID_WORD];
            eventParam = intInd[VP792_REG_INTIND_PARM_WORD];

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
            if (pDevObj->options.debugSelect & VP_DBG_INFO) {
                DisplayVp792Event(deviceId, eventId, eventParam);
            }
#endif
        }

        /* If no event, return FALSE. */
        if (eventId == VP792_REG_INTIND_NO_EVENT) {
            break;
        }

        /* Pass the event ID and parameter to GetEventInternal().  If event is
           handled internally, gotEvent will be FALSE.  In that case, repeat
           the loop. */
        pEvent->pDevCtx = pDevCtx;
        gotEvent = GetEventInternal(pEvent, eventId, eventParam);

        /* ...unless an error occurred. */
        if (pEvent->status != VP_STATUS_SUCCESS) {
            break;
        }

        /* If no API-level event was generated, GetEventInternal() handled the
           event internally.  Repeat the loop to check for more interrupts. */
    } while (!gotEvent);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792GetEvent()-"));

    return gotEvent;
} /* Vp792GetEvent() */

VpStatusType
Vp792GetLineStatus(
    VpLineCtxType *pLineCtx,
    VpInputType input,
    bool *pStatus)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;
    uint8 offset;
    uint16 regData;
    uint16 flagMask;
    bool global;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Decide which register to read, and which bit(s) to look at, based on
       the input type. */
    switch(input) {
        case VP_INPUT_HOOK:
            if (pLineObj->options.pulseMode == VP_OPTION_PULSE_DECODE_ON) {
                offset = VP792_REG_GHOOK_OFFSET;
            } else {
                offset = VP792_REG_GRHOOK_OFFSET;
            }
            global = TRUE;
            flagMask = (1 << channelId);
            break;
        case VP_INPUT_RAW_HOOK:
            offset = VP792_REG_GRHOOK_OFFSET;
            global = TRUE;
            flagMask = (1 << channelId);
            break;
        case VP_INPUT_GKEY:
            offset = VP792_REG_GGKEY_OFFSET;
            global = TRUE;
            flagMask = (1 << channelId);
            break;
        case VP_INPUT_THERM_FLT:
            offset = VP792_REG_CH_SUP_OFFSET;
            global = FALSE;
            flagMask = VP792_REG_CH_SUP_TFT_ST_MASK;
            break;
        case VP_INPUT_AC_FLT:
            offset = VP792_REG_CH_SUP_OFFSET;
            global = FALSE;
            flagMask = VP792_REG_CH_SUP_ACFT_ST_MASK;
            break;
        case VP_INPUT_DC_FLT:
            offset = VP792_REG_CH_SUP_OFFSET;
            global = FALSE;
            flagMask = VP792_REG_CH_SUP_DCFT_ST_MASK;
            break;
        case VP_INPUT_CLK_FLT:
            offset = VP792_REG_CLK_STATUS_OFFSET;
            global = TRUE;
            flagMask = (VP792_REG_CLK_STATUS_PCLK_FAIL_MASK |
                VP792_REG_CLK_STATUS_FS_FAIL_MASK |
                VP792_REG_CLK_STATUS_CFAIL_MASK);
            break;
        case VP_INPUT_BAT1_FLT:
            offset = VP792_REG_SYS_STAT_OFFSET;
            global = TRUE;
            flagMask = VP792_REG_SYS_STAT_LBF_MASK;
            break;
        case VP_INPUT_BAT2_FLT:
            offset = VP792_REG_SYS_STAT_OFFSET;
            global = TRUE;
            flagMask = VP792_REG_SYS_STAT_HBF_MASK;
            break;
        case VP_INPUT_BAT3_FLT:
            offset = VP792_REG_SYS_STAT_OFFSET;
            global = TRUE;
            flagMask = VP792_REG_SYS_STAT_PBF_MASK;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Read the appropriate global or channel register. */
    if (global) {
        status = Vp792HbiDirectPageRead(pDevCtx, offset, 1, &regData);
    } else {
        status = Vp792HbiPagedRead(pDevCtx, channelId, offset, 1, &regData);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    *pStatus = ((regData & flagMask) != 0);
    return status;
} /* Vp792GetLineStatus() */

VpStatusType
Vp792GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;
    uint16 requestIdx;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Save arguments for later.  The actual register read operations
           are done in VpGetResults(). */
        pRequest->requestType = VP792_REQUEST_GET_LOOP_COND;
        pRequest->args.getLoopCond.handle = handle;

        /* Generate a VP792_EVID_USER event, which VpGetEvent() will
           translate into a RD_LOOP event. */
        status = Vp792GenerateUserEvent(pDevCtx,
            VP792_UEVID_RESPONSE_FIRST + requestIdx, channelId);
        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792GetLoopCond() */

VpStatusType
Vp792GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType optionId,
    uint16 handle)
{
    Vp792DeviceObjectType *pDevObj;
    VpDeviceIdType deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channelId;
    uint16 requestIdx;

    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    pDevObj = pDevCtx->pDevObj;
    deviceId = pDevObj->deviceId;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineCtx != VP_NULL) {
        Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    } else {
        channelId = VP_ALL_LINES;
    }

    /* The following options are always line-specific, so pLineCtx is required. */
    switch (optionId) {

        /* Line-specific options */
        case VP_OPTION_ID_PULSE:
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_PCM_HWY:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_ZERO_CROSS:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_LINE_IO_CFG:
        case VP_OPTION_ID_DTMF_SPEC:
        case VP_OPTION_ID_DTMF_MODE:
        case VP_OPTION_ID_DCFEED_SLOPE:
            if (pLineCtx == VP_NULL) {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        /* Unsupported options */
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_RAMP2STBY:
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
        case VP_DEVICE_OPTION_ID_PULSE2:
        case VP_OPTION_ID_SWITCHER_CTRL:
            return VP_STATUS_OPTION_NOT_SUPPORTED;

        default:
            break;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Save arguments for later.  In most cases (except below) the actual
           register read operation is done in VpGetResults(). */
        pRequest->requestType = VP792_REQUEST_GET_OPTION;
        pRequest->args.getOption.handle = handle;
        pRequest->args.getOption.optionId = optionId;
        pRequest->args.getOption.channelId = channelId;

        /* The following special cases require a mailbox read command: */
        if (optionId == VP_OPTION_ID_PULSE) {
            uint16 buf[VP792_CMDSIZE_RD_PULSE_PARAMS], *pBufEnd = buf;
            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_RD_PULSE_PARAMS, channelId);
            *pBufEnd++ = VP792_UEVID_RESPONSE_FIRST + requestIdx;
            status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_RD_PULSE_PARAMS);
        } else if (optionId == VP_OPTION_ID_LOOPBACK) {
            uint16 buf[VP792_CMDSIZE_RD_AFE_CONFIG], *pBufEnd = buf;
            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_RD_AFE_CONFIG, channelId);
            *pBufEnd++ = VP792_UEVID_RESPONSE_FIRST + requestIdx;
            status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_RD_AFE_CONFIG);
        }

        /* ...but in most cases, we just generate a VP792_EVID_USER event
           here, which VpGetEvent() will translate into a RD_OPTION event.
           Later, vpGetResults() will actually read the SLAC registers. */
        else {
            if (channelId == VP_ALL_LINES) {

                /* All user events are channel specific, so we arbitrarily
                   use channel 0 here. */
                channelId = 0;
            }
            status = Vp792GenerateUserEvent(pDevCtx,
                VP792_UEVID_RESPONSE_FIRST + requestIdx, channelId);
        }
        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792GetOption() */

VpStatusType
Vp792GetResults(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 requestIdx = pDevObj->requestIdx;
    Vp792ResponseRequestType *pRequest = &pDevObj->responseRequest[requestIdx];
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792GetResults()+"));

    if (pResults == VP_NULL) {
        /* User wants to know what kind of results are available. */
        if (requestIdx >= VP792_MAX_OUTSTANDING_RESPONSES) {
            return VP_STATUS_MAILBOX_EMPTY;
        } else if (pRequest->outstanding) {
            Vp792IdentifyResponseEvent(pEvent, 0, requestIdx);
        } else {
            pEvent->eventId = 0;
        }
        return VP_STATUS_SUCCESS;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetResultsInt(pEvent, pResults);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792GetResults()-"));

    return status;
} /* Vp792GetResults() */

VpStatusType
Vp792GetResultsInt(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 requestIdx = pDevObj->requestIdx;
    Vp792ResponseRequestType *pRequest = &pDevObj->responseRequest[requestIdx];

    if (requestIdx >= VP792_MAX_OUTSTANDING_RESPONSES) {
        return VP_STATUS_MAILBOX_EMPTY;
    } else if (pRequest->outstanding == FALSE) {
        return VP_STATUS_MAILBOX_EMPTY;
    } else {
        if (pRequest->requestType != VP792_REQUEST_TEST_CMP) {
            /* Make sure the event ID and category are correct. */
            VpEventType tempEvent;
            tempEvent.pDevCtx = pDevCtx;
            Vp792IdentifyResponseEvent(&tempEvent, 0, requestIdx);
            if (
                (tempEvent.eventCategory != pEvent->eventCategory) ||
                (tempEvent.eventId != pEvent->eventId) ||
                (tempEvent.hasResults == FALSE)
            ) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792GetResultsInt() returning ERR_MAILBOX_DATA"));
                Vp792RspMailboxRelease(pDevCtx);
                return VP_STATUS_ERR_MAILBOX_DATA;
            }
        }
    }

    /* Read the response mailbox or the appropriate register. */
    return ReadResponse(pEvent, requestIdx, pResults);
} /* Vp792GetResultsInt() */

bool
Vp792GroupGetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)
{
    return FALSE;
}

/*
 * This function fills in the event struct for response events (i.e., events
 * with hasResults = TRUE), based on the contents of the supplied
 * responseRequest[] struct.
 *
 * If no external event is to be generated (i.e., this is an internal response)
 * this function returns FALSE.
 *
 * If an external event is to be generated, this function returns TRUE and sets
 * the pDevObj->responseIdx field, so that when VpGetResults() is called, it
 * will know what kind of result to return.
 *
 * This function is used:
 *
 *  + In VpGetEvent(), to fill in the event struct when a response event is to
 *    be generated.  There are two cases of this: "real" response events and
 *    "fake" response events.
 *     + For "real" response events, the SLAC generated an actual MB_RSP event.
 *       The HandleFirmwareEvent() function gets the response index from the
 *       handle field in the SLAC's response mailbox.  It passes that index to
 *       this function.
 *     + For "fake" response events, the API wants to generate an event with
 *       hasResults = TRUE, but it doesn't actually correspond to a SLAC mailbox
 *       read command.  In this case, the API requests a USER event from the
 *       SLAC.  When the event arrives, HandleUserEvent() gets the response
 *       index from the USER event ID.  It passes that index to this function.
 *    This function looks in the responseRequest[] array at the specified index,
 *    to find all relevant information about the response.  It fills in the
 *    event struct accordingly, with hasResults = TRUE.  It then sets the
 *    pDevObj->responseIdx field for VpGetResults(), and returns TRUE.
 *    VpGetEvent() then returns TRUE.
 *
 *  + In VpGetEvent(), when an internal MB_RSP event needs to be processed.
 *    In this case, HandleFirmwareEvent() gets the response index from the
 *    handle field in the SLAC's response mailbox.  It passes that index to this
 *    function.  This function looks in the responseRequest[] array at the
 *    specified index, to find all relevant information about the response.
 *    Having determined that this is an internal response, this function calls
 *    the appropriate internal handler function.  The handler function may or
 *    may not generate an external event by updating the event struct and
 *    returning TRUE.
 *
 *  + In VpGetResults(), to verify that the kind of response "in the mailbox"
 *    matches the event type in the event struct passed by the user.  In this
 *    case, VpGetResults() passes idx = pDevObj->responseIdx to this function.
 *    This function looks in the responseRequest[] array at the specified index,
 *    to find all relevant information about the response, and fills in the
 *    event ID and category in the event struct.  VpGetResults() compares these
 *    values with the values in the user-supplied event struct to make sure they
 *    match.
 *
 *  + In VpGetResults(), when pResults = NULL, to identify what kind of response
 *    is "in the mailbox".  In this case VpGetResults() passes idx =
 *    pDevObj->responseIdx to this function.  This function looks in the
 *    responseRequest[] array at the specified index, to find all relevant
 *    information about the response, and fills in the event struct.
 */
bool
Vp792IdentifyResponseEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 idx)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792ResponseRequestType *pRequest = &pDevObj->responseRequest[idx];

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792IdentifyResponseEvent(idx = %d)+", idx));

    /* Are we expecting this response? */
    if (
        (idx >= VP792_MAX_OUTSTANDING_RESPONSES) ||
        (pDevObj->responseRequest[idx].outstanding == FALSE)
    ) {
        return UnexpectedEvent(pEvent, eventIdWord, idx);
    }

    pEvent->eventCategory = VP_EVCAT_RESPONSE;
    pEvent->hasResults = TRUE;

    /* Determine what kind of event should be generated. */
    switch (pRequest->requestType) {
        case VP792_REQUEST_SET_REL_GAIN:
            pEvent->eventId = VP_LINE_EVID_GAIN_CMP;
            pEvent->parmHandle = pRequest->args.setRelGain.handle;
            pEvent->channelId = pRequest->args.setRelGain.channelId;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            {
                Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
                pEvent->lineId = pLineObj->lineId;
            }
            break;
        case VP792_REQUEST_GET_LOOP_COND:
            pEvent->eventId = VP_LINE_EVID_RD_LOOP;
            pEvent->parmHandle = pRequest->args.getLoopCond.handle;
            break;
        case VP792_REQUEST_DEVICE_IO_ACCESS_EXT:
            pEvent->eventId = VP_DEV_EVID_IO_ACCESS_CMP;
            pEvent->eventData = (uint16)pRequest->args.deviceIoAccessExt.deviceIoAccess.direction;
            break;
        case VP792_REQUEST_LINE_IO_ACCESS:
            if (pRequest->args.lineIoAccess.lineIoAccess.direction == VP_IO_READ) {
                pEvent->eventId = VP_LINE_EVID_LINE_IO_RD_CMP;
                /* The response struct will be cleared in VpGetResults(). */
            } else {
                pEvent->eventId = VP_LINE_EVID_LINE_IO_WR_CMP;
                pEvent->hasResults = FALSE;
                /* Free the response struct now, because VpGetResults() won't
                   be called. */
                pRequest->outstanding = FALSE;
            }
            pEvent->parmHandle = pRequest->args.lineIoAccess.handle;
            break;
        case VP792_REQUEST_QUERY:
            pEvent->eventId = VP_LINE_EVID_QUERY_CMP;
            pEvent->parmHandle = pRequest->args.query.handle;
            break;
        case VP792_REQUEST_GET_OPTION:
            pEvent->eventId = VP_LINE_EVID_RD_OPTION;
            pEvent->parmHandle = pRequest->args.getOption.handle;
            pEvent->eventData = (uint16)pRequest->args.getOption.optionId;
            pEvent->channelId = pRequest->args.getOption.channelId;
            if (pEvent->channelId == VP_ALL_LINES) {
                /* A device-specific option was read, but VP792_EVID_USER
                   events always have a specific channel ID, so
                   GetEventInternal() will have set the pLineCtx member. */
                pEvent->pLineCtx = VP_NULL;
            } else {
                /* Could be a MB_RSP event, in which case the pLineCtx member
                   was cleared in HandleFirmwareEvent(). */
                pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
                {
                    Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
                    pEvent->lineId = pLineObj->lineId;
                }
            }
            break;
        case VP792_REQUEST_LOW_LEVEL_CMD_16:
            pEvent->eventId = VP_LINE_EVID_LLCMD_RX_CMP;
            pEvent->channelId = pRequest->args.lowLevelCmd16.channelId;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            pEvent->parmHandle = pRequest->args.lowLevelCmd16.handle;
            pEvent->eventData = pRequest->args.lowLevelCmd16.numReadWords * 2;
            {
                Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
                pEvent->lineId = pLineObj->lineId;
            }
            break;
        case VP792_REQUEST_TEST_INTERNAL:      /* todo:  This needs to be refined.  -- RTL */
            pEvent->eventCategory = VP_EVCAT_TEST;
            pEvent->eventId = pRequest->args.testLine.eventId;
            pEvent->channelId = pRequest->args.testLine.channelId;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            pEvent->parmHandle = pRequest->args.testLine.handle;
            break;
        case VP792_REQUEST_TEST_CMP:
            pEvent->eventCategory = VP_EVCAT_TEST;
            pEvent->eventId = VP_LINE_EVID_TEST_CMP;
            pEvent->channelId = pRequest->args.testLine.channelId;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            pEvent->parmHandle = pRequest->args.testLine.handle;
            break;
        case VP792_REQUEST_SENDSIG_MSGWAIT: {
            Vp792LineObjectType *pLineObj;
            pEvent->channelId = pRequest->args.lowLevelCmd16.channelId;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            pLineObj = pEvent->pLineCtx->pLineObj;
            if (pLineObj->eventHandlers & VP792_EH_SEND_SIGNAL) {
                return EhSendSignal(pEvent, eventIdWord, idx);
            }
            /* Else, fall through to default case (unexpected event) */
        }
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("ProcessResponse(): Invalid requestType (%u)", pRequest->requestType));
            return UnexpectedEvent(pEvent, eventIdWord, idx);
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp792IdentifyResponseEvent()-"));

    /* Save the request index for VpGetResults(). */
    pDevObj->requestIdx = idx;
    return TRUE;
}

/* This is not a top-level API function, so it really belongs in vp792_common.c.
   However, moving this function there would require moving a lot of other static
   functions there and making them extern. */
VpStatusType
Vp792NewSequence(
    VpLineCtxType *pLineCtx,
    uint16 eventHandler,
    uint16 *pCallerBuf,
    uint8 callerBufLen)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 slacId = pDevObj->slacId;
    uint16 channelId = pLineObj->channelId;
    uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE], *pBufEnd = buf;
    VpStatusType status = VP_STATUS_SUCCESS;

    /* If the caller already has a mailbox command to send, add it to the buffer
       first. */
    if (callerBufLen > 0) {
        VpMemCpy(buf, pCallerBuf, callerBufLen * 2);
        pBufEnd += callerBufLen;
    }

    if (pLineObj->sequencer.activeSequence != VP792_SEQTYPE_OFF) {

        /* If a sequence is currently running, we must stop it before starting a
           new sequence.  (If we already sent an abort command, we don't need to
           send another one.) */
        if (pLineObj->sequencer.aborting == FALSE) {
            pLineObj->sequencer.aborting = TRUE;

            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);
            *pBufEnd++ = VP792_SEQ_CTL_STOP;

            /* We also generate a VP792_UEVID_SEQUENCER_READY event so we know when
               to start the new sequence. */
            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_USER_CTL, channelId);
            *pBufEnd++ = VP792_UEVID_SEQUENCER_READY;

            /* Send the commands to the SLAC. */
            status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
        }

        if (status == VP_STATUS_SUCCESS) {
            if (eventHandler == VP792_EH_SET_LINE_STATE) {
                /* VpSetLineTone(), VpSendCid(), VpSetLineState(), or
                   VpSendSignal() may already be waiting for the sequencer,
                   but these are all wiped out by the new VpSetLineState()
                   call. */
                pLineObj->sequencer.deferredHandlers &= ~VP792_EH_SEQ_STOP_MASK;
            } else {
                /* A new VpSetLineTone(), VpSendCid(), or VpSendSignal() call
                   was made.  If a previous VpSetLineState() call was made,
                   we still want to process that, but all previous calls to
                   VpSetLineTone(), VpSendCid(), or VpSendSignal() can be wiped
                   out. */
                pLineObj->sequencer.deferredHandlers &=
                    ~(VP792_EH_SEQ_STOP_MASK & ~VP792_EH_SET_LINE_STATE);
            }
            pLineObj->sequencer.deferredHandlers |= eventHandler;

            /* The reason we need a 'deferredHandlers' field is that we don't
               want to remove any ongoing event handlers that might be waiting
               for the SEQ_CMP event.  Such handlers might want to do some
               cleanup at the end of the sequence.  So we don't modify the
               main eventHandlers field until after the SEQ_CMP event. */

            /* When the VP792_UEVID_SEQUENCER_READY event arrives,
               HandleUserEvent() will activate the deferredHandlers
               by copying the bits into the main eventHandlers field. */
        }

    } else {

        /* If no sequence is running, we can start the new sequence immediately.
           For code simplicity, we just generate and process a fake
           VP792_UEVID_SEQUENCER_READY event. */
        uint16 eventIdWord = (slacId << VP792_INTIND_SLACID_POS) |
            (channelId << VP792_INTIND_FW_CHANID_POS) | VP792_EVID_USER;
        uint16 eventParamWord = VP792_UEVID_SEQUENCER_READY;
        VpEventType fakeEvent;
        fakeEvent.pDevCtx = pDevCtx;

        /* Don't need to send a sequencer-abort mailbox command, but we do need
           to send the caller's mailbox command, if any. */
        if (callerBufLen > 0) {
            status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
            if (status != VP_STATUS_SUCCESS) {
                return status;
            }
        }

        /* Activate the event handler to process the fake event: */
        pLineObj->eventHandlers &= ~VP792_EH_SEQ_STOP_MASK;
        pLineObj->eventHandlers |= eventHandler;

        /* Send the fake event to the normal event processing function: */
        GetEventInternal(&fakeEvent, eventIdWord, eventParamWord);
        status = fakeEvent.status;
    }

    return status;
}

#if (VP_CC_DEBUG_SELECT && VP_DBG_INFO)
VpStatusType
Vp792ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    if (pDevCtx != VP_NULL) {
        int profType, idx, i;
        Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

        VpSysDebugPrintf("\nDumping VP_DEV_792_SERIES Device Object:\n");
        VpSysDebugPrintf("\tpDevCtx->pDevObj->deviceId =");
        VP_PRINT_DEVICE_ID(pDevObj->deviceId);
        VpSysDebugPrintf("\n\tpDevCtx->pDevObj->slacId = %d\n", (int)pDevObj->slacId);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->maxChannels = %d\n", (int)pDevObj->maxChannels);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->rev.device = 0x%4.4X\n", (int)pDevObj->rev.device);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->rev.product = 0x%4.4X\n", (int)pDevObj->rev.product);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->rev.version = 0x%4.4X\n", (int)pDevObj->rev.version);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->rev.patchAddress = 0x%8.8X\n", (int)pDevObj->rev.patchAddress);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->devInit = %d\n", (int)pDevObj->devInit);

        VpSysDebugPrintf("\tpDevCtx->pDevObj->profTable:\n");

        for (profType = 0; profType < VP792_NUM_PROFILE_TYPES; profType++) {
            VpProfilePtrType pProfile = VP_NULL;
            int profTableSize = 0, maxLen = 0;
            char *memberStr = "(error)";

            for (idx = 0; idx < 16; idx++) {
                switch (profType) {
                    case VP_PROFILE_DEVICE:
                        profTableSize = VP_792_DEV_PROF_TABLE_SIZE;
                        maxLen = VP792_DEV_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pDevice[idx];
                        memberStr = "pDevice";
#else
                        pProfile = pDevObj->profTable.device[idx];
                        memberStr = "device";
#endif
                        break;
                    case VP_PROFILE_AC:
                        profTableSize = VP_792_AC_PROF_TABLE_SIZE;
                        maxLen = VP792_AC_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pAc[idx];
                        memberStr = "pAc";
#else
                        pProfile = pDevObj->profTable.ac[idx];
                        memberStr = "ac";
#endif
                        break;
                    case VP_PROFILE_DC:
                        profTableSize = VP_792_DC_PROF_TABLE_SIZE;
                        maxLen = VP792_DC_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pDc[idx];
                        memberStr = "pDc";
#else
                        pProfile = pDevObj->profTable.dc[idx];
                        memberStr = "dc";
#endif
                        break;
                    case VP_PROFILE_RING:
                        profTableSize = VP_792_RINGING_PROF_TABLE_SIZE;
                        maxLen = VP792_RING_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pRinging[idx];
                        memberStr = "pRinging";
#else
                        pProfile = pDevObj->profTable.ringing[idx];
                        memberStr = "ringing";
#endif
                        break;
                    case VP_PROFILE_RINGCAD:
                        profTableSize = VP_792_RING_CADENCE_PROF_TABLE_SIZE;
                        maxLen = VP792_RINGCAD_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pRingCad[idx];
                        memberStr = "pRingCad";
#else
                        pProfile = pDevObj->profTable.ringCad[idx];
                        memberStr = "ringCad";
#endif
                        break;
                    case VP_PROFILE_TONE:
                        profTableSize = VP_792_TONE_PROF_TABLE_SIZE;
                        maxLen = VP792_TONE_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pTone[idx];
                        memberStr = "pTone";
#else
                        pProfile = pDevObj->profTable.tone[idx];
                        memberStr = "tone";
#endif
                        break;
                    case VP_PROFILE_METER:
                        profTableSize = VP_792_METERING_PROF_TABLE_SIZE;
                        maxLen = VP792_MTR_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pMetering[idx];
                        memberStr = "pMetering";
#else
                        pProfile = pDevObj->profTable.metering[idx];
                        memberStr = "metering";
#endif
                        break;
                    case VP_PROFILE_CID:
                        profTableSize = VP_792_CALLERID_PROF_TABLE_SIZE;
                        maxLen = VP792_CID_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pCallerId[idx];
                        memberStr = "pCallerId";
#else
                        pProfile = pDevObj->profTable.callerId[idx];
                        memberStr = "callerId";
#endif
                        break;
                    case VP_PROFILE_TONECAD:
                        profTableSize = VP_792_TONE_CADENCE_PROF_TABLE_SIZE;
                        maxLen = VP792_TONECAD_PROFILE_LEN;
#ifdef VP_COMMON_ADDRESS_SPACE
                        pProfile = pDevObj->profTable.pToneCad[idx];
                        memberStr = "pToneCad";
#else
                        pProfile = pDevObj->profTable.toneCad[idx];
                        memberStr = "toneCad";
#endif
                    default:
                        break;
                }

                if (idx >= profTableSize) {
                    break;
                }

#ifndef VP_COMMON_ADDRESS_SPACE
                {
                    uint16 bit = (1 << idx);
                    if ((pDevObj->profTable.valid[profType] & bit) == 0) {
                        pProfile = VP_NULL;
                    }
                }
#endif

                if (pProfile != VP_NULL) {
                    VpSysDebugPrintf("\tpDevCtx->pDevObj->profTable.%s[%d] = %lu {", memberStr, idx, (unsigned long)pProfile);
                    DisplayVp792Profile(pProfile, maxLen);
                    VpSysDebugPrintf(" }\n");
                }
            }
        }

        VpSysDebugPrintf("\tpDevCtx->pDevObj->pcmClockRate = %d kHz\n", (int)pDevObj->pcmClockRate);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->timeStampHiBits = 0x%4.4X\n", (int)pDevObj->timeStampHiBits);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.criticalFlt.acFltDiscEn = %d\n", (int)pDevObj->options.criticalFlt.acFltDiscEn);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.criticalFlt.dcFltDiscEn = %d\n", (int)pDevObj->options.criticalFlt.acFltDiscEn);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.criticalFlt.thermFltDiscEn = %d\n", (int)pDevObj->options.criticalFlt.acFltDiscEn);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.faults = 0x%4.4X\n", (int)pDevObj->options.eventMask.faults);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.signaling = 0x%4.4X\n", (int)pDevObj->options.eventMask.signaling);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.response = 0x%4.4X\n", (int)pDevObj->options.eventMask.response);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.test = 0x%4.4X\n", (int)pDevObj->options.eventMask.test);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.process = 0x%4.4X\n", (int)pDevObj->options.eventMask.process);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.eventMask.fxo = 0x%4.4X\n", (int)pDevObj->options.eventMask.fxo);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.pulseEnabled = 0x%4.4X\n", (int)pDevObj->options.pulseEnabled);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.dtmfEnabled = 0x%4.4X\n", (int)pDevObj->options.dtmfEnabled);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.parkMode.discTime = 0x%4.4X\n", (int)pDevObj->options.parkMode.discTime);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.parkMode.standbyTime = 0x%4.4X\n", (int)pDevObj->options.parkMode.standbyTime);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->options.debugSelect = 0x%4.4X\n", (int)pDevObj->options.debugSelect);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->intInd[2] = { 0x%4.4X 0x%4.4X }\n", (int)pDevObj->intInd[0], (int)pDevObj->intInd[1]);

        for (idx = 0; idx < VP792_MAX_OUTSTANDING_RESPONSES; idx++) {
            if (pDevObj->responseRequest[idx].outstanding) {
                VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].outstanding = %d\n", idx, (int)pDevObj->responseRequest[idx].outstanding);
                VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].requestType = %d\n", idx, (int)pDevObj->responseRequest[idx].requestType);
                switch (pDevObj->responseRequest[idx].requestType) {
                    case VP792_REQUEST_SET_REL_GAIN:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.setRelGain.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.setRelGain.handle);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.setRelGain.channelId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.setRelGain.channelId);
                        break;
                    case VP792_REQUEST_GET_LOOP_COND:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.getLoopCond.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.getLoopCond.handle);
                        break;
                    case VP792_REQUEST_GET_OPTION:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.getOption.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.getOption.handle);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.getOption.channelId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.getOption.channelId);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.getOption.optionId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.getOption.optionId);
                        break;
                    case VP792_REQUEST_DEVICE_IO_ACCESS_EXT:
                        break;
                    case VP792_REQUEST_LINE_IO_ACCESS:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lineIoAccess.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.lineIoAccess.handle);
                        break;
                    case VP792_REQUEST_QUERY:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.query.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.query.handle);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.query.queryId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.query.queryId);
                        break;
                    case VP792_REQUEST_LOW_LEVEL_CMD_16:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.handle);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.channelId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.channelId);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.cmdType = %d\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.cmdType);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.writeWords[2] = { 0x%4.4X 0x%4.4X }\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.writeWords[0], (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.writeWords[1]);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.numWriteWords = %d\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.numWriteWords);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.lowLevelCmd16.numReadWords = %d\n", idx, (int)pDevObj->responseRequest[idx].args.lowLevelCmd16.numReadWords);
                        break;
                    case VP792_REQUEST_SENDSIG_MSGWAIT:
                        break;
                    case VP792_REQUEST_TEST_INTERNAL:
                        break;
                    case VP792_REQUEST_TEST_CMP:
                        break;
                    case VP792_REQUEST_TIMER_CMP:
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.genTimer.handle = 0x%4.4X\n", idx, (int)pDevObj->responseRequest[idx].args.genTimer.handle);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.genTimer.channelId = %d\n", idx, (int)pDevObj->responseRequest[idx].args.genTimer.channelId);
                        VpSysDebugPrintf("\tpDevCtx->pDevObj->responseRequest[%d].args.genTimer.internal = %d\n", idx, (int)pDevObj->responseRequest[idx].args.genTimer.internal);
                        break;
                    default:
                        break;
                }
            }
        }

        VpSysDebugPrintf("\tpDevCtx->pDevObj->requestIdx = 0x%4.4X\n", (int)pDevObj->requestIdx);
        VpSysDebugPrintf("\tpDevCtx->pDevObj->respMboxCache.count = %d\n", (int)pDevObj->respMboxCache.count);
        for (idx = 0; idx < pDevObj->respMboxCache.count ; idx++) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->respMboxCache.data[%d][] = {", idx);
            for (i = 0; i < VP792_MAX_RSPSIZE; i++) {
                VpSysDebugPrintf(" 0x%4.4X", (int)pDevObj->respMboxCache.data[idx][i]);
            }
            VpSysDebugPrintf("\n");
        }

        VpSysDebugPrintf("\tpDevCtx->pDevObj->eventHandlers = 0x%4.4X\n", (int)pDevObj->eventHandlers);

#ifdef VP_COMMON_ADDRESS_SPACE
        if (pDevObj->ehInitDevice.pDevProfile != VP_NULL) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.pDevProfile = %lu {", (unsigned long)pDevObj->ehInitDevice.pDevProfile);
            DisplayVp792Profile(pDevObj->ehInitDevice.pDevProfile, VP792_DEV_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#else
        if (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_DEVICE)) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.devProfile = {");
            DisplayVp792Profile(pDevObj->ehInitDevice.devProfile, VP792_DEV_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#endif

#ifdef VP_COMMON_ADDRESS_SPACE
        if (pDevObj->ehInitDevice.pAcProfile != VP_NULL) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.pAcProfile = %lu {", (unsigned long)pDevObj->ehInitDevice.pAcProfile);
            DisplayVp792Profile(pDevObj->ehInitDevice.pAcProfile, VP792_AC_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#else
        if (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_AC)) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.acProfile = {");
            DisplayVp792Profile(pDevObj->ehInitDevice.acProfile, VP792_AC_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#endif

#ifdef VP_COMMON_ADDRESS_SPACE
        if (pDevObj->ehInitDevice.pDcProfile != VP_NULL) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.pDcProfile = %lu {", (unsigned long)pDevObj->ehInitDevice.pDcProfile);
            DisplayVp792Profile(pDevObj->ehInitDevice.pDcProfile, VP792_DC_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#else
        if (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_DC)) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.dcProfile = {");
            DisplayVp792Profile(pDevObj->ehInitDevice.dcProfile, VP792_DC_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#endif

#ifdef VP_COMMON_ADDRESS_SPACE
        if (pDevObj->ehInitDevice.pRingProfile != VP_NULL) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.pRingProfile = %lu {", (unsigned long)pDevObj->ehInitDevice.pRingProfile);
            DisplayVp792Profile(pDevObj->ehInitDevice.pRingProfile, VP792_RING_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#else
        if (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_RING)) {
            VpSysDebugPrintf("\tpDevCtx->pDevObj->ehInitDevice.devProfile = {");
            DisplayVp792Profile(pDevObj->ehInitDevice.ringProfile, VP792_RING_PROFILE_LEN);
            VpSysDebugPrintf(" }\n");
        }
#endif

    } else {
        VpSysDebugPrintf("\nDumping VP_DEV_792_SERIES Line Object:\n\t(TBI)\n");
    }

    return status;
}
#endif /* (VP_CC_DEBUG_SELECT && VP_DBG_INFO) */


/* ==================
    Static Functions
   ================== */

/******************************************************************************
 * This function indicates whether the event contained in the pEvent struct is
 * currently masked.
 *
 * \param[in,out] pEvent Event Struct for the event which
 *
 * \retval ::TRUE   The event is not masked.
 * \retval ::FALSE  The event is masked.
 *****************************************************************************/
static bool
CheckEventMask(
    VpEventType *pEvent)
{
    VpOptionEventMaskType eventMask;
    uint16 mask = 0;

    if (pEvent->pLineCtx != VP_NULL) {
        Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
        eventMask = pLineObj->options.eventMask;
    } else {
        Vp792DeviceObjectType *pDevObj = pEvent->pDevCtx->pDevObj;
        eventMask = pDevObj->options.eventMask;
    }

    switch (pEvent->eventCategory) {
        case VP_EVCAT_FAULT:
            mask = eventMask.faults;
            break;
        case VP_EVCAT_SIGNALING:
            mask = eventMask.signaling;
            break;
        case VP_EVCAT_RESPONSE:
            mask = eventMask.response;
            break;
        case VP_EVCAT_TEST:
            mask = eventMask.test;
            break;
        case VP_EVCAT_PROCESS:
            mask = eventMask.process;
            break;
        default: /* can't happen (checked elsewhere) */
            break;
    }

    if (mask & pEvent->eventId) {

        /* Event mask bit 1 => masked. */
        return FALSE;
    } else {

        /* Event mask bit 0 => unmasked. */
        return TRUE;
    }
} /* CheckEventMask() */

static bool
CidAckDetectInterval(
    VpEventType *pEvent,
    int toneIndex1,
    int toneIndex2,
    VpProfilePtrType pCidProfile)
{
    Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
    uint8 *pProfileData;
    uint8 profileDataLength;
    uint8 tone;
    Vp792ProfileSectionType sectionType;
    VpStatusType status = VP_STATUS_SUCCESS;

    /* The VP792_UEVID_ACK_DETECT_.. user events are generated to mark both the
     * beginning and end of the detection interval.  cidDetectInterval is TRUE
     * during the detection interval. */
    if (!pLineObj->callerId.dtmfDetectInterval) {
        sectionType = VP792_PROF_SECTYPE_UNSTRUCTURED;
        status = Vp792ProfileFindSection(pCidProfile, &sectionType, 0, &pProfileData,
            &profileDataLength);

        if (status != VP_STATUS_SUCCESS ||
            sectionType != VP792_PROF_SECTYPE_UNSTRUCTURED)
        {
            VP_ERROR(VpLineCtxType, pEvent->pLineCtx, ("CidAckDetectInterval(): Unstructured data section not found in CID profile"));
            return VP_STATUS_ERR_PROFILE;
        }

        pLineObj->callerId.dtmfTones[0] = VP792_CID_TONE_UNDETECTED;
        pLineObj->callerId.dtmfTones[1] = VP792_CID_TONE_UNDETECTED;

        /* tone 3--. .--tone 2   tone 1--. .--tone 0
         *     0 x # #         ,     0 x # #
         */

        if (toneIndex1 > -1) {
            tone = pProfileData[VP792_CID_PROF_DETECT_TONES + 1 - (toneIndex1 / 2)];
            tone = (uint8)((tone >> ((toneIndex1 & 1) * 8) ) & 0x0F);
            pLineObj->callerId.dtmfTones[0] = tone;
        }
        if (toneIndex2 > -1) {
            tone = pProfileData[VP792_CID_PROF_DETECT_TONES + 1 - (toneIndex2 / 2)];
            tone = (uint8)((tone >> ((toneIndex2 & 1) * 8) ) & 0x0F);
            pLineObj->callerId.dtmfTones[1] = tone;
        }

        pLineObj->callerId.dtmfDetectInterval = TRUE;

    } else {
        if (pLineObj->callerId.dtmfTones[0] == VP792_CID_TONE_DETECTED ||
            pLineObj->callerId.dtmfTones[1] == VP792_CID_TONE_DETECTED) {

            status = SequencerControl(pEvent->pLineCtx, VP792_SEQ_CTL_RESUME);
        } else {
            /* restore things */
            Vp792SetLineState(pEvent->pLineCtx, pLineObj->currentState);

            /* Return the CID_DATA:TX_DONE event from VpGetEvent(). */
            pEvent->eventCategory = VP_EVCAT_PROCESS;
            pEvent->eventId = VP_LINE_EVID_CID_DATA;
            pEvent->eventData = VP_CID_DATA_TX_DONE;
            return TRUE;
        }

        pLineObj->callerId.dtmfDetectInterval = FALSE;
    }

    return FALSE; /* no external event */
} /* CidAckDetectInterval() */

/**
 * ConvertCharToDigitType()
 *  This function is called by the CID sequencer executed internally by the API.
 * It converts a character to a VpDigitType and is used for functions requiring
 * a VpDigitType specifically.
 *
 * Preconditions:
 *  None. Utility function only.
 *
 * Postconditions:
 *  The character passed is converted/returned as a VpDigitType
 */
static VpDigitType
ConvertCharToDigitType(
    char digit)
{
    VpDigitType vpDig;

    switch(digit) {
        case '0':
            vpDig = VP_DIG_ZERO;
            break;

        case 'A':
        case 'a':
            vpDig = VP_DIG_A;
            break;

        case 'B':
        case 'b':
            vpDig = VP_DIG_B;
            break;

        case 'C':
        case 'c':
            vpDig = VP_DIG_C;
            break;

        case 'D':
        case 'd':
            vpDig = VP_DIG_D;
            break;

        case '*':
            vpDig = VP_DIG_ASTER;
            break;

        case '#':
            vpDig = VP_DIG_POUND;
            break;

        default:
            if ((digit <= '9') && (digit > '0')) {
                vpDig = (VpDigitType)(digit - '0');
            } else {
                vpDig = VP_DIG_POUND;
            }
            break;
    }
    return vpDig;
} /* ConvertCharToDigitType */

static void
DeferEvent(
    VpDevCtxType *pDevCtx,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    CopyIntInd(pDevObj->intInd, eventIdWord, eventParamWord);
}

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
static void
DisplayVp792Event(
    VpDeviceIdType deviceId,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    char buf[200], *pBuf = buf;
    uint16 sys = (uint16) (eventIdWord & 0x8000);
    uint16 slacId = (uint16) ((eventIdWord & 0x7000) >> 12);

    if (eventIdWord == 0) {
        return;
    }

    pBuf += sprintf(buf, "vp792 event: %4.4X %4.4X => ", eventIdWord, eventParamWord);

    if (sys) {
        char *bitName[] = {"reset", "preb", "rsvd2", "rsvd3", "hbf", "lbf", "pbf",
            "rsvd7", "rsvd8", "cfail", "wdt", "ovl"};
        int bit;
        bool multi = FALSE;

        pBuf += sprintf(pBuf, "sys dev=%u slac=%u bits=", deviceId, slacId);
        for (bit = 11; bit > -1; bit--) {
            if ((1 << bit) & eventIdWord) {
                pBuf += sprintf(pBuf, "%s%s", (multi ? "|" : ""), bitName[bit]);
                multi = TRUE;
            }
        }
    } else {
        char *evName[] = {"0x00", "boot_cmp", "mb_cmd", "mb_rsp", "mb_err",
            "timer_cmp", "ts_rollover", "cal_cmp", "0x08", "0x09", "reset_cmp",
            "chksum_cmp", "user", "fsk", "ac_flt", "dc_flt", "t_flt",
            "hook", "raw_hook", "gkey", "ring_cad", "digit", "pdigit", "tone",
            "seq", "test", "0x1A", "0x1B", "0x1C", "0x1D", "0x1E", "0x1F"};
        uint16 chanId = (uint16) ((eventIdWord & 0x0E00) >> 9);
        uint16 rsvd = (uint16) ((eventIdWord & 0x01C0) >> 6);
        uint16 evId = (uint16) ((eventIdWord & 0x003E) >> 1);
        uint16 st = (uint16) (eventIdWord & 1);

        pBuf += sprintf(pBuf, "fw dev=%u slac=%u chan=%u ", deviceId, slacId, chanId);
        if (rsvd) {
            pBuf += sprintf(pBuf, "rsvd=%u ", rsvd);
        }

        pBuf += sprintf(pBuf, "evid=%s st=%u", evName[evId], st);
    }

    VP_INFO(VpDeviceIdType, &deviceId, ("%s", buf));
} /* DisplayVp792Event() */

static void
DisplayVp792Profile(
    VpProfilePtrType pProfile,
    uint8 maxLen)
{
    int i;
    uint8 profLen = pProfile[VP792_PROF_FLDOFFSET_CONTENT_LEN] + VP792_PROF_FLDOFFSET_CONTENT;
    if (profLen > maxLen) {
        VpSysDebugPrintf(" [length overflow!]");
        profLen = maxLen;
    }
    for (i = 0; i < profLen; i++) {
        VpSysDebugPrintf(" 0x%2.2X", pProfile[i]);
    }
}
#endif /* VP_CC_DEBUG_SELECT & VP_DBG_INFO */

static bool
EhCal(
    VpEventType *pEvent,
    uint16 eventIdWord)
{
    Vp792DeviceObjectType *pDevObj = pEvent->pDevCtx->pDevObj;
    bool externalEvent = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pEvent->pLineCtx, ("EhCal()+"));

    switch (eventIdWord) {

        case VP792_EVID_SEQ: {

            /* DRIVE_ST change delay is finished.  Run calibration. */
            uint16 buf[VP792_CMDSIZE_CAL_CTL], *pBufEnd = buf;
            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_CAL_CTL, pEvent->channelId);
            pEvent->status = Vp792MailboxSend(pEvent->pDevCtx, buf, pBufEnd, VP792_CMDSIZE_CAL_CTL);
            break;
        }
        case VP792_EVID_CAL_CMP:

            /* Generate CAL_CMP event. */
            pEvent->eventCategory = VP_EVCAT_RESPONSE;
            pEvent->eventId = VP_EVID_CAL_CMP;
            externalEvent = TRUE;

            /* Restore the drive state(s). */
            pEvent->status = RestoreDriveState(pEvent->pDevCtx, pEvent->channelId);

            /* Expect no further events. */
            if (pEvent->channelId == VP_ALL_LINES) {
                pEvent->pLineCtx = VP_NULL;
                pDevObj->eventHandlers &= ~VP792_EH_CAL_CODEC;
                pDevObj->devInit = TRUE;
            } else {
                Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
                pLineObj->eventHandlers &= ~VP792_EH_CAL_LINE;
                pLineObj->lineInit = TRUE;
            }
            break;

        default: /* can't happen */
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pEvent->pLineCtx, ("EhCal()-"));

    return externalEvent;
} /* EhCal() */

/******************************************************************************
 * This function handles the SYS_RESET, BOOT_CMP, and CAL_CMP events that are
 * caused by VpInitDevice().  It continues the device initialization process by
 * calling Vp792BootSlac or Vp792InitDeviceInt().
 *
 * \param[in,out] pEvent        Event struct of the event to be handled
 * \param[in]     eventIdWord   Interrupt indication from VP792 device
 *
 * \retval ::TRUE   A DEV_INIT_CMP event is generated.
 * \retval ::FALSE  No external event is generated.
 *****************************************************************************/
static bool
EhInitDevice(
    VpEventType *pEvent,
    uint16 eventIdWord)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE];
    bool externalEvent = FALSE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("EhInitDevice()+"));

    switch (eventIdWord) {
        case VP792_EVID_BOOT_CMP:

            /* Read the ROM firmware revision code. */
            status = ReadFirmwareRevCode(pDevCtx);
            if (status == VP_STATUS_SUCCESS) {
                VP_INFO(VpDevCtxType, pDevCtx, ("ROM revision info: device 0x%4.4X / product 0x%4.4X / version 0x%4.4X",
                    pDevObj->rev.device, pDevObj->rev.product, pDevObj->rev.version));
            }

            /* ROM firmware boot complete.  Send the PATCH_CTL command (if necessary). */
            if (pDevObj->rev.patchAddress != 0) {
                uint16 *pBufEnd = buf;

                /* Send the PATCH_CTL command to the SLAC. */
                Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_PATCH_CTL, 0);
                *pBufEnd++ = VP792_CMD_PATCH_CTL_CMD_KEY_HIGH;
                *pBufEnd++ = VP792_CMD_PATCH_CTL_CMD_KEY_LOW;
                *pBufEnd++ = (uint16)(pDevObj->rev.patchAddress >> 16);
                *pBufEnd++ = (uint16)pDevObj->rev.patchAddress;

                /* Unmask the USER event. */
                if (status == VP_STATUS_SUCCESS) {
                    uint16 chMask = ~VP792_REG_CH_MASKS_USER_M_MASK;
                    status = Vp792HbiPagedWrite(pDevCtx, 0,
                        VP792_REG_CH_MASKS_OFFSET, VP792_REG_CH_MASKS_LEN, &chMask);
                }

                /* Generate a USER event when patching is complete. */
                Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_USER_CTL, 0);
                *pBufEnd++ = VP792_UEVID_PATCH_READY;
                if (status == VP_STATUS_SUCCESS) {
                    status = Vp792MailboxSend(pDevCtx, buf, pBufEnd,
                        VP792_CMDSIZE_PATCH_CTL + VP792_CMDSIZE_USER_CTL);
                }
                break;
            }
            /* Else, no patch is necessary; fall through to the next case: */

        case VP792_EVID_USER: { /* eventParamWord == VP792_UEVID_PATCH_READY */

            uint16 *pBufEnd = buf;

#ifdef VP_COMMON_ADDRESS_SPACE
            VpProfilePtrType pDevProfile = pDevObj->ehInitDevice.pDevProfile;
            VpProfilePtrType pAcProfile = pDevObj->ehInitDevice.pAcProfile;
            VpProfilePtrType pDcProfile = pDevObj->ehInitDevice.pDcProfile;
            VpProfilePtrType pRingProfile = pDevObj->ehInitDevice.pRingProfile;
#else
            VpProfilePtrType pDevProfile = (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_DEVICE)) ?
                pDevObj->ehInitDevice.devProfile : VP_NULL;
            VpProfilePtrType pAcProfile = (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_AC)) ?
                pDevObj->ehInitDevice.acProfile : VP_NULL;
            VpProfilePtrType pDcProfile = (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_DC)) ?
                pDevObj->ehInitDevice.dcProfile : VP_NULL;
            VpProfilePtrType pRingProfile = (pDevObj->ehInitDevice.profilesValid & (1 << VP_PROFILE_RING)) ?
                pDevObj->ehInitDevice.ringProfile : VP_NULL;
#endif

            /* Read and store the SLAC firmware revision code. */
            status = ReadFirmwareRevCode(pDevCtx);
            if (status == VP_STATUS_SUCCESS) {
                VP_INFO(VpDevCtxType, pDevCtx, ("Patch revision info: device 0x%4.4X / product 0x%4.4X / version 0x%4.4X",
                    pDevObj->rev.device, pDevObj->rev.product, pDevObj->rev.version));
            }

            /* Read and store the PCM clock rate. */
            if (status == VP_STATUS_SUCCESS) {
                static const uint16 clkRateMap[16] = {1536, 1544, 2048, 0xFFFF, 3072, 2088,
                    4096, 0xFFFF, 6144, 6176, 8192, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
                uint16 clkCfg;
                status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_CLK_CFG_OFFSET,
                    VP792_REG_CLK_CFG_LEN, &clkCfg);
                pDevObj->pcmClockRate = clkRateMap[clkCfg & VP792_REG_CLK_CFG_CSEL_MASK];
                if (pDevObj->pcmClockRate == 0xFFFF) {
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Invalid PCM clock rate detected"));
                    return VP_STATUS_FAILURE;
                }
                VP_INFO(VpDevCtxType, pDevCtx, ("PCLK = %d kHz", pDevObj->pcmClockRate));
            }

            /* Send the post-boot portion of the device profile. */
            if (status == VP_STATUS_SUCCESS) {
                Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_REGLIST;
                status = Vp792ProfileProcessSection(pDevCtx, VP_ALL_LINES, pDevProfile, VP_NULL,
                    VP_NULL, &sectionType, VP792_DEV_PROF_SEC_POSTBOOT);
            }

            /* Initialize all lines using the AC, DC, and ring profiles (if any). */
            if (status == VP_STATUS_SUCCESS) {
                status = InitAllLines(pDevCtx, pAcProfile, pDcProfile,
                    pRingProfile, buf, &pBufEnd);
            }

            /* Apply default option values.  We need to set devInit = TRUE here, since
               this function will ultimately call Vp792SetOption(), which checks
               devInit. */
            if (status == VP_STATUS_SUCCESS) {
                pDevObj->devInit = TRUE;
                status = VpImplementDefaultSettings(pDevCtx, VP_NULL);
                pDevObj->devInit = FALSE;
            }

            /* Send the global CAL_CTL command to calibrate all channels. */
            if (status == VP_STATUS_SUCCESS) {
                Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_CAL_CTL, VP_ALL_LINES);
                status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
            }
            break;
        }
        case VP792_EVID_CAL_CMP:

            /* Generate the DEV_INIT_CMP event. */
            pEvent->eventCategory = VP_EVCAT_RESPONSE;
            pEvent->eventId = VP_DEV_EVID_DEV_INIT_CMP;
            externalEvent = TRUE;

            /* Mark the device as initialized, so other API functions can run. */
            pDevObj->devInit = TRUE;

            /* State machine complete. */
            pDevObj->eventHandlers &= ~ VP792_EH_INIT_DEVICE;
            break;

        default: /* can't happen */
            break;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("EhInitDevice()-"));

    pEvent->status = status;
    return externalEvent;
} /* EhInitDevice() */

/******************************************************************************
 * This event handler runs in response to a Vp792InitLine() call and is
 * responsible for applying the AC, DC, and Ring profiles and calibrating the
 * line.
 *
 * \param[in,out] pEvent       Event struct of the event to be handled
 *****************************************************************************/
static bool
EhInitLine(
    VpEventType *pEvent,
    uint16 eventIdWord)
{
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    bool externalEvent = FALSE;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhInitLine()+"));

    switch (eventIdWord) {
        case VP792_EVID_RESET_CMP: {
            uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE], *pBufEnd = buf;

            /* Init the relay state. */
            status = Vp792InitRelayState(pEvent->pLineCtx);

            if (status == VP_STATUS_SUCCESS) {

#ifdef VP_COMMON_ADDRESS_SPACE
                VpProfilePtrType pAcProfile = pLineObj->profiles.pAc;
                VpProfilePtrType pDcProfile = pLineObj->profiles.pDc;
                VpProfilePtrType pRingProfile = pLineObj->profiles.pRinging;
#else
                VpProfilePtrType pAcProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_AC)) ?
                    pLineObj->profiles.ac : VP_NULL;
                VpProfilePtrType pDcProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_DC)) ?
                    pLineObj->profiles.dc : VP_NULL;
                VpProfilePtrType pRingProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_RING)) ?
                    pLineObj->profiles.ringing : VP_NULL;
#endif

                status = Vp792ConfigLineInternal(pDevCtx, channelId, &pAcProfile,
                    &pDcProfile, &pRingProfile, buf, &pBufEnd, VP792_PROF_SECTYPE_ANY);
            }

            /* Reset line object variables. */
            if (status == VP_STATUS_SUCCESS) {
                Vp792ResetLineVars(pLineCtx);
                pLineObj->eventHandlers = VP792_EH_INIT_LINE;
            }

            /* Send the CAL_CTL command for this channel. */
            if (status == VP_STATUS_SUCCESS) {
                Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_CAL_CTL, channelId);
                status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
            }

            /* Wait for the CAL_CMP event before continuing. */
            break;
        }

        case VP792_EVID_CAL_CMP:

            /* Apply all default option settings for this line. */
            status = VpImplementDefaultSettings(VP_NULL,
                pDevCtx->pLineCtx[channelId]);

            /* Generate the LINE_INIT_CMP event. */
            pEvent->eventCategory = VP_EVCAT_RESPONSE;
            pEvent->eventId = VP_LINE_EVID_LINE_INIT_CMP;
            externalEvent = TRUE;

            /* Mark the line as initialized, so other API functions can run. */
            pLineObj->lineInit = TRUE;

            /* Don't need to receive any more events, so deactivate. */
            pLineObj->eventHandlers &= ~VP792_EH_INIT_LINE;
            break;

        default: /* can't happen */
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhInitLine()-"));

    pEvent->status = status;
    return externalEvent;
} /* EhInitLine() */

/******************************************************************************
 * This function completes the functionality of VpSendCid().  It programs the
 * sequencer with the sequence found in the stored CID profile.
 *
 * \param[in]     pLineCtx   Line to which the CID sequence will be applied
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 * \retval ::VP_STATUS_DEVICE_BUSY  Sequence is already running on the channel
 *****************************************************************************/
static bool
EhSendCid(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_REGLIST;
    bool externalEvent = FALSE;
    VpStatusType status = VP_STATUS_SUCCESS;

#ifdef VP_COMMON_ADDRESS_SPACE
    VpProfilePtrType pCidProfile = pLineObj->callerId.pCidProfile;
#else
    VpProfilePtrType pCidProfile = (pLineObj->callerId.cidProfileValid & (1 << VP_PROFILE_CID)) ?
        pLineObj->callerId.cidProfile : VP_NULL;
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendCid()+"));

    switch (eventIdWord) {
        case VP792_EVID_USER:

            switch (eventParamWord) {
                case VP792_UEVID_SEQUENCER_READY: {

                    /* Apply FSK/DTMF settings to the device. */
                    status = Vp792ProfileProcessSection(pDevCtx, channelId, pCidProfile,
                        VP_NULL, VP_NULL, &sectionType, 0);

                    /* Prime the device's FSK data buffers. */
                    pLineObj->callerId.fsk.bufStatus = VP792_FSKEVENT_BUF1_EMPTY | VP792_FSKEVENT_BUF2_EMPTY;
                    if ((status == VP_STATUS_SUCCESS) && (pLineObj->callerId.isFsk) && (pLineObj->callerId.data.nonempty)) {
                        status = Vp792SendFskData(pLineCtx);        /* buffer 1 */
                        if (status == VP_STATUS_SUCCESS) {
                            status = Vp792SendFskData(pLineCtx);    /* buffer 2 */
                        }
                    }
                    /* Start the Caller ID sequence. */
                    if (status == VP_STATUS_SUCCESS) {
                        status = Vp792StartSavedSequence(pLineCtx, VP792_SEQTYPE_CID);
                    }
                    break;
                }

                case VP792_UEVID_DTMF_DATA: {
                    int bufBytes;

                    /* Zero out tone generator registers. */
                    uint16 resetSigGen[VP792_REG_SIG_GEN_AMPD_OFFSET - VP792_REG_SIG_GEN_AMPA_OFFSET];
                    VpMemSet(resetSigGen, 0, sizeof(resetSigGen));
                    Vp792HbiPagedWrite(pDevCtx, channelId,
                        VP792_REG_SIG_GEN_AMPA_OFFSET,
                        VP792_REG_SIG_GEN_AMPD_OFFSET - VP792_REG_SIG_GEN_AMPA_OFFSET, resetSigGen);

                    pEvent->status = Vp792ProfileProcessSection(pDevCtx, channelId,
                        pCidProfile, VP_NULL, VP_NULL, &sectionType, 0);

                    status = SendDtmfData(pEvent->pLineCtx);
                    if (status != VP_STATUS_SUCCESS) {
                        break;
                    }

                    /* Is there enough room in the line object buffer for more data? */
                    bufBytes = (int)pLineObj->callerId.data.head - pLineObj->callerId.data.tail;
                    if (bufBytes < 0) {
                        bufBytes += (VP_SIZEOF_CID_MSG_BUFFER * 2);
                    }

                    /* If we generate an event, it will be a CID_DATA event. */
                    pEvent->eventCategory = VP_EVCAT_PROCESS;
                    pEvent->eventId = VP_LINE_EVID_CID_DATA;

                    if (pLineObj->callerId.data.nonempty == FALSE) {

                        /* Data buffer empty.  Return the CID_DATA:TX_DONE event
                           from VpGetEvent(). */
                        externalEvent = TRUE;
                        pEvent->eventData = VP_CID_DATA_TX_DONE;

                        /* Transmit the final DTMF digit and end the sequence. */
                        status = SequencerControl(pLineCtx, VP792_SEQ_CTL_BREAK_AND_RESUME);
                        break;

                    } else if (bufBytes == VP_SIZEOF_CID_MSG_BUFFER) {

                        /* Data buffer is half full.  Return the
                           CID_DATA:NEED_MORE_DATA event from VpGetEvent(). */
                        externalEvent = TRUE;
                        pEvent->eventData = VP_CID_DATA_NEED_MORE_DATA;
                    }

                    /* Continue the sequence to transmit the next DTMF digit. */
                    status = SequencerControl(pEvent->pLineCtx, VP792_SEQ_CTL_RESUME);
                    break;
                }

                /* Caller ID tone 'n' is stored in the CID profile as a register list
                   section at index n + 1. */
                case VP792_UEVID_CID_TONE_1:
                case VP792_UEVID_CID_TONE_2:
                case VP792_UEVID_CID_TONE_3: {
                    int toneIndex = eventParamWord - VP792_UEVID_CID_TONE_1 + 1;
                    status = Vp792ProfileProcessSection(pDevCtx, channelId,
                        pCidProfile, VP_NULL, VP_NULL, &sectionType, toneIndex);
                    break;
                }

                /* The CID profile includes 4 possible tones to detect, and we can look
                 * for any one or two of them at a time. */
                case VP792_UEVID_ACK_DETECT_0:
                case VP792_UEVID_ACK_DETECT_1:
                case VP792_UEVID_ACK_DETECT_1_0:
                case VP792_UEVID_ACK_DETECT_2:
                case VP792_UEVID_ACK_DETECT_2_0:
                case VP792_UEVID_ACK_DETECT_2_1:
                case VP792_UEVID_ACK_DETECT_3:
                case VP792_UEVID_ACK_DETECT_3_0:
                case VP792_UEVID_ACK_DETECT_3_1:
                case VP792_UEVID_ACK_DETECT_3_2: {
                    int toneIndex1, toneIndex2;
                    GetCidAckToneIndices(eventParamWord, &toneIndex1, &toneIndex2);
                    return CidAckDetectInterval(pEvent, toneIndex1, toneIndex2, pCidProfile);
                }

                default: /* can't happen */
                    break;
            } /* switch (eventParamWord) */
            break;

        case VP792_EVID_FSK: {
            uint8 prevDataLength = GetFskDataLength(pLineObj);

            /* If both buffers are empty, data transfer is complete. */
            if (eventParamWord == VP792_FSKEVENT_FSK_CMP) {

                /* Return the CID_DATA:TX_DONE event from VpGetEvent(). */
                pEvent->eventCategory = VP_EVCAT_PROCESS;
                pEvent->eventId = VP_LINE_EVID_CID_DATA;
                pEvent->eventData = VP_CID_DATA_TX_DONE;
                externalEvent = TRUE;

                /* Resume the sequencer */
                pEvent->status = SequencerControl(pEvent->pLineCtx, VP792_SEQ_CTL_RESUME);
                break;
            }

            /* Send the next 6 bytes of FSK data to the device. */
            pLineObj->callerId.fsk.bufStatus |= eventParamWord;
            pEvent->status = Vp792SendFskData(pLineCtx);

            /* If the data buffer has just fallen below half-full... */
            if (
                (prevDataLength > VP_SIZEOF_CID_MSG_BUFFER) &&
                (GetFskDataLength(pLineObj) <= VP_SIZEOF_CID_MSG_BUFFER)
            ) {

                /* ...return CID_DATA:NEED_MORE_DATA event from VpGetEvent(). */
                pEvent->eventCategory = VP_EVCAT_PROCESS;
                pEvent->eventId = VP_LINE_EVID_CID_DATA;
                pEvent->eventData = VP_CID_DATA_NEED_MORE_DATA;
                externalEvent = TRUE;
                break;
            }

            /* No external event. */
            break;
        }

        case VP792_EVID_DDIGIT:
            if (pLineObj->callerId.dtmfDetectInterval) {
                if ((pEvent->eventData & VP792_DIGIT_MASK) == pLineObj->callerId.dtmfTones[0]) {
                    pLineObj->callerId.dtmfTones[0] = VP792_CID_TONE_DETECTED;
                }
                if ((pEvent->eventData & VP792_DIGIT_MASK) == pLineObj->callerId.dtmfTones[1]) {
                    pLineObj->callerId.dtmfTones[1] = VP792_CID_TONE_DETECTED;
                }
            } else {
                /* We are not in a CID ACK detect interval, so report the DTMF
                   event. */
                externalEvent = TRUE;
            }
            break;

        case VP792_EVID_SEQ: { /* eventParamWord = VP792_SEQEVENT_CMP */
            uint16 fskCtl;

            /* Disable any active FSK generation */
            status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_FSK_CTL_OFFSET,
                VP792_REG_FSK_CTL_LEN, &fskCtl);
            if ((status == VP_STATUS_SUCCESS) && (fskCtl & VP792_REG_FSK_CTL_EFSK_MASK)) {
                fskCtl &= ~VP792_REG_FSK_CTL_EFSK_MASK;
                status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_FSK_CTL_OFFSET,
                    VP792_REG_FSK_CTL_LEN, &fskCtl);
            }

            /* Disable this event handler; Caller ID sequence is complete. */
            pLineObj->eventHandlers &= ~ VP792_EH_SEND_CID;
            break;
        }

        default: /* can't happen */
            break;
    } /* switch (eventIdWord) */

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendCid()-"));

    pEvent->status = status;
    return externalEvent;
} /* EhSendCid() */

static bool
EhSendSignal(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpSendSignalType currentSignal = pLineObj->sendSig.currentSignal;
    VpStatusType status = VP_STATUS_SUCCESS;
    bool externalEvent = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendSignal()+"));

    switch (eventIdWord & VP792_INTIND_FW_EVID_MASK) {
        case VP792_EVID_USER:
            switch (eventParamWord) {
                case VP792_UEVID_SEQUENCER_READY:

                    if (pLineObj->sendSig.newSignal == VP_SENDSIG_MSG_WAIT_PULSE) {

                        /* For message-waiting signal, before we start, we need
                           to read the DC feed parameters. */
                        status = SendSigMsgWaitReadDcParams(pLineCtx);
                    } else {

                        /* For all other sequence types, we can start the new
                           sequencer program. */

                        /* First, read the DRIVE_ST register so we know what
                           value to restore to it at signal completion. */
                        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_DRIVE_ST_OFFSET,
                            VP792_REG_DRIVE_ST_LEN, &pLineObj->sendSig.driveSt);

                        /* Start the sequencer. */
                        if (status == VP_STATUS_SUCCESS) {
                            status = Vp792StartSavedSequence(pLineCtx, VP792_SEQTYPE_SENDSIG);
                        }
                    }
                    if (status == VP_STATUS_SUCCESS) {
                        pLineObj->sendSig.currentSignal = pLineObj->sendSig.newSignal;
                    }
                    break;

                case VP792_UEVID_MSG_WAIT_ON:
                    if (currentSignal != VP_SENDSIG_MSG_WAIT_PULSE) {
                        break;
                    }

                    /* Set DC feed parameters for ON interval: */
                    status = SendSigMsgWaitWriteDcParams(pLineCtx,
                        pLineObj->sendSig.msgWait.dcParams.on.v1,
                        pLineObj->sendSig.msgWait.dcParams.on.vas,
                        pLineObj->sendSig.msgWait.dcParams.on.ila);
                    break;

                case VP792_UEVID_MSG_WAIT_OFF:
                    if (currentSignal != VP_SENDSIG_MSG_WAIT_PULSE) {
                        break;
                    }

                    if (pLineObj->sendSig.msgWait.aborting) {
                        /* Abort was requested.  Stop the sequencer. */
                        status = SequencerControl(pLineCtx, VP792_SEQ_CTL_STOP);
                        break;
                    }

                    /* Set DC feed parameters back to defaults: */
                    status = SendSigMsgWaitWriteDcParams(pLineCtx,
                        pLineObj->sendSig.msgWait.dcParams.off.v1,
                        pLineObj->sendSig.msgWait.dcParams.off.vas,
                        pLineObj->sendSig.msgWait.dcParams.off.ila);
                    break;

                default: /* can't happen */
                    break;
            }
            break;

        case VP792_EVID_MB_RSP:
            if (currentSignal != VP_SENDSIG_MSG_WAIT_PULSE) {
                break;
            }
            status = SendSigMsgWaitSaveDcParams(pEvent);
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792StartSavedSequence(pLineCtx, VP792_SEQTYPE_SENDSIG);
            }
            break;

        case VP792_EVID_HOOK:
            if (currentSignal != VP_SENDSIG_MSG_WAIT_PULSE) {
                break;
            }

            /* If we haven't already aborted... */
            if (pLineObj->sequencer.deferredHandlers == VP792_EH_NONE) {

                /* Abort the sequencer. */
                pEvent->status = Vp792NewSequence(pLineCtx, VP792_EH_NONE, VP_NULL, 0);
            }

            /* Here we fall through to the next case, pretending that we already
               received the VP792_EVID_SEQ event.  (When the real event arrives,
               it will be ignored.) */

        case VP792_EVID_SEQ:
            if (currentSignal == VP_SENDSIG_MSG_WAIT_PULSE) {
                /* Set DC feed parameters back to defaults: */
                status = SendSigMsgWaitWriteDcParams(pLineCtx,
                    pLineObj->sendSig.msgWait.dcParams.off.v1,
                    pLineObj->sendSig.msgWait.dcParams.off.vas,
                    pLineObj->sendSig.msgWait.dcParams.off.ila);
            } else if (
                (currentSignal == VP_SENDSIG_POLREV_PULSE) ||
                (currentSignal == VP_SENDSIG_FWD_DISCONNECT)
            ) {
                /* Restore DRIVE_ST register to the value it had before the
                   signal started. */
                status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_DRIVE_ST_OFFSET,
                    VP792_REG_DRIVE_ST_LEN, &pLineObj->sendSig.driveSt);
            }

            /* Generate the SIGNAL_CMP event. */
            pEvent->eventCategory = VP_EVCAT_PROCESS;
            pEvent->eventId = VP_LINE_EVID_SIGNAL_CMP;
            pEvent->eventData = (uint16)currentSignal;
            externalEvent = TRUE;

            /* Disable this event handler. */
            pLineObj->eventHandlers &= ~VP792_EH_SEND_SIGNAL;
            break;


        default: /* can't happen */
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendSignal()-"));

    pEvent->status = status;
    return externalEvent;
} /* EhSendSignal() */

/******************************************************************************
 * This function implements the main functionality of VpSetLineState().  If no
 * sequence is running with Vp792SetLineState() is called, this function gets
 * called immediately to enter the new line state.  Otherwise,
 * Vp792SetLineState() aborts the currently-running sequence, and this function
 * gets called by VpGetEvent() when the "sequencer ready" event occurs.
 *
 * If entering cadenced ringing, this function calls StartSequence() to
 * program the sequence register and start the sequencer.
 *
 * Otherwise, this function sets the device's drive state and PCM TX/RX cut-off
 * registers according to the selected line state.
 *
 * If no errors occurred, it then updates the currentState member in the line
 * object to reflect the new line state.
 *
 * \param[in]     pLineCtx   Line to which the new line state will be applied
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 * \retval ::VP_STATUS_INVALID_ARG
 * \retval ::VP_STATUS_DEVICE_BUSY  Sequence is already running on the channel
 *****************************************************************************/
static VpStatusType
EhSetLineState(
    VpEventType *pEvent,
    uint16 eventIdWord)
{
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    uint8 channelId = pLineObj->channelId;
    VpLineStateType state = pLineObj->ehSetLineState.newState;
    bool externalEvent = TRUE;
    bool readyForTransition = FALSE;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSetLineState()+"));

    switch (eventIdWord) {
        case VP792_EVID_USER: /* eventParamWord = VP792_UEVID_SEQUENCER_READY */
            pLineObj->ehSetLineState.waitForSequencerReady = FALSE;
            readyForTransition = !pLineObj->ehSetLineState.waitForMeterAbort;
            break;

        case VP792_EVID_MTR_CMP: /* eventParamWord = 0 ("aborted") */
            pLineObj->ehSetLineState.waitForMeterAbort = FALSE;
            readyForTransition = !pLineObj->ehSetLineState.waitForSequencerReady;
            break;

        case VP792_EVID_SEQ: { /* eventParamWord = VP792_SEQEVENT_CMP */
            if (
                (pLineObj->currentState == VP_LINE_RINGING) ||
                (pLineObj->currentState == VP_LINE_RINGING_POLREV)
            ) {
                /* Ring cadence complete.  If the cadence ran to completion
                   (wasn't aborted), generate the RING_CAD:DONE event. */
                pEvent->eventCategory = VP_EVCAT_PROCESS;
                pEvent->eventId = VP_LINE_EVID_RING_CAD;
                pEvent->eventData = VP_RING_CAD_DONE;
                externalEvent = !pLineObj->sequencer.aborting;
            }

            /* Disable this event handler. */
            pLineObj->eventHandlers &= ~VP792_EH_SET_LINE_STATE;
            break;
        }
        default: /* can't happen */
            break;
    }

    if (readyForTransition) {
        switch (state) {
            case VP_LINE_RINGING:
            case VP_LINE_RINGING_POLREV:
                status = RingEnter(pEvent, (state == VP_LINE_RINGING_POLREV));
                break;
            case VP_LINE_PARK:
                status = ParkMode(pLineCtx);
                break;
            default: {
                uint16 driveState;

                /* Set the DRIVE_ST register based on requested line state. */
                status = Vp792MapLineState(state, &driveState);
                if (status == VP_STATUS_SUCCESS) {
                    status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_DRIVE_ST_OFFSET,
                        VP792_REG_DRIVE_ST_LEN, &driveState);
                }

                /* Update PCM TX/RX mode per requested API line state and API option
                   settings. */
                if (status == VP_STATUS_SUCCESS) {
                    status = Vp792SetPcmTxRxMode(pLineCtx, state);
                }

                /* We're not starting the sequencer, so there is no need to
                   wait for more events. */
                pLineObj->eventHandlers &= ~VP792_EH_SET_LINE_STATE;
            }
        }

        /* Save new line state into line object. */
        if (status == VP_STATUS_SUCCESS) {
            pLineObj->currentState = state;
        }
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSetLineState()-"));

    pEvent->status = status;
    return externalEvent;
} /* EhSetLineState() */


/******************************************************************************
 * This function implements the main functionality of VpSetLineTone().  It
 * applies a tone (cadenced or noncadenced) to the specified line.  The tone
 * profile and tone cadence profile pointers are taken from the line object.
 *
 * \param[in]     pLineCtx   Line to which the tone/cadence will be applied
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 * \retval ::VP_STATUS_DEVICE_BUSY  Sequence is already running on the channel
 *****************************************************************************/
static VpStatusType
EhSetLineTone(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    bool externalEvent = FALSE;
    VpStatusType status;

#ifdef VP_COMMON_ADDRESS_SPACE
    VpProfilePtrType pToneProfile = pLineObj->profiles.pTone;
    VpProfilePtrType pToneCadProfile = pLineObj->profiles.pToneCad;
#else
    VpProfilePtrType pToneProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_TONE)) ?
        pLineObj->profiles.tone : VP_NULL;
    VpProfilePtrType pToneCadProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_TONECAD)) ?
        pLineObj->profiles.toneCad : VP_NULL;
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendSetLineTone()+"));

    switch (eventIdWord) {
        case VP792_EVID_USER: { /* eventParamWord = VP792_UEVID_SEQUENCER_READY */

            switch (eventParamWord) {
                case VP792_UEVID_SEQUENCER_READY:
                    if (pToneProfile == VP_NULL) {
                        /* User just wanted to abort the previous tone. */
                        uint16 sigGenCtl = VP792_REG_SIG_GEN_CTL_DISABLE;
                        pEvent->status = Vp792HbiPagedWrite(pDevCtx, channelId,
                            VP792_REG_SIG_GEN_OFFSET + VP792_REG_SIG_GEN_CTL_WORD,
                            1, &sigGenCtl);
                        pLineObj->eventHandlers &= ~VP792_EH_SET_LINE_TONE;
                        break;
                    }

                    /* Program the signal generators per the new tone profile. */
                    status = Vp792ProfileProcessSections(pDevCtx, channelId, pToneProfile,
                        VP_NULL, VP_NULL, VP792_PROF_SECTYPE_REGLIST);

                    /* Cadenced tone: */
                    if (pToneCadProfile != VP_NULL) {
                        uint8 *pSequence, length;
                        Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_SEQUENCE;
                        status = Vp792ProfileFindSection(pToneCadProfile, &sectionType,
                            0, &pSequence, &length);
                        if (status == VP_STATUS_SUCCESS) {
                            status = StartSequence(pLineCtx, pSequence, FALSE, VP792_SEQTYPE_TONE);
                        }
                        break;
                    }

                    /* Uncadenced ("always on") tone:  Tone profile may contain an
                       unstructured data section, containing the value for the
                       VP792_REG_SIG_GEN_CTL_WORD register. */
                    status = Vp792ProfileProcessSections(pDevCtx, channelId, pToneProfile,
                        VP_NULL, VP_NULL, VP792_PROF_SECTYPE_UNSTRUCTURED);
                    break;

                case VP792_UEVID_TONE_CAD:
                    pEvent->eventCategory = VP_EVCAT_PROCESS;
                    pEvent->eventId = VP_LINE_EVID_TONE_CAD;
                    externalEvent = TRUE;

                    /* Profile Wizard puts this event at the end of a sequence.
                       When we get the VP792_SEQEVENT_CMP event, we shouldn't
                       disable the tone generators, because the user wants the
                       tone to continue forever.  Disable this event handler;
                       when the VP792_SEQEVENT_CMP arrives, it will be ignored. */
                    pLineObj->eventHandlers &= ~VP792_EH_SET_LINE_TONE;
                    break;

                default: /* can't happen */
                    break;
            }
            break;
        }
        case VP792_EVID_SEQ: { /* eventParamWord = VP792_SEQEVENT_CMP */
            /* Cadence complete.   If the cadence was aborted, disable the
               signal generators.  Otherwise, the cadence ran to completion and
               we should generate a TONE_CAD event. */
            if (pLineObj->sequencer.aborting) {
                uint16 sigGenCtl = VP792_REG_SIG_GEN_CTL_DISABLE;
                pEvent->status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_SIG_GEN_OFFSET + VP792_REG_SIG_GEN_CTL_WORD,
                    1, &sigGenCtl);
            } else {
                pEvent->eventCategory = VP_EVCAT_PROCESS;
                pEvent->eventId = VP_LINE_EVID_TONE_CAD;
                externalEvent = TRUE;
            }

            /* Not expecting any more events. */
            pLineObj->eventHandlers &= ~VP792_EH_SET_LINE_TONE;
            break;
        }
        default: /* can't happen */
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("EhSendSetLineTone()-"));

    return externalEvent;
} /* EhSetLineTone() */

static void
GetCidAckToneIndices(
    uint16 userEventId,
    int *pIndex1,
    int *pIndex2)
{
    int index1, index2;

    switch (userEventId) {
        case VP792_UEVID_ACK_DETECT_0:
            index1 =  0;
            index2 = -1;
            break;

        case VP792_UEVID_ACK_DETECT_1:
            index1 =  1;
            index2 = -1;
            break;

        case VP792_UEVID_ACK_DETECT_1_0:
            index1 =  1;
            index2 =  0;
            break;

        case VP792_UEVID_ACK_DETECT_2:
            index1 =  2;
            index2 = -1;
            break;

        case VP792_UEVID_ACK_DETECT_2_0:
            index1 =  2;
            index2 =  0;
            break;

        case VP792_UEVID_ACK_DETECT_2_1:
            index1 =  2;
            index2 =  1;
            break;

        case VP792_UEVID_ACK_DETECT_3:
            index1 =  3;
            index2 = -1;
            break;

        case VP792_UEVID_ACK_DETECT_3_0:
            index1 =  3;
            index2 =  0;
            break;

        case VP792_UEVID_ACK_DETECT_3_1:
            index1 =  3;
            index2 =  1;
            break;

        case VP792_UEVID_ACK_DETECT_3_2:
            index1 =  3;
            index2 =  2;
            break;

        default:
            index1 = -1;
            index2 = -1;
    }

    *pIndex1 = index1;
    *pIndex2 = index2;
}  /*  GetCidAckToneIndices()  */

static bool
GetEventInternal(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = VP_NULL;
    bool returnEvent = FALSE;
    bool eventHandlingRequired = TRUE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("GetEventInternal()+"));

    /* This is the same for all events: */
    pEvent->deviceId = pDevObj->deviceId;

    /* Set some sensible defaults in the pEvent struct; they may be overwritten
       later, before this function returns. */
    pEvent->hasResults = FALSE;
    pEvent->status = VP_STATUS_SUCCESS;
    pEvent->eventData = pEvent->parmHandle = eventParamWord;

    /* Get the channel information from the event ID word. */
    pEvent->channelId = (eventIdWord & VP792_INTIND_FW_CHANID_MASK) >> VP792_INTIND_FW_CHANID_POS;
    pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
    if (pEvent->pLineCtx != VP_NULL) {
        pLineObj = pEvent->pLineCtx->pLineObj;
        pEvent->lineId = pLineObj->lineId;
    }

#ifdef VP792_INCLUDE_TESTLINE_CODE
    if (pDevObj->eventHandlers & VP792_EH_TEST_LINE) {
        eventHandlingRequired = Vp792PmEventHandler(pEvent, &eventIdWord, &eventParamWord);
    }
#endif

    /* Handle the event.  When these functions return, the pEvent struct will
       be fully filled in.  If the event is handled internally, returnEvent will be
       FALSE. */
    if (eventHandlingRequired) {
        if (eventIdWord & VP792_INTIND_SYS_MASK) {
            returnEvent = HandleSystemInt(pEvent, eventIdWord, eventParamWord);
        } else if ((eventIdWord & VP792_INTIND_FW_EVID_MASK) == VP792_EVID_USER) {
            returnEvent = HandleUserEvent(pEvent, eventIdWord, eventParamWord);
        } else {
            returnEvent = HandleFirmwareEvent(pEvent, eventIdWord, eventParamWord);
        }
    }

    /* If the event could not be handled because the command mailbox was busy,
       we need to save the interrupt indication so VpGetEvent() can be called
       again. */
    if (pEvent->status == VP_STATUS_MAILBOX_BUSY) {
        CopyIntInd(pDevObj->intInd, eventIdWord, eventParamWord);
    }

    /* If the event is masked, get rid of it. */
    if (returnEvent) {
        returnEvent = CheckEventMask(pEvent);
#if 0
        if (!returnEvent) {
            VP_INFO(VpDevCtxType, pDevCtx, ("Event masked in API (0x%4.4X:0x%4.4X)", pEvent->eventCategory, pEvent->eventId));
        }
#endif
    }

    /* Some legacy applications (e.g. VPD) assume that channelId will be 0 for
       device-specific events.  The correct behavior is to ignore channelId if
       pLineCtx = VP_NULL. */
    if (pEvent->channelId == VP_ALL_LINES) {
        pEvent->channelId = 0;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("GetEventInternal()-"));

    return returnEvent;
} /* GetEventInternal() */

static uint8
GetFskDataLength(
    Vp792LineObjectType *pLineObj)
{
    /* Compute the amount of data currently in the FSK data buffer in the line
       object. */
    int bufBytes = (int)pLineObj->callerId.data.head - (int)pLineObj->callerId.data.tail;
    if (bufBytes < 0) {
        bufBytes += (VP_SIZEOF_CID_MSG_BUFFER * 2);
    }
    return (uint8)bufBytes;
}

/******************************************************************************
 * This function decodes a VP792 firmware event indication (supplied in the
 * eventIdWord and eventParamWord parameters) and fills the supplied event
 * struct (pEvent).  For VP792 events which don't directly correspond to API-
 * level events, this function returns FALSE.  Error codes (listed below) are
 * saved in pEvent->status.
 *
 * \param[in,out] pEvent       Event struct to be filled
 * \param[in]     eventIdWord  Interrupt indication value from VP792 device
 * \param[in]     eventIdParam Interrupt indication value from VP792 device
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_FAILURE  pEvent->pLineCtx = VP_NULL
 * \retval ::VP_STATUS_ERR_HBI  Invalid eventIdWord
 *****************************************************************************/
static bool
HandleFirmwareEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    Vp792DeviceObjectType *pDevObj = pEvent->pDevCtx->pDevObj;
    uint8 evId = (uint8)(eventIdWord & VP792_INTIND_FW_EVID_MASK);
    bool active = (bool)(eventIdWord & VP792_INTIND_FW_ACTIVE_MASK);
    bool externalEvent;

    /* Make temporary local copies of event struct members: */
    VpStatusType status               = pEvent->status;
    uint8 channelId                   = pEvent->channelId;
    VpLineCtxType *pLineCtx           = pEvent->pLineCtx;
    VpDeviceIdType deviceId           = pEvent->deviceId;
    VpDevCtxType *pDevCtx             = pEvent->pDevCtx;
    VpEventCategoryType eventCategory = pEvent->eventCategory;
    uint16 eventId                    = pEvent->eventId;
    uint16 parmHandle                 = pEvent->parmHandle;
    uint16 eventData                  = pEvent->eventData;
    bool hasResults                   = pEvent->hasResults;
    VpLineIdType lineId               = pEvent->lineId;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("HandleFirmwareEvent()+"));

    /* If it's not a line-specific event, clear pLineCtx. */
    if ((evId >= VP792_FIRST_GLOBAL_EVID) && (evId <= VP792_LAST_GLOBAL_EVID)) {
        channelId = pEvent->channelId = VP_ALL_LINES;
        pLineCtx = pEvent->pLineCtx = VP_NULL;
    } else if (pEvent->pLineCtx == VP_NULL) {

        /* Event occurred on a line with no line context. */
        VP_INFO(VpDevCtxType, pDevCtx, ("HandleFirmwareEvent(): No line context for channel %d!", channelId));
        return FALSE;
    }

    switch (evId) {
        case VP792_EVID_BOOT_CMP:
            pDevObj->devInit = FALSE;
            if (pDevObj->eventHandlers & VP792_EH_INIT_DEVICE) {

                /* VpInitDevice() was called, so we are expecting this event. */
                return EhInitDevice(pEvent, VP792_EVID_BOOT_CMP);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }

        case VP792_EVID_RESET_CMP: {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            pLineObj->lineInit = FALSE;

            if (pLineObj->eventHandlers & VP792_EH_INIT_LINE) {

                /* VpInitLine() was called, so we are expecting this event. */
                return EhInitLine(pEvent, VP792_EVID_RESET_CMP);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }
        }
        case VP792_EVID_FSK: {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            if (pLineObj->eventHandlers & VP792_EH_SEND_CID) {
                /* VpSendCid() or VpInitCid() was called, so we are expecting
                   this event. */
                return EhSendCid(pEvent, VP792_EVID_FSK, eventParamWord);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }
        }
        case VP792_EVID_MB_RSP: {
            uint16 idx;

            /* Are we handling a cached response? */
            if (pDevObj->respMboxCache.count > 0) {
                idx = pDevObj->respMboxCache.data[0][VP792_MBOX_PAYLOAD_INDEX];
            }

            /* Otherwise, we got a real MB_RSP event. */
            else {
                uint16 flags;
                status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
                        VP792_REG_MB_FLAG_LEN, &flags);
                if (VP_STATUS_SUCCESS == status) {

                    /* Response Mailbox flag: 0 => host owns it; 1=> VP792 owns it. */
                    if (flags & VP792_REG_MB_FLAG_RSP_MBOX_MASK) {

                        /* The response was already handled due to mailbox
                           congestion; discard the useless MB_RSP event. */
                        externalEvent = FALSE;
                        break;
                    }

                    /* Read the request index from the SLAC's response mailbox. */
                    status = Vp792HbiPagedRead(pDevCtx, VP792_HBI_RESPONSE_MBOX_PAGE,
                        VP792_MBOX_PAYLOAD_INDEX, 1, &idx);
                }
            }

            /* Fill in event struct based on the type of the response. */
            if (status == VP_STATUS_SUCCESS) {
                return Vp792IdentifyResponseEvent(pEvent, eventIdWord, idx - VP792_UEVID_RESPONSE_FIRST);
            }
            externalEvent = TRUE;
            break;
        }
        case VP792_EVID_MB_ERR:
            return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);

        case VP792_EVID_TS_ROLLOVER: {

#if (TS_DIVISOR > 0)
            /* Received SLAC timestamp rollover.  The API timestamp granularity
               is coarser that of the SLAC, so we don't report every event. */
            uint16 increment = (uint16)(0x10000UL / TS_DIVISOR);
            pDevObj->timeStampHiBits += increment;
#endif

            eventCategory = VP_EVCAT_SIGNALING;
            eventId = VP_DEV_EVID_TS_ROLLOVER;
            externalEvent = (pDevObj->timeStampHiBits == 0);
            break;
        }
        case VP792_EVID_TIMER_CMP: {
            Vp792LineObjectType *pLineObj;
            Vp792ResponseRequestType *pRequest;
            uint16 idx = eventParamWord - VP792_UEVID_RESPONSE_FIRST;
            pRequest = &pDevObj->responseRequest[idx];

            if (
                (eventParamWord < VP792_UEVID_RESPONSE_FIRST) ||
                (eventParamWord > VP792_UEVID_RESPONSE_LAST) ||
                (pRequest->outstanding != TRUE)
            ) {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }

            /* Generate timer-complete event in response to a VpGenTimerCtrl()
               call. */
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_GEN_TIMER;
            channelId = pRequest->args.genTimer.channelId;
            parmHandle = pRequest->args.genTimer.handle;
            pLineCtx = pDevCtx->pLineCtx[channelId];
            pLineObj = pLineCtx->pLineObj;
            lineId = pLineObj->lineId;
            eventData = VP_GEN_TIMER_STATUS_CMP;
            externalEvent = TRUE;

            /* Free the response struct. */
            pRequest->outstanding = FALSE;
            break;
        }
        case VP792_EVID_CAL_CMP: {
            Vp792LineObjectType *pLineObj;

            /* Device-specific event handlers: */
            if (pDevObj->eventHandlers & VP792_EH_INIT_DEVICE) {

                /* VpInitDevice() was called, so we are expecting this event. */
                pEvent->pLineCtx = VP_NULL;
                pEvent->channelId = VP_ALL_LINES;
                return EhInitDevice(pEvent, VP792_EVID_CAL_CMP);
            } else if (pDevObj->eventHandlers & VP792_EH_CAL_CODEC) {

                /* VpCalCodec() was called, so we are expecting this event. */
                pEvent->pLineCtx = VP_NULL;
                pEvent->channelId = VP_ALL_LINES;
                return EhCal(pEvent, VP792_EVID_CAL_CMP);
            }

            /* Remaining cases are line-specific. */
            pEvent->channelId = (eventIdWord & VP792_INTIND_FW_CHANID_MASK) >> VP792_INTIND_FW_CHANID_POS;
            pEvent->pLineCtx = pDevCtx->pLineCtx[pEvent->channelId];
            if (pEvent->pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }

            /* Line-specific event handlers: */
            if (pLineObj->eventHandlers & VP792_EH_INIT_LINE) {

                /* VpInitLine() was called, so we are expecting this event. */
                return EhInitLine(pEvent, VP792_EVID_CAL_CMP);
            } else if (pLineObj->eventHandlers & VP792_EH_CAL_LINE) {

                /* VpCalLine() was called, so we are expecting this event. */
                return EhCal(pEvent, VP792_EVID_CAL_CMP);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }
        }
        case VP792_EVID_AC_FLT:
            eventCategory = VP_EVCAT_FAULT;
            eventId = VP_LINE_EVID_AC_FLT;
            eventData = active;
            externalEvent = TRUE;

            /* Transition to disconnect if VP_DEVICE_OPTION_ID_CRITICAL_FLT is enabled. */
            if ((active) && (pDevObj->options.criticalFlt.acFltDiscEn)) {
                status = VpSetLineState(pEvent->pLineCtx, VP_LINE_DISCONNECT);
            }
            break;

        case VP792_EVID_DC_FLT:
            eventCategory = VP_EVCAT_FAULT;
            eventId = VP_LINE_EVID_DC_FLT;
            eventData = active;
            externalEvent = TRUE;

            /* Transition to disconnect if VP_DEVICE_OPTION_ID_CRITICAL_FLT is enabled. */
            if ((active) && (pDevObj->options.criticalFlt.dcFltDiscEn)) {
                status = VpSetLineState(pEvent->pLineCtx, VP_LINE_DISCONNECT);
            }
            break;

        case VP792_EVID_T_FLT:
            eventCategory = VP_EVCAT_FAULT;
            eventId = VP_LINE_EVID_THERM_FLT;
            eventData = active;
            externalEvent = TRUE;

            /* Transition to disconnect if VP_DEVICE_OPTION_ID_CRITICAL_FLT is enabled. */
            if ((active) && (pDevObj->options.criticalFlt.thermFltDiscEn)) {
                status = VpSetLineState(pEvent->pLineCtx, VP_LINE_DISCONNECT);
            }
            break;

        case VP792_EVID_PREQ_HOOK:
            eventCategory = VP_EVCAT_SIGNALING;
            eventId = VP_LINE_EVID_HOOK_PREQUAL;
            parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord);
            eventData = active ? VP_HOOK_PREQUAL_START : VP_HOOK_PREQUAL_ABORT;
            externalEvent = TRUE;
            break;

        case VP792_EVID_RAW_HOOK:
        case VP792_EVID_HOOK: {
            /* At most, one of these two events will be unmasked at a time. */
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            eventCategory = VP_EVCAT_SIGNALING;
            eventId = (active ? VP_LINE_EVID_HOOK_OFF : VP_LINE_EVID_HOOK_ON);
            parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord);

            if (eventId == VP_LINE_EVID_HOOK_OFF) {

                /* If a message-waiting signal is ongoing (and hasn't already
                   been aborted), abort it and generate the SIGNAL_CMP event. */
                if (pLineObj->eventHandlers & VP792_EH_SEND_SIGNAL) {
                    EhSendSignal(pEvent, VP792_EVID_HOOK, 0);

                    /* We'll generate the HOOK_OFF event next time the user
                       calls VpGetEvent(). */
                    DeferEvent(pDevCtx, eventIdWord, eventParamWord);
                } else {

                    /* If in ringing, switch to the ring-trip exit state. */
                    status = RingExit(pLineCtx);
                }
            }

            externalEvent = TRUE;
            break;
        }
        case VP792_EVID_GKEY:
            eventCategory = VP_EVCAT_SIGNALING;
            eventId = (active ? VP_LINE_EVID_GKEY_DET : VP_LINE_EVID_GKEY_REL);
            parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord);
            externalEvent = TRUE;
            break;

        case VP792_EVID_RING_CAD:
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_RING_CAD;
            eventData = (active ? VP_RING_CAD_MAKE : VP_RING_CAD_BREAK);
            parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord);
            externalEvent = TRUE;
            break;

        case VP792_EVID_DDIGIT: {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            uint16 sense;
            if (active) {
                /* SLAC reports DTMF make event. */
                if (pLineObj->dtmfDigitDetected & VP_DIG_SENSE_MAKE) {
                    return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
                }
                sense = VP_DIG_SENSE_MAKE;
            } else {
                /* SLAC reports DTMF break event. */
                if ((pLineObj->dtmfDigitDetected & VP_DIG_SENSE_MAKE) == 0) {
                    return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
                }
                sense = VP_DIG_SENSE_BREAK;
            }

            pLineObj->dtmfDigitDetected = sense | (eventParamWord & VP792_DIGIT_MASK);
            pEvent->eventData = pLineObj->dtmfDigitDetected;

            pEvent->parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord & VP792_DIGIT_TS_MASK);
            ReportDtmfEvent(pEvent, sense, FALSE);

            if (pLineObj->eventHandlers & VP792_EH_SEND_CID) {
                /* If in a Caller ID sequence, don't generate a DTMF event if we
                   are waiting for a CPE acknowledgement. */
                externalEvent = EhSendCid(pEvent, VP792_EVID_DDIGIT, eventParamWord);
            } else {
                externalEvent = TRUE;
            }
            return (externalEvent && (pLineObj->options.dtmfControlMode == VP_OPTION_DTMF_DECODE_ON));
        }
        case VP792_EVID_FLASH:
            eventCategory = VP_EVCAT_SIGNALING;
            eventId = (active ? VP_LINE_EVID_EXTD_FLASH : VP_LINE_EVID_FLASH);
            parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord & VP792_DIGIT_TS_MASK);
            externalEvent = TRUE;
            break;

        case VP792_EVID_PDIGIT:
            eventCategory = VP_EVCAT_SIGNALING;
            if (active) {
                eventId = VP_LINE_EVID_STARTPULSE;
                parmHandle = CALCULATE_TIMESTAMP(pDevObj, eventParamWord & VP792_DIGIT_TS_MASK);
            } else {
                eventId = VP_LINE_EVID_PULSE_DIG;
                eventData = (eventParamWord & VP792_DIGIT_MASK);
                /* One to Fifteen pulses can be reported. */
                if ((eventData == 0) || (eventData > 15)) {
                    eventData = VP_DIG_NONE;
                }
            }
            externalEvent = TRUE;
            break;

        case VP792_EVID_TONE:
            eventCategory = VP_EVCAT_SIGNALING;
            eventId = VP_LINE_EVID_MTONE;
            externalEvent = TRUE;
            break;

        case VP792_EVID_MB_CMD:
            eventCategory = VP_EVCAT_RESPONSE;
            eventId = VP_DEV_EVID_DNSTR_MBOX;
            externalEvent = TRUE;
            break;

        case VP792_EVID_SEQ:
            switch (eventParamWord) {
                case VP792_SEQEVENT_CMP: {
                    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
                    pLineObj->sequencer.activeSequence = VP792_SEQTYPE_OFF;

                    if ((channelId == 0) && (pDevObj->eventHandlers & VP792_EH_CAL_CODEC)) {
                        pEvent->channelId = VP_ALL_LINES;
                        externalEvent = EhCal(pEvent, VP792_EVID_SEQ);
                    } else if (pLineObj->eventHandlers & VP792_EH_CAL_LINE) {
                        externalEvent = EhCal(pEvent, VP792_EVID_SEQ);
                    } else if (pLineObj->eventHandlers & (VP792_EH_SEND_CID | VP792_EH_SET_LINE_STATE)) {
                        externalEvent = FALSE;
                        /* In the case of ringing with caller ID, these two event
                           handlers can be enabled at the same time. */
                        if (pLineObj->eventHandlers & VP792_EH_SEND_CID) {
                            EhSendCid(pEvent, VP792_EVID_SEQ, VP792_SEQEVENT_CMP);
                            /* Return value is ignored, because it won't generate an event. */
                        }
                        if (pLineObj->eventHandlers & VP792_EH_SET_LINE_STATE) {
                            externalEvent = EhSetLineState(pEvent, VP792_EVID_SEQ);
                        }
                    } else if (pLineObj->eventHandlers & VP792_EH_SET_LINE_TONE) {
                        externalEvent = EhSetLineTone(pEvent, VP792_EVID_SEQ, VP792_SEQEVENT_CMP);
                    } else if (pLineObj->eventHandlers & VP792_EH_SEND_SIGNAL) {
                        externalEvent = EhSendSignal(pEvent, VP792_EVID_SEQ, 0);
                    } else {
                        /* No event handlers are waiting for this event. */
                        externalEvent = FALSE;
                    }

                    pLineObj->sequencer.aborting = FALSE;
                    return externalEvent;
                }
                case VP792_SEQEVENT_PAUSED:
                    externalEvent = FALSE;
                    break;

                case VP792_SEQEVENT_BAD_INST:
                case VP792_SEQEVENT_BAD_BR:
                default:
                    return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
            }
            break;

        case VP792_EVID_MTR_CAD: {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_MTR_CAD;
            externalEvent = ((eventData % pLineObj->metering.eventRate) == 0);
            break;
        }
        case VP792_EVID_MTR_CMP: {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
            pLineObj->metering.active = FALSE;
            eventCategory = VP_EVCAT_PROCESS;
            externalEvent = TRUE;
            if (active) {
                eventId = VP_LINE_EVID_MTR_CMP;
            } else {
                eventId = VP_LINE_EVID_MTR_ABORT;
                if (pLineObj->eventHandlers & VP792_EH_SET_LINE_STATE) {
                    EhSetLineState(pEvent, VP792_EVID_MTR_CMP);
                }
            }
            break;
        }
        case VP792_EVID_MTR_ROLLOVER:
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_MTR_ROLLOVER;
            externalEvent = TRUE;
            break;

        /* VP792_EVID_USER events are handled in HandleUserEvent(). */

        default:
            return UnexpectedEvent(pEvent, eventIdWord, eventParamWord);
    }

    /* Copy temporary local variables back into the event struct. */
    pEvent->status = status;
    if (externalEvent) {
        pEvent->channelId = channelId;
        pEvent->pLineCtx = pLineCtx;
        pEvent->deviceId = deviceId;
        pEvent->pDevCtx = pDevCtx;
        pEvent->eventCategory = eventCategory;
        pEvent->eventId = eventId;
        pEvent->parmHandle = parmHandle;
        pEvent->eventData = eventData;
        pEvent->hasResults = hasResults;
        pEvent->lineId = lineId;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("HandleFirmwareEvent()-"));

    return externalEvent;
} /* HandleFirmwareEvent() */

static bool
HandleSequenceIntervalEvent(
    VpEventType *pEvent,
    uint16 userEventId)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    switch (userEventId) {
        case VP792_UEVID_RX_DISABLE: {
            uint16 vpCfg2;

            /* Read, modify, write.  */
            status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
                VP792_REG_VP_CFG2_LEN, &pLineObj->sequencer.lastVpCfg2);
            if (status == VP_STATUS_SUCCESS) {
                vpCfg2 = (uint16) (pLineObj->sequencer.lastVpCfg2 | VP792_REG_VP_CFG2_CRP_MASK);
                status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
                    VP792_REG_VP_CFG2_LEN, &vpCfg2);
            }
            break;
        }
        case VP792_UEVID_RX_RESTORE:
            status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
                VP792_REG_VP_CFG2_LEN, &pLineObj->sequencer.lastVpCfg2);
            break;

        case VP792_UEVID_HOOK_MASK:
            /* The VP792_UEVID_HOOK_MASK events should occur in pairs in
               CID sequences.  The first one means to mask the hook events; the
               second one means to unmask them.

               If not currently masking hooks, set the callerId.hookMask flag.
               Then set the SLAC's hook debounce to maximum time and save its
               old value.  (Maximizing the hook debounce time is a way of
               masking hook events without actually modifying the event mask
               and without worrying about missing a genuine hook state change.) */
            if (pLineObj->sequencer.hookMaskInterval == FALSE) {
                status = Vp792HbiPagedRead(pDevCtx, channelId,
                    VP792_REG_LOOP_SUP_OFFSET + VP792_REG_LOOP_SUP_DSH_WORD,
                    1, &pLineObj->sequencer.lastDsh);
                if (status == VP_STATUS_SUCCESS) {
                    uint16 maxDsh = 0xFFFF;
                    status = Vp792HbiPagedWrite(pDevCtx, channelId,
                        VP792_REG_LOOP_SUP_OFFSET + VP792_REG_LOOP_SUP_DSH_WORD,
                        1, &maxDsh);
                }
                pLineObj->sequencer.hookMaskInterval = TRUE;
            } else {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_LOOP_SUP_OFFSET + VP792_REG_LOOP_SUP_DSH_WORD,
                    1, &pLineObj->sequencer.lastDsh);
                pLineObj->sequencer.hookMaskInterval = FALSE;
            }
            break;

        case VP792_UEVID_VOICE_EN:
            /* This event is generated by "Mute off" CID sequence commands and
               linestate ring/tone cadence commands that request a talk state.
               Restore the cut tx/rx settings based on the PCM_TX_RX option
               setting for a talk state.
               We're sending the VP_LINE_OHT parameter just because it is a
               known talk state.  Using VP_LINE_TALK would be equivalent. */
            status = Vp792SetPcmTxRxMode(pLineCtx, VP_LINE_OHT);
            break;

        case VP792_UEVID_VOICE_DIS:
            /* This event is generated by "Mute on" CID sequence commands and
               linestate ring/tone cadence commands that request a non-talk
               state.  Cut tx and rx paths.
               We're sending the VP_LINE_ACTIVE parameter just because it is a
               known non-talk state.  Using VP_LINE_ACTIVE would be
               equivalent. */
            status = Vp792SetPcmTxRxMode(pLineCtx, VP_LINE_ACTIVE);
            break;

        default: /* can't happen */
            status = VP_STATUS_FAILURE;
    }

    pEvent->status = status;
    return FALSE;
}

static bool
HandleSystemInt(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 bit;
    bool externalEvent;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("HandleSystemInt()+"));

    /* We potentially generate a separate API-level event for each system
       interrupt bit.  Find the rightmost bit that is set. */
    for (bit = 1; bit & VP792_INTIND_SYSINT_FLAGS_MASK; bit <<= 1) {
        if (eventIdWord & bit) {

            /* Found it. */
            eventIdWord &= ~bit;
            break;
        }
    }

    /* In the default case (unexpected system interrupt) we generate a
       SYSTEM_FLT event for each bit. */
    pEvent->pLineCtx = VP_NULL;
    pEvent->channelId = VP_ALL_LINES;
    pEvent->eventCategory = VP_EVCAT_FAULT;
    pEvent->eventId = VP_DEV_EVID_SYSTEM_FLT;
    pEvent->eventData = (uint16)((eventIdWord & ~VP792_INTIND_SYSINT_FLAGS_MASK) | bit);
    pEvent->parmHandle = eventParamWord;
    externalEvent = TRUE;

    /* For some bits we generate an API event and/or do some other handling. */
    switch (bit) {
        case VP792_INTIND_SYSINT_WDT_MASK:
            pEvent->eventId = VP_DEV_EVID_WDT_FLT;
            pDevObj->devInit = FALSE;
            break;

        case VP792_INTIND_SYSINT_OVL_MASK:
            pEvent->eventId = VP_DEV_EVID_EVQ_OFL_FLT;
            break;

        case VP792_INTIND_SYSINT_RESET_MASK:
            pDevObj->devInit = FALSE;
        case VP792_INTIND_SYSINT_PREB_MASK:
            if (pDevObj->eventHandlers & VP792_EH_INIT_DEVICE) {
                externalEvent = EhInitDevice(pEvent, VP792_INTIND_SYS_MASK | bit);
            } /* else it's a system fault */
            break;

        case VP792_INTIND_SYSINT_HBF_MASK:
        case VP792_INTIND_SYSINT_LBF_MASK:
        case VP792_INTIND_SYSINT_PBF_MASK:
            pEvent->eventId = VP_DEV_EVID_BAT_FLT;
            pEvent->eventData = (uint16)((eventParamWord & VP792_REG_SYS_STAT_BF_MASK) >> VP792_REG_SYS_STAT_BF_SHIFT);
            break;

        case VP792_INTIND_SYSINT_CFAIL_MASK:
            pEvent->eventId = VP_DEV_EVID_CLK_FLT;
            pEvent->eventData = (uint16) ((eventParamWord & VP792_REG_SYS_STAT_CFAIL_MASK) ? 1 : 0);
            break;

        default:
            /* Unexpected system fault event. */
            break;
    }

    /* If more system interrupts occurred, process them in the next call to
       VpGetEvent(). */
    if ((eventIdWord & VP792_INTIND_SYSINT_FLAGS_MASK) != 0) {
        DeferEvent(pEvent->pDevCtx, eventIdWord, eventParamWord);
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("HandleSystemInt()-"));

    return externalEvent;
} /* HandleSystemInt() */

static bool
HandleUserEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 userEventId)
{
    Vp792LineObjectType *pLineObj;
    bool externalEvent;

    /* Make temporary local copies of event struct members: */
    VpStatusType status               = pEvent->status;
    uint8 channelId                   = pEvent->channelId;
    VpLineCtxType *pLineCtx           = pEvent->pLineCtx;
    VpDeviceIdType deviceId           = pEvent->deviceId;
    VpDevCtxType *pDevCtx             = pEvent->pDevCtx;
    VpEventCategoryType eventCategory = pEvent->eventCategory;
    uint16 eventId                    = pEvent->eventId;
    uint16 parmHandle                 = pEvent->parmHandle;
    uint16 eventData                  = pEvent->eventData;
    bool hasResults                   = pEvent->hasResults;
    VpLineIdType lineId               = pEvent->lineId;

    VP_API_FUNC_INT(VpLineCtxType, pEvent->pLineCtx, ("HandleUserEvent()+"));

    if (pLineCtx != VP_NULL) {
        pLineObj = pLineCtx->pLineObj;
    } else {

        /* Event occurred on a line with no line context. */
        VP_INFO(VpDevCtxType, pDevCtx, ("HandleUserEvent(): No line context for channel %d!", channelId));
        return UnexpectedEvent(pEvent, eventIdWord, userEventId);
    }

    /* Handle 'artificial' response events (e.g. LINE_IO_RD_CMP): */
    if (
        (userEventId >= VP792_UEVID_RESPONSE_FIRST) &&
        (userEventId <= VP792_UEVID_RESPONSE_LAST)
    ) {
        uint16 idx = userEventId - VP792_UEVID_RESPONSE_FIRST;
        return Vp792IdentifyResponseEvent(pEvent, userEventId, idx);
    }

    switch (userEventId) {

        case VP792_UEVID_PATCH_READY: {
            Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
            if (pDevObj->eventHandlers & VP792_EH_INIT_DEVICE) {
                return EhInitDevice(pEvent, VP792_EVID_USER);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, userEventId);
            }
        }
        case VP792_UEVID_SEQUENCER_READY: {
            uint16 tempEventHandlers;

            /* Enable the deferred event handlers that are waiting for this
               event. */
            pLineObj->eventHandlers |= pLineObj->sequencer.deferredHandlers;
            pLineObj->sequencer.deferredHandlers = 0;
            tempEventHandlers = pLineObj->eventHandlers;

            /* User may have called VpSetLineState() followed by some other
               sequencer-using function such as VpSetLineTone(); therefore,
               we may call two event handlers here. */
            if (pLineObj->eventHandlers & VP792_EH_SET_LINE_STATE) {
                EhSetLineState(pEvent, VP792_EVID_USER);
                /* EhSetLineState() may modify pLineObj->eventHandlers; hence
                   the use of the tempEventHandlers variable below. */
            }
            if (tempEventHandlers & VP792_EH_SEND_SIGNAL) {
                return EhSendSignal(pEvent, VP792_EVID_USER, VP792_UEVID_SEQUENCER_READY);
            } else if (tempEventHandlers & VP792_EH_SET_LINE_TONE) {
                return EhSetLineTone(pEvent, VP792_EVID_USER, VP792_UEVID_SEQUENCER_READY);
            } else if (tempEventHandlers & VP792_EH_SEND_CID) {
                return EhSendCid(pEvent, VP792_EVID_USER, VP792_UEVID_SEQUENCER_READY);
            }

            externalEvent = FALSE;
            break;
        }
        case VP792_UEVID_IO_READ_CMP:
            eventCategory = VP_EVCAT_RESPONSE;
            eventId = VP_DEV_EVID_IO_ACCESS_CMP;
            eventData = VP_DEVICE_IO_READ;
            hasResults = TRUE;
            externalEvent = TRUE;
            break;

        case VP792_UEVID_IO_WRITE_CMP:
            eventCategory = VP_EVCAT_RESPONSE;
            eventId = VP_DEV_EVID_IO_ACCESS_CMP;
            eventData = VP_DEVICE_IO_WRITE;
            hasResults = FALSE;
            externalEvent = TRUE;
            break;

        case VP792_UEVID_TONE_CAD:
            if (pLineObj->eventHandlers & VP792_EH_SET_LINE_TONE) {
                return EhSetLineTone(pEvent, VP792_EVID_USER, VP792_UEVID_TONE_CAD);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, VP792_UEVID_TONE_CAD);
            }

        case VP792_UEVID_MSG_WAIT_ON:
        case VP792_UEVID_MSG_WAIT_OFF:
            if (pLineObj->eventHandlers & VP792_EH_SEND_SIGNAL) {
                return EhSendSignal(pEvent, VP792_EVID_USER, userEventId);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, userEventId);
            }

        case VP792_UEVID_CAL_BUSY:
            eventCategory = VP_EVCAT_RESPONSE;
            eventId = VP_EVID_CAL_BUSY;
            pLineCtx = VP_NULL;
            channelId = VP_ALL_LINES;
            externalEvent = TRUE;
            break;

        /* The following events can be generated in any sequence to demarcate
           the beginning/end of RX disable, hook mask, and mute intervals. */
        case VP792_UEVID_RX_DISABLE:
        case VP792_UEVID_RX_RESTORE:
        case VP792_UEVID_HOOK_MASK:
        case VP792_UEVID_VOICE_EN:
        case VP792_UEVID_VOICE_DIS: {
            if (pLineObj->sequencer.activeSequence != VP792_SEQTYPE_OFF) {
                return HandleSequenceIntervalEvent(pEvent, userEventId);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, userEventId);
            }
        }

        case VP792_UEVID_DTMF_DATA:
        case VP792_UEVID_CID_TONE_1:
        case VP792_UEVID_CID_TONE_2:
        case VP792_UEVID_CID_TONE_3:
        case VP792_UEVID_ACK_DETECT_0:
        case VP792_UEVID_ACK_DETECT_1:
        case VP792_UEVID_ACK_DETECT_1_0:
        case VP792_UEVID_ACK_DETECT_2:
        case VP792_UEVID_ACK_DETECT_2_0:
        case VP792_UEVID_ACK_DETECT_2_1:
        case VP792_UEVID_ACK_DETECT_3:
        case VP792_UEVID_ACK_DETECT_3_0:
        case VP792_UEVID_ACK_DETECT_3_1:
        case VP792_UEVID_ACK_DETECT_3_2:
            /* These events should only occur in Caller ID sequences. */
            if (pLineObj->eventHandlers & VP792_EH_SEND_CID) {
                return EhSendCid(pEvent, VP792_EVID_USER, userEventId);
            } else {
                return UnexpectedEvent(pEvent, eventIdWord, userEventId);
            }

        case VP792_UEVID_DTMF_DISABLE:
            if (pLineObj->options.dtmfControlMode == VP_OPTION_DTMF_DECODE_OFF) {
                /* User made redundant VpSetOption() calls; do nothing. */
                externalEvent = FALSE;
            } else {
                pLineObj->options.dtmfControlMode = VP_OPTION_DTMF_DECODE_OFF;
                if (pLineObj->dtmfDigitDetected & VP_DIG_SENSE_MAKE) {

                    /* A DTMF digit is ongoing, but we have now "turned off"
                       DTMF detection, so we must report a BREAK event. */
                    return ReportDtmfEvent(pEvent, VP_DIG_SENSE_BREAK, TRUE);
                }
                externalEvent = FALSE;
            }
            break;

        case VP792_UEVID_DTMF_ENABLE:
            if (pLineObj->options.dtmfControlMode == VP_OPTION_DTMF_DECODE_ON) {
                /* User made redundant VpSetOption() calls; do nothing. */
                externalEvent = FALSE;
            } else {
                pLineObj->options.dtmfControlMode = VP_OPTION_DTMF_DECODE_ON;
                if (pLineObj->dtmfDigitDetected & VP_DIG_SENSE_MAKE) {

                    /* A DTMF digit is ongoing, but we are only now "turning on"
                       DTMF detection, so we must report a MAKE event. */
                    return ReportDtmfEvent(pEvent, VP_DIG_SENSE_MAKE, TRUE);
                }
                externalEvent = FALSE;
            }
            break;

        case VP792_UEVID_MTR_ABORT:
            pLineObj->metering.active = FALSE;
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_MTR_ABORT;
            eventData = 0;
            externalEvent = TRUE;
            break;

        case VP792_UEVID_LLCMD_TX_CMP:
            eventCategory = VP_EVCAT_RESPONSE;
            eventId = VP_LINE_EVID_LLCMD_TX_CMP;
            externalEvent = TRUE;
            break;

        case VP792_UEVID_TST_PRIMITIVE_CMP:
            /* Struct members have already been set in Vp792PmEventHandler(). */
            externalEvent = TRUE;
            break;

        case VP792_UEVID_USER_0:
        case VP792_UEVID_USER_1:
        case VP792_UEVID_USER_2:
        case VP792_UEVID_USER_3:
        case VP792_UEVID_USER_4:
        case VP792_UEVID_USER_5:
        case VP792_UEVID_USER_6:
        case VP792_UEVID_USER_7:
            /* User event requested by the end-user in Profile Wizard. */
            eventCategory = VP_EVCAT_PROCESS;
            eventId = VP_LINE_EVID_USER;
            externalEvent = TRUE;
            break;

        default:
            return UnexpectedEvent(pEvent, eventIdWord, userEventId);
    }

    /* Copy temporary local variables back into the event struct. */
    pEvent->status = status;
    if (externalEvent) {
        pEvent->channelId = channelId;
        pEvent->pLineCtx = pLineCtx;
        pEvent->deviceId = deviceId;
        pEvent->pDevCtx = pDevCtx;
        pEvent->eventCategory = eventCategory;
        pEvent->eventId = eventId;
        pEvent->parmHandle = parmHandle;
        pEvent->eventData = eventData;
        pEvent->hasResults = hasResults;
        pEvent->lineId = lineId;
    }

    VP_API_FUNC_INT(VpLineCtxType, pEvent->pLineCtx, ("HandleUserEvent()-"));

    return externalEvent;
} /* HandleUserEvent() */

static uint16
Hypotenuse(
    uint16 a,
    uint16 b)
{
    uint32 sum_of_squares = (uint32)a * a + (uint32)b * b;

    return VpComputeSquareRoot(sum_of_squares);
}

static VpStatusType
InitAllLines(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    uint16p pBuf,
    uint16p *ppBufEnd)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channel;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("InitAllLines()+"));
    for (channel = 0; channel < pDevObj->maxChannels; channel++) {
        VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channel];

        /* Don't bother initializing channels with no Line Context. */
        if (pLineCtx == VP_NULL) {
            continue;
        }

        /* Apply the AC, DC, and Ring profile to each line.  Since
           Vp792ConfigLineInternal() ultimately calls Vp792SetOption(), we need
           to temporarily set devInit = TRUE. */
        pDevObj->devInit = TRUE;
        status = Vp792ConfigLineInternal(pDevCtx, channel, &pAcProfile, &pDcProfile,
            &pRingProfile, pBuf, ppBufEnd, VP792_PROF_SECTYPE_ANY);
        pDevObj->devInit = FALSE;
        if (status != VP_STATUS_SUCCESS) {
            break;
        }
        status = Vp792MailboxSend(pDevCtx, pBuf, *ppBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
        *ppBufEnd = pBuf;
        if (status != VP_STATUS_SUCCESS) {
            break;
        }

        /* Reset line object variables. */
        Vp792ResetLineVars(pLineCtx);

        /* Initialize relay state to the default for the termination type. */
        status = Vp792InitRelayState(pLineCtx);
        if (status != VP_STATUS_SUCCESS) {
            break;
        }
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("InitAllLines()-"));

    return status;
} /* InitAllLines() */

static VpStatusType
LineIoAccessIntRead(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 *pData)
{
    VpStatusType status;
    uint16 temp;

    /* The CH_STAT register contains the GPIO sense bits. */
    status = Vp792HbiPagedRead(pDevCtx, channelId,
        VP792_REG_CH_STAT_OFFSET, VP792_REG_CH_STAT_LEN, &temp);

    /* Extract the IOx_ST bits. */
    temp &= VP792_REG_CH_STAT_IO_ST_MASK;
    temp >>= VP792_REG_CH_STAT_IO_ST_SHIFT;
    *pData = (uint8)temp;

    return status;
}

#if 0
static void
Display792Sequence(
    uint8p pSequence)
{
    uint8 offset = 0;
    uint16 seqLen = ((uint16)pSequence[0] << 8) | pSequence[1];
    uint16 cmd;

    printf("\n"
           "Addr  Value  Field  Meaning\n"
           "----  -----  -----  ----------------------------------------\n");

    printf(" 00   %4.4X   SEQLN  Sequence length = ", seqLen);
    seqLen = (seqLen & 0x1F) + 1;
    printf("%d words\n", seqLen);

    while (seqLen--) {
        pSequence += 2;
        cmd = ((uint16)pSequence[0] << 8) | pSequence[1];
        offset++;
        printf(" %2.2X   %4.4X   SEQ%-2d  ", (int)offset, cmd, (int)offset - 1);

        /* Standard commands: */
        if ((cmd & 0xF000) == 0) {
            switch ((cmd & 0x0F00) >> 8) {
                case 0x01: {
                    char *uevent_name;
                    printf("User event 0x%2.2X: ", cmd & 0xFF);
                    switch (cmd & 0xFF) {
                        case VP792_UEVID_USER_0        : uevent_name = "USER_0        "; break;
                        case VP792_UEVID_USER_1        : uevent_name = "USER_1        "; break;
                        case VP792_UEVID_USER_2        : uevent_name = "USER_2        "; break;
                        case VP792_UEVID_USER_3        : uevent_name = "USER_3        "; break;
                        case VP792_UEVID_USER_4        : uevent_name = "USER_4        "; break;
                        case VP792_UEVID_USER_5        : uevent_name = "USER_5        "; break;
                        case VP792_UEVID_USER_6        : uevent_name = "USER_6        "; break;
                        case VP792_UEVID_USER_7        : uevent_name = "USER_7        "; break;
                        case VP792_UEVID_MSG_WAIT_ON   : uevent_name = "MSG_WAIT_ON   "; break;
                        case VP792_UEVID_MSG_WAIT_OFF  : uevent_name = "MSG_WAIT_OFF  "; break;
                        case VP792_UEVID_TONE_CAD      : uevent_name = "TONE_CAD      "; break;
                        case VP792_UEVID_START_CID_SEQ : uevent_name = "START_CID_SEQ "; break;
                        case VP792_UEVID_RX_DISABLE    : uevent_name = "RX_DISABLE    "; break;
                        case VP792_UEVID_RX_RESTORE    : uevent_name = "RX_RESTORE    "; break;
                        case VP792_UEVID_HOOK_MASK     : uevent_name = "HOOK_MASK     "; break;
                        case VP792_UEVID_ACK_DETECT_0  : uevent_name = "ACK_DETECT_0  "; break;
                        case VP792_UEVID_ACK_DETECT_1  : uevent_name = "ACK_DETECT_1  "; break;
                        case VP792_UEVID_ACK_DETECT_1_0: uevent_name = "ACK_DETECT_1_0"; break;
                        case VP792_UEVID_ACK_DETECT_2  : uevent_name = "ACK_DETECT_2  "; break;
                        case VP792_UEVID_ACK_DETECT_2_0: uevent_name = "ACK_DETECT_2_0"; break;
                        case VP792_UEVID_ACK_DETECT_2_1: uevent_name = "ACK_DETECT_2_1"; break;
                        case VP792_UEVID_ACK_DETECT_3  : uevent_name = "ACK_DETECT_3  "; break;
                        case VP792_UEVID_ACK_DETECT_3_0: uevent_name = "ACK_DETECT_3_0"; break;
                        case VP792_UEVID_ACK_DETECT_3_1: uevent_name = "ACK_DETECT_3_1"; break;
                        case VP792_UEVID_ACK_DETECT_3_2: uevent_name = "ACK_DETECT_3_2"; break;
                        case VP792_UEVID_VOICE_EN      : uevent_name = "VOICE_EN      "; break;
                        case VP792_UEVID_VOICE_DIS     : uevent_name = "VOICE_DIS     "; break;
                        case VP792_UEVID_DTMF_DATA     : uevent_name = "DTMF_DATA     "; break;
                        case VP792_UEVID_CID_TONE_1    : uevent_name = "CID_TONE_1    "; break;
                        case VP792_UEVID_CID_TONE_2    : uevent_name = "CID_TONE_2    "; break;
                        case VP792_UEVID_CID_TONE_3    : uevent_name = "CID_TONE_3    "; break;
                        default                        : uevent_name = "(unknown)     "; break;
                    }
                    printf("%s\n", uevent_name);
                    break;
                }
                case 0x02: {
                    char *drive_state = "(invalid)";
                    printf("Drive state 0x%2.2X: ", cmd & 0xFF);
                    switch (cmd & 0x7) {
                        case 0: drive_state = (cmd & 0x8) ? "TEST" : "DISABLED"; break;
                        case 1: drive_state = "DISCONNECT"; break;
                        case 2: drive_state = "NULL FEED"; break;
                        case 3: drive_state = "STANDBY"; break;
                        case 4: drive_state = "LEAD OPEN"; break;
                        case 5: drive_state = "ACTIVE"; break;
                        case 6: drive_state = "RINGING"; break;
                        case 7: drive_state = "HOWLER"; break;
                    }
                    printf("%s (%s polarity)\n", drive_state, (cmd & 0x80) ? ((cmd & 0x10) ? "reverse" : "normal") : "maintain");
                    break;
                }
                case 0x03:
                    printf("Polarity reversal\n");
                    break;
                case 0x04:
                    printf("IO_ST = 0x%2.2X\n", cmd & 0xFF);
                    break;
                case 0x05: {
                    uint8 bit, enable = (cmd & 0xFF);
                    printf("SIG_GEN[0:7] = ");
                    for (bit = 1; bit < 0x40; bit <<= 1) {
                        if ((enable & bit) == 0) {
                            printf("!");
                        }
                        switch (bit) {
                            case 0x01:
                                printf("a");
                                break;
                            case 0x02:
                                printf("b");
                                break;
                            case 0x04:
                                printf("c");
                                break;
                            case 0x08:
                                printf("d");
                                break;
                            case 0x10:
                                printf("bias");
                                break;
                            case 0x20:
                                printf("rev");
                                break;
                        }
                        printf(" ");
                    }
                    printf("\n");
                    break;
                }
                case 0x06:
                    printf("Start/stop metering\n");
                    break;
                case 0x07: {
                    uint8 bit, enable = (cmd & 0xFF);
                    printf("VP_CFG1[0:7] = ");
                    for (bit = 1; bit < 0x40; bit <<= 1) {
                        if ((enable & bit) == 0) {
                            printf("!");
                        }
                        switch (bit) {
                            case 0x01:
                                printf("b");
                                break;
                            case 0x02:
                                printf("z");
                                break;
                            case 0x04:
                                printf("r");
                                break;
                            case 0x08:
                                printf("x");
                                break;
                            case 0x10:
                                printf("gx");
                                break;
                            case 0x20:
                                printf("gr");
                                break;
                        }
                        printf(" ");
                    }
                    switch (cmd & 0xC0) {
                        case 0x00:
                            printf("alaw");
                            break;
                        case 0x40:
                            printf("ulaw");
                            break;
                        case 0x80:
                            printf("linear");
                            break;
                        case 0xC0:
                            printf("INVALID");
                            break;
                    }
                    printf("\n");
                    break;
                }
                case 0x08: {
                    uint8 bit, enable = (cmd & 0xFF);
                    printf("VP_CFG2[0:7] = ");
                    for (bit = 1; bit != 0; bit <<= 1) {
                        if (enable & 0x0A)
                            continue;
                        if ((enable & bit) == 0) {
                            printf("!");
                        }
                        switch (bit) {
                            case 0x01:
                                printf("tone");
                                break;
                            case 0x04:
                                printf("tslb");
                                break;
                            case 0x10:
                                printf("lgr");
                                break;
                            case 0x20:
                                printf("dhp");
                                break;
                            case 0x40:
                                printf("crp");
                                break;
                            case 0x80:
                                printf("ctp");
                                break;
                        }
                        printf(" ");
                    }
                    printf("\n");
                    break;
                }
                case 0x09:
                    printf("Pause\n");
                    break;
                case 0x0A:
                    printf("Save timestamp\n");
                    break;
                case 0x0B:
                    printf("%sable FSK generator\n", (cmd & 1) ? "En" : "Dis");
                    break;
                default:
                    printf("INVALID standard command\n");
                    break;
            }

        /* Long data commands: */
        } else if ((cmd & 0xC000) == 0) {

            switch (cmd & 0x3000) {

                case 0x1000: {
                    uint16 target = cmd & 0x1F;
                    uint16 iterations = (cmd & 0x0FE0) > 5;
                    printf("Branch ");
                    if (iterations == 0) {
                        printf("forever");
                    } else {
                        printf("%d times", iterations);
                    }
                    printf(" to SEQ%d\n", target);
                    break;
                }
                default:
                    printf("INVALID long data command\n");
                    break;
            }

        /* Very long data commands: */
        } else {
            switch (cmd & 0xC000) {
                case 0x4000:
                    if (cmd & 0x2000) {
                        printf("Relative");
                    } else {
                        printf("Absolute");
                    }
                    printf(" delay %d ms\n", cmd & 0x1FFF);
                    break;

                case 0x8000:
                    printf("Gen B amplitude = 0x%4.4X\n", (cmd & 0x1FFF) >> 2);
                    break;

                default:
                    printf("INVALID very long data command\n");
                    break;
            }
        }
    }
    printf("\n");
}
#endif

static VpStatusType
MergeSequences(
    uint8p pRingSeq,
    uint8p pCidSeq,
    uint16p pMergedSeq)
{
    uint8 cidIndex;

    /* Get length fields. */
    uint8 ringSeqLen = (uint8)((pRingSeq[1] & VP792_SEQ_CTL_LENGTH_MASK) + 1);
    uint8 cidSeqLen = (uint8)((pCidSeq[1] & VP792_SEQ_CTL_LENGTH_MASK) + 1);

    /* Find the index of the "start CID sequence" user event command. */
    for (cidIndex = 0; cidIndex < ringSeqLen; cidIndex++) {

        /* Get next command from the ring sequence. */
        uint8p pCmd = &pRingSeq[(cidIndex + 1) * 2];
        uint16 cmd = (uint16)(((uint16)pCmd[0] << 8) | pCmd[1]);

        /* Is it a "start CID sequence" user event? */
        if (cmd == (VP792_SEQ_CMD_USER_EVENT | VP792_UEVID_START_CID_SEQ)) {
            break;
        }
    }

    /* Save length field into new sequencer program.  Subtract 1 if the
       "start CID sequence" user event command was present in the ring
       sequence -- this command is just a placeholder and is not included in
       the merged sequence. */
    pMergedSeq[0] = (uint16)(ringSeqLen + cidSeqLen - (cidIndex < ringSeqLen) - 1);
    if (pMergedSeq[0] > VP792_SEQ_CTL_LENGTH_MASK) {
        VP_ERROR(None, VP_NULL, ("MergeSequences(): Ring cadence and CID profile can't be combined: sequence too long"));
        return VP_STATUS_ERR_PROFILE;
    }

    /* If "start CID sequence" user event command not found in the ring
       sequence, we don't know where to insert the CID sequence.  In this case,
       the CID sequence just gets concatenated onto the end of the ring
       sequence. */

    /* Copy all commands before the "start CID sequence" user event command
       from the ring sequence into the merged sequence, without changes. */
    RelocateSeqCmds(pRingSeq, 0, pMergedSeq, 0, cidIndex);

    /* Add the CID sequence to the end of the new sequence. */
    RelocateSeqCmds(pCidSeq, 0, pMergedSeq, cidIndex, cidSeqLen);

    /* Add the remainder of the ring sequence (if any) to the end of the new
       sequence. */
    if (cidIndex < ringSeqLen) {
        RelocateSeqCmds(pRingSeq, (uint8)(cidIndex + 1), pMergedSeq, (uint8)(cidIndex + cidSeqLen),
            (uint8)(ringSeqLen - cidIndex - 1));
    }

    return VP_STATUS_SUCCESS;
} /* MergeSequences() */

static VpStatusType
ParkMode(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 discTime = pDevObj->options.parkMode.discTime;
    uint16 standbyTime = pDevObj->options.parkMode.standbyTime;
    uint16 buf[VP792_CMDSIZE_WR_SEQ_CTL], *pBufEnd = buf;
    uint16 branchTarget;

    *pBufEnd++ = VP792_SEQ_CTL_START;
    *pBufEnd++ = VP792_SEQ_CMD_DRIVE_ST | VP792_REG_DRIVE_ST_ST_DISCONNECT;
    branchTarget = (pBufEnd - buf) - 1;
    *pBufEnd++ = VP792_SEQ_CMD_DELAY | 500 /* ms */;
    if (discTime > 1) {
        *pBufEnd++ = VP792_SEQ_CMD_BRANCH | (discTime - 1) << 5 | branchTarget;
    }
    *pBufEnd++ = VP792_SEQ_CMD_DRIVE_ST | VP792_REG_DRIVE_ST_ST_STANDBY;
    branchTarget = (pBufEnd - buf) - 1;
    *pBufEnd++ = VP792_SEQ_CMD_DELAY | 100 /* ms */;
    if (discTime > 1) {
        *pBufEnd++ = VP792_SEQ_CMD_BRANCH | (standbyTime - 1) << 5 | branchTarget;
    }
    *pBufEnd++ = VP792_SEQ_CMD_BRANCH; /* infinite branch to beginning */

    /* Update the sequence length field. */
    buf[0] |= (pBufEnd - buf) - 2;

    return StartSequence(pLineCtx, (uint8 *)&buf, TRUE, VP792_SEQTYPE_PARK);
}

static VpStatusType
ReadResponse(
    VpEventType *pEvent,
    uint16 requestIdx,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792ResponseRequestType *pRequest = &pDevObj->responseRequest[requestIdx];
    uint16 buf[VP792_HBI_RESPONSE_MBOX_SIZE], *pPayload = &buf[VP792_MBOX_PAYLOAD_INDEX];
    VpResultsType *pUserBuf = (VpResultsType *)pResults;
    uint8 mailboxLen;
    uint16 mbLen, mbChan, mbCmd, mbContext;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_INT_ENTER(VpDevCtxType, pDevCtx, "ReadResponse");

    pRequest->outstanding = FALSE;

    /* Determine how much data we need to read from the SLAC's response
       mailbox (if any). */
    switch (pRequest->requestType) {
        case VP792_REQUEST_SET_REL_GAIN:
            mailboxLen = VP792_RSPSIZE_RD_REL_GAIN;
            break;
        case VP792_REQUEST_GET_LOOP_COND:
        case VP792_REQUEST_DEVICE_IO_ACCESS_EXT:
        case VP792_REQUEST_LINE_IO_ACCESS:
        case VP792_REQUEST_QUERY:
            mailboxLen = 0;
            break;
        case VP792_REQUEST_TEST_INTERNAL:      /*  todo:  This needs to be refined.  -- RTL */
        case VP792_REQUEST_TEST_CMP:
            mailboxLen = pRequest->args.testLine.respSize;
            break;
        case VP792_REQUEST_GET_OPTION:
            switch (pRequest->args.getOption.optionId) {
                case VP_OPTION_ID_PULSE:
                    mailboxLen = VP792_RSPSIZE_RD_PULSE_PARAMS;
                    break;
                case VP_OPTION_ID_LOOPBACK:
                    mailboxLen = VP792_RSPSIZE_RD_AFE_CONFIG;
                    break;
                default:
                    mailboxLen = 0;
            }
            break;
        case VP792_REQUEST_SENDSIG_MSGWAIT:
        case VP792_REQUEST_LOW_LEVEL_CMD_16:
            if (pRequest->args.lowLevelCmd16.cmdType == VP_LOWLEV_MBOX_RD) {
                mailboxLen = pRequest->args.lowLevelCmd16.numReadWords;
            } else {
                mailboxLen = 0;
            }
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("ReadResponse(): Invalid requestType (%u)", pRequest->requestType));
            return VP_STATUS_FAILURE;
    }

    /* If we need to read the response mailbox, do it now. */
    if (mailboxLen > 0) {

        /* Handle cached response, if any. */
        if (pDevObj->respMboxCache.count > 0) {

            /* Copy the response from the head of the cache. */
            VpMemCpy(buf, pDevObj->respMboxCache.data[0], sizeof(buf));

            /* Remove the response from the head of the cache. */
            pDevObj->respMboxCache.count--;
            if (pDevObj->respMboxCache.count > 0) {
                VpMemCpy(pDevObj->respMboxCache.data[0], pDevObj->respMboxCache.data[1],
                         sizeof(pDevObj->respMboxCache.data));
            }
        }

        /* Otherwise, read the response mailbox. */
        else {
            uint16 flags;

            /* Verify that the host owns the response mailbox. */
            status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
                VP792_REG_MB_FLAG_LEN, &flags);
            if (status |= VP_STATUS_SUCCESS) {
                return status;
            }

            /* Response mailbox flag: 0 => host owns it; 1=> VP792 owns it. */
            if (flags & VP792_REG_MB_FLAG_RSP_MBOX_MASK) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("ReadResponse(): Response mailbox empty"));
                return VP_STATUS_MAILBOX_EMPTY;
            }

            /* Read the response from the mailbox. */
            buf[VP792_MBOX_LENGTH_INDEX] = mailboxLen - VP792_MBOX_PAYLOAD_INDEX;
            status = Vp792RspMailboxRead(pDevCtx, mailboxLen, buf);
            if (status |= VP_STATUS_SUCCESS) {
                return status;
            }
        }

        /* Extract header fields. */
        mbLen = buf[VP792_MBOX_LENGTH_INDEX];
        mbCmd = (buf[VP792_MBOX_ID_CHAN_INDEX] & VP792_MBOX_IDCHAN_CMD_MASK)
                >> VP792_MBOX_IDCHAN_CMD_POS;
        mbChan = (buf[VP792_MBOX_ID_CHAN_INDEX] & VP792_MBOX_IDCHAN_CHAN_MASK)
                >> VP792_MBOX_IDCHAN_CHAN_POS;
        mbContext = *pPayload++;
    }

    /* Process the response. */
    switch (pRequest->requestType) {

        case VP792_REQUEST_LOW_LEVEL_CMD_16: {
            uint8 i;

            /* If doing a mailbox read, we have already read the data (above).
               For page read, we need to issue the command now. */
            if (pRequest->args.lowLevelCmd16.cmdType == VP_LOWLEV_PAGE_RD) {
                uint8 numWriteWords = pRequest->args.lowLevelCmd16.numWriteWords;
                uint8 channelId = pRequest->args.lowLevelCmd16.channelId;
                uint16 pageCmd = HBI_SELECT_PAGE(pDevObj->slacId, channelId);
                uint16 readCmd = pRequest->args.lowLevelCmd16.writeWords[numWriteWords - 1];
                VpDeviceIdType deviceId = pDevObj->deviceId;

                /* Select the channel page. */
                if (VP_HAL_HBI_CMD(deviceId, pageCmd) == FALSE) {
                   return VP_STATUS_ERR_HBI;
                }

                /* Send optional command word (if any). */
                if (numWriteWords == 2) {
                    uint16 cmd = pRequest->args.lowLevelCmd16.writeWords[0];
                    if (VP_HAL_HBI_CMD(deviceId, cmd) == FALSE) {
                       return VP_STATUS_ERR_HBI;
                    }
                }

                /* Read the data from the device. */
                mailboxLen = pRequest->args.lowLevelCmd16.numReadWords;
                if (VP_HAL_HBI_READ(deviceId, readCmd, mailboxLen - 1, buf) == FALSE) {
                    return VP_STATUS_ERR_HBI;
                }
            }

            /* Copy the response into the user's buffer.  Copying from the
               uint16 mailbox buffer to the uint8 result buffer must be done
               in an endian-independent manner */
            for (i = 0; i < mailboxLen; i++) {
                pUserBuf->lowLevelCmd[i * 2] = buf[i] >> 8;
                pUserBuf->lowLevelCmd[i * 2 + 1] = buf[i] & 0xFF;
            }

            /* Check for HBI desync. */
            status = Vp792HbiSync(pDevCtx);
            break;
        }
        case VP792_REQUEST_SENDSIG_MSGWAIT: {
            uint8 i;

            /* Copy the response into the user's buffer.  Copying from the
               uint16 mailbox buffer to the uint8 result buffer must be done
               in an endian-independent manner */
            for (i = 0; i < mailboxLen; i++) {
                pUserBuf->lowLevelCmd[i * 2] = buf[i] >> 8;
                pUserBuf->lowLevelCmd[i * 2 + 1] = buf[i] & 0xFF;
            }
            break;
        }
        case VP792_REQUEST_SET_REL_GAIN:

            /* Skip REL_TX and REL_RX fields. */
            pPayload += 2;

            /* Read the response from the SLAC's response mailbox into the
               user's buffer. */
            pUserBuf->setRelGain.gResult = *pPayload++ & VP_GAIN_BOTH_OOR;
            pUserBuf->setRelGain.gxValue = *pPayload++;
            pUserBuf->setRelGain.grValue = *pPayload++;
            break;

        case VP792_REQUEST_GET_LOOP_COND: {
            uint16 batSense[VP792_REG_BAT_LEN];
            uint16 loopCond[VP792_REG_LOOP_COND_LEN];
            uint16 mGen[VP792_REG_MGEN_LEN];
            uint16 chStatus;
            Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
            uint8 channelId = pLineObj->channelId;

            /* Get battery voltages. */
            status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_BAT_OFFSET,
                VP792_REG_BAT_LEN, batSense);

            /* Read Metering Cancel Amplitude. */
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_MGEN_OFFSET,
                    VP792_REG_MGEN_LEN, mGen);
            }

            /* Read CH_STAT register for battery and DC feed status. */
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_CH_STAT_OFFSET,
                    VP792_REG_CH_STAT_LEN, &chStatus);
            }

            /* Get the remaining loop conditions registers. */
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_LOOP_COND_OFFSET,
                    VP792_REG_LOOP_COND_LEN, loopCond);
            }

            /* Fill in the results struct using the register values. */
            pUserBuf->getLoopCond.rloop = (int16)loopCond[VP792_REG_LOOP_COND_RLOOP_WORD];
            if (pUserBuf->getLoopCond.rloop < 0) {
                /* SLAC saturates at 65 kohms, but the API saturates at 33 kohms. */
                pUserBuf->getLoopCond.rloop = 32767;
            }
            pUserBuf->getLoopCond.ilg   = (int16)loopCond[VP792_REG_LOOP_COND_ILG_WORD];
            pUserBuf->getLoopCond.imt   = (int16)loopCond[VP792_REG_LOOP_COND_IMT_WORD];
            pUserBuf->getLoopCond.vsab  = (int16)loopCond[VP792_REG_LOOP_COND_VSAB_WORD];
            pUserBuf->getLoopCond.vbat1 = (int16)batSense[VP792_REG_BAT_L_WORD];
            pUserBuf->getLoopCond.vbat2 = (int16)batSense[VP792_REG_BAT_H_WORD];
            pUserBuf->getLoopCond.vbat3 = (int16)batSense[VP792_REG_BAT_P_WORD];
            pUserBuf->getLoopCond.mspl  = Hypotenuse(mGen[0], mGen[1]);
            switch (chStatus & VP792_REG_CH_STAT_BSTAT_MASK) {
                case VP792_REG_CH_STAT_BSTAT_LBAT:
                    pUserBuf->getLoopCond.selectedBat = VP_BATTERY_1;
                    break;
                case VP792_REG_CH_STAT_BSTAT_HBAT:
                    pUserBuf->getLoopCond.selectedBat = VP_BATTERY_2;
                    break;
                case VP792_REG_CH_STAT_BSTAT_PBAT:
                    pUserBuf->getLoopCond.selectedBat = VP_BATTERY_3;
                    break;
                default:
                    pUserBuf->getLoopCond.selectedBat = VP_BATTERY_UNDEFINED;
            }
            switch (chStatus & VP792_REG_CH_STAT_DCFR_MASK) {
                case VP792_REG_CH_STAT_DCFR_VAS:
                    pUserBuf->getLoopCond.dcFeedReg = VP_DF_ANTI_SAT_REG;
                    break;
                case VP792_REG_CH_STAT_DCFR_IL:
                    pUserBuf->getLoopCond.dcFeedReg = VP_DF_CNST_CUR_REG;
                    break;
                case VP792_REG_CH_STAT_DCFR_RF:
                    pUserBuf->getLoopCond.dcFeedReg = VP_DF_RES_FEED_REG;
                    break;
                default:
                    pUserBuf->getLoopCond.dcFeedReg = VP_DF_UNDEFINED;
            }
            break;
        }
        case VP792_REQUEST_DEVICE_IO_ACCESS_EXT: {
            uint8 chan;

            VpMemSet(&pUserBuf->deviceIoAccessExt, 0, sizeof(VpDeviceIoAccessExtType));
            pUserBuf->deviceIoAccessExt.direction = pRequest->args.deviceIoAccessExt.deviceIoAccess.direction;

            for (chan = 0; chan < pDevObj->maxChannels; chan++) {
                uint8 *pData = &(pUserBuf->deviceIoAccessExt.lineIoBits[chan].data);

                pUserBuf->deviceIoAccessExt.lineIoBits[chan].mask = pRequest->args.deviceIoAccessExt.deviceIoAccess.lineIoBits[chan].mask;

                status = LineIoAccessIntRead(pDevCtx, chan, pData);
                if (status != VP_STATUS_SUCCESS) {
                    break;
                }
            }
            break;
        }
        case VP792_REQUEST_LINE_IO_ACCESS: {
            uint8 mask = pRequest->args.lineIoAccess.lineIoAccess.ioBits.mask;
            uint8 *pData = &(pUserBuf->lineIoAccess.ioBits.data);

            pUserBuf->lineIoAccess.direction = VP_IO_READ;
            pUserBuf->lineIoAccess.ioBits.mask = mask;

            status = LineIoAccessIntRead(pDevCtx, pEvent->channelId, pData);
            break;
        }
        case VP792_REQUEST_QUERY:
            VP_ERROR(VpDevCtxType, pDevCtx, ("TBI in %s at %d", __FILE__, __LINE__));
            status = VP_STATUS_FUNC_NOT_SUPPORTED;
            break;

        case VP792_REQUEST_GET_OPTION: {
            Vp792LineObjectType *pLineObj = VP_NULL;
            uint8 channelId = pEvent->channelId;
            uint16 temp;
            if (pEvent->pLineCtx != VP_NULL) {
                pLineObj = pEvent->pLineCtx->pLineObj;
            }
            switch (pRequest->args.getOption.optionId) {
                case VP_DEVICE_OPTION_ID_PULSE:
                    return VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;

                case VP_DEVICE_OPTION_ID_DEV_IO_CFG:

                    /* Call ReadLineIoCfg() once for each channel. */
                    for (channelId = 0; channelId < pDevObj->maxChannels; channelId++) {
                        VpOptionLineIoConfigType *pLineIoCfg =
                            &(pUserBuf->getOption.deviceIoConfig.lineIoConfig[channelId]);
                        status = ReadLineIoCfg(pDevCtx, channelId, pLineIoCfg);
                        if (status != VP_STATUS_SUCCESS) {
                            break;
                        }
                    }
                    break;

                case VP_DEVICE_OPTION_ID_PARK_MODE:
                    pUserBuf->getOption.parkMode = pDevObj->options.parkMode;
                    break;

                case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
                    pUserBuf->getOption.criticalFlt = pDevObj->options.criticalFlt;
                    break;

                case VP_OPTION_ID_ZERO_CROSS:
                    pUserBuf->getOption.zeroCross = pLineObj->options.zeroCross;
                    break;

                case VP_OPTION_ID_PULSE:
                    pUserBuf->getOption.linePulse.breakMin = *pPayload++;
                    pUserBuf->getOption.linePulse.breakMax = *pPayload++;
                    pUserBuf->getOption.linePulse.flashMin = *pPayload++;
                    pUserBuf->getOption.linePulse.flashMax = *pPayload++;
                    pUserBuf->getOption.linePulse.onHookMin = *pPayload++;
                    pUserBuf->getOption.linePulse.offHookMin = *pPayload++;
                    pUserBuf->getOption.linePulse.makeMin = *pPayload++;
                    pUserBuf->getOption.linePulse.makeMax = *pPayload++;
                    pUserBuf->getOption.linePulse.interDigitMin = *pPayload++;
                    break;

                case VP_OPTION_ID_PULSE_MODE:
                    pUserBuf->getOption.pulseMode = pLineObj->options.pulseMode;
                    break;

                case VP_OPTION_ID_TIMESLOT:
                    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
                        VP792_REG_TXTS_CFG_LEN, &temp);
                    pUserBuf->getOption.timeslot.tx = (uint8)(temp & VP792_REG_TXTS_CFG_TTS_MASK);
                    if (status == VP_STATUS_SUCCESS) {
                        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
                            VP792_REG_RXTS_CFG_LEN, &temp);
                    }
                    pUserBuf->getOption.timeslot.rx = (uint8)(temp & VP792_REG_RXTS_CFG_RTS_MASK);
                    break;

                case VP_OPTION_ID_CODEC:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_VP_CFG1_OFFSET, VP792_REG_VP_CFG1_LEN, &temp);
                    pUserBuf->getOption.codec = (VpOptionCodecType)(temp >> VP792_REG_VP_CFG1_CODEC_SHIFT);
                    break;

                case VP_OPTION_ID_PCM_HWY: {
                    uint16 txCfg, rxCfg;
                    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
                        VP792_REG_TXTS_CFG_LEN, &txCfg);
                    if (status == VP_STATUS_SUCCESS) {
                        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
                            VP792_REG_RXTS_CFG_LEN, &rxCfg);
                    }
                    if (status == VP_STATUS_SUCCESS) {
                        txCfg &= VP792_REG_TXTS_CFG_THWY_MASK;
                        rxCfg &= VP792_REG_RXTS_CFG_RHWY_MASK;

                        switch (txCfg) {
                            case VP792_REG_TXTS_CFG_THWY_A:
                                if (rxCfg == VP792_REG_RXTS_CFG_RHWY_A) {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_A;
                                } else {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_TX_A_RX_B;
                                }
                                break;
                            case VP792_REG_TXTS_CFG_THWY_B:
                                if (rxCfg == VP792_REG_RXTS_CFG_RHWY_B) {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_B;
                                } else {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_TX_B_RX_A;
                                }
                                break;
                            case VP792_REG_TXTS_CFG_THWY_AB:
                                if (rxCfg == VP792_REG_RXTS_CFG_RHWY_A) {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_TX_AB_RX_A;
                                } else {
                                    pUserBuf->getOption.pcmHwy = VP_OPTION_HWY_TX_AB_RX_B;
                                }
                                break;
                            default:
                                status = VP_STATUS_ERR_HBI;
                        }
                    }
                    break;
                }
                case VP_OPTION_ID_LOOPBACK:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_VP_CFG2_OFFSET, VP792_REG_VP_CFG2_LEN, &temp);
                    if (temp & VP792_REG_VP_CFG2_TSLB_MASK) {
                        pUserBuf->getOption.loopback = VP_OPTION_LB_TIMESLOT;

                    /* We also did a RD_AFE_CONFIG command.  Check the FDLB bit. */
                    } else if (*pPayload & VP792_MB_AFE_CONFIG_FDLB_MASK) {
                        pUserBuf->getOption.loopback = VP_OPTION_LB_DIGITAL;
                    } else {
                        pUserBuf->getOption.loopback = VP_OPTION_LB_OFF;
                    }
                    pLineObj->options.afeConfig = *pPayload;
                    break;

                case VP_OPTION_ID_LINE_STATE:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_DC_FEED_OFFSET, VP792_REG_DC_FEED_LEN, &temp);
                    pUserBuf->getOption.lineState.battRev = ((temp & VP792_REG_DC_FEED_POLRR_ABRUPT) != 0);
                    switch (temp & VP792_REG_DC_FEED_BSET_MASK) {
                        case VP792_REG_DC_FEED_BSET_AUTO:
                            pUserBuf->getOption.lineState.bat = VP_OPTION_BAT_AUTO;
                            break;
                        case VP792_REG_DC_FEED_BSET_HBAT:
                            pUserBuf->getOption.lineState.bat = VP_OPTION_BAT_HIGH;
                            break;
                        case VP792_REG_DC_FEED_BSET_LBAT:
                            pUserBuf->getOption.lineState.bat = VP_OPTION_BAT_LOW;
                            break;
                        case VP792_REG_DC_FEED_BSET_BOOST:
                            pUserBuf->getOption.lineState.bat = VP_OPTION_BAT_BOOST;
                            break;
                        default: /* can't happen */
                            break;
                    }
                    break;

                case VP_OPTION_ID_EVENT_MASK:
                    /*
                     * In SetOption(), we force all line-specific bits in the
                     * deviceEventsMask to zero.  Likewise, we force all device-
                     * specific bits in the lineEventsMask to zero.  This allows
                     * us to simply OR the two together here.
                     */
                    pUserBuf->getOption.eventMask.faults =
                        pLineObj->options.eventMask.faults |
                        pDevObj->options.eventMask.faults;
                    pUserBuf->getOption.eventMask.signaling =
                        pLineObj->options.eventMask.signaling |
                        pDevObj->options.eventMask.signaling;
                    pUserBuf->getOption.eventMask.response =
                        pLineObj->options.eventMask.response |
                        pDevObj->options.eventMask.response;
                    pUserBuf->getOption.eventMask.test =
                        pLineObj->options.eventMask.test |
                        pDevObj->options.eventMask.test;
                    pUserBuf->getOption.eventMask.process =
                        pLineObj->options.eventMask.process |
                        pDevObj->options.eventMask.process;
                    pUserBuf->getOption.eventMask.fxo = VP_EVENT_MASK_ALL;
                    pUserBuf->getOption.eventMask.packet = VP_EVENT_MASK_ALL;
                    break;

                case VP_OPTION_ID_RING_CNTRL:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_LOOP_SUP_OFFSET + VP792_REG_LOOP_SUP_BOSH_WORD, 1,
                        &temp);
                    pUserBuf->getOption.ringControl.zeroCross = pLineObj->options.ringControl.zeroCross;
                    pUserBuf->getOption.ringControl.ringExitDbncDur = temp * 4;
                    pUserBuf->getOption.ringControl.ringTripExitSt = pLineObj->options.ringControl.ringTripExitSt;
                    break;

                case VP_OPTION_ID_PCM_TXRX_CNTRL:
                    pUserBuf->getOption.pcmTxRxCntrl = pLineObj->options.pcmTxRxCntrl;
                    break;

                case VP_OPTION_ID_DTMF_MODE:
                    pUserBuf->getOption.dtmfMode.dtmfControlMode = pLineObj->options.dtmfControlMode;
                    pUserBuf->getOption.dtmfMode.direction = VP_DIRECTION_US;
                    Vp792GetDtmfResources(pDevCtx, &pUserBuf->getOption.dtmfMode);
                    break;

                case VP_OPTION_ID_DTMF_SPEC:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_LOOP_SUP_OFFSET, 1, &temp);
                    if (status == VP_STATUS_SUCCESS) {
                        switch (temp & VP792_REG_LOOP_SUP_DTMF_REG_MASK) {
                            case VP792_REG_LOOP_SUP_DTMF_REG_ATT:
                                pUserBuf->getOption.dtmfSpec = VP_OPTION_DTMF_SPEC_ATT;
                                break;
                            case VP792_REG_LOOP_SUP_DTMF_REG_NTT:
                                pUserBuf->getOption.dtmfSpec = VP_OPTION_DTMF_SPEC_NTT;
                                break;
                            case VP792_REG_LOOP_SUP_DTMF_REG_AUS:
                                pUserBuf->getOption.dtmfSpec = VP_OPTION_DTMF_SPEC_AUS;
                                break;
                            case VP792_REG_LOOP_SUP_DTMF_REG_BRZL:
                                pUserBuf->getOption.dtmfSpec = VP_OPTION_DTMF_SPEC_BRZL;
                                break;
                            case VP792_REG_LOOP_SUP_DTMF_REG_ETSI:
                                pUserBuf->getOption.dtmfSpec = VP_OPTION_DTMF_SPEC_ETSI;
                                break;
                            default:
                                /* Unknown DTMF spec. */
                                status = VP_STATUS_FAILURE;
                                break;
                        }
                    }
                    break;

                case VP_OPTION_ID_LINE_IO_CFG:
                    status = ReadLineIoCfg(pDevCtx, channelId, &pUserBuf->getOption.lineIoConfig);
                    break;

                case VP_OPTION_ID_DCFEED_SLOPE:
                    status = Vp792HbiPagedRead(pDevCtx, channelId,
                        VP792_REG_DC_FEED_SLOPE_OFFSET, VP792_REG_DC_FEED_SLOPE_LEN,
                        &pUserBuf->getOption.dcFeedSlope);
                    break;

               default: /* can't happen (checked elsewhere) */
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
            }
            break;
        }

#ifdef VP792_INCLUDE_TESTLINE_CODE
        case VP792_REQUEST_TEST_INTERNAL:
            VpMemCpy(&pUserBuf->lowLevelCmd, (uint8 *)(buf + VP792_HEADER_FLD + VP792_HANDLE_FLD),
                                               (mailboxLen - (VP792_HEADER_FLD + VP792_HANDLE_FLD)) * 2);
            break;

        case VP792_REQUEST_TEST_CMP:
            status = Vp792PmGetResults(pEvent, pResults);
            break;
#endif

        default: /* can't happen (checked elsewhere) */
            break;
    }

    VP_API_INT_EXIT(VpDevCtxType, pDevCtx, "ReadResponse", status);
    return status;
} /* ReadResponse() */

static VpStatusType
ReadFirmwareRevCode(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpStatusType status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_REVCODE_OFFSET,
        VP792_REG_REVCODE_LEN, &pDevObj->rev.device);
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_DEVCODE_OFFSET,
            VP792_REG_DEVCODE_LEN, &pDevObj->rev.product);
    }
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_ICODE_OFFSET,
            VP792_REG_DEVCODE_LEN, &pDevObj->rev.version);
    }

    return status;
}

static VpStatusType
ReadLineIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpOptionLineIoConfigType *pLineIoCfg)
{
    uint16 temp;
    VpStatusType status = Vp792HbiPagedRead(pDevCtx, channelId,
        VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &temp);
    pLineIoCfg->direction =
        (temp & VP792_REG_IO_ST_IO_X_CFG_MASK) >> VP792_REG_IO_ST_IO_X_CFG_SHIFT;
    pLineIoCfg->outputType =
        (temp & VP792_REG_IO_ST_IO_0_MODE_MASK) >> VP792_REG_IO_ST_IO_0_MODE_SHIFT;
    /* I/O pin 1 is always a CMOS output, so the outputType bit should be 0. */
    return status;
}

static void
RelocateSeqCmds(
    uint8p pSrcSeq,
    uint8 srcIndex,
    uint16p pDestSeq,
    uint8 destIndex,
    uint8 numCmds)
{
    uint8 branchTargetOffset = (uint8)(destIndex - srcIndex);

    /* Skip to the specified command indices. */
    pSrcSeq = &pSrcSeq[(srcIndex + 1) * 2];
    pDestSeq = &pDestSeq[destIndex + 1];

    while (numCmds--) {
        uint16 cmd;

        /* Get next command from the source sequence. */
        cmd = (uint16)((uint16)*pSrcSeq++ << 8);
        cmd |= *pSrcSeq++;

        /* Branch instructions may need to have their targets modified. */
        if ((cmd & VP792_SEQ_LONG_CMD_MASK) == VP792_SEQ_CMD_BRANCH) {
            uint16 branchTarget = (uint16)(cmd & VP792_SEQ_BRANCH_TARGET_MASK);

            /* Check whether the target is within the range of commands that
               are being relocated.  If so, modify the instruction. */
            if (branchTarget >= srcIndex) {
                cmd += branchTargetOffset;
            }
        }

        /* Save the command into the new sequence. */
        *pDestSeq++ = cmd;
    }
} /* RelocateSeqCmds() */

static bool
ReportDtmfEvent(
    VpEventType *pEvent,
    VpDigitSenseType sense,
    bool needTimeStamp)
{
    Vp792LineObjectType *pLineObj = pEvent->pLineCtx->pLineObj;
    Vp792DeviceObjectType *pDevObj = pEvent->pDevCtx->pDevObj;
    uint16 slacTimeStamp;

    pEvent->eventCategory = VP_EVCAT_SIGNALING;
    pEvent->eventId = VP_LINE_EVID_DTMF_DIG;
    pEvent->eventData = pLineObj->dtmfDigitDetected & ~VP_DIG_SENSE_MAKE;
    pEvent->eventData |= sense;

    if (needTimeStamp) {
        /* Need a timestamp.  There is a race condition here (if the
           timestamp has already rolled over, but the TS_ROLLOVER event
           is still in the queue), but it's not really avoidable. */
        pEvent->status = Vp792HbiDirectPageRead(pEvent->pDevCtx,
            VP792_REG_TSTAMP_OFFSET, VP792_REG_TSTAMP_LEN, &slacTimeStamp);
        pEvent->parmHandle = CALCULATE_TIMESTAMP(pDevObj, slacTimeStamp);
    }
    return TRUE;
} /* ReportDtmfEvent() */


static VpStatusType
RestoreDriveState(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 c;

    for (c = pDevObj->maxChannels; c > 0; ) {
        c--;
        if ((channelId == VP_ALL_LINES) || (c == channelId)) {
            VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[c];
            if (pLineCtx != VP_NULL) {
                Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
                uint16 driveSt;

                /* Set the DRIVE_ST register based on line state. */
                status = Vp792MapLineState(pLineObj->currentState, &driveSt);
                if (status != VP_STATUS_SUCCESS) {
                    break;
                }
                status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_DRIVE_ST_OFFSET,
                    VP792_REG_DRIVE_ST_LEN, &driveSt);
                if (status != VP_STATUS_SUCCESS) {
                    break;
                }
            }
        }
    }

    return status;
}

static VpStatusType
RingEnter(
    VpEventType *pEvent,
    bool polrev)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 *pRingSeq, *pCidSeq, length;
    Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_SEQUENCE;
    VpStatusType status;

#ifdef VP_COMMON_ADDRESS_SPACE
    VpProfilePtrType pRingCadProfile = pLineObj->profiles.pRingCad;
    VpProfilePtrType pCidProfile = pLineObj->profiles.pCallerId;
#else
    VpProfilePtrType pRingCadProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_RINGCAD)) ?
        pLineObj->profiles.ringCad : VP_NULL;
    VpProfilePtrType pCidProfile = (pLineObj->profiles.valid & (1 << VP_PROFILE_CID)) ?
        pLineObj->profiles.callerId : VP_NULL;
#endif

    /* If a RING profile is NULL, set it to the default "always on" profile. */
    if (pRingCadProfile == VP_NULL) {
        static const uint8 defaultRingCadProfile[] = {
            /* Profile header ----------------------------------------------------- */
                /* version */               0x00,
                /* type */                  0x08,       /* 0x08 = ring cadence */
                /* number of sections */    0x01,
                /* content length */        0x08,       /* (2 + 6) */
            /* Section 0 ---------------------------------------------------------- */
                /* section type */          0x02,       /* 0x02 = sequence */
                /* content length */        0x06,       /* 6 */
                /* data */                  0x00, 0x01, /* sequence length = 2 */
                                            0x02, 0x06, /* 00 - Line State */
                                            0x08, 0xC0  /* CTP, CRP */
            /* Unstructured data -------------------------------------------------- */
                /* none */
        };
        pRingCadProfile = defaultRingCadProfile;
    }

    /* Locate the ring cadence sequence. */
    status = Vp792ProfileFindSection(pRingCadProfile, &sectionType, 0, &pRingSeq, &length);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("EhSetLineState(): Invalid ring cadence profile"));
        return status;
    }

    /* Set the polarity bit in the DRIVE_ST register. */
    if (status == VP_STATUS_SUCCESS) {
        uint16 oldValue, newValue;

        status = Vp792HbiPagedRead(pDevCtx, pEvent->channelId, VP792_REG_DRIVE_ST_OFFSET,
            VP792_REG_DRIVE_ST_LEN, &oldValue);

        newValue = oldValue & ~VP792_REG_DRIVE_ST_POL_MASK;
        if (polrev) {
            newValue |= VP792_REG_DRIVE_ST_POL_REVERSE;
        }

        if ((status == VP_STATUS_SUCCESS) && (newValue != oldValue)) {
            status = Vp792HbiPagedWrite(pDevCtx, pEvent->channelId, VP792_REG_DRIVE_ST_OFFSET,
                VP792_REG_DRIVE_ST_LEN, &newValue);
        }
    }
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Locate the Caller ID sequence. */
    if (pCidProfile != VP_NULL) {
        uint16 mergedSeq[VP792_CMDSIZE_WR_SEQ_CTL];

        status = Vp792ProfileFindSection(pCidProfile, &sectionType, 0, &pCidSeq, &length);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("EhSetLineState(): Caller ID profile"));
            return status;
        }

        /* Merge the caller ID sequence with the ring cadence sequence,
           and save the resulting composite sequence into the line object. */
        status = MergeSequences(pRingSeq, pCidSeq, mergedSeq);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("EhSetLineState(): failed to merge Caller ID and Tone Cadence"));
            return status;
        }
        Vp792SaveSequence(pLineCtx, (uint8 *)mergedSeq, TRUE);

        /* Start the sequence, just as in VpSendCid(). */
#ifdef VP_COMMON_ADDRESS_SPACE
        pLineObj->callerId.pCidProfile = pCidProfile;
#else
        Vp792SaveProfile(pCidProfile, pLineObj->callerId.cidProfile,
            &pLineObj->callerId.cidProfileValid, VP_PROFILE_CID);
#endif
        EhSendCid(pEvent, VP792_EVID_USER, VP792_UEVID_SEQUENCER_READY);
        pLineObj->eventHandlers |= VP792_EH_SEND_CID;
        return pEvent->status;
    } else {

        /* No Caller ID sequence.  Just start the ring cadence sequence. */
        return StartSequence(pLineCtx, pRingSeq, FALSE, VP792_SEQTYPE_RING);
    }
}

static VpStatusType
RingExit(
    VpLineCtxType *pLineCtx)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpLineStateType ringTripExitSt = pLineObj->options.ringControl.ringTripExitSt;

    if (
        (pLineObj->currentState != VP_LINE_RINGING) &&
        (pLineObj->currentState != VP_LINE_RINGING_POLREV)
    ) {
        /* Not ringing; nothing to do. */
        return VP_STATUS_SUCCESS;
    }

    if (
        (ringTripExitSt == VP_LINE_RINGING) ||
        (ringTripExitSt == VP_LINE_RINGING_POLREV)
    ) {
        /* Automatic ring exit state control is disabled. */
        return VP_STATUS_SUCCESS;
    }

    return Vp792SetLineState(pLineCtx, ringTripExitSt);
} /* RingExit() */

/******************************************************************************
 * This function programs the next DTMF amplitude/frequency information to
 * the device's signal generators.
 *
 * \param[in]     pLineCtx      Line context for line on which to transmit
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_FAILURE  The device is not currently transmitting FSK
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
static VpStatusType
SendDtmfData(
    VpLineCtxType *pLineCtx)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 word;
    uint8 tail;
    VpStatusType status = VP_STATUS_SUCCESS;

    /* Get byte from circular buffer, increment tail pointer. */
    tail = pLineObj->callerId.data.tail;
    word = pLineObj->callerId.data.buf[tail++];
    pLineObj->callerId.data.tail = (uint8)(tail % (VP_SIZEOF_CID_MSG_BUFFER * 2));

    /* Check for buffer-empty condition. */
    if (pLineObj->callerId.data.tail == pLineObj->callerId.data.head) {
        pLineObj->callerId.data.nonempty = FALSE;
    } else {
    }

    status = SendDtmfProgSigGen(pLineCtx, ConvertCharToDigitType((char) word));

    return VP_STATUS_SUCCESS;
} /* SendDtmfData() */

static VpStatusType
SendDtmfProgSigGen(
    VpLineCtxType *pLineCtx,
    VpDigitType digit)
{

#define VP792_SIGGEN_GENA_FREQ_1209 0xB021
#define VP792_SIGGEN_GENA_FREQ_1336 0xC083
#define VP792_SIGGEN_GENA_FREQ_1477 0x4396
#define VP792_SIGGEN_GENA_FREQ_1633 0x4189

#define VP792_SIGGEN_GENB_FREQ_697  0x164E
#define VP792_SIGGEN_GENB_FREQ_770  0x18A4
#define VP792_SIGGEN_GENB_FREQ_852  0x1B44
#define VP792_SIGGEN_GENB_FREQ_941  0x1E1D

#define VP792_SIGGEN_GENC_FREQ_1209 0x26B0
#define VP792_SIGGEN_GENC_FREQ_1336 0x2AC0
#define VP792_SIGGEN_GENC_FREQ_1477 0x2F43
#define VP792_SIGGEN_GENC_FREQ_1633 0x3441

    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status = VP_STATUS_SUCCESS;

    static const struct {
        uint16 sigGenFreqC;
        uint16 sigGenFreqB;
    } sigGenReg[] = {
        { VP792_SIGGEN_GENC_FREQ_1633, VP792_SIGGEN_GENB_FREQ_941}, /* VP_DIG_D */
        { VP792_SIGGEN_GENC_FREQ_1209, VP792_SIGGEN_GENB_FREQ_697}, /* VP_DIG_1 */
        { VP792_SIGGEN_GENC_FREQ_1336, VP792_SIGGEN_GENB_FREQ_697}, /* VP_DIG_2 */
        { VP792_SIGGEN_GENC_FREQ_1477, VP792_SIGGEN_GENB_FREQ_697}, /* VP_DIG_3 */
        { VP792_SIGGEN_GENC_FREQ_1209, VP792_SIGGEN_GENB_FREQ_770}, /* VP_DIG_4 */
        { VP792_SIGGEN_GENC_FREQ_1336, VP792_SIGGEN_GENB_FREQ_770}, /* VP_DIG_5 */
        { VP792_SIGGEN_GENC_FREQ_1477, VP792_SIGGEN_GENB_FREQ_770}, /* VP_DIG_6 */
        { VP792_SIGGEN_GENC_FREQ_1209, VP792_SIGGEN_GENB_FREQ_852}, /* VP_DIG_7 */
        { VP792_SIGGEN_GENC_FREQ_1336, VP792_SIGGEN_GENB_FREQ_852}, /* VP_DIG_8 */
        { VP792_SIGGEN_GENC_FREQ_1477, VP792_SIGGEN_GENB_FREQ_852}, /* VP_DIG_9 */
        { VP792_SIGGEN_GENC_FREQ_1336, VP792_SIGGEN_GENB_FREQ_941}, /* VP_DIG_ZERO */
        { VP792_SIGGEN_GENC_FREQ_1209, VP792_SIGGEN_GENB_FREQ_941}, /* VP_DIG_ASTER */
        { VP792_SIGGEN_GENC_FREQ_1477, VP792_SIGGEN_GENB_FREQ_941}, /* VP_DIG_POUND */
        { VP792_SIGGEN_GENC_FREQ_1633, VP792_SIGGEN_GENB_FREQ_697}, /* VP_DIG_A */
        { VP792_SIGGEN_GENC_FREQ_1633, VP792_SIGGEN_GENB_FREQ_770}, /* VP_DIG_B */
        { VP792_SIGGEN_GENC_FREQ_1633, VP792_SIGGEN_GENB_FREQ_852}  /* VP_DIG_C */
    };

    status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_SIG_GEN_FRQB_OFFSET, 1,
        (uint16p)&sigGenReg[digit].sigGenFreqB);

    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_SIG_GEN_FRQC_OFFSET, 1,
            (uint16p)&sigGenReg[digit].sigGenFreqC);
    }

    return status;
} /* SendDtmfProgSigGen() */

static VpStatusType
SendSigMsgWaitReadDcParams(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 requestIdx;
    VpStatusType status;

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);
        uint16 buf[VP792_CMDSIZE_RD_DC_PARAM], *pBufEnd = buf;

        if (pLineObj->sendSig.msgWait.readingDcParams) {
            /* User started two message-waiting signals in quick succession.  We
               only need to read the DC params once. */
            return VP_STATUS_SUCCESS;
        }
        pLineObj->sendSig.msgWait.readingDcParams = TRUE;

        /* Send the RD_DC_PARAMS mailbox command. */
        Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_RD_DC_PARAM, channelId);
        *pBufEnd++ = VP792_UEVID_RESPONSE_FIRST + requestIdx;
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_RD_DC_PARAM);

        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
            pRequest->requestType = VP792_REQUEST_SENDSIG_MSGWAIT;
            pRequest->args.lowLevelCmd16.channelId = channelId;
            pRequest->args.lowLevelCmd16.cmdType = VP_LOWLEV_MBOX_RD;
            pRequest->args.lowLevelCmd16.numReadWords = VP792_RSPSIZE_RD_DC_PARAM;
        }
    }

    return status;
} /* SendSigMsgWaitReadDcParams() */

static VpStatusType
SendSigMsgWaitWriteDcParams(
    VpLineCtxType *pLineCtx,
    uint16 v1,
    uint16 vas,
    uint16 ila)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 buf[VP792_CMDSIZE_WR_DC_PARAM], *pBufEnd = buf;

    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_DC_PARAM, pLineObj->channelId);

    *pBufEnd++ = v1;
    *pBufEnd++ = vas;
    *pBufEnd++ = pLineObj->sendSig.msgWait.dcParams.vasOffset;
    *pBufEnd++ = pLineObj->sendSig.msgWait.dcParams.rfd;
    *pBufEnd++ = pLineObj->sendSig.msgWait.dcParams.rptc;
    *pBufEnd++ = ila;

    return Vp792MailboxSend(pLineCtx->pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_DC_PARAM);
} /* SendSigMsgWaitWriteDcParams() */

static VpStatusType
SendSigMsgWaitSaveDcParams(
    VpEventType *pEvent)
{
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 requestIdx = pLineObj->sendSig.msgWait.requestIdx;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_RSPSIZE_RD_DC_PARAM];
    int16 bStat, hBat, lBat, pBat, hBatAdj, maxVoltage;
    int16 sigVoltage = ABS(pLineObj->sendSig.msgWait.voltage);
    VpStatusType status;

    /* Get the DC parameters from the response mailbox.  Save them in
       the line object so we can restore them during the OFF intervals. */
    status = ReadResponse(pEvent, requestIdx, buf);

    if (status == VP_STATUS_SUCCESS) {
        uint16 *pBufEnd = &buf[VP792_MBOX_PAYLOAD_INDEX + 1];
        pLineObj->sendSig.msgWait.dcParams.off.v1 = *pBufEnd++;
        pLineObj->sendSig.msgWait.dcParams.off.vas = *pBufEnd++;
        pLineObj->sendSig.msgWait.dcParams.vasOffset = *pBufEnd++;
        pLineObj->sendSig.msgWait.dcParams.rfd = *pBufEnd++;
        pLineObj->sendSig.msgWait.dcParams.rptc = *pBufEnd++;
        pLineObj->sendSig.msgWait.dcParams.off.ila = *pBufEnd++;
    }

    /* Get battery voltages. */
    status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_BAT_OFFSET,
        VP792_REG_BAT_LEN, buf);
    hBat = ((uint32)buf[VP792_REG_BAT_H_WORD] * 200) / 32768;
    lBat = ((uint32)buf[VP792_REG_BAT_L_WORD] * 200) / 32768;
    pBat = ((uint32)buf[VP792_REG_BAT_P_WORD] * 200) / 32768;

    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_HBAT_ADJ_OFFSET,
            VP792_REG_HBAT_ADJ_LEN, &buf[0]);
    }
    hBatAdj = ((uint32)buf[0] * 200) / 32768;

    /* Read CH_STAT register for battery status. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_CH_STAT_OFFSET,
            VP792_REG_CH_STAT_LEN, &buf[0]);
    }
    bStat = (buf[0] & VP792_REG_CH_STAT_BSTAT_MASK);

    /* Calculate the maximum voltage that can be produced by the currently-
       selected batteries. */
    if (bStat == VP792_REG_CH_STAT_BSTAT_PBAT) {
        maxVoltage = pBat - hBat - hBatAdj;
    } else if (bStat == VP792_REG_CH_STAT_BSTAT_HBAT) {
        maxVoltage = 0 - hBat - hBatAdj;
    } else if (bStat == VP792_REG_CH_STAT_BSTAT_LBAT) {
        maxVoltage = 0 - lBat;
    } else {
        maxVoltage = pBat - lBat;
    }

    /* Is it possible toproduce the voltage that was requested, given the
       current battery selection? */
    if (sigVoltage > maxVoltage) {
        sigVoltage = maxVoltage;
    }

#define MSGWAIT_V1_150_V            0xFFFF
#define MSGWAIT_ILA_5_MA            0x1248

    /* Calculate alternate DC parameters for ON intervals. */
    pLineObj->sendSig.msgWait.dcParams.on.v1 = MSGWAIT_V1_150_V;
    pLineObj->sendSig.msgWait.dcParams.on.vas =
        (uint32)(maxVoltage - sigVoltage) * 65536 / 150;
    pLineObj->sendSig.msgWait.dcParams.on.ila = MSGWAIT_ILA_5_MA;

    pLineObj->sendSig.msgWait.readingDcParams = FALSE;
    return status;
} /* SendSigMsgWaitSaveDcParams() */

static VpStatusType
SequencerControl(
    VpLineCtxType *pLineCtx,
    uint16 cmd)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 mboxBuf[VP792_CMDSIZE_WR_SEQ_CTL * 2];
    uint16p pBufEnd = mboxBuf;

    if (cmd == VP792_SEQ_CTL_STOP) {
        if (pLineObj->sequencer.aborting == TRUE) {
            /* Abort request already sent.  No need to send another. */
            return VP_STATUS_SUCCESS;
        }

    /* VP792_SEQ_CTL_BREAK_AND_RESUME is not implemented in the SLAC
       firmware as a single operation.  The API requests it by stuffing two
       consecutive sequencer commands into the mailbox. */
    } else if (cmd == VP792_SEQ_CTL_BREAK_AND_RESUME) {
        Vp792MailboxAddCmdHeader(mboxBuf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);
        *pBufEnd++ = VP792_SEQ_CTL_BREAK;
        cmd = VP792_SEQ_CTL_RESUME;
    }

    Vp792MailboxAddCmdHeader(mboxBuf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);
    *pBufEnd++ = cmd;

    return Vp792MailboxSend(pDevCtx, mboxBuf, pBufEnd, VP792_CMDSIZE_WR_SEQ_CTL * 2);
} /* SequencerControl() */

/******************************************************************************
 * This function starts the specified sequencer program (pSequence) on the
 * specified line (pLineCtx).
 *
 * \param[in]  pLineCtx     Line on which to start the sequence
 * \param[in]  pSequence    Sequencer program to be started
 * \param[in]  endian       Flag indicating whether the buffer is in the host
 *                          processor's endianness (TRUE) or big-endian (FALSE)
 * \param[in]  seqType      Sequencer program type
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_MAILBOX_BUSY
 *****************************************************************************/
static VpStatusType
StartSequence(
    VpLineCtxType *pLineCtx,
    uint8p pSequence,
    bool endian,
    uint16 seqType)
{

    /* Copy the sequencer program into the line object buffer. */
    Vp792SaveSequence(pLineCtx, pSequence, endian);

    /* Send the sequence to the device. */
    return Vp792StartSavedSequence(pLineCtx, seqType);
}

static bool
UnexpectedEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 eventParamWord)
{
    VP_INFO(VpDevCtxType, pEvent->pDevCtx, ("Unexpected event: %4.4X %4.4X", eventIdWord, eventParamWord));
    return FALSE;
}

#endif /* VP_CC_792_SERIES */
