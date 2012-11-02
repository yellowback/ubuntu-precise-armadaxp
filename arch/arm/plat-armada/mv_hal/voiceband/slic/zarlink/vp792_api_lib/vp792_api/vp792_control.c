/** \file vp792_control.c
 * vp792_control.c
 *
 *  This file contains the implementation of the VP-API 792 Series
 *  Control Functions.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 7149 $
 * $LastChangedDate: 2010-05-12 15:07:23 -0500 (Wed, 12 May 2010) $
 */

#include "vp_api.h"

#if defined (VP_CC_792_SERIES)  /* Compile only if required */

#define VP_HAL_DEVICE_TYPE VP_DEV_792_SERIES

#include "vp_api_int.h"
#include "vp792_api_int.h"


/* =================================
    Prototypes for Static Functions
   ================================= */

static bool
IsMeteringState(
    VpLineStateType state);

static bool
IsSigGenState(
    VpLineStateType state);

static VpStatusType
LineIoAccessIntWrite(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 ioMask,
    uint8 ioData);


static VpStatusType
LowLevelMboxRead(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle);

static VpStatusType
LowLevelMboxWrite(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords);

static VpStatusType
LowLevelPageRead(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle);

static VpStatusType
LowLevelPageWrite(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords);

static VpStatusType
MaskDedicatedPins(
    VpTermType termType,
    uint8 *pMask);

static VpStatusType
SetOptionDevice(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue);

static VpStatusType
SetOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pValue);

static VpStatusType
SetOptionLineEventMask(
    VpLineCtxType *pLineCtx,
    VpOptionEventMaskType *eventMask);

static VpStatusType
SetOptionLineIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpOptionLineIoConfigType *pLineIoCfg);

static VpStatusType
SetOptionLinePcmHwy(
    VpLineCtxType *pLineCtx,
    VpOptionPcmHwyType pcmHwy);

static VpStatusType
SetOptionLineTimeslot(
    VpLineCtxType *pLineCtx,
    VpOptionTimeslotType *timeSlots);

static VpStatusType
SetOptionLineZeroCross(
    VpLineCtxType *pLineCtx,
    VpOptionZeroCrossType zeroCrossMode);

static VpStatusType
UpdateCh1Mask(
    VpLineCtxType *pLineCtx);


/* ===================
    CONTROL FUNCTIONS
   =================== */

VpStatusType
Vp792ContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    int freeBytes;

    /* Calculate number of free bytes in caller ID buffer in line object. */
    if (pLineObj->callerId.data.nonempty == FALSE) {
        freeBytes = VP_SIZEOF_CID_MSG_BUFFER * 2;
    } else {
        freeBytes = pLineObj->callerId.data.tail - pLineObj->callerId.data.head;
        if (freeBytes < 0) {
            freeBytes += VP_SIZEOF_CID_MSG_BUFFER * 2;
        }
    }

    /* Get out if device state is not ready */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    if ((length > freeBytes) || (length == 0)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ContinueCid(): Data length out of range (%d > %d)", length, freeBytes));
        return VP_STATUS_INVALID_ARG;
    }
    if (pCidData == VP_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ContinueCid(): %s.", "Data pointer must not be NULL"));
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Copy FSK data into line object buffer. */
    while (length--) {
        pLineObj->callerId.data.buf[pLineObj->callerId.data.head++] = *pCidData++;
        pLineObj->callerId.data.head %= (VP_SIZEOF_CID_MSG_BUFFER * 2);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
} /* Vp792ContinueCid() */

VpStatusType
Vp792DeviceIoAccessExt(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessExtType *pDeviceIoAccess)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpIoDirectionType direction = pDeviceIoAccess->direction;
    bool dedicatedPins = FALSE;
    VpStatusType status = VP_STATUS_SUCCESS;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    if ((direction != VP_IO_READ) && (direction != VP_IO_WRITE)) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    if (direction == VP_IO_READ) {
        uint16 requestIdx;
        status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
        if (status == VP_STATUS_SUCCESS) {
            Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

            /* Save arguments for later.  The actual register read operations
               are done in VpGetResults(). */
            pRequest->requestType = VP792_REQUEST_DEVICE_IO_ACCESS_EXT;
            pRequest->args.deviceIoAccessExt.deviceIoAccess = *pDeviceIoAccess;

            /* Generate a VP792_EVID_USER event, which VpGetEvent() will
               translate into a VP_DEV_EVID_IO_ACCESS_CMP event. */
            status = Vp792GenerateUserEvent(pDevCtx,
                VP792_UEVID_RESPONSE_FIRST + requestIdx, 0);
            if (status == VP_STATUS_SUCCESS) {
                pRequest->outstanding = TRUE;
            }
        }

    } else /* (direction == VP_IO_WRITE) */ {
        uint8 chan;

        for (chan = 0; chan < pDevObj->maxChannels; chan++) {
            uint8 mask = pDeviceIoAccess->lineIoBits[chan].mask;
            uint8 data = pDeviceIoAccess->lineIoBits[chan].data;

            if (mask != 0) {
                status = LineIoAccessIntWrite(pDevCtx, chan, mask, data);
            }
            if (status == VP_STATUS_DEDICATED_PINS) {
                dedicatedPins = TRUE;
            } else if (status != VP_STATUS_SUCCESS) {
                break;
            }
        }

        /* Generate a VP792_EVID_USER event, which VpGetEvent() will
           translate into a VP_DEV_EVID_IO_ACCESS_CMP event. */
        if ((status == VP_STATUS_SUCCESS) || (status == VP_STATUS_DEDICATED_PINS)) {
            status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_IO_WRITE_CMP, 0);
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* If any of the LineIoAccessIntWrite() calls returned VP_STATUS_DEDICATED_PINS,
       we should return that value here. */
    if ((status == VP_STATUS_SUCCESS) && dedicatedPins) {
        status = VP_STATUS_DEDICATED_PINS;
    }

    return status;
} /* Vp792DeviceIoAccessExt */

VpStatusType
Vp792GenTimerCtrl(
    VpLineCtxType *pLineCtx,
    VpGenTimerCtrlType timerCtrl,
    uint32 duration,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_CMDSIZE_TIMER_CTL], *pBufEnd = buf;
    uint16 requestIdx;
    uint16 control;
    VpStatusType status;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792GenTimerCtrl()+"));

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    switch (timerCtrl) {
        case VP_GEN_TIMER_START:
            control = VP792_CMD_GEN_TIMER_START;
            break;
        case VP_GEN_TIMER_CANCEL:
            control = VP792_CMD_GEN_TIMER_STOP;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    if (duration > VP792_CMD_GEN_TIMER_DELAY_MAX) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Add the standard header information to the mailbox buffer. */
        Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_TIMER_CTL, channelId);

        /* Add the handle and control arguments to the mailbox buffer. */
        *pBufEnd++ = VP792_UEVID_RESPONSE_FIRST + requestIdx;
        *pBufEnd++ = control;

        /* Add the duration to the mailbox buffer. */
        *pBufEnd++ = (uint16)duration;

        /* Send the command to the VP792's mailbox. */
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_TIMER_CTL);

        /* Save arguments for VpGetEvent(). */
        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
            pRequest->requestType = VP792_REQUEST_TIMER_CMP;
            pRequest->args.genTimer.handle = handle;
            pRequest->args.genTimer.channelId = channelId;
            pRequest->args.genTimer.internal = FALSE;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792GenTimerCtrl()-"));

    return status;
} /* Vp792GenTimerCtrl */

VpStatusType
Vp792LineIoAccess(
    VpLineCtxType *pLineCtx,
    VpLineIoAccessType *pLineIoData,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpIoDirectionType direction = pLineIoData->direction;
    uint16 requestIdx;
    bool dedicatedPins = FALSE;
    VpStatusType status = VP_STATUS_SUCCESS;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    if ((direction != VP_IO_READ) && (direction != VP_IO_WRITE)) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Save arguments for VpGetEvent(). */
        pRequest->requestType = VP792_REQUEST_LINE_IO_ACCESS;
        pRequest->args.lineIoAccess.handle = handle;
        pRequest->args.lineIoAccess.lineIoAccess = *pLineIoData;

        /* For read operations, the actual register accesses are done in
           VpGetEvent().  For writes, we do them now. */
        if (direction == VP_IO_WRITE) {
            uint8 mask = pLineIoData->ioBits.mask;
            uint8 data = pLineIoData->ioBits.data;

            status = LineIoAccessIntWrite(pDevCtx, channelId, mask, data);
            if (status == VP_STATUS_DEDICATED_PINS) {
                dedicatedPins = TRUE;
                status = VP_STATUS_SUCCESS;
            }
        }

        /* Generate a VP792_EVID_USER event, which VpGetEvent() will
           translate into a VP_LINE_EVID_LINE_IO_RD_CMP or
           VP_LINE_EVID_LINE_IO_WR_CMP event. */
        if (status == VP_STATUS_SUCCESS) {
            status = Vp792GenerateUserEvent(pDevCtx,
                VP792_UEVID_RESPONSE_FIRST + requestIdx, channelId);
        }
        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    if ((status == VP_STATUS_SUCCESS) && dedicatedPins) {
        status = VP_STATUS_DEDICATED_PINS;
    }

    return status;
} /* Vp792LineIoAccess() */

VpStatusType
Vp792LowLevelCmd16(
    VpLineCtxType *pLineCtx,
    VpLowLevelCmdType cmdType,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_INVALID_ARG;

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    switch (cmdType) {
        case VP_LOWLEV_PAGE_WR:
            status = LowLevelPageWrite(pLineCtx, writeWords, numWriteWords);
            break;
        case VP_LOWLEV_MBOX_WR:
            status = LowLevelMboxWrite(pLineCtx, writeWords, numWriteWords);
            break;
        case VP_LOWLEV_PAGE_RD:
            status = LowLevelPageRead(pLineCtx, writeWords, numWriteWords,
                numReadWords, handle);
            break;
        case VP_LOWLEV_MBOX_RD:
            status = LowLevelMboxRead(pLineCtx, writeWords, numWriteWords,
                numReadWords, handle);
            break;
        default:
            break;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792LowLevelCmd16() */

VpStatusType
Vp792SendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    /* Get out if device state is not ready */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments.  NULL CID profile is not acceptable. */
    if (pCidProfile == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_CID, &pCidProfile);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Check arguments. */
    if ((length > (VP_SIZEOF_CID_MSG_BUFFER * 2)) || (length == 0)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SendCid(): %s.", "Data length out of range"));
        return VP_STATUS_INVALID_ARG;
    }
    if (pCidData == VP_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SendCid(): %s.", "Data pointer must not be NULL"));
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Save Caller ID profile pointer for EhSendCid(). */
#ifdef VP_COMMON_ADDRESS_SPACE
    pLineObj->callerId.pCidProfile = pCidProfile;
#else
    Vp792SaveProfile(pCidProfile, pLineObj->callerId.cidProfile,
        &pLineObj->callerId.cidProfileValid, VP_PROFILE_CID);
#endif

    /* Copy the Caller ID sequencer program into the line object. */
    {
        Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_SEQUENCE;
        uint8 *pSequence, temp;
        status = Vp792ProfileFindSection(pCidProfile, &sectionType,
            0, &pSequence, &temp);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SendCid(): %s.", "Bad Caller ID profile"));
        } else {
            Vp792SaveSequence(pLineCtx, pSequence, FALSE);
        }
    }

    /* Load CID data and preamble bits into the line object. */
    Vp792LoadCidData(pLineCtx, length, pCidData);

    /* Get channel seizure duration and mark duration from CID profile. */
    status = Vp792ProfileProcessSections(pDevCtx, channelId,
        pCidProfile, VP_NULL, VP_NULL,
        VP792_PROF_SECTYPE_UNSTRUCTURED);

    /* Start the Caller ID sequence in EhSendCid(): */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792NewSequence(pLineCtx, VP792_EH_SEND_CID, VP_NULL, 0);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
}

VpStatusType
Vp792SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType signalType,
    void *pSignalData)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    uint16 onCmd, offCmd, onTime, offTime, cycles;
    VpStatusType status = VP_STATUS_SUCCESS;

    if (pSignalData == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    switch (signalType) {
        case VP_SENDSIG_MSG_WAIT_PULSE: {
            VpSendMsgWaitType *pMsgWait = pSignalData;
            onCmd = VP792_SEQ_CMD_USER_EVENT | VP792_UEVID_MSG_WAIT_ON;
            offCmd = VP792_SEQ_CMD_USER_EVENT | VP792_UEVID_MSG_WAIT_OFF;
            onTime = pMsgWait->onTime;
            offTime = pMsgWait->offTime;
            cycles = pMsgWait->cycles;
            pLineObj->sendSig.msgWait.voltage = pMsgWait->voltage;

            /* If abort was requested, wait for the next VP792_UEVID_MSG_WAIT_OFF
               event, and then stop the sequencer.  This flag is checked in
               EhSendSignal(). */
            pLineObj->sendSig.msgWait.aborting = (onTime == 0);
            break;
        }
        case VP_SENDSIG_FWD_DISCONNECT: {
            onCmd = VP792_SEQ_CMD_DRIVE_ST | VP792_REG_DRIVE_ST_ST_DISCONNECT;
            offCmd = VP792_SEQ_CMD_DELAY; /* delay 0ms => "No Op" */
            onTime = *(uint16 *)pSignalData;
            offTime = 100;
            cycles = 1;
            break;
        }
        case VP_SENDSIG_POLREV_PULSE: {
            onCmd = VP792_SEQ_CMD_POLREV;
            offCmd = VP792_SEQ_CMD_POLREV;
            onTime = *(uint16 *)pSignalData;
            offTime = 100;
            cycles = 1;
            break;
        }
        default:
            onTime = offTime = onCmd = offCmd = cycles = 0; /* silence warnings */
            status = VP_STATUS_INVALID_ARG;
    }

    /* Build a sequencer program and save it in the line object: */
    if ((onTime > 0) && (status == VP_STATUS_SUCCESS)) {
        uint16 *buf = pLineObj->sequencer.program, *pBufEnd = buf;
        uint16 programSize, moreCycles = 0;

        if (cycles > VP792_SEQ_BRANCH_ITERATE_MAX + 1) {

            /* Cycles is more than the SLAC's sequencer can do natively.
               We need more iterations, so we will add a second loop. */
            moreCycles = VP792_SEQ_BRANCH_ITERATE_MAX + 1;
            cycles -= moreCycles;
        }

        Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);

        /* Add first loop */
        *pBufEnd++ = VP792_SEQ_CTL_START;
        *pBufEnd++ = onCmd;
        *pBufEnd++ = VP792_SEQ_CMD_DELAY | onTime;
        *pBufEnd++ = offCmd;
        *pBufEnd++ = VP792_SEQ_CMD_DELAY | offTime;

        /* Add first loop branch instruction (if needed) */
        if (cycles == 0) {
            *pBufEnd++ = VP792_SEQ_CMD_BRANCH;  /* branch forever */
        } else if (cycles > 1) {
            *pBufEnd++ = VP792_SEQ_CMD_BRANCH |
                ((cycles - 1) << VP792_SEQ_BRANCH_ITERATE_POS);
        } /* else cycles = 1, so no branch is necessary */

        /* Add second loop (if needed) */
        if (moreCycles > 0) {
            uint16 secondLoopStartPos = pBufEnd - buf - VP792_MBOX_PAYLOAD_INDEX - 1;
            *pBufEnd++ = onCmd;
            *pBufEnd++ = VP792_SEQ_CMD_DELAY | onTime;
            *pBufEnd++ = offCmd;
            *pBufEnd++ = VP792_SEQ_CMD_DELAY | offTime;

            /* Add second loop branch instruction (if needed) */
            if (moreCycles > 1) {
                *pBufEnd++ = VP792_SEQ_CMD_BRANCH |
                    ((moreCycles - 1) << VP792_SEQ_BRANCH_ITERATE_POS) |
                    secondLoopStartPos;
            }
        }

        /* Update the length field. */
        programSize = pBufEnd - buf - 1; /* exclude CTL field */
        buf[VP792_MBOX_PAYLOAD_INDEX] |= programSize - VP792_MBOX_PAYLOAD_INDEX - 1;

        /* Start the saved sequence in EhSendSignal(): */
        pLineObj->sendSig.newSignal = signalType;
        status = Vp792NewSequence(pLineCtx, VP792_EH_SEND_SIGNAL, VP_NULL, 0);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792SendSignal() */

VpStatusType
Vp792SetBFilter(
    VpLineCtxType *pLineCtx,
    VpBFilterModeType bFiltMode,
    VpProfilePtrType pAcProfile)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;
    uint16 oldValue, newValue;
    uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE];
    uint16p pBufEnd = buf;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792SetBFilter()+"));

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    if (
        ((pAcProfile == VP_PTABLE_NULL) && (bFiltMode == VP_BFILT_EN)) ||
        ((bFiltMode != VP_BFILT_DIS) && (bFiltMode != VP_BFILT_EN))
    ) {
        return VP_STATUS_INVALID_ARG;
    }
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_AC, &pAcProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetBFilter(): Invalid AC profile"));
        return status;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_VP_CFG1_OFFSET,
                               VP792_REG_VP_CFG1_LEN, &oldValue);

    if (status == VP_STATUS_SUCCESS) {
        if (bFiltMode == VP_BFILT_DIS) {
            newValue = oldValue & ~VP792_REG_VP_CFG1_EB_MASK;
        } else {
            newValue = oldValue | VP792_REG_VP_CFG1_EB_MASK;

            /* Send the WR_AC_PARAMS command to the VP792 device. */
            status = Vp792ProfileProcessSections(pDevCtx, channelId, pAcProfile,
                        buf, &pBufEnd, VP792_PROF_SECTYPE_MBOXCMD);
            if (status == VP_STATUS_SUCCESS) {

                /* Only update the B-filter. */
                buf[VP792_MBOX_PAYLOAD_INDEX] = VP792_MB_WR_AC_PARAM_AC_MASK_BFIL;
                status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
            }
        }
    }

    if ((newValue != oldValue) && (status == VP_STATUS_SUCCESS)) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_VP_CFG1_OFFSET,
                                    VP792_REG_VP_CFG1_LEN, &newValue);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792SetBFilter()-"));

    return status;
} /* Vp792SetBFilter() */

VpStatusType
Vp792SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    /* Get out if device state is not ready */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    return Vp792SetLineStateInt(pLineCtx, state, TRUE);
} /* Vp792SetLineState() */

VpStatusType
Vp792SetLineStateInt(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    bool criticalSection)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    bool preserveActiveTone, preserveActiveMetering;
    uint16 buf[VP792_CMDSIZE_WR_MTR_CTL + VP792_CMDSIZE_WR_SEQ_CTL + VP792_CMDSIZE_USER_CTL], *pBufEnd = buf;
    uint16 activeSequence;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792SetLineStateInt()+"));

    /* Argument check */
    switch (state) {
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
        case VP_LINE_PARK:
        case VP_LINE_HOWLER:
        case VP_LINE_TESTING:
        case VP_LINE_DISABLED:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_NULLFEED:
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    /* When switching among line states that support signal generation, if
       there's an ongoing tone (cadenced or not), we don't want to disturb it. */
    preserveActiveTone = IsSigGenState(pLineObj->currentState) && IsSigGenState(state);

    /* When switching among line states that support metering, if
       there's an ongoing metering signal, we don't want to disturb it. */
    preserveActiveMetering = IsMeteringState(pLineObj->currentState) && IsMeteringState(state);

    if (criticalSection) {
        VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);
    }

    /* If there's an ongoing tone cadence and we're entering a line state in
       which the tone cadence should continue, we should not abort the
       sequencer.  In order to achieve that, temporarily clear the
       activeSequence member before calling Vp792NewSequence(). */
    activeSequence = pLineObj->sequencer.activeSequence;
    if (preserveActiveTone && (activeSequence == VP792_SEQTYPE_TONE)) {
        pLineObj->sequencer.activeSequence = VP792_SEQTYPE_OFF;
    } else if (!preserveActiveTone && (activeSequence == VP792_SEQTYPE_OFF)) {
        /* Make sure to kill any ongoing uncadenced tones.  Cadenced tones
           will be stopped in Vp792NewSequence(). */
        uint16 sigGenCtl = VP792_REG_SIG_GEN_CTL_DISABLE;
        status = Vp792HbiPagedWrite(pDevCtx, channelId,
            VP792_REG_SIG_GEN_OFFSET + VP792_REG_SIG_GEN_CTL_WORD,
            1, &sigGenCtl);
    }

    /* If we need to cancel metering, add the command to the mailbox buffer. */
    if (pLineObj->metering.active && !preserveActiveMetering) {
        pLineObj->ehSetLineState.waitForMeterAbort = TRUE;
        Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_MTR_CTL, channelId);
        *pBufEnd++ = VP792_MTR_CTL_STOP;
    } else {
        pLineObj->ehSetLineState.waitForMeterAbort = FALSE;
    }

    /* If a sequence is currently running, we must stop it before entering a
       new line state.  Complete the line state transition in EhSetLineState(). */
    if (status == VP_STATUS_SUCCESS) {
        pLineObj->ehSetLineState.newState = state;
        pLineObj->ehSetLineState.waitForSequencerReady = TRUE; /* real or fake */
        status = Vp792NewSequence(pLineCtx, VP792_EH_SET_LINE_STATE, buf, pBufEnd - buf);
    }

    /* We changed line states, but there may still be an ongoing tone. */
    if (preserveActiveTone) {
        pLineObj->sequencer.activeSequence = activeSequence;
    }

    if (criticalSection) {
        VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp792SetLineStateInt()-"));

    return status;
} /* VpSetLineStateInt() */

VpStatusType
Vp792SetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pToneCadProfile,
    VpDtmfToneGenType *pDtmfControl)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status;

    VP_API_INT_ENTER(VpLineCtxType, pLineCtx, "Vp792SetLineTone");

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check the validity of the tone profile */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_TONE, &pToneProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetLineTone(): Invalid tone profile"));
        return status;
    }

    /* Check the validity of the tone cadence profile. */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_TONECAD, &pToneCadProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetLineTone(): Invalid tone cadence profile"));
        return status;
    }

    if (pDtmfControl != VP_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetLineTone(): pDtmfControl not implemented"));
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

#ifdef VP_COMMON_ADDRESS_SPACE
    /* Save the profile pointers into the line object. */
    pLineObj->profiles.pTone = pToneProfile;
    pLineObj->profiles.pToneCad = pToneCadProfile;
#else
    Vp792SaveProfile(pToneProfile, pLineObj->profiles.tone,
        &pLineObj->profiles.valid, VP_PROFILE_TONE);
    Vp792SaveProfile(pToneCadProfile, pLineObj->profiles.toneCad,
        &pLineObj->profiles.valid, VP_PROFILE_TONECAD);
#endif

    /* Start the sequence in EhSetLineTone(): */
    status = Vp792NewSequence(pLineCtx, VP792_EH_SET_LINE_TONE, VP_NULL, 0);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_INT_EXIT(VpLineCtxType, pLineCtx, "Vp792SetLineTone", status);
    return status;
} /* Vp792SetLineTone() */

VpStatusType
Vp792SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue)
{
    Vp792DeviceObjectType *pDevObj;
    VpLineCtxType *pLineCtxLocal;
    VpDeviceIdType deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channelId;

    if (pDevCtx != VP_NULL) {
        pDevObj = pDevCtx->pDevObj;
    } else {
        pDevObj = pLineCtx->pDevCtx->pDevObj;
    }
    deviceId = pDevObj->deviceId;

    /* Get out if device state is not ready */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    switch (option) {
        case VP_DEVICE_OPTION_ID_PARK_MODE:
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
        case VP_DEVICE_OPTION_ID_DEV_IO_CFG:
            if (pDevCtx != VP_NULL) {
                status = SetOptionDevice(pDevCtx, option, pValue);
            } else {
                status = VP_STATUS_INVALID_ARG;
            }
            break;
        case VP_OPTION_ID_PULSE:
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_PCM_HWY:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_EVENT_MASK:
        case VP_OPTION_ID_ZERO_CROSS:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_LINE_IO_CFG:
        case VP_OPTION_ID_DTMF_SPEC:
        case VP_OPTION_ID_DTMF_MODE:
        case VP_OPTION_ID_DCFEED_SLOPE:
            if (pDevCtx != VP_NULL) {
                /*
                 * Loop through all of the valid channels associated with this
                 * device.
                 */
                for (channelId = 0; channelId < pDevObj->maxChannels; channelId++) {
                    pLineCtxLocal = pDevCtx->pLineCtx[channelId];
                    if (pLineCtxLocal != VP_NULL) {
                        status = SetOptionLine(pLineCtxLocal, option, pValue);
                        if (status != VP_STATUS_SUCCESS) {
                            break;
                        }
                    }
                }
            } else {
                status = SetOptionLine(pLineCtx, option, pValue);
            }
            break;

#if (VP_CC_DEBUG_SELECT != 0)
        case VP_OPTION_ID_DEBUG_SELECT:
            if (pLineCtx == VP_NULL) {
                status = SetOptionDevice(pDevCtx, VP_OPTION_ID_DEBUG_SELECT, pValue);
            } else {
                status = SetOptionLine(pLineCtx, VP_OPTION_ID_DEBUG_SELECT, pValue);
            }
            break;
#endif

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            break;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);
    return status;
} /* Vp792SetOption() */

VpStatusType
Vp792SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint16 oldValue, newValue;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792HbiPagedRead(pDevCtx, channelId,
        VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &oldValue);
    newValue = oldValue;

    switch (pLineObj->termType) {
        case VP_TERM_FXS_GENERIC:
            switch (rState) {
                case VP_RELAY_NORMAL:
                case VP_RELAY_TALK:
                    newValue &= ~VP792_REG_IO_ST_TSW_MASK;
                    break;
                case VP_RELAY_BRIDGED_TEST:
                    newValue |= VP792_REG_IO_ST_TSW_MASK;
                    break;
                default:
                    status = VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_TERM_FXS_TI:
            switch (rState) {
                case VP_RELAY_NORMAL:
                case VP_RELAY_TALK:
                    newValue &= ~VP792_REG_IO_ST_TSW_MASK;
                    newValue |= VP792_REG_IO_ST_IO_0_MASK;
                    break;
                case VP_RELAY_TEST:
                case VP_RELAY_DISCONNECT:
                    newValue &= ~VP792_REG_IO_ST_TSW_MASK;
                    newValue &= ~VP792_REG_IO_ST_IO_0_MASK;
                    break;
                case VP_RELAY_BRIDGED_TEST:
                    newValue |= VP792_REG_IO_ST_TSW_MASK;
                    newValue |= VP792_REG_IO_ST_IO_0_MASK;
                    break;
                case VP_RELAY_SPLIT_TEST:
                    newValue |= VP792_REG_IO_ST_TSW_MASK;
                    newValue &= ~VP792_REG_IO_ST_IO_0_MASK;
                    break;
                default:
                    status = VP_STATUS_INVALID_ARG;
            }
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
    }

    if ((newValue != oldValue) && (status == VP_STATUS_SUCCESS)) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId,
            VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &newValue);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792SetRelayState() */

VpStatusType
Vp792SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_CMDSIZE_WR_REL_GAIN + VP792_CMDSIZE_RD_REL_GAIN], *pBufEnd = buf;
    uint16 requestIdx;
    Vp792ResponseRequestType *pRequest;
    bool readResult = FALSE;
    VpStatusType status;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Add the VP792_CMD_WR_REL_GAIN command to the mailbox buffer. */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_REL_GAIN, channelId);

    /* Add the arguments to the mailbox buffer. */
    *pBufEnd++ = 0; /* reserved */
    *pBufEnd++ = txLevel;
    *pBufEnd++ = rxLevel;

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);
    pRequest = &(pDevObj->responseRequest[requestIdx]);

    /* If the VP_LINE_EVID_GAIN_CMP event is unmasked, add a RD_REL_GAIN
       command. */
    if (!(pLineObj->options.eventMask.response & VP_LINE_EVID_GAIN_CMP)) {
        readResult = TRUE;
        if (status == VP_STATUS_SUCCESS) {
            Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_RD_REL_GAIN, channelId);
            *pBufEnd++ = VP792_UEVID_RESPONSE_FIRST + requestIdx;
        }
    }

    /* Send the command(s) to the VP792's mailbox. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd,
            VP792_CMDSIZE_WR_REL_GAIN + VP792_CMDSIZE_RD_REL_GAIN);
    }

    /* Save arguments for VpGetEvent(). */
    if (readResult && (status == VP_STATUS_SUCCESS)) {
        pRequest->outstanding = TRUE;
        pRequest->requestType = VP792_REQUEST_SET_REL_GAIN;
        pRequest->args.setRelGain.handle = handle;
        pRequest->args.setRelGain.channelId = channelId;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792SetRelGain() */

VpStatusType
Vp792StartMeter32Q(
    VpLineCtxType *pLineCtx,
    uint32 minDelay,
    uint32 onTime,
    uint32 offTime,
    uint16 numMeters,
    uint16 eventRate)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_CMDSIZE_WR_MTR_CTL], *pBufEnd = buf;
    uint16 ctl;
    VpStatusType status;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Select operation (start or abort): */
    if ((onTime == 0) && (numMeters == 0)) {
        ctl = VP792_MTR_CTL_STOP;
    } else {
        ctl = VP792_MTR_CTL_START;
    }

    /* Handle special argument values. */
    if (onTime == 0) {
        onTime = 0xFFFFFFFF;
    }
    if (eventRate == 0) {
        eventRate = 1;
    }

    /* Add the VP792_CMDSIZE_WR_MTR_CTL command to the mailbox buffer. */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_MTR_CTL, channelId);
    *pBufEnd++ = ctl;
    *pBufEnd++ = numMeters;
    *pBufEnd++ = (uint16)(minDelay >> 16);
    *pBufEnd++ = (uint16)(minDelay & 0xFFFF);
    *pBufEnd++ = (uint16)(offTime >> 16);
    *pBufEnd++ = (uint16)(offTime & 0xFFFF);
    *pBufEnd++ = (uint16)(onTime >> 16);
    *pBufEnd++ = (uint16)(onTime & 0xFFFF);

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* If we're currently in a line state that doesn't support metering, and
       not in the middle of a line state transition, we should generate a
       MTR_ABORT event. */
    if (
        !IsMeteringState(pLineObj->currentState) &&
        !(pLineObj->eventHandlers & VP792_EH_SET_LINE_STATE)
    ) {
        status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_MTR_ABORT, channelId);
        pLineObj->metering.active = FALSE;
    }

    /* If trying to start metering and we're in a line-state transition to a
       state that doesn't support metering (and we haven't already requested a
       MTR_ABORT), we should generate a MTR_ABORT event. */
    else if (
        (pLineObj->eventHandlers & VP792_EH_SET_LINE_STATE) &&
        !IsMeteringState(pLineObj->ehSetLineState.newState) &&
        !pLineObj->ehSetLineState.waitForMeterAbort
    ) {
        status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_MTR_ABORT, channelId);
        pLineObj->metering.active = FALSE;
    }

    else {
        /* Send the command to the VP792's mailbox. */
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_MTR_CTL);

        /* Save eventRate into the Line Object. */
        if ((status == VP_STATUS_SUCCESS) && (ctl == VP792_MTR_CTL_START)) {
            pLineObj->metering.active = TRUE;
            pLineObj->metering.eventRate = eventRate;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792StartMeter32Q() */


/* ===================
    Static Functions
   =================== */

/**
 * This internal function specifies whether metering is allowed in the
 * given line state.
 *
 * \param[in]       state    VP-API-II line state
 *
 * \returns TRUE if metering is allowed.
 */
static bool
IsMeteringState(
    VpLineStateType state)
{
    switch (state) {
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
        case VP_LINE_PARK:
        case VP_LINE_TESTING:
        case VP_LINE_DISABLED:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_NULLFEED:
        /* To do: Metering should be allowed in OHT and NULLFEED states, but
           currently it is disabled for behavioral equivalence with VCP.  When
           this is fixed, IsMeteringState() will be identical to IsSigGenState()
           and thus can be removed. */
            return FALSE;

        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_HOWLER:
        default:
            return TRUE;
    }
}

/**
 * This internal function specifies whether signal generation is allowed in the
 * given line state.
 *
 * \param[in]       state    VP-API-II line state
 *
 * \returns TRUE if signal generation is allowed.
 */
static bool
IsSigGenState(
    VpLineStateType state)
{
    switch (state) {
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
        case VP_LINE_PARK:
        case VP_LINE_TESTING:
        case VP_LINE_DISABLED:
            return FALSE;

        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_HOWLER:
        case VP_LINE_NULLFEED:
        default:
            return TRUE;
    }
}

static VpStatusType
LineIoAccessIntWrite(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 mask,
    uint8 data)
{
    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
    VpStatusType status;
    uint16 oldValue, newValue;

    /* So as not to disturb the other bits in the IO_ST register, we do a
       read-modify-write sequence. */
    status = Vp792HbiPagedRead(pDevCtx, channelId,
        VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &oldValue);

    /* For writes, modify the access mask to exclude GPIO pins dedicated
       to the line termination type. */
    if ((pLineCtx != VP_NULL) && (status == VP_STATUS_SUCCESS)) {
        Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
        status = MaskDedicatedPins(pLineObj->termType, &mask);
    }

    /* Zero out the dedicated bits in the IO_ST register, and OR in the new
       values. */
    newValue = (oldValue & ~(uint16)mask) | (data & mask);

    /* If necessary, write the new register value. */
    if (newValue != oldValue) {
        VpStatusType tempStatus = Vp792HbiPagedWrite(pDevCtx, channelId,
            VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &newValue);
        if (tempStatus != VP_STATUS_SUCCESS) {
            status = tempStatus;
        }
    }

    return status;
}

static VpStatusType
LowLevelPageWrite(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    uint16 cmd = HBI_SELECT_PAGE(pDevObj->slacId, channelId);
    VpStatusType status;

    /* Write length of 0 would be pointless. */
    if (numWriteWords == 0) {
        return VP_STATUS_INVALID_ARG;
    }

    if (!VP_HAL_HBI_WRITE(deviceId, cmd, numWriteWords - 1, writeWords)) {
        return VP_STATUS_ERR_HBI;
    }

    /* Check for HBI desync. */
    status = Vp792HbiSync(pDevCtx);

    /* If the LLCMD_TX_CMP event is masked, don't bother generating the event. */
    if (
        ((pLineObj->options.eventMask.response & VP_LINE_EVID_LLCMD_TX_CMP) == 0) &&
        (status == VP_STATUS_SUCCESS)
    ) {
        status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_LLCMD_TX_CMP, channelId);
    }

    return status;
} /* LowLevelPageWrite() */

static VpStatusType
LowLevelMboxWrite(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    /* Need at least 2 words to create a valid mailbox command */
    if (numWriteWords < 2) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Acquire exclusive access to the command mailbox. */
    status = Vp792CmdMailboxAcquire(pDevCtx);

    /* Dump the mailbox buffer to the VP792's command mailbox. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, VP792_HBI_COMMAND_MBOX_PAGE, 0,
            numWriteWords, writeWords);
    }

    /* Release control of the mailbox to the VP792. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792CmdMailboxRelease(pDevCtx);
    }

    /* Check for HBI desync. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiSync(pDevCtx);
    }

    /* If the LLCMD_TX_CMP event is masked, don't bother generating the event. */
    if (
        ((pLineObj->options.eventMask.response & VP_LINE_EVID_LLCMD_TX_CMP) == 0) &&
        (status == VP_STATUS_SUCCESS)
    ) {
        status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_LLCMD_TX_CMP, channelId);
    }

    return status;
} /* LowLevelMboxWrite() */

static VpStatusType
LowLevelPageRead(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 requestIdx;
    VpStatusType status;

    /* Must be 1 or 2 write words.  2 allows for page select followed by read
       command for reading mailbox or codeload pages. */
    if (numWriteWords < 1 || numWriteWords > 2) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Read words must be at least 1 and no more than 128 */
    if (numReadWords < 1 || numReadWords > 128) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Reserve a request buffer in the device object. */
    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);

    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Save arguments for later.  The actual register read operations
           are done in VpGetResults(). */
        pRequest->requestType = VP792_REQUEST_LOW_LEVEL_CMD_16;
        pRequest->args.lowLevelCmd16.handle = handle;
        pRequest->args.lowLevelCmd16.channelId = channelId;
        pRequest->args.lowLevelCmd16.cmdType = VP_LOWLEV_PAGE_RD;
        pRequest->args.lowLevelCmd16.numReadWords = numReadWords;
        pRequest->args.lowLevelCmd16.numWriteWords = numWriteWords;
        pRequest->args.lowLevelCmd16.writeWords[0] = writeWords[0];
        if (numWriteWords > 1) {
            pRequest->args.lowLevelCmd16.writeWords[1] = writeWords[1];
        }

        /* Generate a VP792_EVID_USER event, which VpGetEvent() will
           translate into a VP_LINE_EVID_LLCMD_RX_CMP event. */
        status = Vp792GenerateUserEvent(pDevCtx,
            VP792_UEVID_RESPONSE_FIRST + requestIdx, channelId);
        if (status == VP_STATUS_SUCCESS) {
            pRequest->outstanding = TRUE;
        }
    }

    return status;
} /* LowLevelPageRead() */

static VpStatusType
LowLevelMboxRead(
    VpLineCtxType *pLineCtx,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 requestIdx;
    VpStatusType status;

    /* Need at least 2 words to create a valid mailbox command */
    if (numWriteWords < 2) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Read words must be at least 1 and no more than 128 */
    if (numReadWords < 1 || numReadWords > 128) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Reserve a request buffer in the device object. */
    status = Vp792GetFreeRequestIdx(pDevCtx, &requestIdx);

    /* Use an internal handle instead of the user-supplied one. */
    writeWords[VP792_MBOX_PAYLOAD_INDEX] = VP792_UEVID_RESPONSE_FIRST + requestIdx;

    /* Acquire exclusive access to the command mailbox. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792CmdMailboxAcquire(pDevCtx);
    }

    /* Dump the mailbox buffer to the VP792's command mailbox. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, VP792_HBI_COMMAND_MBOX_PAGE, 0,
            numWriteWords, writeWords);
    }

    /* Release control of the mailbox to the VP792. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792CmdMailboxRelease(pDevCtx);
    }

    if (status == VP_STATUS_SUCCESS) {
        Vp792ResponseRequestType *pRequest = &(pDevObj->responseRequest[requestIdx]);

        /* Save arguments for VpGetResults(). */
        pRequest->requestType = VP792_REQUEST_LOW_LEVEL_CMD_16;
        pRequest->args.lowLevelCmd16.handle = handle;
        pRequest->args.lowLevelCmd16.channelId = channelId;
        pRequest->args.lowLevelCmd16.cmdType = VP_LOWLEV_MBOX_RD;
        pRequest->args.lowLevelCmd16.numReadWords = numReadWords;
        pRequest->outstanding = TRUE;
    }

    return status;
} /* LowLevelMboxRead() */

/**
 * This internal function modifies the specified GPIO pin access mask to
 * conform with the line termination type selected for the specified line.
 *
 * \param[in]       pLineCtx    Line Context for a VCP2 line
 * \param[in,out]   pMask       Access mask
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_DEDICATED_PINS   The pMask argument was modified, because
 *                                      Some pins are controlled automatically
 *                                      according to the line termination type.
 */
static VpStatusType
MaskDedicatedPins(
    VpTermType termType,
    uint8 *pMask)
{
    uint8 requestedMask = *pMask;

    switch (termType) {
        case VP_TERM_FXS_GENERIC:
            *pMask &= VP792_REG_IO_ST_IO_X_MASK;
            break;
        case VP_TERM_FXS_TI:
            *pMask &= VP792_REG_IO_ST_IO_1_MASK;
            break;
        default:
            /* Unknown termination type.  No restrictions. */
            break;
    }

    if (*pMask != requestedMask) {
        return VP_STATUS_DEDICATED_PINS;
    } else {
        return VP_STATUS_SUCCESS;
    }
}

/*******************************************************************************
 * This function implements a given device option.
 *
 * \param[in] pDevCtx Pointer to the Device Context identifying the device to
 * apply the option to.
 * \param[in] option VpOptionIdType specifying which VP-API option to set.
 * \param[in] pValue Pointer to an option data structure.  Type is determined
 * base on the 'option' parameter.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_OPTION_NOT_SUPPORTED
 * \retval ::VP_STATUS_INVALID_ARG
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionDevice(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpOptionValueType *pUserBuf = pValue;
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (option) {
        case VP_DEVICE_OPTION_ID_PARK_MODE: {
            uint16 discTime = pUserBuf->parkMode.discTime;
            uint16 standbyTime = pUserBuf->parkMode.standbyTime;

            if (
                (discTime == 0) ||
                (discTime > 240 /* measured in 500ms increments */) ||
                (standbyTime == 0) ||
                (standbyTime > 80 /* measured in 100ms increments */)
            ) {
                status = VP_STATUS_INVALID_ARG;
                break;
            }

            pDevObj->options.parkMode = pUserBuf->parkMode;
            break;
        }
        case VP_DEVICE_OPTION_ID_DEV_IO_CFG: {
            uint8 channelId;

            /* Call SetOptionLineIoCfg() once for each channel. */
            for (channelId = 0; channelId < pDevObj->maxChannels; channelId++) {
                VpOptionLineIoConfigType *pLineIoCfg =
                    &(pUserBuf->deviceIoConfig.lineIoConfig[channelId]);
                status = SetOptionLineIoCfg(pDevCtx, channelId, pLineIoCfg);
                if (status != VP_STATUS_SUCCESS) {
                    break;
                }
            }
        }
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
            pDevObj->options.criticalFlt = pUserBuf->criticalFlt;
            break;

#if (VP_CC_DEBUG_SELECT != 0)
        case VP_OPTION_ID_DEBUG_SELECT:
            pDevObj->options.debugSelect = pUserBuf->debugSelect;
            break;
#endif

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            break;
    }

    return status;
} /* SetOptionDevice() */

/*******************************************************************************
 * This function implements a given line option.
 *
 * \param[in] pLineCtx Pointer to the Line Context identifying the line to apply
 * the option to.
 * \param[in] option VpOptionIdType specifying which VP-API option to set.
 * \param[in] pValue Pointer to an option data structure.  Type is determined
 * base on the 'option' parameter.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_OPTION_NOT_SUPPORTED
 * \retval ::VP_STATUS_INVALID_ARG
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pValue)
{
#define VP792_MAX_SETOPTION_CMD_LEN MAX(VP792_CMDSIZE_WR_PULSE_PARAMS, VP792_CMDSIZE_WR_AFE_CONFIG)

    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    VpOptionValueType *pUserBuf = pValue;
    uint16 buf[VP792_MAX_SETOPTION_CMD_LEN], *pBufEnd = buf;
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("SetOptionLine(%s):", VpGetString_VpOptionIdType(option)));

    switch (option) {
        case VP_OPTION_ID_ZERO_CROSS:
            status = SetOptionLineZeroCross(pLineCtx, pUserBuf->zeroCross);
            break;

        case VP_OPTION_ID_PULSE_MODE:
            switch (pUserBuf->pulseMode) {
                case VP_OPTION_PULSE_DECODE_OFF:
                    pLineObj->options.pulseMode = VP_OPTION_PULSE_DECODE_OFF;
                    pDevObj->options.pulseEnabled &= ~(1 << channelId);
                    status = UpdateCh1Mask(pLineCtx);
                    break;
                case VP_OPTION_PULSE_DECODE_ON:
                    pLineObj->options.pulseMode = VP_OPTION_PULSE_DECODE_ON;
                    pDevObj->options.pulseEnabled |= (1 << channelId);
                    status = UpdateCh1Mask(pLineCtx);
                    break;
                default:
                    status = VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_OPTION_ID_PULSE:

            /* Send the WR_PULSE_PARAMS command */
             Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_PULSE_PARAMS, channelId);
            *pBufEnd++ = pUserBuf->linePulse.breakMin;
            *pBufEnd++ = pUserBuf->linePulse.breakMax;
            *pBufEnd++ = pUserBuf->linePulse.flashMin;
            *pBufEnd++ = pUserBuf->linePulse.flashMax;
            *pBufEnd++ = pUserBuf->linePulse.onHookMin;
            *pBufEnd++ = pUserBuf->linePulse.offHookMin;
            *pBufEnd++ = pUserBuf->linePulse.makeMin;
            *pBufEnd++ = pUserBuf->linePulse.makeMax;
            *pBufEnd++ = pUserBuf->linePulse.interDigitMin;
            status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_MAX_SETOPTION_CMD_LEN);
            break;

        case VP_OPTION_ID_TIMESLOT:
            status = SetOptionLineTimeslot(pLineCtx, &pUserBuf->timeslot);
            break;

        case VP_OPTION_ID_CODEC: {
            uint16 vpCfg1;

            /* So as not to disturb the other bits in the VP_CFG1 register,
               we need to do a read-modify-write sequence. */
            status = Vp792HbiPagedRead(pDevCtx, channelId,
                VP792_REG_VP_CFG1_OFFSET, VP792_REG_VP_CFG1_LEN, &vpCfg1);
            if (status == VP_STATUS_SUCCESS) {
                vpCfg1 &= ~VP792_REG_VP_CFG1_CODEC_MASK;

                switch (pUserBuf->codec) {
                    case VP_OPTION_ALAW:
                    case VP_OPTION_MLAW:
                    case VP_OPTION_LINEAR:
                        vpCfg1 |= pUserBuf->codec << VP792_REG_VP_CFG1_CODEC_SHIFT;
                        break;
                    case VP_OPTION_WIDEBAND:
                        status = VP_STATUS_OPTION_NOT_SUPPORTED;
                        break;
                    default:
                        status = VP_STATUS_INVALID_ARG;
                        break;
                }
            }
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_VP_CFG1_OFFSET, VP792_REG_VP_CFG1_LEN, &vpCfg1);
            }
            break;
        }
        case VP_OPTION_ID_PCM_HWY:
            status = SetOptionLinePcmHwy(pLineCtx, pUserBuf->pcmHwy);
            break;

        case VP_OPTION_ID_LOOPBACK: {
            uint16 vpCfg2;
            uint16 newAfeConfig = pLineObj->options.afeConfig;

            /* So as not to disturb the other bits in the VP_CFG2
               register, we need to do a read-modify-write sequence. */
            status = Vp792HbiPagedRead(pDevCtx, channelId,
                VP792_REG_VP_CFG2_OFFSET, VP792_REG_VP_CFG2_LEN, &vpCfg2);
            switch (pUserBuf->loopback) {
                case VP_OPTION_LB_TIMESLOT:
                    vpCfg2 |= VP792_REG_VP_CFG2_TSLB_MASK;
                    newAfeConfig &= ~VP792_MB_AFE_CONFIG_FDLB_MASK;
                    break;
                case VP_OPTION_LB_DIGITAL:
                    vpCfg2 &= ~VP792_REG_VP_CFG2_TSLB_MASK;
                    newAfeConfig |= VP792_MB_AFE_CONFIG_FDLB_MASK;
                    break;
                case VP_OPTION_LB_OFF:
                    vpCfg2 &= ~VP792_REG_VP_CFG2_TSLB_MASK;
                    newAfeConfig &= ~VP792_MB_AFE_CONFIG_FDLB_MASK;
                    break;
                default:
                    return VP_STATUS_INVALID_ARG;
            }
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_VP_CFG2_OFFSET, VP792_REG_VP_CFG2_LEN, &vpCfg2);
            }

            /* We cache the AFE Config value in the line object, because of the
               inconvenience of doing a read-modify-write of a mailbox register. */
            if (newAfeConfig != pLineObj->options.afeConfig) {
                Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_AFE_CONFIG, channelId);
                *pBufEnd++ = newAfeConfig;
                status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_MAX_SETOPTION_CMD_LEN);
            }
            if (status == VP_STATUS_SUCCESS) {
                pLineObj->options.afeConfig = newAfeConfig;
            }
            break;
        }
        case VP_OPTION_ID_LINE_STATE: {
            uint16 dcFeed;

            /* So as not to disturb the other bits in the DC_FEED
               register, we need to do a read-modify-write sequence. */
            status = Vp792HbiPagedRead(pDevCtx, channelId,
                VP792_REG_DC_FEED_OFFSET, VP792_REG_DC_FEED_LEN, &dcFeed);
            dcFeed &= ~(VP792_REG_DC_FEED_POLRR_MASK | VP792_REG_DC_FEED_BSET_MASK);
            if (pUserBuf->lineState.battRev) {
                dcFeed |= VP792_REG_DC_FEED_POLRR_ABRUPT;
            }
            switch(pUserBuf->lineState.bat) {
                case VP_OPTION_BAT_AUTO:
                    dcFeed |= VP792_REG_DC_FEED_BSET_AUTO;
                    break;
                case VP_OPTION_BAT_HIGH:
                    dcFeed |= VP792_REG_DC_FEED_BSET_HBAT;
                    break;
                case VP_OPTION_BAT_LOW:
                    dcFeed |= VP792_REG_DC_FEED_BSET_LBAT;
                    break;
                case VP_OPTION_BAT_BOOST:
                    dcFeed |= VP792_REG_DC_FEED_BSET_BOOST;
                    break;
                default:
                    status = VP_STATUS_INVALID_ARG;
            }
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_DC_FEED_OFFSET, VP792_REG_DC_FEED_LEN, &dcFeed);
            }
            break;
        }
        case VP_OPTION_ID_EVENT_MASK:
            status = SetOptionLineEventMask(pLineCtx, &pUserBuf->eventMask);
            break;

        case VP_OPTION_ID_RING_CNTRL: {
            uint16 oldvalue, newValue;

            /* Update the zero cross option: */
            if (status == VP_STATUS_SUCCESS) {
                status = SetOptionLineZeroCross(pLineCtx, pUserBuf->ringControl.zeroCross);
            }

            /* So as not to disturb the other bits in the RING
               register, we need to do a read-modify-write sequence. */
            status = Vp792HbiPagedRead(pDevCtx, channelId,
                VP792_REG_RING_OFFSET + VP792_REG_RING_RTRIP_ST_WORD, 1,
                &oldvalue);
            if (status == VP_STATUS_SUCCESS) {
                uint16 newState;
                status = Vp792MapLineState(
                    ((VpOptionRingControlType *)pValue)->ringTripExitSt,
                    &newState);
                newValue = (oldvalue & ~VP792_REG_RING_RTRIP_ST_MASK) | newState;
            }
            if ((newValue != oldvalue) && (status == VP_STATUS_SUCCESS)) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_RING_OFFSET + VP792_REG_RING_RTRIP_ST_WORD, 1,
                    &newValue);
            }

            /* Ring exit debounce duration: */
            if (status == VP_STATUS_SUCCESS) {
                uint16 bosh = pUserBuf->ringControl.ringExitDbncDur / 4;
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_LOOP_SUP_OFFSET + VP792_REG_LOOP_SUP_BOSH_WORD,
                    1, &bosh);
            }

            if (status == VP_STATUS_SUCCESS) {
                pLineObj->options.ringControl.ringTripExitSt = pUserBuf->ringControl.ringTripExitSt;
            }
            break;
        }
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
            pLineObj->options.pcmTxRxCntrl = pUserBuf->pcmTxRxCntrl;
            status = Vp792SetPcmTxRxMode(pLineCtx, pLineObj->currentState);
            break;

        case VP_OPTION_ID_DTMF_SPEC: {
            uint16 dtmfReg;

            /* So as not to disturb the other bits in the LOOP_SUP
               register, we need to do a read-modify-write sequence. */
            status = Vp792HbiPagedRead(pDevCtx, channelId,
                VP792_REG_LOOP_SUP_OFFSET, 1, &dtmfReg);
            dtmfReg &= ~VP792_REG_LOOP_SUP_DTMF_REG_MASK;
            switch (pUserBuf->dtmfSpec) {
                case VP_OPTION_DTMF_SPEC_ATT:
                    dtmfReg |= VP792_REG_LOOP_SUP_DTMF_REG_ATT;
                    break;
                case VP_OPTION_DTMF_SPEC_NTT:
                    dtmfReg |= VP792_REG_LOOP_SUP_DTMF_REG_NTT;
                    break;
                case VP_OPTION_DTMF_SPEC_AUS:
                    dtmfReg |= VP792_REG_LOOP_SUP_DTMF_REG_AUS;
                    break;
                case VP_OPTION_DTMF_SPEC_BRZL:
                    dtmfReg |= VP792_REG_LOOP_SUP_DTMF_REG_BRZL;
                    break;
                case VP_OPTION_DTMF_SPEC_ETSI:
                    dtmfReg |= VP792_REG_LOOP_SUP_DTMF_REG_ETSI;
                    break;
                default:
                    status = VP_STATUS_INVALID_ARG;
                    break;
            }
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId,
                    VP792_REG_LOOP_SUP_OFFSET, 1, &dtmfReg);
            }
            break;
        }
        case VP_OPTION_ID_DTMF_MODE:
            status = Vp792SetOptionLineDtmfMode(pLineCtx, &pUserBuf->dtmfMode);
            break;

        case VP_OPTION_ID_LINE_IO_CFG:
            status = SetOptionLineIoCfg(pDevCtx, channelId, &pUserBuf->lineIoConfig);
            break;

        case VP_OPTION_ID_DCFEED_SLOPE:
            status = Vp792HbiPagedWrite(pDevCtx, channelId,
                VP792_REG_DC_FEED_SLOPE_OFFSET, VP792_REG_DC_FEED_SLOPE_LEN,
                &pUserBuf->dcFeedSlope);
            break;

#if (VP_CC_DEBUG_SELECT != 0)
        case VP_OPTION_ID_DEBUG_SELECT:
            pLineObj->options.debugSelect = *(uint32 *)(pValue);
            break;
#endif

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            break;
    }

    VP_ASSERT((pBufEnd - buf) <= VP792_MAX_SETOPTION_CMD_LEN);

    VP_API_INT_EXIT(VpLineCtxType, pLineCtx, "SetOptionLine", status);
    return status;
} /* SetOptionLine() */

/*******************************************************************************
 * SetOptionLineEventMask()
 * This function implements the EVENT_MASK line option.  The API event masks are
 * set based on the user input, known unmaskable events, and known unsupported
 * events.
 *
 * \param[in] pLineCtx Pointer to the Line Context identifying the line to apply
 * the option to.
 * \param[in] pNewEventMask Point to data structure containing user-input event
 * masks.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionLineEventMask(
    VpLineCtxType *pLineCtx,
    VpOptionEventMaskType *pNewEventMask)
{

#define VP792_INVALID_FAULT_EVENTS       (VP_LINE_EVID_RES_LEAK_FLT | VP_EVCAT_FAULT_UNDEFINED)
#define VP792_INVALID_SIGNALING_EVENTS   (VP_LINE_EVID_US_TONE_DETECT | VP_LINE_EVID_DS_TONE_DETECT | VP_DEV_EVID_SEQUENCER | VP_EVCAT_SIGNALING_UNDEFINED)
#define VP792_INVALID_RESPONSE_EVENTS    (VP_DEV_EVID_BOOT_CMP | VP_LINE_EVID_SLAC_INIT_CMP | VP_LINE_EVID_QUERY_CMP | VP_EVCAT_RESPONSE_UNDEFINED)
#define VP792_INVALID_TEST_EVENTS        ~(VP_LINE_EVID_TEST_CMP | VP_LINE_EVID_DTONE_DET | VP_LINE_EVID_DTONE_LOSS | VP_DEV_EVID_CHKSUM)
#define VP792_INVALID_PROCESS_EVENTS     (VP_EVCAT_PROCESS_UNDEFINED)
#define VP792_INVALID_FXO_EVENTS         VP_EVENT_MASK_ALL
#define VP792_INVALID_PACKET_EVENTS      VP_EVENT_MASK_ALL

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    /* We don't want to disturb the user's struct, so we make a local copy
       that we can modify. */
    VpOptionEventMaskType tempMask = *pNewEventMask;

    /* Mask events that are never generated for VP792 devices: */
    tempMask.faults    |= VP792_INVALID_FAULT_EVENTS;
    tempMask.signaling |= VP792_INVALID_SIGNALING_EVENTS;
    tempMask.response  |= VP792_INVALID_RESPONSE_EVENTS;
    tempMask.test      |= VP792_INVALID_TEST_EVENTS;
    tempMask.process   |= VP792_INVALID_PROCESS_EVENTS;
    tempMask.fxo       |= VP792_INVALID_PROCESS_EVENTS;
    tempMask.packet    |= VP792_INVALID_PROCESS_EVENTS;

    /* Zero out the line-specific bits while setting the device event masks. */
    pDevObj->options.eventMask.faults    = tempMask.faults    & VP_EVCAT_FAULT_DEV_EVENTS;
    pDevObj->options.eventMask.signaling = tempMask.signaling & VP_EVCAT_SIGNALING_DEV_EVENTS;
    pDevObj->options.eventMask.response  = tempMask.response  & (VP_EVCAT_RESPONSE_DEV_EVENTS | VP_EVID_CAL_BUSY | VP_EVID_CAL_CMP);
    pDevObj->options.eventMask.test      = tempMask.test      & VP_EVCAT_TEST_DEV_EVENTS;
    pDevObj->options.eventMask.process   = tempMask.process   & VP_EVCAT_PROCESS_DEV_EVENTS;

    /* Zero out the device-specific bits while setting the line event masks. */
    pLineObj->options.eventMask.faults    = tempMask.faults    & ~VP_EVCAT_FAULT_DEV_EVENTS;
    pLineObj->options.eventMask.signaling = tempMask.signaling & ~VP_EVCAT_SIGNALING_DEV_EVENTS;
    pLineObj->options.eventMask.response  = tempMask.response  & ~VP_EVCAT_RESPONSE_DEV_EVENTS;
    pLineObj->options.eventMask.test      = tempMask.test      & ~VP_EVCAT_TEST_DEV_EVENTS;
    pLineObj->options.eventMask.process   = tempMask.process   & ~VP_EVCAT_PROCESS_DEV_EVENTS;

    /* Unmask the unmaskable defined in vp_api_event.h */
    VpImplementNonMaskEvents(&pLineObj->options.eventMask, &pDevObj->options.eventMask);

    /* The GAIN_CMP event is maskable for this device, but not other devices. */
    pLineObj->options.eventMask.response |= (tempMask.response & VP_LINE_EVID_GAIN_CMP);

    /* Set the GMASK and CH_MASKx registers in the SLAC here.  This is mostly
       an efficiency optimization, since event masking is also applied for
       each event at the end of GetEventInternal().

       Some SLAC events need to be masked/unmasked based on whether pulse digit
       decoding is enabled.  This is done in UpdateCh1Mask(), called below. */
    {
        uint16 mask;

        /* Set SYS_MASK register. */
        mask = VP792_REG_SYS_MASK_UNMASK_ALL;
        status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_SYS_MASK_OFFSET,
            VP792_REG_SYS_MASK_LEN, &mask);

        /* Set GMASK register. */
        mask = VP792_REG_GMASK_UNMASK_ALL;
        if (pDevObj->options.eventMask.response & VP_DEV_EVID_DNSTR_MBOX) {
            mask |= VP792_REG_GMASK_CMD_M_MASK;
        }
        if (status == VP_STATUS_SUCCESS) {
            status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_GMASK_OFFSET,
                VP792_REG_GMASK_LEN, &mask);
        }

        /* Set the CH_MASK1 register. */
        if (status == VP_STATUS_SUCCESS) {
            status = UpdateCh1Mask(pLineCtx);
        }

        /* Set the CH_MASK2 register. */
        mask = VP792_REG_CH_MASKS2_UNMASK_ALL;
        if (status == VP_STATUS_SUCCESS) {
            status = Vp792HbiPagedWrite(pDevCtx, channelId,
                VP792_REG_CH_MASKS2_OFFSET, VP792_REG_CH_MASKS2_LEN, &mask);
        }
    }

    return status;
} /* SetOptionLineEventMask() */

/*******************************************************************************
 * SetOptionLineIoCfg()
 * This function implements the LINE_IO_CFG option.  Only bits 0 and 1 from
 * the IO direction parameter are used, corresponding to IO_0 and IO_1
 * respectively.  Only bit 0 from the outputType parameter is used,
 * corresponding to IO_0.
 *
 * \param[in] pDevCtx Pointer to the Device Context identifying the device to
 * apply the option to.
 * \param[in] pDeviceIo Pointer to the VpOptionLineIoConfigType data structure.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionLineIoCfg(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpOptionLineIoConfigType *pLineIoCfg)
{
    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
    Vp792LineObjectType *pLineObj = (pLineCtx != VP_NULL) ? pLineCtx->pLineObj : VP_NULL;
    VpTermType termType = (pLineObj != VP_NULL) ? pLineObj->termType : VP_TERM_FXS_GENERIC;
    uint8 freePins = VP792_REG_IO_ST_IO_X_MASK;
    uint16 ioStReg, userValue, protectedBits = 0xFFFF;
    VpStatusType status;

    /* Shift the user-supplied bitmasks into position for the register. */
    userValue = (uint16)pLineIoCfg->direction << VP792_REG_IO_ST_IO_X_CFG_SHIFT
        | VP792_REG_IO_ST_IO_X_CFG_MASK;
    userValue |= (uint16)pLineIoCfg->outputType << VP792_REG_IO_ST_IO_0_MODE_SHIFT
        | VP792_REG_IO_ST_IO_0_MODE_MASK;

    /* So as not to disturb the other bits in the IO_ST register, we do a
       read-modify-write sequence. */
    status = Vp792HbiPagedRead(pDevCtx, channelId,
        VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &ioStReg);

    /* Don't disturb the configuration for bits that are dedicated in the
       termination type. */
    MaskDedicatedPins(termType, &freePins);
    if (freePins & VP792_REG_IO_ST_IO_0_MASK) {
        protectedBits &= ~(VP792_REG_IO_ST_IO_0_CFG_MASK | VP792_REG_IO_ST_IO_0_MODE_MASK);
    }
    if (freePins & VP792_REG_IO_ST_IO_1_MASK) {
        protectedBits &= ~VP792_REG_IO_ST_IO_1_CFG_MASK;
    }

    /* Or in the new bits. */
    ioStReg = (ioStReg & protectedBits) | (userValue & ~protectedBits);

    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId,
            VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &ioStReg);
    }

    return status;
} /* SetOptionLineIoCfg() */

/*******************************************************************************
 * SetOptionLinePcmHwy()
 * This function implements the PCM_HWY option.
 *
 * \param[in] pLineCtx Pointer to the Line Context identifying the line to apply
 * the option to.
 * \param[in] pcmHwy PCM Highway option data.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_INVALID_ARG Invalid pcmHwy
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionLinePcmHwy(
    VpLineCtxType *pLineCtx,
    VpOptionPcmHwyType pcmHwy)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint16 txCfg, rxCfg;

    /* So as not to disturb the other bits in the TXTS_CFG and RXTS_CFG
       registers, we do a read-modify-write sequence for each. */
    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
        VP792_REG_TXTS_CFG_LEN, &txCfg);
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
            VP792_REG_RXTS_CFG_LEN, &rxCfg);
    }

    txCfg &= ~VP792_REG_TXTS_CFG_THWY_MASK;
    rxCfg &= ~VP792_REG_RXTS_CFG_RHWY_MASK;

    switch (pcmHwy) {
        case VP_OPTION_HWY_A:
            txCfg |= VP792_REG_TXTS_CFG_THWY_A;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_A;
            break;
        case VP_OPTION_HWY_B:
            txCfg |= VP792_REG_TXTS_CFG_THWY_B;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_B;
            break;
        case VP_OPTION_HWY_TX_A_RX_B:
            txCfg |= VP792_REG_TXTS_CFG_THWY_A;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_B;
            break;
        case VP_OPTION_HWY_TX_B_RX_A:
            txCfg |= VP792_REG_TXTS_CFG_THWY_B;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_A;
            break;
        case VP_OPTION_HWY_TX_AB_RX_A:
            txCfg |= VP792_REG_TXTS_CFG_THWY_AB;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_A;
            break;
        case VP_OPTION_HWY_TX_AB_RX_B:
            txCfg |= VP792_REG_TXTS_CFG_THWY_AB;
            rxCfg |= VP792_REG_RXTS_CFG_RHWY_B;
            break;
        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
            VP792_REG_TXTS_CFG_LEN, &txCfg);
    }
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
            VP792_REG_RXTS_CFG_LEN, &rxCfg);
    }

    return status;
} /* SetOptionLinePcmHwy() */

/*******************************************************************************
 * SetOptionLineTimeslot()
 * This function implements the TIMESLOT option.  Timeslot values must be lower
 * than (kHz)PCM clock rate/64.  Ex: If the clock rate is 8192, timeslots must
 * be 127 or below.
 *
 * \param[in] pLineCtx Pointer to the Line Context identifying the line to apply
 * the option to.
 * \param[in] timeSlots Pointer to structure containing desired timeslot values.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_INVALID_ARG Timeslot(s) out of range.
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
static VpStatusType
SetOptionLineTimeslot(
    VpLineCtxType *pLineCtx,
    VpOptionTimeslotType *timeSlots)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint16 tsReg;

    if (timeSlots->tx >= pDevObj->pcmClockRate / 64) {
        return VP_STATUS_INVALID_ARG;
    }
    if (timeSlots->rx >= pDevObj->pcmClockRate / 64) {
        return VP_STATUS_INVALID_ARG;
    }

    /* So as not to disturb the other bits in the TXTS_CFG and RXTS_CFG
       registers, we do a read-modify-write sequence for each. */
    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
        VP792_REG_TXTS_CFG_LEN, &tsReg);
    tsReg &= ~VP792_REG_TXTS_CFG_TTS_MASK;
    tsReg |= timeSlots->tx;
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_TXTS_CFG_OFFSET,
            VP792_REG_TXTS_CFG_LEN, &tsReg);
    }

    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
            VP792_REG_RXTS_CFG_LEN, &tsReg);
    }
    tsReg &= ~VP792_REG_RXTS_CFG_RTS_MASK;
    tsReg |= timeSlots->rx;
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_RXTS_CFG_OFFSET,
            VP792_REG_RXTS_CFG_LEN, &tsReg);
    }

    return status;
} /* SetOptionLineTimeslot() */

/*******************************************************************************
 * SetOptionLineZeroCross()
 * This function implements the ZERO_CROSS option and the zero cross control
 * portion of the RING_CNTRL option.
 *
 * \param[in] pLineCtx Pointer to the Line Context identifying the line to apply
 * the option to.
 * \param[in] zeroCrossMode Desired zero cross setting.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_OPTION_NOT_SUPPORTED VP_OPTION_ZC_NONE is not supported
 * for 792.
 * \retval ::VP_STATUS_INVALID_ARG Invalid zeroCrossMode
 ******************************************************************************/
static VpStatusType
SetOptionLineZeroCross(
    VpLineCtxType *pLineCtx,
    VpOptionZeroCrossType zeroCrossMode)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    /*
     * Zero cross control is effectively always on for 792.  M4B/B4M are
     * both valid, but meaningless.  We store the input in a variable so
     * that it can be read back later, but there is no change.
     */
    if (zeroCrossMode == VP_OPTION_ZC_M4B || zeroCrossMode == VP_OPTION_ZC_B4M) {
        pLineObj->options.ringControl.zeroCross = zeroCrossMode;
    } else if (zeroCrossMode == VP_OPTION_ZC_NONE) {
        status = VP_STATUS_OPTION_NOT_SUPPORTED;
    } else {
        status = VP_STATUS_INVALID_ARG;
    }

    return status;
} /* SetOptionLineZeroCross() */

static VpStatusType
UpdateCh1Mask(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 mask = 0;
    VpStatusType status = VP_STATUS_SUCCESS;

#if 0
    /* Read the CH_MASK1 register. */
    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_CH_MASKS_OFFSET,
        VP792_REG_CH_MASKS_LEN, &mask);
#endif

    /* Mask either the HOOK or RAW_HOOK event, depending on whether pulse
       decoding is enabled. */
    if (pLineObj->options.pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
        mask &= ~VP792_REG_CH_MASKS_HKR_M_MASK;
        mask |= VP792_REG_CH_MASKS_HK_M_MASK | VP792_REG_CH_MASKS_PDG_M_MASK |
            VP792_REG_CH_MASKS_FLASH_M_MASK | VP792_REG_CH_MASKS_PREQ_HK_M_MASK;
    } else {
        mask &= ~(VP792_REG_CH_MASKS_HK_M_MASK | VP792_REG_CH_MASKS_PDG_M_MASK |
            VP792_REG_CH_MASKS_FLASH_M_MASK | VP792_REG_CH_MASKS_PREQ_HK_M_MASK);
        mask |= VP792_REG_CH_MASKS_HKR_M_MASK;
    }

    /* Write the CH_MASK1 register. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiPagedWrite(pDevCtx, channelId,
            VP792_REG_CH_MASKS_OFFSET, VP792_REG_CH_MASKS_LEN, &mask);
    }

    return status;
}

#endif /* VP_CC_792_SERIES */
