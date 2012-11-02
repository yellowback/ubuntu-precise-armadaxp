/** \file apiseq.c
 * apiseq.c
 *
 *  This file contains the VP880 functions called by the API-II Caller ID or
 * sequencer. It is seperated from "normal" API functions for users that want
 * to remove this section of code from the API-II.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 6419 $
 * $LastChangedDate: 2010-02-12 16:40:10 -0600 (Fri, 12 Feb 2010) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES)
#ifdef CSLAC_SEQ_EN

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "sys_service.h"

/**< Function called by Send Signal only. Implements message waiting pulse. */
static VpStatusType
Vp880SendMsgWaitPulse(
    VpLineCtxType *pLineCtx,
    VpSendMsgWaitType *pMsgWait);

/**< Function called by Send Signal only. Implements Forward Disconnect and
 * Tip Open pulse.
 */
static VpStatusType
Vp880SendPulse(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    uint16 timeInMs);

/**< Function called by Send Signal only. Implements Polarity Reversal Pulse */
VpStatusType
Vp880SendPolRevPulse(
    VpLineCtxType *pLineCtx,
    uint16 timeIn1Ms);

/**< Function called by Send Signal only. Implements FXO digit generation */
static VpStatusType
Vp880SendDigit(
    VpLineCtxType *pLineCtx,
    VpDigitGenerationType digitType,
    VpDigitType digit);

/**< Function called by Send Signal only. Implements FXO Momentary Loop Open */
static VpStatusType
Vp880MomentaryLoopOpen(
    VpLineCtxType *pLineCtx);

/**
 * Vp880CommandInstruction()
 *  This function implements the Sequencer Command instruction for the Vp880
 * device type.
 *
 * Preconditions:
 *  The line must first be initialized and the sequencer data must be valid.
 *
 * Postconditions:
 *  The command instruction currently being pointed to by the sequencer
 * instruction passed is acted upon.  The sequencer may or may not be advanced,
 * depending on the specific command instruction being executed.
 */
VpStatusType
Vp880CommandInstruction(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pSeqData)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;
    uint8 channelId = pLineObj->channelId;
    uint8 sigGenCtrl, lineState;
    uint8 lsConfig[VP880_LOOP_SUP_LEN];
    uint16 tempFreq, tempLevel;

    /*
     * We know the current value "pSeqData[0]" is 0, now we need to determine if
     * the next command is generator control operator followed by time, or a
     * Line state command -- No other options supported
     */
    switch (pSeqData[0] & VP_SEQ_SUBTYPE_MASK) {
        case VP_SEQ_SUBCMD_SIGGEN:
            VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Generator Control 0x%02X 0x%02X",
                pSeqData[0], pSeqData[1]));

            sigGenCtrl = VP880_GEN_ALLOFF;

            /* Get the signal generator bits and set. */
            sigGenCtrl |= ((pSeqData[1] & 0x01) ?  VP880_GENA_EN : 0);
            sigGenCtrl |= ((pSeqData[1] & 0x02) ?  VP880_GENB_EN : 0);
            sigGenCtrl |= ((pSeqData[1] & 0x04) ?  VP880_GENC_EN : 0);
            sigGenCtrl |= ((pSeqData[1] & 0x08) ?  VP880_GEND_EN : 0);

            VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT,
                VP880_GEN_CTRL_LEN, &sigGenCtrl);
            break;

        case VP_SEQ_SUBCMD_LINE_STATE:
            VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Line State Control 0x%02X 0x%02X",
                pSeqData[0], pSeqData[1]));

            switch(pSeqData[1]) {
                case VP_PROFILE_CADENCE_STATE_STANDBY:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_STANDBY);
                    break;

                case VP_PROFILE_CADENCE_STATE_POLREV_STANDBY:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_STANDBY_POLREV);
                    break;

                case VP_PROFILE_CADENCE_STATE_TIP_OPEN:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_TIP_OPEN);
                    break;

                case VP_PROFILE_CADENCE_STATE_TALK:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_TALK);
                    break;

                case VP_PROFILE_CADENCE_STATE_ACTIVE:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_ACTIVE);
                    break;

                case VP_PROFILE_CADENCE_STATE_OHT:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_OHT);
                    break;

                case VP_PROFILE_CADENCE_STATE_DISCONNECT:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_DISCONNECT);
                    break;

                case VP_PROFILE_CADENCE_STATE_RINGING:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_RINGING);
                    break;

                case VP_PROFILE_CADENCE_STATE_POLREV_RINGING:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_RINGING_POLREV);
                    break;

                case VP_PROFILE_CADENCE_STATE_POLREV_ACTIVE:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_ACTIVE_POLREV);
                    break;

                case VP_PROFILE_CADENCE_STATE_POLREV_TALK:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_TALK_POLREV);
                    break;

                case VP_PROFILE_CADENCE_STATE_POLREV_OHT:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_OHT_POLREV);
                    break;

                case VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_FXO_LOOP_OPEN);
                    break;

                case VP_PROFILE_CADENCE_STATE_FXO_OHT:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_FXO_OHT);
                    break;

                case VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_FXO_LOOP_CLOSE);
                    break;

                case VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK:
                    Vp880SetLineStateInt(pLineCtx, VP_LINE_FXO_TALK);
                    break;

                case VP_PROFILE_CADENCE_STATE_MSG_WAIT_NORM:
                case VP_PROFILE_CADENCE_STATE_MSG_WAIT_POLREV:
                    VpMemCpy(lsConfig, pLineObj->loopSup, VP880_LOOP_SUP_LEN);
                    if (lsConfig[VP880_LOOP_SUP_RT_MODE_BYTE]
                        & VP880_RING_TRIP_AC) {
                        if (!(pLineObj->status & VP880_BAD_LOOP_SUP)) {
                            pLineObj->status |= VP880_BAD_LOOP_SUP;
                        }

                        /* Force DC Trip */
                        lsConfig[VP880_LOOP_SUP_RT_MODE_BYTE] &=
                            ~VP880_RING_TRIP_AC;
                        VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT,
                            VP880_LOOP_SUP_LEN, lsConfig);
                    }

                    lineState =
                        (pSeqData[1] == VP_PROFILE_CADENCE_STATE_MSG_WAIT_NORM) ?
                        VP880_SS_BALANCED_RINGING :
                        VP880_SS_BALANCED_RINGING_PR;

                    Vp880LLSetSysState(deviceId, pLineCtx, lineState, TRUE);
                    break;

                default:
                    return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_SEQ_SUBCMD_START_CID:
        case VP_SEQ_SUBCMD_WAIT_ON:
            if (pLineObj->pCidProfileType1 != VP_PTABLE_NULL) {
                pLineObj->callerId.pCliProfile = pLineObj->pCidProfileType1;

                pLineObj->callerId.status |= VP_CID_IN_PROGRESS;
                if ((pSeqData[0] & VP_SEQ_SUBTYPE_MASK) == VP_SEQ_SUBCMD_WAIT_ON) {
                    VP_CID(VpLineCtxType, pLineCtx, ("880 Command WAIT_ON_CID"));
                    pLineObj->callerId.status |= VP_CID_WAIT_ON_ACTIVE;
                } else {
                    VP_CID(VpLineCtxType, pLineCtx, ("880 Command START_CID"));
                }
                pLineObj->callerId.cliTimer = 1;

                pLineObj->callerId.cliIndex = 0;
                pLineObj->callerId.cliMPIndex = 0;
                pLineObj->callerId.cliMSIndex = 0;

                pLineObj->callerId.status &= ~VP_CID_SIG_B_VALID;

                pLineObj->callerId.status |= VP_CID_PRIMARY_IN_USE;
                pLineObj->callerId.status &= ~VP_CID_SECONDARY_IN_USE;
            }
            break;

        case VP_SEQ_SUBCMD_RAMP_GENERATORS:
            VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Ramp Generators Control 0x%02X 0x%02X",
                pSeqData[0], pSeqData[1]));

            tempFreq = (pLineObj->cadence.regData[3] << 8);
            tempFreq |= pLineObj->cadence.regData[4];

            tempLevel = (pLineObj->cadence.regData[5] << 8);
            tempLevel |= pLineObj->cadence.regData[6];

            if (pLineObj->cadence.isFreqIncrease == TRUE) {
                /* Check if we're at or above the max frequency */
                if (tempFreq >= pLineObj->cadence.stopFreq) {
                    pLineObj->cadence.isFreqIncrease = FALSE;
                    tempFreq -= pLineObj->cadence.freqStep;
                } else {
                    tempFreq += pLineObj->cadence.freqStep;
                }
            } else {
                if (tempFreq <
                    (pLineObj->cadence.startFreq - pLineObj->cadence.freqStep)) {
                    pLineObj->cadence.isFreqIncrease = TRUE;
                    tempFreq += pLineObj->cadence.freqStep;
                } else {
                    tempFreq -= pLineObj->cadence.freqStep;
                }
            }
            pLineObj->cadence.regData[3] = (tempFreq >> 8) & 0xFF;
            pLineObj->cadence.regData[4] = tempFreq & 0xFF;

            /*
             * Check if we're at or above the max level, but make sure we don't
             * wrap around
             */
            if (tempLevel <
                (pLineObj->cadence.stopLevel -
                    ((tempLevel * pLineObj->cadence.levelStep) / 10000))) {

                tempLevel += ((tempLevel * pLineObj->cadence.levelStep) / 10000);

                pLineObj->cadence.regData[5] = (tempLevel >> 8) & 0xFF;
                pLineObj->cadence.regData[6] = tempLevel & 0xFF;
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SIGAB_PARAMS_WRT,
                VP880_SIGAB_PARAMS_LEN, pLineObj->cadence.regData);
            break;

        case VP_SEQ_SUBCMD_METERING:
            VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Metering Control 0x%02X 0x%02X",
                pSeqData[0], pSeqData[1]));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_SLIC_STATE_RD,
                VP880_SLIC_STATE_LEN, &lineState);

            if (pSeqData[1]) {  /* Metering On */
                lineState |= VP880_SS_METERING_MASK;
                pLineObj->cadence.meteringBurst++;
            } else {    /* Metering Off */
                lineState &= ~VP880_SS_METERING_MASK;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP880_SLIC_STATE_WRT,
                VP880_SLIC_STATE_LEN, &lineState);
            break;

        default:
            return VP_STATUS_INVALID_ARG;
    }

    /*
     * Check to see if there is more sequence data, and if so, move the
     * sequence pointer to the next command. Otherwise, end this cadence
     */
    pLineObj->cadence.index+=2;
    if (pLineObj->cadence.index <
       (pLineObj->cadence.length + VP_PROFILE_LENGTH + 1)) {
        pSeqData+=2;
        pLineObj->cadence.pCurrentPos = pSeqData;
    } else {
        VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Ending Cadence Length %d at Index %d",
            pLineObj->cadence.length, pLineObj->cadence.index));
        switch(pLineObj->cadence.pActiveCadence[VP_PROFILE_TYPE_LSB]) {
            case VP_PRFWZ_PROFILE_METERING_GEN:
                pLineObj->lineEvents.process |= VP_LINE_EVID_MTR_CMP;
                break;

            case VP_PRFWZ_PROFILE_RINGCAD:
                pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
                pLineObj->processData = VP_RING_CAD_DONE;
                break;

            case VP_PRFWZ_PROFILE_TONECAD:
                pLineObj->lineEvents.process |= VP_LINE_EVID_TONE_CAD;
                break;

            case VP_PRFWZ_PROFILE_HOOK_FLASH_DIG_GEN:
                pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
                pLineObj->processData = VP_SENDSIG_HOOK_FLASH;
                break;

            case VP_PRFWZ_PROFILE_DIAL_PULSE_DIG_GEN:
                pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
                pLineObj->processData = VP_SENDSIG_PULSE_DIGIT;
                break;

            case VP_PRFWZ_PROFILE_MOMENTARY_LOOP_OPEN_INT:
                pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
                pLineObj->processData = VP_SENDSIG_MOMENTARY_LOOP_OPEN;
                if (pDevObj->intReg[channelId] & VP880_LIU1_MASK) {
                    pLineObj->lineEventHandle = 1;
                } else {
                    pLineObj->lineEventHandle = 0;
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT,
                    VP880_LOOP_SUP_LEN, pLineObj->loopSup);
                break;

            case VP_PRFWZ_PROFILE_DTMF_DIG_GEN:
                pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
                pLineObj->processData = VP_SENDSIG_DTMF_DIGIT;
                Vp880MuteChannel(pLineCtx, FALSE);
                break;

            case VP_PRFWZ_PROFILE_MSG_WAIT_PULSE_INT:
                pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
                pLineObj->processData = VP_SENDSIG_MSG_WAIT_PULSE;
                VpSetLineState(pLineCtx, pLineObj->lineState.usrCurrent);
                break;

            default:
                break;

        }
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
        pLineObj->cadence.pActiveCadence = VP_PTABLE_NULL;
    }

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880InitRing()
 *  This function is used to initialize the ringing profile and caller ID
 * cadence on a given line.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  The line pointed to by the line context passed is initialized with the
 * ringing and caller ID profile specified.  The profiles may be specified as
 * either an index into the devic profile table or by profile pointers. This
 * function returns the success code if the device has been initialized and both
 * indexes (if indexes are passed) are within the range of the device profile
 * table.
 */
VpStatusType
Vp880InitRing(
    VpLineCtxType *pLineCtx,        /**< Line Context to modify Ringing
                                     * Parameters for
                                     */
    VpProfilePtrType pCadProfile,   /**< Pointer of a Ringing Cadence profile,
                                     * or the index into the Ringing Cadence
                                     * profile table.
                                     */
    VpProfilePtrType pCidProfile)   /**< Pointer of a Caller ID profile, or the
                                     * index into the Caller ID profile table.
                                     */
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpProfilePtrType *pProfileTable;

    int cadIndex = VpGetProfileIndex(pCadProfile);
    int cidIndex = VpGetProfileIndex(pCidProfile);

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->status.state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * If the profile passed is an index, make sure it's in the valid range
     * and if so, set the currently used profile to it.
     */
    if ((cadIndex >= 0) && (cadIndex < VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE)) {
        /* Valid Cadence index.  Set it if it's not an invalid table entry */
        if (!(pDevObj->profEntry.ringCadProfEntry & (0x01 << cadIndex))) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_PROFILE;
        }

        pProfileTable = pDevObj->devProfileTable.pRingingCadProfileTable;
        pLineObj->pRingingCadence = pProfileTable[cadIndex];
    } else if (cadIndex >= VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE) {
        /* It's an index, but it's out of range */
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    } else {
        /* This is a pointer. Set it if it's the correct type */
        if(VpVerifyProfileType(VP_PROFILE_RINGCAD, pCadProfile) == TRUE) {
            pLineObj->pRingingCadence = pCadProfile;
        } else {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_PROFILE;
        }
    }

    /* Setup Caller ID profile like Ringing Cadence. */
    if ((cidIndex >= 0) && (cidIndex < VP_CSLAC_CALLERID_PROF_TABLE_SIZE)) {
        /* Valid Caller ID index.  Set it if it's not an invalid table entry */
        if (!(pDevObj->profEntry.cidCadProfEntry & (0x01 << cidIndex))) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_ERR_PROFILE;
        }

        pProfileTable = pDevObj->devProfileTable.pCallerIdProfileTable;
        pLineObj->pCidProfileType1 = pProfileTable[cidIndex];
    } else {
        if (cidIndex > VP_CSLAC_CALLERID_PROF_TABLE_SIZE) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_INVALID_ARG;
        } else {
            /* This is a pointer. Set it if it's the correct type */
            if(VpVerifyProfileType(VP_PROFILE_CID, pCidProfile) == TRUE) {
                pLineObj->pCidProfileType1 = pCidProfile;
            } else {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_ERR_PROFILE;
            }
        }
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp880InitRing */

/**
 * Vp880InitCid()
 *  This function is used to send caller ID information. It accepts an amount of
 * CID message data up to a "full" buffer amount (2 times the amount of the
 * size used for ContinueCID). It low fills the primary buffer such that the
 * application is interrupted at the earliest time when the API is ready to
 * accept more data.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function. This function needs to be called before placing the line in to
 * ringing state.
 *
 * Postconditions:
 *  This function transmits the given CID information on the line (when the line
 * is placed in the ringing state).
 */
VpStatusType
Vp880InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 primaryByteCount, secondaryByteCount;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->status.state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length > (2 * VP_SIZEOF_CID_MSG_BUFFER)) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length == 0) {
        return VP_STATUS_SUCCESS;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->callerId.cliIndex = 0;
    pLineObj->callerId.cliMPIndex = 0;
    pLineObj->callerId.cliMSIndex = 0;

    /* Stop CID if it was in progress */
    pLineObj->callerId.cliTimer = 0;
    pLineObj->callerId.status = VP_CID_RESET_VALUE;
    pLineObj->suspendCid = FALSE;
    pLineObj->callerId.dtmfStatus = VP_CID_DTMF_RESET_VALUE;
    pLineObj->callerId.cidCheckSum = 0;

    /*
     * If length is within the size of just the primary buffer size, then only
     * fill the primary buffer. Otherwise (the length exceeds the size of the
     * primary buffer size) "low fill" the primary buffer and max fill the
     * secondary buffer. This has the affect of causing a CID Data event
     * quickly and giving the application a maximum amount of time to refill
     * the message buffer
     */
    if (length <= VP_SIZEOF_CID_MSG_BUFFER) {
        pLineObj->callerId.primaryMsgLen = length;
        pLineObj->callerId.secondaryMsgLen = 0;
    } else {
        pLineObj->callerId.primaryMsgLen = (length - VP_SIZEOF_CID_MSG_BUFFER);
        pLineObj->callerId.secondaryMsgLen = VP_SIZEOF_CID_MSG_BUFFER;
    }

    /*
     * Copy the message data to the primary API buffer. If we're here, there's
     * at least one byte of primary message data. So a check is not necessary
     */
    pLineObj->callerId.status |= VP_CID_PRIMARY_FULL;
    for (primaryByteCount = 0;
         (primaryByteCount < pLineObj->callerId.primaryMsgLen);
         primaryByteCount++) {
        pLineObj->callerId.primaryBuffer[primaryByteCount]
            = pCidData[primaryByteCount];
        pLineObj->callerId.cidCheckSum += pCidData[primaryByteCount];
        pLineObj->callerId.cidCheckSum = pLineObj->callerId.cidCheckSum % 256;
    }

    /* Copy the message data to the secondary API buffer if there is any */
    if (pLineObj->callerId.secondaryMsgLen > 0) {
        pLineObj->callerId.status |= VP_CID_SECONDARY_FULL;
        for (secondaryByteCount = 0;
             (secondaryByteCount < pLineObj->callerId.secondaryMsgLen);
             secondaryByteCount++) {
            pLineObj->callerId.secondaryBuffer[secondaryByteCount] =
                pCidData[secondaryByteCount + primaryByteCount];
            pLineObj->callerId.cidCheckSum +=
                pCidData[secondaryByteCount + primaryByteCount];
            pLineObj->callerId.cidCheckSum =
                pLineObj->callerId.cidCheckSum % 256;
        }
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SendCid()
 *  This function may be used to send Caller ID information on-demand. It
 * accepts an amount of CID message data up to a "full" buffer amount (2 times
 * the amount of the size used for ContinueCID). It low fills the primary buffer
 * such that the application is interrupted at the earliest time when the API
 * is ready to accept more data.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. The length of the
 * message (indicated by the length field passed) must not exceed the buffer
 * size.
 *
 * Postconditions:
 * Caller ID information is transmitted on the line.
 */
VpStatusType
Vp880SendCid(
    VpLineCtxType *pLineCtx,        /**< Line to send CID on */
    uint8 length,                   /**< Length of the current message data, not
                                     * to exceed the buffer size
                                     */
    VpProfilePtrType pCidProfile,   /**< CID Profile or Profile index to use */
    uint8p pCidData)                /**< CID Message data */
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpProfilePtrType *pProfileTable;
    VpProfilePtrType pCidProfileLocal;

    uint8 primaryByteCount, secondaryByteCount;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    int cidIndex = VpGetProfileIndex(pCidProfile);

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->status.state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length > (2 * VP_SIZEOF_CID_MSG_BUFFER)) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length == 0) {
        return VP_STATUS_SUCCESS;
    }

    /* Determine if it's an index or profile */
    if ((cidIndex >= 0) && (cidIndex < VP_CSLAC_CALLERID_PROF_TABLE_SIZE)) {
        /* It's an index. Set the line profile to the device indexed profile */
        if (!(pDevObj->profEntry.cidCadProfEntry & (0x0001 << cidIndex))) {
            /* The profile is invalid -- error. */
            return VP_STATUS_ERR_PROFILE;
        }
        pProfileTable = pDevObj->devProfileTable.pCallerIdProfileTable;
        pCidProfileLocal = pProfileTable[cidIndex];
    } else {
        if (cidIndex > VP_CSLAC_CALLERID_PROF_TABLE_SIZE) {
            return VP_STATUS_INVALID_ARG;
        } else {
            /*
             * This is a pointer. Make sure it's the correct type, and if so,
             * set it to the line object
             */
            if(VpVerifyProfileType(VP_PROFILE_CID, pCidProfile) == TRUE) {
                pCidProfileLocal = pCidProfile;
            } else {
                return VP_STATUS_ERR_PROFILE;
            }
        }
    }

    if (pCidProfileLocal == VP_PTABLE_NULL) {
        return VP_STATUS_ERR_PROFILE;
    }

    /* If we're here, all parameters passed are valid */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->callerId.pCliProfile = pCidProfileLocal;
    pLineObj->callerId.cliIndex = 0;
    pLineObj->callerId.cliMPIndex = 0;
    pLineObj->callerId.cliMSIndex = 0;

    pLineObj->callerId.status |= VP_CID_IN_PROGRESS;
    pLineObj->callerId.status &= ~VP_CID_SIG_B_VALID;
    pLineObj->callerId.status &= ~VP_CID_REPEAT_MSG;
    pLineObj->callerId.status &= ~VP_CID_END_OF_MSG;

    pLineObj->callerId.cliTimer = 1;
    pLineObj->callerId.cidCheckSum = 0;

    /*
     * If length is within the size of just the primary buffer size, then only
     * fill the primary buffer. Otherwise (the length exceeds the size of the
     * primary buffer size) "low fill" the primary buffer and max fill the
     * secondary buffer. This has the affect of causing a CID Data event
     * quickly and giving the application a maximum amount of time to refill
     * the message buffer
     */
    if (length <= VP_SIZEOF_CID_MSG_BUFFER) {
        pLineObj->callerId.primaryMsgLen = length;
        pLineObj->callerId.secondaryMsgLen = 0;
    } else {
        pLineObj->callerId.primaryMsgLen = (length - VP_SIZEOF_CID_MSG_BUFFER);
        pLineObj->callerId.secondaryMsgLen = VP_SIZEOF_CID_MSG_BUFFER;
    }

    /*
     * Copy the message data to the primary API buffer. If we're here, there's
     * at least one byte of primary message data. So a check is not necessary
     */
    pLineObj->callerId.status |= VP_CID_PRIMARY_FULL;
    for (primaryByteCount = 0;
         (primaryByteCount < pLineObj->callerId.primaryMsgLen);
         primaryByteCount++) {
        pLineObj->callerId.primaryBuffer[primaryByteCount]
            = pCidData[primaryByteCount];
        pLineObj->callerId.cidCheckSum += pCidData[primaryByteCount];
        pLineObj->callerId.cidCheckSum = pLineObj->callerId.cidCheckSum % 256;
    }

    /* Copy the message data to the secondary API buffer if there is any */
    if (pLineObj->callerId.secondaryMsgLen > 0) {
        pLineObj->callerId.status |= VP_CID_SECONDARY_FULL;
        for (secondaryByteCount = 0;
             (secondaryByteCount < pLineObj->callerId.secondaryMsgLen);
             secondaryByteCount++) {
            pLineObj->callerId.secondaryBuffer[secondaryByteCount] =
                pCidData[secondaryByteCount + primaryByteCount];
            pLineObj->callerId.cidCheckSum +=
                pCidData[secondaryByteCount + primaryByteCount];
            pLineObj->callerId.cidCheckSum =
                pLineObj->callerId.cidCheckSum % 256;
        }
    }

    pLineObj->callerId.status |= VP_CID_PRIMARY_IN_USE;
    pLineObj->callerId.status &= ~VP_CID_SECONDARY_IN_USE;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880ContinueCid()
 *  This function is called to provide more caller ID data (in response to
 * Caller ID data event from the VP-API). See VP-API-II  documentation
 * for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Continues to transmit Caller ID information on the line.
 */
VpStatusType
Vp880ContinueCid(
    VpLineCtxType *pLineCtx,    /**< Line to continue CID on */
    uint8 length,               /**< Length of data passed not to exceed the
                                 * buffer length
                                 */
    uint8p pCidData)            /**< CID message data */
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 byteCount = 0;
    uint8 *pMsgLen;
    uint8 *pBuffer;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    pLineObj->callerId.status &= ~VP_CID_END_OF_MSG;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->status.state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length >  VP_SIZEOF_CID_MSG_BUFFER) {
        return VP_STATUS_INVALID_ARG;
    }

    if (length == 0) {
        return VP_STATUS_SUCCESS;
    }

    /*
     * When this function is called, the buffer that is in use is flagged
     * by the VpCliGetEncodeByte() function in vp_api_common.c file. That
     * function implements the logic of when to switch between the primary
     * and secondary buffer. This function just needs to fill the bufffer that
     * is not currently in use, starting with the primary (because the primary
     * buffer is also used first for the first part of the message).
     */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (!(pLineObj->callerId.status & VP_CID_PRIMARY_IN_USE)) {
        /* Fill the primary buffer */
        pLineObj->callerId.status |= VP_CID_PRIMARY_FULL;
        pMsgLen = &(pLineObj->callerId.primaryMsgLen);
        pBuffer = &(pLineObj->callerId.primaryBuffer[0]);
    } else {
        /* Fill the secondary buffer */
        pLineObj->callerId.status |= VP_CID_SECONDARY_FULL;
        pMsgLen = &(pLineObj->callerId.secondaryMsgLen);
        pBuffer = &(pLineObj->callerId.secondaryBuffer[0]);
    }

    *pMsgLen = length;

    /* Copy the message data to the API buffer */
    for (byteCount = 0; (byteCount < *pMsgLen); byteCount++) {
        pBuffer[byteCount] = pCidData[byteCount];

        pLineObj->callerId.cidCheckSum += pBuffer[byteCount];
        pLineObj->callerId.cidCheckSum = pLineObj->callerId.cidCheckSum % 256;
    }
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp880ContinueCid() */

/**
 * Vp880CtrlSetCliTone()
 *  This function is called by the API internally to enable or disable the
 * signal generator used for Caller ID.
 *
 * Preconditions:
 *  The line context must be valid (pointing to a Vp880 line object type
 *
 * Postconditions:
 *  The signal generator used for CID tones is enabled/disabled indicated by
 * the mode parameter passed.
 */
VpStatusType
Vp880CtrlSetCliTone(
    VpLineCtxType *pLineCtx,    /**< Line affected by the CLI tones */
    bool mode)                  /**< TRUE = enabled, FALSE = disable tones */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;
    uint8 sigGenCtrl;

    /*
     * This function should only be called when the Caller ID sequence is
     * generating an alerting tone. We're using the C/D generators, so disable
     * A/B and enable C/D only (if mode == TRUE).
     */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_RD, VP880_GEN_CTRL_LEN,
        &sigGenCtrl);
    sigGenCtrl &= ~(VP880_GEN_ALLON | VP880_GEN_CTRL_EN_BIAS);

    if (mode == TRUE) {
        sigGenCtrl |= (VP880_GENC_EN | VP880_GEND_EN);
    }
    VP_CID(VpLineCtxType, pLineCtx, ("Writing 0x%02X to SignGen Ctrl", sigGenCtrl));
    VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT, VP880_GEN_CTRL_LEN,
        &sigGenCtrl);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880CtrlSetFSKGen()
 *  This function is called by the CID sequencer executed internally by the API
 *
 * Preconditions:
 *  The line context must be valid (pointing to a VP880 line object type
 *
 * Postconditions:
 *  The data indicated by mode and data is applied to the line. Mode is used
 * to indicate whether the data is "message", or a special character. The
 * special characters are "channel siezure" (alt. 1/0), "mark" (all 1), or
 * "end of transmission".
 */
void
Vp880CtrlSetFSKGen(
    VpLineCtxType *pLineCtx,    /**< Line affected by the mode and data */
    VpCidGeneratorControlType mode,      /**< Indicates the type of data being sent.
                                 * Affects the start and stop bit used
                                 */
    uint8 data)                 /**< 8-bit message data */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;
    uint8 fskParam[VP880_CID_PARAM_LEN];
    bool moreData = TRUE;

    VpMpiCmdWrapper(deviceId, ecVal, VP880_CID_PARAM_RD, VP880_CID_PARAM_LEN,
        fskParam);
    fskParam[0] &= ~(VP880_CID_FRAME_BITS);

    switch(mode) {
        case VP_CID_SIGGEN_EOT:
            if (data == 0) {
                /* Stop Transmission Immediately */
                Vp880MuteChannel(pLineCtx, FALSE);
                fskParam[0] |= VP880_CID_DIS;
            } else {
                /* Wait until the device is complete */
                pLineObj->suspendCid = TRUE;
            }
            moreData = FALSE;
            break;

        case VP_CID_GENERATOR_DATA:
            Vp880MuteChannel(pLineCtx, TRUE);

            fskParam[0] |= (VP880_CID_FB_START_0 | VP880_CID_FB_STOP_1);
            fskParam[0] &= ~(VP880_CID_DIS);
            if ((pLineObj->callerId.status & VP_CID_END_OF_MSG) ||
                (pLineObj->callerId.status & VP_CID_MID_CHECKSUM)) {
                fskParam[0] |= VP880_CID_EOM;
            } else {
                fskParam[0] &= ~(VP880_CID_EOM);
            }
            break;

        case VP_CID_GENERATOR_KEYED_CHAR:
            Vp880MuteChannel(pLineCtx, TRUE);
            fskParam[0] &= ~(VP880_CID_EOM | VP880_CID_DIS);

            switch(data) {
                case VP_FSK_CHAN_SEIZURE:
                    fskParam[0] |=
                        (VP880_CID_FB_START_0 | VP880_CID_FB_STOP_1);
                    break;

                case VP_FSK_MARK_SIGNAL:
                    fskParam[0] |=
                        (VP880_CID_FB_START_1 | VP880_CID_FB_STOP_1);
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }

    VP_CID(VpLineCtxType, pLineCtx, ("Writing 0x%02X to CID Params", fskParam[0]));
    VpMpiCmdWrapper(deviceId, ecVal, VP880_CID_PARAM_WRT, VP880_CID_PARAM_LEN,
        fskParam);

    if (moreData == TRUE) {
        VP_CID(VpLineCtxType, pLineCtx, ("Writing 0x%02X to CID Data", data));
        VpMpiCmdWrapper(deviceId, ecVal, VP880_CID_DATA_WRT, VP880_CID_DATA_LEN,
            &data);
    }

    return;
}

/**
 * Vp880SendSignal()
 *  This function sends a signal on the line. The type of signal is specified
 * by the type parameter passed. The structure passed specifies the parameters
 * associated with the signal.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The signal specified is applied to the line.
 */
VpStatusType
Vp880SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    void *pStruct)
{
    VpDigitType *pDigit;
    VpDigitType digit = VP_DIG_NONE;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->status.state & VP_DEV_IN_CAL) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pStruct == VP_NULL) {
        pDigit = &digit;
    } else {
        pDigit = pStruct;
    }

    switch(type) {
        case VP_SENDSIG_MSG_WAIT_PULSE:
            status = Vp880SendMsgWaitPulse(pLineCtx, pStruct);
            break;

        case VP_SENDSIG_DTMF_DIGIT:
            status = Vp880SendDigit(pLineCtx, VP_DIGIT_GENERATION_DTMF, *pDigit);
            break;

        case VP_SENDSIG_PULSE_DIGIT:
            pDigit = (VpDigitType *)pStruct;
            status = Vp880SendDigit(pLineCtx, VP_DIGIT_GENERATION_DIAL_PULSE,
                *pDigit);
            break;

        case VP_SENDSIG_HOOK_FLASH:
            /* prevent case of *pDigit when user passes VP_NULL */
            status = Vp880SendDigit(pLineCtx, VP_DIGIT_GENERATION_DIAL_HOOK_FLASH,
                VP_DIG_NONE);
            break;

        case VP_SENDSIG_FWD_DISCONNECT:
        case VP_SENDSIG_TIP_OPEN_PULSE:
            if (pStruct != VP_NULL) {
                VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Pulse Time %d", *((uint16 *)pStruct)));
                status = Vp880SendPulse(pLineCtx, type, *((uint16 *)pStruct));
            } else {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_SENDSIG_POLREV_PULSE:
            if (pStruct != VP_NULL) {
                status = Vp880SendPolRevPulse(pLineCtx, *((uint16 *)pStruct));
            } else {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_SENDSIG_MOMENTARY_LOOP_OPEN:
            status = Vp880MomentaryLoopOpen(pLineCtx);
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    return status;
}

/**
 * Vp880SendMsgWaitPulse()
 *  This function sends a message waiting pulse to the line specified by the
 * by the pMsgWait parameter passed. The structure specifies a voltage, on-time,
 * off-time, and number of pulses.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The message waiting signal specified is applied to the line.
 */
VpStatusType
Vp880SendMsgWaitPulse(
    VpLineCtxType *pLineCtx,
    VpSendMsgWaitType *pMsgWait)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;

    uint16 tempTime, firstTimer, secondTimer;
    uint16 tickRate = pDevObj->devProfileData.tickRate;

    VpLineStateType currentState = pLineObj->lineState.usrCurrent;

    uint8 seqByte, branchCount;
    uint8 addStep = 0;

    uint32 aVolt;
    int32 userVolt;
    uint8 cmdLen = 0x08;  /* Minimum Cadence with infinite on */

    /*
     * Set the signal generator parameters to set the A amplitude and frequency
     * "very low". We'll adjust the bias to the user defined MsgWait voltage
     */
    uint8 sigGenBytes[VP880_SIGA_PARAMS_LEN] = {
        0x00, 0x29, 0x73, 0x04, 0x44, 0x00, 0x15, 0x7F, 0xFD, 0x00, 0x00};

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    /*
     * If we're already in Ringing, return a failure since we're using a
     * shared resource to accomplish this function.
     */
    if ((currentState == VP_LINE_RINGING) || (currentState == VP_LINE_RINGING_POLREV)) {
        return VP_STATUS_FAILURE;
    }

    /*
     * If the voltage is 0, it (previously) meant to use the maximum voltage
     * supported by the line. However, that function has been removed so instead
     * of stopping Message Waiting, just return error code to maintain a bit of
     * backward compatibility (max voltage isn't applied, but it isn't stopped
     * either).
     */
    if ((pMsgWait != VP_NULL) && (pMsgWait->voltage == 0)) {
        return VP_STATUS_INVALID_ARG;
    }

    /* All parameters passed are good -- proceed */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;    /* No active status */

    for (seqByte = 0; seqByte < VP880_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /*
     * If we were previously running a Message Waiting cadence, stop it and
     * generate the event.
     * If we were previously running another cadence, let it continue and
     * return.
     */
    if ((pMsgWait == VP_NULL) || (pMsgWait->onTime == 0)) {
        VpSetLineState(pLineCtx, currentState);
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
        pLineObj->cadence.pActiveCadence = VP_PTABLE_NULL;
        pLineObj->lineEvents.process |= VP_LINE_EVID_SIGNAL_CMP;
        pLineObj->processData = VP_SENDSIG_MSG_WAIT_PULSE;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /*
     * Compute the new signal generator A values from the voltage and set the
     * line state that is used to apply the message waiting pulse
     */
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    if (pMsgWait->voltage > 0) {
        userVolt = pMsgWait->voltage;
        pLineObj->intSequence[9] = VP_PROFILE_CADENCE_STATE_MSG_WAIT_NORM;
    } else {
        userVolt = -pMsgWait->voltage;
        pLineObj->intSequence[9] = VP_PROFILE_CADENCE_STATE_MSG_WAIT_POLREV;
    }

    /* Scale by same factor as bit resolution */
    aVolt = userVolt * (uint32)VP880_RINGING_BIAS_FACTOR;

    /* Scale down by the bit resolution of the device */
    aVolt /= (uint32)VP880_RINGING_BIAS_SCALE;

    sigGenBytes[VP880_SIGA_BIAS_MSB] = (aVolt >> 8) & 0xFF;
    sigGenBytes[VP880_SIGA_BIAS_LSB] = (aVolt & 0xFF);

    /* Write the new signal generator parameters */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_RINGER_PARAMS_WRT,
       VP880_RINGER_PARAMS_LEN, sigGenBytes);

    /*
     * Build the rest of the cadence defined by the user input (message state
     * set above). Start by setting the type of profile to an API Message Wait
     * Pulse type
     */
    pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
        VP_PRFWZ_PROFILE_MSG_WAIT_PULSE_INT;

    /*
     * Set the timers for on/off pulse duration, scale from mS to tickRate
     * and prevent rounding down to 0
     */
    if (pMsgWait->onTime < (tickRate >> 8)) {
        firstTimer = 3;
    } else {
        firstTimer = MS_TO_TICKRATE(pMsgWait->onTime, tickRate);

        /* Prevent 0 for time (because that means "forever") */
        if (firstTimer <= 2) {
            firstTimer = 3;
        }
    }

    branchCount = 0;
    if (firstTimer > 8192) {
        /* Special Handling for using 16-bit time in 14-bit data fields */
        for (; firstTimer > 8192; branchCount++) {
            firstTimer = ((firstTimer >> 1) & 0x3FFF);
        }
        cmdLen+=2;
    }

    pLineObj->intSequence[10] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    tempTime = ((firstTimer - 2) >> 8) & 0x1F;
    pLineObj->intSequence[10] |= tempTime;

    tempTime = ((firstTimer - 2) & 0x00FF);
    pLineObj->intSequence[11] = tempTime;

    if (branchCount) {
        pLineObj->intSequence[12] = VP_SEQ_SPRCMD_BRANCH_INSTRUCTION;
        pLineObj->intSequence[12] |= 0x01;  /* On-Time is the step 1 (0 base) */
        pLineObj->intSequence[13] = branchCount;
        addStep+=2;
    }

    /*
     * If the off-time is 0, we will stay in the previous state forever so the
     * cadencer needs to stop where it is
     */
    if (pMsgWait->offTime == 0) {
        pLineObj->intSequence[VP_PROFILE_LENGTH] = cmdLen;
        pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] =
            (0x04 + addStep);
    } else {
        cmdLen+=4;   /* Add two for the next state and two for the off-time */

        /* In-between pulses we'll return to the current state */
        pLineObj->intSequence[12+addStep]
            = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);
        pLineObj->intSequence[13+addStep] =
            ConvertApiState2PrfWizState(currentState);

        /*
         * Set the timers for on/off pulse duration, scale from mS to tickRate
         * and prevent rounding down to 0
         */
        if (pMsgWait->offTime < (tickRate >> 8)) {
            secondTimer = 3;
        } else {
            secondTimer = MS_TO_TICKRATE(pMsgWait->offTime, tickRate);

            /* Prevent 0 for time (because that means "forever") */
            if (secondTimer <= 2) {
                secondTimer = 3;
            }
        }

        branchCount = 0;
        if (secondTimer > 8192) {
            cmdLen+=2;   /* Add two for the off-time branch loop */
            /* Special Handling for using 16-bit time in 14-bit data fields */
            for (; secondTimer > 8192; branchCount++) {
                secondTimer = ((secondTimer >> 1) & 0x3FFF);
            }
        }

        pLineObj->intSequence[14+addStep] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
        tempTime = ((secondTimer - 2) >> 8) & 0x1F;
        pLineObj->intSequence[14+addStep] |= tempTime;

        tempTime = ((secondTimer - 2) & 0x00FF);
        pLineObj->intSequence[15+addStep] = tempTime;

        if (branchCount) {
            pLineObj->intSequence[16+addStep] = VP_SEQ_SPRCMD_BRANCH_INSTRUCTION;
            pLineObj->intSequence[16+addStep] |= (0x03 + (addStep / 2));
            pLineObj->intSequence[17+addStep] = branchCount;
            addStep+=2;
        }

        /*
         * If the number of cycles is 0, set the branch to repeat forever. If
         * it's 1, don't add a branch statement because the sequence should end
         * after the first cycle, otherwise subtract 1 from the total number of
         * cycles to force the correct number of "repeats" (branch)
         */

        if (pMsgWait->cycles != 1) {
            cmdLen+=2; /* Two more for this last branch operator */
            pLineObj->intSequence[16+addStep] = VP_SEQ_SPRCMD_BRANCH_INSTRUCTION;
            pLineObj->intSequence[17+addStep] = (pMsgWait->cycles) ?
                (pMsgWait->cycles - 1) : pMsgWait->cycles;
        }
    }

    /*
     * Set the line object cadence variables to this sequence and activate the
     * sequencer
     */
    pLineObj->intSequence[VP_PROFILE_LENGTH] = cmdLen;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = cmdLen - 4;

    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
    pLineObj->cadence.length = pLineObj->intSequence[VP_PROFILE_LENGTH];

    pLineObj->cadence.pActiveCadence = &pLineObj->intSequence[0];
    pLineObj->cadence.pCurrentPos = &pLineObj->intSequence[8];

    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
    pLineObj->cadence.status |= VP_CADENCE_STATUS_SENDSIG;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SendPulse()
 *  This function sends either a forward disconnect or Tip Open pulse to the
 * line specified for a duration given in mS.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  A disconnect or tip open has been applied to the line, the line state is
 * restored to what it was prior to this function being called.
 */
VpStatusType
Vp880SendPulse(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    uint16 timeInMs)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 seqByte, index, targetState;
    uint16 timeIn5mS = 0;

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    cadenceState = ConvertApiState2PrfWizState(currentState);

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = 0x0000;    /* No active status */

    for (seqByte = 0; seqByte < VP880_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Set the cadence type and target state */
    if (type == VP_SENDSIG_FWD_DISCONNECT) {
        pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_FWD_DISC_INT;
        targetState = VP_PROFILE_CADENCE_STATE_DISCONNECT;
    } else {
        pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_TIP_OPEN_INT;
        targetState = VP_PROFILE_CADENCE_STATE_TIP_OPEN;
    }

    /* First step is to go to target state */
    index = 0;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = targetState;

    /* Then wait for the time specified -- rounded to 5mS increments */
    if (timeInMs < 5) {
        timeIn5mS = 1;
    } else {
        timeIn5mS = timeInMs / 5;
    }
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        |= (timeIn5mS >> 8) & 0x1F;

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (timeIn5mS & 0xFF);

    /* Restore the line state */
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = cadenceState;

    /* Then wait for 100mS for the detector to become stable */
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = 20; /* 5mS per increment */

    index++; /* Adjust one more for length values */
    /*
     * Set the line object cadence variables to this sequence and activate the
     * sequencer
     */
    pLineObj->intSequence[VP_PROFILE_LENGTH] = index + 4;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = index;

    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
    pLineObj->cadence.length = pLineObj->intSequence[VP_PROFILE_LENGTH];

    pLineObj->cadence.pActiveCadence = &pLineObj->intSequence[0];
    pLineObj->cadence.pCurrentPos =
        &pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START];

    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
    pLineObj->cadence.status |= VP_CADENCE_STATUS_SENDSIG;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SendPolRevPulse()
 *  This function sends a Pol Rev pulse to the line specified with a duration
 * given in mS.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  A Pol Rev pulse has been applied to the line, the line state is restored
 * to what it was prior to this function being called.
 */
VpStatusType
Vp880SendPolRevPulse(
    VpLineCtxType *pLineCtx,
    uint16 timeInMs)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState, polRevState;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 seqByte, index;
    uint16 timeIn5mS = 0;

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    if (currentState == VP_LINE_DISCONNECT) {
        return VP_STATUS_INVALID_ARG;
    }

    cadenceState = ConvertApiState2PrfWizState(currentState);
    polRevState = ConvertApiState2PrfWizState(VpGetReverseState(currentState));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = 0x0000;    /* No active status */

    for (seqByte = 0; seqByte < VP880_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Set the cadence type */
    pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
        VP_PRFWZ_PROFILE_POLREV_PULSE_INT;

    /* First step is to go to polrev state */
    index = 0;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = polRevState;

    /* Then wait for the time specified -- rounded to 5mS increments */
    if (timeInMs < 5) {
        timeIn5mS = 1;
    } else {
        timeIn5mS = timeInMs / 5;
    }
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        |= (timeIn5mS >> 8) & 0x1F;

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (timeIn5mS & 0xFF);

    /* Restore the line state */
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = cadenceState;

    /* Then wait for 100mS for the detector to become stable */
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = 20; /* 5mS per increment */

    index++; /* Adjust one more for length values */
    /*
     * Set the line object cadence variables to this sequence and activate the
     * sequencer
     */
    pLineObj->intSequence[VP_PROFILE_LENGTH] = index + 4;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = index;

    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
    pLineObj->cadence.length = pLineObj->intSequence[VP_PROFILE_LENGTH];

    pLineObj->cadence.pActiveCadence = &pLineObj->intSequence[0];
    pLineObj->cadence.pCurrentPos =
        &pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START];

    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
    pLineObj->cadence.status |= VP_CADENCE_STATUS_SENDSIG;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880MomentaryLoopOpen()
 *  This function applies a Momentary Loop Open to an FXO line and tests for
 * a parallel off-hook.
 *
 * Preconditions:
 *  The line must first be initialized and must be of FXO type.
 *
 * Postconditions:
 *  A 10ms loop open is applied to the line and line state returns to previous
 * condition. An event is generated indicating if there exists a parallel phone
 * off-hook or not.
 */
VpStatusType
Vp880MomentaryLoopOpen(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState;

    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;

    uint8 seqByte, index;
    uint16 timeIn5mS = 0;
    uint8 loopSup[VP880_LOOP_SUP_LEN] = {0x18, 0xE1, 0x79, 0xEB};

    if (!(pLineObj->status & VP880_IS_FXO)) {
        return VP_STATUS_INVALID_ARG;
    }

    switch(currentState) {
        case VP_LINE_FXO_OHT:
            cadenceState = VP_PROFILE_CADENCE_STATE_FXO_OHT;
            break;

        case VP_LINE_FXO_LOOP_OPEN:
            cadenceState = VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;
            break;

        case VP_LINE_FXO_LOOP_CLOSE:
            cadenceState = VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE;
            break;

        case VP_LINE_FXO_TALK:
            cadenceState = VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK;
            break;

        default:
            return VP_STATUS_FAILURE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = 0x0000;    /* No active status */

    for (seqByte = 0; seqByte < VP880_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Set the cadence type */
    pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
        VP_PRFWZ_PROFILE_MOMENTARY_LOOP_OPEN_INT;

    VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN,
        loopSup);

    /* First step is to go to Loop Open */
    index = 0;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;

    /* Then wait for at least 10ms. The time is higher by 2*ApiTick value */
    timeIn5mS = 2;  /* Cadencer Tick is 5ms increments */

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        |= (timeIn5mS >> 8) & 0x1F;

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (timeIn5mS & 0xFF);

    /* Restore the line state */
    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = cadenceState;

    index++; /* Adjust for length values */
    /*
     * Set the line object cadence variables to this sequence and activate the
     * sequencer
     */
    pLineObj->intSequence[VP_PROFILE_LENGTH] = index + 4;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = index;

    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
    pLineObj->cadence.length = pLineObj->intSequence[VP_PROFILE_LENGTH];

    pLineObj->cadence.pActiveCadence = &pLineObj->intSequence[0];
    pLineObj->cadence.pCurrentPos =
        &pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START];

    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
    pLineObj->cadence.status |= VP_CADENCE_STATUS_SENDSIG;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SendDigit()
 *  This function sends a DTMF or Dial Pulse digit on an FXO line. It creates
 * a sequencer compatible profile to control the FXO loop open, loop close, and
 * time operators.
 *
 * Preconditions:
 *  The line must first be initialized and must be of FXO type.
 *
 * Postconditions:
 *  The digit specified is sent on the line in the form specified (DTMF or Dial
 * Pulse).  This function returns the success code if the line is an FXO type of
 * line, if the digit is between 0 - 9, and if the digit type is either DTMF or
 * Dial Pulse.
 */
VpStatusType
Vp880SendDigit(
    VpLineCtxType *pLineCtx,            /**< Line to send a digit on */
    VpDigitGenerationType digitType,    /**< Type of digit to send. May indicate
                                         * DTMF, Dial Pulse, or Hook Flash
                                         */
    VpDigitType digit)                  /**< The digit to send. Used if type of
                                         * digit is DTMF or Dial Pulse
                                         */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tempTime, firstTimer, secondTimer;
    uint16 tickAdjustment;

    uint8 seqByte;

    VpDeviceIdType deviceId = pDevObj->deviceId;

    if (!(pLineObj->status & VP880_IS_FXO)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Function invalid for FXS"));
        return VP_STATUS_INVALID_ARG;
    }

    switch(digitType) {
        case VP_DIGIT_GENERATION_DIAL_PULSE:
            if ((pLineObj->lineState.currentState != VP_LINE_FXO_TALK)
             && (pLineObj->lineState.currentState != VP_LINE_FXO_LOOP_CLOSE)) {
                return VP_STATUS_INVALID_ARG;
            }

        case VP_DIGIT_GENERATION_DTMF:
            if ((VpIsDigit(digit) == FALSE) || (digit == VP_DIG_NONE)) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Invalid digit"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_DIGIT_GENERATION_DIAL_HOOK_FLASH:
            if ((pLineObj->lineState.currentState != VP_LINE_FXO_TALK)
             && (pLineObj->lineState.currentState != VP_LINE_FXO_LOOP_CLOSE)) {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Invalid digitType"));
            return VP_STATUS_INVALID_ARG;
    }

    /* Parameters passed are good -- proceed */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;    /* No active status */

    for (seqByte = 0; seqByte < VP880_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Tick adjustment in 5ms cadence time units */
    tickAdjustment = TICKS_TO_MS(1, pDevObj->devProfileData.tickRate) / 5;

    switch(digitType) {
        case VP_DIGIT_GENERATION_DTMF:
            Vp880MuteChannel(pLineCtx, TRUE);
            Vp880SetDTMFGenerators(pLineCtx, VP_CID_NO_CHANGE, digit);

            /* Fixed total length and sequence length for DTMF generation */
            pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x0C;
            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = 0x08;

            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_SIGGEN);

            pLineObj->intSequence[12]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_SIGGEN);

            pLineObj->intSequence[9] =
                (VP_SEQ_SIGGEN_A_EN | VP_SEQ_SIGGEN_B_EN);

            pLineObj->intSequence[13] = VP_SEQ_SIGGEN_ALL_DISABLED;

            firstTimer = pLineObj->digitGenStruct.dtmfOnTime;
            if (firstTimer > tickAdjustment) {
                firstTimer -= tickAdjustment;
            } else {
                firstTimer = 1;
            }
            pLineObj->intSequence[10] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
            tempTime = (firstTimer >> 8) & 0x03;
            pLineObj->intSequence[10] |= tempTime;

            tempTime = (firstTimer & 0x00FF);
            pLineObj->intSequence[11] |= tempTime;

            secondTimer = pLineObj->digitGenStruct.dtmfOffTime;
            if (secondTimer > tickAdjustment) {
                secondTimer -= tickAdjustment;
            } else {
                secondTimer = 1;
            }
            pLineObj->intSequence[14] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
            tempTime = (secondTimer >> 8) & 0x03;
            pLineObj->intSequence[14] |= tempTime;

            tempTime = (secondTimer & 0x00FF);
            pLineObj->intSequence[15] |= tempTime;

            pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
                VP_PRFWZ_PROFILE_DTMF_DIG_GEN;
            break;

        case VP_DIGIT_GENERATION_DIAL_PULSE:
            /* Fixed total length and sequence length for DP generation */
            pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x10;
            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = 0x0C;

            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

            pLineObj->intSequence[12]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

            if (pLineObj->lineState.currentState == VP_LINE_FXO_TALK) {
                pLineObj->intSequence[9] =
                    VP_PROFILE_CADENCE_STATE_FXO_OHT;
                pLineObj->intSequence[13] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK;
            } else {
                pLineObj->intSequence[9] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;
                pLineObj->intSequence[13] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE;
            }

            firstTimer = pLineObj->digitGenStruct.breakTime;
            if (firstTimer > tickAdjustment) {
                firstTimer -= tickAdjustment;
            } else {
                firstTimer = 1;
            }
            pLineObj->intSequence[10] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
            tempTime = (firstTimer >> 8) & 0x03;
            pLineObj->intSequence[10] |= tempTime;

            tempTime = (firstTimer & 0x00FF);
            pLineObj->intSequence[11] |= tempTime;

            secondTimer = pLineObj->digitGenStruct.makeTime;
            if (secondTimer > tickAdjustment * 2) {
                secondTimer -= tickAdjustment * 2;
            } else {
                secondTimer = 1;
            }
            pLineObj->intSequence[14] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
            tempTime = (secondTimer >> 8) & 0x03;
            pLineObj->intSequence[14] |= tempTime;

            tempTime = (secondTimer & 0x00FF);
            pLineObj->intSequence[15] |= tempTime;

            firstTimer = pLineObj->digitGenStruct.dpInterDigitTime;
            if (digit > 1) {
                pLineObj->intSequence[16] = VP_SEQ_SPRCMD_BRANCH_INSTRUCTION;
                pLineObj->intSequence[17] = digit - 1;

                pLineObj->intSequence[18] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
                tempTime = (firstTimer >> 8) & 0x03;
                pLineObj->intSequence[18] |= tempTime;
                tempTime = (firstTimer & 0x00FF);
                pLineObj->intSequence[19] |= tempTime;
            } else {
                pLineObj->intSequence[16] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
                tempTime = (firstTimer >> 8) & 0x03;
                pLineObj->intSequence[16] |= tempTime;
                tempTime = (firstTimer & 0x00FF);
                pLineObj->intSequence[17] |= tempTime;

                pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x0E;
                pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB]
                    = 0x0A;
            }

            pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
                VP_PRFWZ_PROFILE_DIAL_PULSE_DIG_GEN;
            break;

        case VP_DIGIT_GENERATION_DIAL_HOOK_FLASH:
            /* Fixed total length and sequence length for FLASH generation */
            pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x0A;
            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = 0x06;

            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

            pLineObj->intSequence[12]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

            if (pLineObj->lineState.currentState == VP_LINE_FXO_TALK) {
                pLineObj->intSequence[9] =
                    VP_PROFILE_CADENCE_STATE_FXO_OHT;
                pLineObj->intSequence[13] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK;
            } else {
                pLineObj->intSequence[9] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;
                pLineObj->intSequence[13] =
                    VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE;
            }

            firstTimer = pLineObj->digitGenStruct.flashTime;
            if (firstTimer > tickAdjustment) {
                firstTimer -= tickAdjustment;
            } else {
                firstTimer = 1;
            }
            pLineObj->intSequence[10] = VP_SEQ_SPRCMD_TIME_INSTRUCTION;
            tempTime = (firstTimer >> 8) & 0x03;
            pLineObj->intSequence[10] |= tempTime;

            tempTime = (firstTimer & 0x00FF);
            pLineObj->intSequence[11] |= tempTime;

            pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
                VP_PRFWZ_PROFILE_HOOK_FLASH_DIG_GEN;
            break;

        default:
            /*
             * This can only occur if there is an error in the error checking
             * above.
             */
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_INVALID_ARG;
    }

    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
    pLineObj->cadence.length = pLineObj->intSequence[VP_PROFILE_LENGTH];

    pLineObj->cadence.pActiveCadence = &pLineObj->intSequence[0];
    pLineObj->cadence.pCurrentPos = &pLineObj->intSequence[8];

    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
    pLineObj->cadence.status |= VP_CADENCE_STATUS_SENDSIG;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880CliGetEncodedByte()
 *  This function returns an encoded byte of data that is suitable for writing
 * the FSK generator (device dependent).
 *
 * Preconditions
 *  Must have a valid CLI packet in to work from.
 *
 * Postconditions
 *  The per-channel caller ID buffer will be updated with encoded data.
 *
 */
bool
Vp880CliGetEncodedByte(
    VpLineCtxType *pLineCtx,
    uint8 *pByte)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;

    uint8 nextByte = '\0';

    VpOptionEventMaskType *pLineEvents = &(pLineObj->lineEvents);
    VpCallerIdType *pCidStruct = &(pLineObj->callerId);

    uint8 checkSumIndex = VP_CID_PROFILE_FSK_PARAM_LEN +
        pLineObj->callerId.pCliProfile[VP_CID_PROFILE_FSK_PARAM_LEN] +
        VP_CID_PROFILE_CHECKSUM_OFFSET_LSB;

    if (pLineObj->callerId.status & VP_CID_MID_CHECKSUM) {
        pLineObj->callerId.status &= ~VP_CID_MID_CHECKSUM;
        pCidStruct->status |= VP_CID_END_OF_MSG;
        *pByte = '\0';
        return FALSE;
    }

    /* Check to determine which buffer is in use to index the message data */
    if (pCidStruct->status & VP_CID_PRIMARY_IN_USE) {
        /*
         * If the index is at the length of the buffer, we need to switch
         * buffers if there is more data
         */
        if (pCidStruct->cliMPIndex >= pCidStruct->primaryMsgLen) {
            /*
             * At the end of the Primary Buffer. Flag an event and indicate to
             * the API that this buffer is no longer being used and we can
             * accept more data
             */
            pCidStruct->status &= ~VP_CID_PRIMARY_IN_USE;
            pCidStruct->status &= ~VP_CID_PRIMARY_FULL;

            if (pCidStruct->status & VP_CID_SECONDARY_FULL) {
                pLineEvents->process |= VP_LINE_EVID_CID_DATA;
                pLineObj->processData = VP_CID_DATA_NEED_MORE_DATA;

                pCidStruct->status |= VP_CID_SECONDARY_IN_USE;
                pCidStruct->cliMSIndex = 1;
                *pByte = pCidStruct->secondaryBuffer[0];
                nextByte = pCidStruct->secondaryBuffer[1];
            } else {
                if (pLineObj->callerId.pCliProfile[checkSumIndex]) {
                    *pByte = (uint8)(~pLineObj->callerId.cidCheckSum + 1);
                    pLineObj->callerId.status |= VP_CID_MID_CHECKSUM;
                } else {
                    *pByte = '\0';
                }
            }
        } else {
            *pByte = pCidStruct->primaryBuffer[pCidStruct->cliMPIndex];

            /* Get the next byte to be sent after the current byte */
            if ((pCidStruct->cliMPIndex+1) >= pCidStruct->primaryMsgLen) {
                if (pCidStruct->status & VP_CID_SECONDARY_FULL) {
                    nextByte = pCidStruct->secondaryBuffer[0];
                }
            } else {
                nextByte =
                    pCidStruct->primaryBuffer[pCidStruct->cliMPIndex+1];
            }
        }
        pCidStruct->cliMPIndex++;
    } else if (pCidStruct->status & VP_CID_SECONDARY_IN_USE) {
        /*
         * If the index is at the length of the buffer, we need to switch
         * buffers if there is more data
         */
        if (pCidStruct->cliMSIndex >= pCidStruct->secondaryMsgLen) {
            /*
             * At the end of the Secondary Buffer. Flag an event and indicate to
             * the API that this buffer is no longer being used and is empty
             */
            pLineEvents->process |= VP_LINE_EVID_CID_DATA;
            pLineObj->processData = VP_CID_DATA_NEED_MORE_DATA;

            pCidStruct->status &= ~VP_CID_SECONDARY_IN_USE;
            pCidStruct->status &= ~VP_CID_SECONDARY_FULL;

            if (pCidStruct->status & VP_CID_PRIMARY_FULL) {
                pLineEvents->process |= VP_LINE_EVID_CID_DATA;
                pLineObj->processData = VP_CID_DATA_NEED_MORE_DATA;

                pCidStruct->status |= VP_CID_PRIMARY_IN_USE;
                pCidStruct->cliMPIndex = 1;
                *pByte = pCidStruct->primaryBuffer[0];
                nextByte = pCidStruct->primaryBuffer[1];
            } else {
                /* There is no more data in either buffer */
                if (pLineObj->callerId.pCliProfile[checkSumIndex]) {
                    *pByte = (uint8)(~pLineObj->callerId.cidCheckSum + 1);
                    pLineObj->callerId.status |= VP_CID_MID_CHECKSUM;
                } else {
                    *pByte = '\0';
                }
            }
        } else {
            *pByte = pCidStruct->secondaryBuffer[pCidStruct->cliMSIndex];

            /* Get the next byte to be sent after the current byte */
            if ((pCidStruct->cliMSIndex+1) >= pCidStruct->secondaryMsgLen) {
                if (pCidStruct->status & VP_CID_PRIMARY_FULL) {
                    nextByte = pCidStruct->primaryBuffer[0];
                }
            } else {
                nextByte =
                    pCidStruct->secondaryBuffer[pCidStruct->cliMSIndex+1];
            }
        }
        pCidStruct->cliMSIndex++;
    }

    if ((!(pCidStruct->status & VP_CID_PRIMARY_IN_USE))
     && (!(pCidStruct->status & VP_CID_SECONDARY_IN_USE))) {
        if(pCidStruct->status & VP_CID_MID_CHECKSUM) {
            return TRUE;
        } else {
            return FALSE;
        }
    }

    if ((nextByte == '\0')
    && (!(pLineObj->callerId.pCliProfile[checkSumIndex]))) {
        pCidStruct->status |= VP_CID_END_OF_MSG;
    }

    return TRUE;
}

#endif
#endif
