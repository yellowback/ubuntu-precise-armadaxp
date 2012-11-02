/** \file vp890_control.c
 * vp890_control.c
 *
 *  This file contains the implementation of the VP-API 890 Series
 *  Control Functions.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 4881 $
 * $LastChangedDate: 2009-04-09 08:14:55 -0500 (Thu, 09 Apr 2009) $
 */

/* INCLUDES */
#include    "vp_api.h"

#if defined (VP_CC_890_SERIES)  /* Compile only if required */

#include    "vp_api_int.h"
#include    "vp890_api_int.h"
#include    "sys_service.h"

#ifdef CSLAC_SEQ_EN

static VpStatusType
SendMsgWaitPulse(
    VpLineCtxType           *pLineCtx,
    VpSendMsgWaitType       *pMsgWait);

static VpStatusType
SendFwdDisc(
    VpLineCtxType           *pLineCtx,
    uint16                  timeInMs);

static VpStatusType
SendPolRevPulse(
    VpLineCtxType           *pLineCtx,
    uint16                  timeInMs);

static VpStatusType
MomentaryLoopOpen(
    VpLineCtxType           *pLineCtx);

static VpStatusType
SendDigit(
    VpLineCtxType           *pLineCtx,
    VpDigitGenerationType   digitType,
    VpDigitType             digit);

/*******************************************************************************
 * Vp890SendSignal()
 *  This function sends a signal on the line. The type of signal is specified
 * by the type parameter passed. The structure passed specifies the parameters
 * associated with the signal.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The signal specified is applied to the line.
 ******************************************************************************/
VpStatusType
Vp890SendSignal(
    VpLineCtxType       *pLineCtx,
    VpSendSignalType    type,
    void                *pStruct)
{
    VpDigitType           *pDigit;
    VpDigitType           digit    = VP_DIG_NONE;
    VpDevCtxType          *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType          status;

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->status.state, TRUE)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pStruct == VP_NULL) {
        pDigit = &digit;
    } else {
        pDigit = pStruct;
    }

    switch(type) {
        case VP_SENDSIG_MSG_WAIT_PULSE:
            status = SendMsgWaitPulse(pLineCtx, pStruct);
            break;

        case VP_SENDSIG_DTMF_DIGIT:
            status = SendDigit(pLineCtx, VP_DIGIT_GENERATION_DTMF, *pDigit);
            break;

        case VP_SENDSIG_PULSE_DIGIT:
            pDigit = (VpDigitType *)pStruct;
            status = SendDigit(pLineCtx, VP_DIGIT_GENERATION_DIAL_PULSE,
                *pDigit);
            break;

        case VP_SENDSIG_HOOK_FLASH:
            /* prevent case of *pDigit when user passes VP_NULL */
            status = SendDigit(pLineCtx, VP_DIGIT_GENERATION_DIAL_HOOK_FLASH,
                VP_DIG_NONE);
            break;

        case VP_SENDSIG_FWD_DISCONNECT:
            if (pStruct != VP_NULL) {
                status = SendFwdDisc(pLineCtx, *((uint16 *)pStruct));
            } else {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendSignal() - VP_NULL invalid for FWD_DISCONNECT"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_SENDSIG_POLREV_PULSE:
            if (pStruct != VP_NULL) {
                status = SendPolRevPulse(pLineCtx, *((uint16 *)pStruct));
            } else {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendSignal() - VP_NULL invalid for POLREV_PULSE"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_SENDSIG_MOMENTARY_LOOP_OPEN:
            status = MomentaryLoopOpen(pLineCtx);
            break;

        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendSignal() - Invalid signal type"));
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    return status;
} /* Vp890SendSignal() */

/******************************************************************************
 * SendMsgWaitPulse()
 *  This function sends a message waiting pulse to the line specified by the
 * by the pMsgWait parameter passed. The structure specifies a voltage, on-time,
 * off-time, and number of pulses.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The message waiting signal specified is applied to the line.
 ******************************************************************************/
static VpStatusType
SendMsgWaitPulse(
    VpLineCtxType           *pLineCtx,
    VpSendMsgWaitType       *pMsgWait)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    uint8                   ecVal       = pLineObj->ecVal;
    uint16                  tickRate    = pDevObj->devProfileData.tickRate;
    VpLineStateType         currentState = pLineObj->lineState.usrCurrent;
    uint8                   addStep     = 0;
    uint32                  aVolt;
    int32                   userVolt;
    uint8                   seqByte, branchCount;
    uint16                  tempTime, firstTimer, secondTimer;
    uint8                   cmdLen      = 0x08; /* Min Cadence with infinite on */


    /*
     * Set the signal generator parameters to set the A amplitude and frequency
     * "very low". We'll adjust the bias to the user defined MsgWait voltage
     */
    uint8 sigGenBytes[VP890_SIGA_PARAMS_LEN] = {
        0x00, 0x29, 0x73, 0x04, 0x44, 0x00, 0x15, 0x7F, 0xFD, 0x00, 0x00};

    if (pLineObj->status & VP890_IS_FXO) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendMsgWaitPulse() - Function invalid for FXO"));
        return VP_STATUS_INVALID_ARG;
    }

    /*
     * If we're already in Ringing, return a failure since we're using a
     * shared resource to accomplish this function.
     */
    if ((currentState == VP_LINE_RINGING) || (currentState == VP_LINE_RINGING_POLREV)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendMsgWaitPulse() - Shared resource in use - ringing"));
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
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendMsgWaitPulse() - 0 voltage not supported"));
        return VP_STATUS_INVALID_ARG;
    }

    /* All parameters passed are good -- proceed */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;    /* No active status */

    for (seqByte = 0; seqByte < VP890_INT_SEQ_LEN; seqByte++) {
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
    aVolt = userVolt * (uint32)VP890_RINGING_BIAS_FACTOR;

    /* Scale down by the bit resolution of the device */
    aVolt /= (uint32)VP890_RINGING_BIAS_SCALE;

    sigGenBytes[VP890_SIGA_BIAS_MSB] = (aVolt >> 8) & 0xFF;
    sigGenBytes[VP890_SIGA_BIAS_LSB] = (aVolt & 0xFF);

    /* Write the new signal generator parameters */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_RINGER_PARAMS_WRT,
       VP890_RINGER_PARAMS_LEN, sigGenBytes);

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
} /* SendMsgWaitPulse() */

/******************************************************************************
 * SendFwdDisc()
 *  This function sends a forward disconnect to the line specified for a duration
 * given in mS.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  A disconnect has been applied to the line, the line state is restored to what
 * it was prior to this function being called. No events are generated while a
 * disconnect is occuring -- the application should know that it is not possible
 * to detect a line condition while no feed is being presented.
 ******************************************************************************/
static VpStatusType
SendFwdDisc(
    VpLineCtxType               *pLineCtx,
    uint16                      timeInMs)
{
    Vp890LineObjectType         *pLineObj     = pLineCtx->pLineObj;
    VpLineStateType             currentState  = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState;

    VpDevCtxType                *pDevCtx      = pLineCtx->pDevCtx;
    Vp890DeviceObjectType       *pDevObj      = pDevCtx->pDevObj;
    VpDeviceIdType              deviceId      = pDevObj->deviceId;

    uint16                      timeIn5mS     = 0;
    uint8                       seqByte, index;

    if (pLineObj->status & VP890_IS_FXO) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendFwdDisc() - Function invalid for FXO"));
        return VP_STATUS_INVALID_ARG;
    }

    cadenceState = ConvertApiState2PrfWizState(currentState);

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = 0x0000;    /* No active status */

    for (seqByte = 0; seqByte < VP890_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Set the cadence type */
    pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_FWD_DISC_INT;

    /* First step is to go to disconnect */
    index = 0;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_PROFILE_CADENCE_STATE_DISCONNECT;

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
} /* SendFwdDisc() */

/******************************************************************************
 * SendPolRevPulse()
 *  This function sends a polarity reversal pulse to the line specified for a
 * duration given in ms.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  A polarity reversal has been applied to the line, the line state is restored
 * to what it was prior to this function being called. A hook event may be
 * generated while the polarity reversal is occuring. The application should
 * ignore as appropriate based on expected loop conditions.
 ******************************************************************************/
static VpStatusType
SendPolRevPulse(
    VpLineCtxType               *pLineCtx,
    uint16                      timeInMs)
{
    Vp890LineObjectType         *pLineObj     = pLineCtx->pLineObj;
    VpLineStateType             currentState  = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState, polRevState;

    VpDevCtxType                *pDevCtx      = pLineCtx->pDevCtx;
    Vp890DeviceObjectType       *pDevObj      = pDevCtx->pDevObj;
    VpDeviceIdType              deviceId      = pDevObj->deviceId;

    uint16                      timeIn5mS     = 0;
    uint8                       seqByte, index;

    if (pLineObj->status & VP890_IS_FXO) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SendPolRevPulse() - Function invalid for FXO"));
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

    for (seqByte = 0; seqByte < VP890_INT_SEQ_LEN; seqByte++) {
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
} /* SendPolRevPulse() */

/******************************************************************************
 * MomentaryLoopOpen()
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
 ******************************************************************************/
static VpStatusType
MomentaryLoopOpen(
    VpLineCtxType               *pLineCtx)
{
    Vp890LineObjectType         *pLineObj    = pLineCtx->pLineObj;
    VpLineStateType             currentState = pLineObj->lineState.usrCurrent;
    VpProfileCadencerStateTypes cadenceState;

    VpDevCtxType                *pDevCtx     = pLineCtx->pDevCtx;
    Vp890DeviceObjectType       *pDevObj     = pDevCtx->pDevObj;
    VpDeviceIdType              deviceId     = pDevObj->deviceId;

    uint16                      timeIn5mS    = 0;
    uint8                       seqByte, index;

    if (!(pLineObj->status & VP890_IS_FXO)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("MomentaryLoopOpen() - Function invalid for FXS"));
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
            VP_ERROR(VpLineCtxType, pLineCtx, ("MomentaryLoopOpen() - Invalid state"));
            return VP_STATUS_FAILURE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * This is implemented with the cadencer so we have to stop all previous
     * sequences first
     */
    pLineObj->cadence.status = 0x0000;    /* No active status */

    for (seqByte = 0; seqByte < VP890_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Set the cadence type */
    pLineObj->intSequence[VP_PROFILE_TYPE_LSB] =
        VP_PRFWZ_PROFILE_MOMENTARY_LOOP_OPEN_INT;

    /* First step is to go to Loop Open */
    index = 0;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_LINE_STATE);

    index++;
    pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START + index]
        = VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN;

    /* Then wait for at least 320ms. The time is higher by 2*ApiTick value
       Afterward, when the cadence ends, CommandInstruction will read the LIU
       bit and generate an event if necessary */
    timeIn5mS = 64;  /* Cadencer Tick is 5ms increments */

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
} /* MomentaryLoopOpen() */

/******************************************************************************
 * SendDigit()
 *  This function sends a DTMF or Dial Pulse digit on an FXO line. It creates
 * a sequencer compatible profile to control the FXO loop open, loop close, and
 * time operators.
 *
 * Arguments:
 *  *pLineCtx - Line to send a digit on
 *  digitType - Type of digit to send. May indicate DTMF, Dial Pulse,
 *               or Hook Flash
 *  digit     - The digit to send. Used if type of digit is DTMF or Dial Pulse
 *
 * Preconditions:
 *  The line must first be initialized and must be of FXO type.
 *
 * Postconditions:
 *  The digit specified is sent on the line in the form specified (DTMF or Dial
 * Pulse).  This function returns the success code if the line is an FXO type of
 * line, if the digit is between 0 - 9, and if the digit type is either DTMF or
 * Dial Pulse.
 ******************************************************************************/
static VpStatusType
SendDigit(
    VpLineCtxType           *pLineCtx,
    VpDigitGenerationType   digitType,
    VpDigitType             digit)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    uint16                  tempTime, firstTimer, secondTimer;
    uint8                   seqByte;
    uint16                  tickAdjustment;

    switch(digitType) {
        case VP_DIGIT_GENERATION_DTMF:
        case VP_DIGIT_GENERATION_DIAL_PULSE:
            if (!(pLineObj->status & VP890_IS_FXO)) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Function invalid for FXS"));
                return VP_STATUS_INVALID_ARG;
            }
            if ((VpIsDigit(digit) == FALSE) || (digit == VP_DIG_NONE)) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Invalid digit"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_DIGIT_GENERATION_DIAL_HOOK_FLASH:
            if (!(pLineObj->status & VP890_IS_FXO)) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Function invalid for FXS"));
                return VP_STATUS_INVALID_ARG;
            }

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

    for (seqByte = 0; seqByte < VP890_INT_SEQ_LEN; seqByte++) {
        pLineObj->intSequence[seqByte] = 0x00;
    }

    /* Tick adjustment in 5ms cadence time units */
    tickAdjustment = TICKS_TO_MS(1, pDevObj->devProfileData.tickRate) / 5;

    switch(digitType) {
        case VP_DIGIT_GENERATION_DTMF:
            Vp890MuteChannel(pLineCtx, TRUE);
            Vp890SetDTMFGenerators(pLineCtx, VP_CID_NO_CHANGE, digit);

            /* Fixed total length and sequence length for FLASH generation */
            pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x0C;
            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = 0x08;

            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_START]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_SIGGEN);

            pLineObj->intSequence[12]
                = (VP_SEQ_SPRCMD_COMMAND_INSTRUCTION | VP_SEQ_SUBCMD_SIGGEN);

            pLineObj->intSequence[9] =
                (VP_SEQ_SIGGEN_C_EN | VP_SEQ_SIGGEN_D_EN);

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
            pLineObj->intSequence[VP_PROFILE_LENGTH] = 0x0C;
            pLineObj->intSequence[VP_PROFILE_TYPE_SEQUENCER_COUNT_LSB] = 0x08;

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

            secondTimer = pLineObj->digitGenStruct.dpInterDigitTime;
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
                VP_PRFWZ_PROFILE_HOOK_FLASH_DIG_GEN;
            break;

        default:
            /*
             * This can only occur if there is an error in the error checking
             * above.
             */
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_ERROR(VpLineCtxType, pLineCtx, ("SendDigit() - Invalid digitType; should not get here"));
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
} /* SendDigit() */

/*******************************************************************************
 * Vp890SendCid()
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
 ******************************************************************************/
VpStatusType
Vp890SendCid(
    VpLineCtxType           *pLineCtx,
    uint8                   length,
    VpProfilePtrType        pCidProfile,
    uint8p                  pCidData)
{
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpCallerIdType          *pCid       = &pLineObj->callerId;

    VpProfilePtrType        pCidProfileLocal;

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->status.state, TRUE)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP890_IS_FXO) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendCid() - FXO does not support CID"));
        return VP_STATUS_INVALID_ARG;
    }

    if (length > (2 * VP_SIZEOF_CID_MSG_BUFFER)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendCid() - length exceeds internal msg buffer"));
        return VP_STATUS_INVALID_ARG;
    }

    if (length == 0) {
        return VP_STATUS_SUCCESS;
    }

    /* Check the legality of the Ring CID profile */
    if (!Vp890IsProfileValid(VP_PROFILE_CID,
        VP_CSLAC_CALLERID_PROF_TABLE_SIZE,
        pDevObj->profEntry.cidCadProfEntry,
        pDevObj->devProfileTable.pCallerIdProfileTable,
        pCidProfile, &pCidProfileLocal))
    {
        return VP_STATUS_ERR_PROFILE;
    }

    /* Can't send a null profile */
    if (pCidProfileLocal == VP_PTABLE_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SendCid() - NULL CID profile is not allowed"));
        return VP_STATUS_ERR_PROFILE;
    }

    /* If we're here, all parameters passed are valid */
    VpSysEnterCritical(pDevObj->deviceId, VP_CODE_CRITICAL_SEC);

    pCid->pCliProfile = pCidProfileLocal;
    pCid->cliIndex = 0;
    pCid->cliMPIndex = 0;
    pCid->cliMSIndex = 0;

    pCid->status |= VP_CID_IN_PROGRESS;
    pCid->status &= ~VP_CID_SIG_B_VALID;
    pCid->status &= ~VP_CID_REPEAT_MSG;
    pCid->status &= ~VP_CID_END_OF_MSG;
    pCid->cliTimer = 1;
    pCid->cidCheckSum = 0;

    /* load up the internal CID buffers */
    Vp890LoadCidBuffers(length, pCid, pCidData);

    pCid->status |= VP_CID_PRIMARY_IN_USE;
    pCid->status &= ~VP_CID_SECONDARY_IN_USE;

    VpSysExitCritical(pDevObj->deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;

} /* Vp890SendCid() */

/*******************************************************************************
 * Vp890ContinueCid()
 *  This function is called to provide more caller ID data (in response to
 *  Caller ID data event from the VP-API). See VP-API-II  documentation
 *  for more information about this function.
 *
 *  When this function is called, the buffer that is in use is flagged
 *  by the VpCliGetEncodeByte() function in vp_api_common.c file. That
 *  function implements the logic of when to switch between the primary
 *  and secondary buffer. This function just needs to fill the bufffer that
 *  is not currently in use, starting with the primary (because the primary
 *  buffer is also used first for the first part of the message).
 *
 * Arguments:
 *  *pLineCtx   -
 *  length      -
 *  pCidData    -
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 *  devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Continues to transmit Caller ID information on the line.
 ******************************************************************************/
VpStatusType
Vp890ContinueCid(
    VpLineCtxType           *pLineCtx,
    uint8                   length,
    uint8p                  pCidData)
{
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpCallerIdType          *pCid       = &pLineObj->callerId;

    uint8                   byteCount   = 0;
    uint8                   *pBuffer;

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->status.state, TRUE)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP890_IS_FXO) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890ContinueCid() - FXO does not support CID"));
        return VP_STATUS_INVALID_ARG;
    }

    if (length > (VP_SIZEOF_CID_MSG_BUFFER)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890ContinueCid() - length exceeds internal msg buffer"));
        return VP_STATUS_INVALID_ARG;
    }

    if (length == 0) {
        return VP_STATUS_SUCCESS;
    }

    VpSysEnterCritical(pDevObj->deviceId, VP_CODE_CRITICAL_SEC);

    if (!(pLineObj->callerId.status & VP_CID_PRIMARY_IN_USE)) {
        /* Fill the primary buffer */
        pCid->status |= VP_CID_PRIMARY_FULL;
        pCid->primaryMsgLen = length;
        pBuffer = &(pCid->primaryBuffer[0]);
    } else {
        /* Fill the secondary buffer */
        pCid->status |= VP_CID_SECONDARY_FULL;
        pCid->secondaryMsgLen = length;
        pBuffer = &(pCid->secondaryBuffer[0]);
    }

    /* Copy the message data to the API buffer */
    for (byteCount = 0; (byteCount < length); byteCount++) {
        pBuffer[byteCount] = pCidData[byteCount];

        pCid->cidCheckSum += pBuffer[byteCount];
        pCid->cidCheckSum = pCid->cidCheckSum % 256;
    }
    VpSysExitCritical(pDevObj->deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
} /* Vp890ContinueCid() */

#endif /* CSLAC_SEQ_EN */
#endif
