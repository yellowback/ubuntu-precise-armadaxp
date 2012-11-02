/** \file vp792_init.c
 * vp792_init.c
 *
 *  This file contains the implementation of the VP-API 792 Series
 *  Initialization Functions.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 7004 $
 * $LastChangedDate: 2010-04-27 17:31:09 -0500 (Tue, 27 Apr 2010) $
 */
#include "vp_api.h"

#if defined (VP_CC_792_SERIES)  /* Compile only if required */

#include "vp_api_int.h"
#include "vp792_api_int.h"


/* ==============
    Local Macros
   ============== */

#define VP792_IMAGE_BLOCKS_SIZE_WORDS 64


/* =================================
    Prototypes for Static Functions
   ================================= */

static VpStatusType
CalPrepare(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

static VpStatusType
CheckSlacBusy(
    VpDevCtxType *pDevCtx,
    bool *pBusy);

static VpStatusType
LoadPatch(
    VpDevCtxType *pDevCtx,
    uint16 *pImage,
    uint16 blocks);


/* ==========================
    INITIALIZATION FUNCTIONS
   ========================== */

VpStatusType
Vp792CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    bool busy = FALSE;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    if (mode == VP_DEV_CAL_NBUSY) {
        status = CheckSlacBusy(pDevCtx, &busy);

        if ((status == VP_STATUS_SUCCESS) && busy) {
            /* A sequence is running, and/or the DRIVE_ST is not Disabled for
               one or more channels.  Generate VP_EVID_CAL_BUSY event. */
            status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_CAL_BUSY, 0);
        }
    }

    if (!busy && (status == VP_STATUS_SUCCESS)) {

        /* Don't let any other functions run during calibration. */
        pDevObj->devInit = FALSE;

        /* Call CalPrepare(), which will set all lines to Disabled, wait 1 ms
           and generate a VP792_EVID_SEQ event.  We know the DRIVE_ST is
           already Disabled for all channels.  This is just in case the
           DRIVE_ST was changed less than 1 ms ago on one or more channels. */
        status = CalPrepare(pDevCtx, VP_ALL_LINES);
        if (status == VP_STATUS_SUCCESS) {

            /* Enable the event handler, which will send the CAL_CTL command
               when the VP792_EVID_SEQ event is received from the SLAC.  The
               same event handler will generate the VP_EVID_CAL_CMP event when
               the VP792_EVID_CAL_CMP event is received. */
            pDevObj->eventHandlers |= VP792_EH_CAL_CODEC;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792CalCodec() */

VpStatusType
Vp792CalLine(
    VpLineCtxType *pLineCtx)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status = VP_STATUS_SUCCESS;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    if (pLineObj->eventHandlers & VP792_EH_CAL_LINE) {

        /* Calibration is already in progress; generate VP_EVID_CAL_BUSY. */
        status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_CAL_BUSY, channelId);
    } else {

        /* Call CalPrepare(), which will set the drive state to Disabled,
           wait 1 ms and generate a VP792_EVID_SEQ event. */
        status = CalPrepare(pDevCtx, channelId);
        if (status == VP_STATUS_SUCCESS) {

            /* Enable the event handler, which will send the CAL_CTL command
               and when the VP792_EVID_SEQ event is received from the SLAC.  The
               same event handler will generate the VP_EVID_CAL_CMP event when
               the VP792_EVID_CAL_CMP event is received. */
            pLineObj->eventHandlers |= VP792_EH_CAL_LINE;
        }
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792CalLine() */

VpStatusType
Vp792ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE];
    uint16p pBufEnd = buf;
    VpStatusType status;

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Process the AC, DC, and ringing profiles: */
    status = Vp792ConfigLineInternal(pDevCtx, channelId, &pAcProfile, &pDcProfile,
        &pRingProfile, buf, &pBufEnd, VP792_PROF_SECTYPE_ANY);

    /* Send the command(s) (if any) to the VP792 device. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;

} /* Vp792ConfigLine() */

VpStatusType
Vp792InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

#ifdef VP_COMMON_ADDRESS_SPACE
    VpProfilePtrType pCidProfile = pLineObj->profiles.pCallerId;
#else
    VpProfilePtrType pCidProfile = (pLineObj->profiles.valid & VP_PROFILE_CID) ?
        pLineObj->profiles.callerId : VP_NULL;
#endif

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check arguments. */
    if ((length > (VP_SIZEOF_CID_MSG_BUFFER * 2)) || (length == 0)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitCid(): %s.", "Data length out of range"));
        return VP_STATUS_INVALID_ARG;
    }
    if (pCidData == VP_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitCid(): %s.", "Data pointer must not be NULL"));
        return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Load CID data into the line object. */
    Vp792LoadCidData(pLineCtx, length, pCidData);

    /* If VpInitRing() has not been called since the last time we sent the
       Caller ID message, we need to reinitialize the preamble bits in the line
       object. */
    if (pCidProfile != VP_NULL) {
        status = Vp792ProfileProcessSections(pDevCtx, channelId,
            pCidProfile, VP_NULL, VP_NULL, VP792_PROF_SECTYPE_UNSTRUCTURED);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792InitCid() */

VpStatusType
Vp792InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status;
    uint16 sysStat = 0;
    int i;
    uint16 *pImage;
    uint32 length;

    /* Check the validity of the profile arguments. */
    if (pDevProfile == VP_PTABLE_NULL) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792InitDevice(): Device profile must be non-NULL"));
        return VP_STATUS_INVALID_ARG;
    }
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_DEVICE, &pDevProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792InitDevice(): Invalid device profile"));
        return status;
    }
    status = Vp792ConfigLineInternal(pDevCtx, 0, &pAcProfile, &pDcProfile, &pRingProfile,
        VP_NULL, VP_NULL, VP792_PROF_SECTYPE_NONE);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Initialize device object fields. */
    /*pDevObj->maxChannels = 0;*/
    pDevObj->rev.device = 0;
    pDevObj->rev.product = 0;
    pDevObj->rev.version = 0;
    pDevObj->rev.patchAddress = 0;
    pDevObj->devInit = FALSE;
    pDevObj->pcmClockRate = 0xFFFF;
    pDevObj->timeStampHiBits = 0;
    pDevObj->options.eventMask.response &= ~VP_DEV_EVID_DEV_INIT_CMP;
    pDevObj->options.dtmfEnabled = 0;
    pDevObj->options.pulseEnabled = 0;
    pDevObj->options.debugSelect = VP_OPTION_DEFAULT_DEBUG_SELECT;
    pDevObj->intInd[0] = VP792_REG_INTIND_NO_EVENT;
    for (i = 0; i < VP792_MAX_OUTSTANDING_RESPONSES; i++) {
        pDevObj->responseRequest[i].outstanding = FALSE;
    }
    pDevObj->requestIdx = VP792_MAX_OUTSTANDING_RESPONSES;
    pDevObj->eventHandlers = VP792_EH_NONE;

    /* Save profiles for Vp792InitDeviceInt(). */
#ifdef VP_COMMON_ADDRESS_SPACE
    pDevObj->ehInitDevice.pDevProfile = pDevProfile;
    pDevObj->ehInitDevice.pAcProfile = pAcProfile;
    pDevObj->ehInitDevice.pDcProfile = pDcProfile;
    pDevObj->ehInitDevice.pRingProfile = pRingProfile;
#else
    Vp792SaveProfile(pDevProfile, pDevObj->ehInitDevice.devProfile,
        &pDevObj->ehInitDevice.profilesValid, VP_PROFILE_DEVICE);
    Vp792SaveProfile(pAcProfile, pDevObj->ehInitDevice.acProfile,
        &pDevObj->ehInitDevice.profilesValid, VP_PROFILE_AC);
    Vp792SaveProfile(pDcProfile, pDevObj->ehInitDevice.dcProfile,
        &pDevObj->ehInitDevice.profilesValid, VP_PROFILE_DC);
    Vp792SaveProfile(pRingProfile, pDevObj->ehInitDevice.ringProfile,
        &pDevObj->ehInitDevice.profilesValid, VP_PROFILE_RING);
#endif

    /* Run customer HAL initialization function. */
    if (VpHalHbiInit(deviceId) == FALSE) {
        status = VP_STATUS_ERR_HBI;
    }

    /* Clear the SLAC's HBI state machine. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792HbiSync(pDevCtx);
    }

    /* Reset the device. */
    if (status == VP_STATUS_SUCCESS) {
        uint16 clkStatus = VP792_REG_CLK_STATUS_HW_RST_MASK;
        status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_CLK_STATUS_OFFSET,
                    VP792_REG_CLK_STATUS_LEN, &clkStatus);
    }

    /* Maximum DCLK rate is 8.192 MHz.  400 ms is a very conservative spin-wait
       time.  Maximum theoretical HBI data transfer during that period is 204800
       words.  Each loop iteration sends one command word and reads one data word. */
#define VP792_SYS_RESET_WAIT_ITERATIONS 102400

    /* Poll the SYS_STAT register for SYS_RESET indication. */
    i = 0;
    while ((status == VP_STATUS_SUCCESS) && (i++ < VP792_SYS_RESET_WAIT_ITERATIONS)) {
        const uint16 sysResetMask = (VP792_INTIND_SYS_MASK | VP792_REG_SYS_STAT_SYS_RESET_MASK);
        /* Some SLAC revisions can return 0xFFFF here shortly after reset.
           Therefore we are waiting for the exact value 0x8001, instead of
           ignoring the bits in the middle. */
        if (sysStat == sysResetMask) {
            break;
        }
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_SYS_STAT_OFFSET,
            VP792_REG_SYS_STAT_LEN, &sysStat);
    }

    /* Issue the HBI pin config command. */
    if (VP_HAL_HBI_CMD(deviceId, HBI_CMD_CONFIGURE_INT | HBI_PINCONFIG) == FALSE) {
        status = VP_STATUS_ERR_HBI;
    }

    /* Send the pre-boot portion of the device profile. */
    if (status == VP_STATUS_SUCCESS) {
        Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_REGLIST;
        status = Vp792ProfileProcessSection(pDevCtx, VP_ALL_LINES, pDevProfile,
            VP_NULL, VP_NULL, &sectionType, VP792_DEV_PROF_SEC_PREBOOT);
    }

    /* Poll the SYS_STAT register for SYS_PREBOOT indication. */
    i = 0;
    while ((status == VP_STATUS_SUCCESS) && (i++ < VP792_SYS_RESET_WAIT_ITERATIONS)) {
        const uint16 sysPrebootMask = (VP792_INTIND_SYS_MASK | VP792_REG_SYS_STAT_SYS_PREB);
        if ((sysStat & sysPrebootMask) == sysPrebootMask) {
            break;
        }
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_SYS_STAT_OFFSET,
            VP792_REG_SYS_STAT_LEN, &sysStat);
    }

    /* Select a firmware patch image based on the revision code. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792SelectPatch(pDevCtx, &pImage, &length, &pDevObj->rev.patchAddress);
    }

    /* Load the patch (if any). */
    if ((status == VP_STATUS_SUCCESS) && (length > 0)) {
        status = LoadPatch(pDevCtx, pImage, length / VP792_IMAGE_BLOCKS_SIZE_WORDS);
    }

    /* Kick off the ROMmed SLAC firmware. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792CmdMailboxRelease(pDevCtx);
    }

    /* Return now, and continue when we receive the BOOT_CMP event. */
    if (status == VP_STATUS_SUCCESS) {
        pDevObj->eventHandlers |= VP792_EH_INIT_DEVICE;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
}

VpStatusType
Vp792InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    uint16 buf[VP792_HBI_COMMAND_MBOX_SIZE];
    uint16p pBufEnd = buf;
    VpStatusType status;
    VP_API_INT_ENTER(VpLineCtxType, pLineCtx, "Vp792InitLine");

    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Add channel-reset command to mailbox buffer. */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_RESET_CTL, channelId);

    /* Process AC, DC, and ringing profiles.  Pass VP792_PROF_SECTYPE_MBOXCMD to
     * process the mailbox commands, but ignore the register commands, for now. */
    status = Vp792ConfigLineInternal(pDevCtx, channelId, &pAcProfile, &pDcProfile,
        &pRingProfile, buf, &pBufEnd, VP792_PROF_SECTYPE_MBOXCMD);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Initialize line/device object fields. */
    Vp792ResetLineVars(pLineCtx);
    pDevObj->options.dtmfEnabled &= ~(1 << channelId);
    pDevObj->options.pulseEnabled &= ~(1 << channelId);

#ifdef VP_COMMON_ADDRESS_SPACE
    /* Save profile pointers into the line object so the event handler can
       pass them to Vp792ConfigLine(). */
    pLineObj->profiles.pAc = pAcProfile;
    pLineObj->profiles.pDc = pDcProfile;
    pLineObj->profiles.pRinging = pRingProfile;
#else
    Vp792SaveProfile(pAcProfile, pLineObj->profiles.ac,
        &pLineObj->profiles.valid, VP_PROFILE_AC);
    Vp792SaveProfile(pDcProfile, pLineObj->profiles.dc,
        &pLineObj->profiles.valid, VP_PROFILE_DC);
    Vp792SaveProfile(pRingProfile, pLineObj->profiles.ringing,
        &pLineObj->profiles.valid, VP_PROFILE_RING);
#endif

    /* Send the RESET_CTL and profile commands to the VP792 device. */
    if (status == VP_STATUS_SUCCESS) {
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_HBI_COMMAND_MBOX_SIZE);
    }

    /* Activate an event handler which calls Vp792ConfigLineInternal() and runs
       calibration when the RESET_CMP event arrives. */
    if (status == VP_STATUS_SUCCESS) {
        pLineObj->eventHandlers = VP792_EH_INIT_LINE;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    VP_API_INT_EXIT(VpLineCtxType, pLineCtx, "Vp792InitLine", status);
    return status;
} /* Vp792InitLine() */

VpStatusType
Vp792InitMeter(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pMeterProfile)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 buf[VP792_CMDSIZE_WR_MTR_CTL], *pBufEnd = buf;
    Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_UNSTRUCTURED;
    uint8 length, *pData;
    uint16 mtr_type, ramp_time, ramp_step;
    VpStatusType status;

    /* Make sure device is initialized. */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check profile type and profile size limit. */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_METER, &pMeterProfile);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Extract the metering register settings from the profile. */
    status = Vp792ProfileFindSection(pMeterProfile, &sectionType, 0, &pData,
        &length);

    /* Prevent seg. faults */
    if (length < 2) {
        return VP_STATUS_ERR_PROFILE;
    } else {
        mtr_type = ((uint16)pData[0] << 8) | pData[1];
    }
    if ((length < 6) && (
        (mtr_type == VP792_MTR_TYPE_12KHZ_TONE) ||
        (mtr_type == VP792_MTR_TYPE_16KHZ_TONE)
    )) {
        return VP_STATUS_ERR_PROFILE;
    } else {
        ramp_time = ((uint16)pData[2] << 8) | pData[3];
        ramp_step = ((uint16)pData[4] << 8) | pData[5];
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Send metering register settings to the SLAC. */
    if (status == VP_STATUS_SUCCESS) {
        VpMemSet(buf, 0, sizeof(buf));
        Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_MTR_CTL, channelId);
        *pBufEnd++ = VP792_MTR_CTL_STOP;
        pBufEnd += 7;
        *pBufEnd++ = mtr_type;
        *pBufEnd++ = ramp_time;
        *pBufEnd++ = ramp_step;
        status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_MTR_CTL);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792InitMeter() */

VpStatusType
Vp792InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType profileType,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status;

    /* Check arguments. */
    int profIndex = VpGetProfileIndex(pProfileIndex);
    if ((profIndex == -1) || (VpGetProfileIndex(pProfile) != -1)) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("VP7892InitProfile(): First profile argument must be an index; second profile argument must be a pointer or VP_NULL."));
        return VP_STATUS_INVALID_ARG;
    }

    /* Check profile type and profile size limit. */
    status = Vp792GetProfileArg(pDevCtx, profileType, &pProfile);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Write the new profile into the table. */
    status = Vp792WriteProfileTable(pDevCtx, profileType, profIndex, pProfile);

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
}

VpStatusType
Vp792InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pRingCadProfile,
    VpProfilePtrType pCidProfile)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8p pRingSeq, pCidSeq;
    uint8 length;
    Vp792ProfileSectionType sectionType = VP792_PROF_SECTYPE_SEQUENCE;
    VpStatusType status;

    /* Make sure device is initialized. */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* If a CID profile is specified, a ring cadence profile is required. */
    if ((pRingCadProfile == VP_NULL) && (pCidProfile != VP_NULL)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitRing(): If CID profile is specified, ring cadence profile is required"));
        return VP_STATUS_INVALID_ARG;
    }

    /* Check the validity of the caller ID profile */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_CID, &pCidProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitRing(): Invalid caller ID profile"));
        return status;
    }
    if (pCidProfile != VP_NULL) {
        status = Vp792ProfileFindSection(pCidProfile, &sectionType,
            0, &pCidSeq, &length);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitRing(): Invalid caller ID profile"));
            return status;
        }
    }

    /* Check the validity of the ring cadence profile */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_RINGCAD, &pRingCadProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitRing(): Invalid ring cadence profile"));
        return status;
    }
    if (pRingCadProfile != VP_NULL) {
        status = Vp792ProfileFindSection(pRingCadProfile, &sectionType,
            0, &pRingSeq, &length);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792InitRing(): Invalid ring cadence profile"));
            return status;
        }
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

#ifdef VP_COMMON_ADDRESS_SPACE
    pLineObj->profiles.pRingCad = pRingCadProfile;
    pLineObj->profiles.pCallerId = pCidProfile;
#else
    Vp792SaveProfile(pRingCadProfile, pLineObj->profiles.ringCad,
        &pLineObj->profiles.valid, VP_PROFILE_RINGCAD);
    Vp792SaveProfile(pCidProfile, pLineObj->profiles.callerId,
        &pLineObj->profiles.valid, VP_PROFILE_CID);
#endif

    /* Get channel seizure duration and mark duration from CID profile. */
    if (pCidProfile != VP_NULL) {
        status = Vp792ProfileProcessSections(pDevCtx, channelId,
            pCidProfile, VP_NULL, VP_NULL, VP792_PROF_SECTYPE_UNSTRUCTURED);
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);

    return status;
} /* Vp792InitRing() */

VpStatusType
Vp792SetBatteries(
    VpLineCtxType *pLineCtx,
    VpBatteryModeType battMode,
    VpBatteryValuesType *battVal)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status;
    uint16 buf[VP792_CMDSIZE_WR_BAT_OVR];
    uint16p pBufEnd = buf;

    /* Get out if device state is not ready */
    if (!pDevObj->devInit) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Argument check */
    switch(battMode) {
        case VP_BATT_MODE_DIS:
            break;

        case VP_BATT_MODE_EN:
            if (battVal == VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetBatteries(): VP_NULL invalid for VP_BATT_MODE_EN"));
                return VP_STATUS_INVALID_ARG;
            }
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    VpSysEnterCritical(deviceId, VP_HBI_CRITICAL_SEC);

    /* Send the global WR_BAT_OVR command */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_BAT_OVR, 0);

    /* Battery Override Control */
    *pBufEnd++ = battMode;

    /*  If battMode == DIS, then battVal can be NULL.
     *    This condition is checked above in the argument check.
     *    Of course, if battMode == EN, then battVal must be valid.  */
    if (battVal != VP_NULL) {
        *pBufEnd++ = battVal->batt2;
        *pBufEnd++ = battVal->batt1;
        *pBufEnd++ = battVal->batt3;
    }

    /* Send the command(s) to the VP792 device. */
    status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_BAT_OVR);

    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792SetBatteries(): Failed sending WR_BAT_OVR command"));
        return status;
    }

    VpSysExitCritical(deviceId, VP_HBI_CRITICAL_SEC);
    return status;
} /* Vp792SetBatteries() */


/* ===================
    Static Functions
   =================== */

static VpStatusType
CalPrepare(
    VpDevCtxType *pDevCtx,
    uint8 channelId)
{
    Vp792LineObjectType *pLineObj;
    VpStatusType status;
    uint16 buf[VP792_CMDSIZE_WR_SEQ_CTL], *pBufEnd = buf;

    /* A trivial SLAC sequence that waits for 1 ms and then generates a
       SEQ event: */
    uint16 delaySeq[2] = {
        VP792_SEQ_CTL_START | 0,
        VP792_SEQ_CMD_DELAY | 1
    };

    /* Make sure the drive state for specified line(s) is set to disabled
       before running the CAL_CTL command. */
    uint16 driveSt = VP792_REG_DRIVE_ST_ST_DISABLED;
    status = Vp792WriteMultiplePages(pDevCtx, channelId, VP792_REG_DRIVE_ST_OFFSET,
        VP792_REG_DRIVE_ST_LEN, &driveSt);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* It can take up to 1 ms for the drive state change to take effect.
       Therefore we start a sequence to provide this delay.  The sequence
       will generate a VP792_EVID_SEQ event.  The event handler
       will then send the CAL_CTL command. */

    if (channelId == VP_ALL_LINES) {
        /* In the case of a gobal CAL_CTL command, we will run the sequence
           on channel 0. */
        channelId = 0;
    }

    /* Get the line object for this channel. */
    if (pDevCtx->pLineCtx[channelId] == VP_NULL) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("CalPrepare(): No line context for channel %d!", channelId));
        status = VP_STATUS_FAILURE;
    } else {
        pLineObj = pDevCtx->pLineCtx[channelId]->pLineObj;
        pLineObj->sequencer.activeSequence = VP792_SEQTYPE_CAL;
    }

    /* Add the new sequencer command to the mailbox, and start the sequence. */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);
    *pBufEnd++ = delaySeq[0];
    *pBufEnd++ = delaySeq[1];
    status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_SEQ_CTL);

    /* The caller of this function will now enable the appropriate event
       handler for the expected VP792_EVID_SEQ event. */
    return status;
} /* CalPrepare() */

static VpStatusType
CheckSlacBusy(
    VpDevCtxType *pDevCtx,
    bool *pBusy)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 chan;

    /*  Make sure each line in the 'disabled' state and not running a sequence. */
    for (chan = 0; chan < pDevObj->maxChannels; chan++) {
        VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[chan];
        if (pLineCtx != VP_NULL) {
            Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;

            /* Make sure a sequence is not currently running; such a
               sequence might change the DRIVE_ST register. */
            if (pLineObj->sequencer.activeSequence != VP792_SEQTYPE_OFF) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Channel %d is running a sequence; device not calibrated", (int)chan));
                *pBusy = TRUE;
                break;
            } else if (
                (pLineObj->currentState != VP_LINE_DISABLED) &&
                (pLineObj->currentState != VP_LINE_DISCONNECT)
            ) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Channel %d is not in VP_LINE_DISABLED or VP_LINE_DISCONNECT state; device not calibrated", (int)chan));
                *pBusy = TRUE;
                break;
            }

        } else {

            /* No line context for this channel.  Read the channel registers
               to check the sequencer status and line state. */
            uint16 chanStatus, driveSt;
            status = Vp792HbiPagedRead(pDevCtx, chan, VP792_REG_DRIVE_ST_OFFSET,
                VP792_REG_DRIVE_ST_LEN, &driveSt);
            if (status == VP_STATUS_SUCCESS) {
                status = Vp792HbiPagedRead(pDevCtx, chan, VP792_REG_CH_STAT_OFFSET,
                    VP792_REG_CH_STAT_LEN, &chanStatus);
                if (status != VP_STATUS_SUCCESS) {
                    break;
                }
            }
            if (chanStatus & VP792_REG_CH_STAT_SEQ_MASK) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Vp792CalCodec(): Channel %d is running a sequence; device not calibrated", (int)chan));
                *pBusy = TRUE;
                break;
            } else if (
                ((driveSt & VP792_REG_DRIVE_ST_ST_MASK) != VP792_REG_DRIVE_ST_ST_DISABLED) &&
                ((driveSt & VP792_REG_DRIVE_ST_ST_MASK) != VP792_REG_DRIVE_ST_ST_DISCONNECT)
            ) {
                VP_WARNING(VpDevCtxType, pDevCtx, ("Channel %d is not in VP_LINE_DISABLED or VP_LINE_DISCONNECT state; device not calibrated", (int)chan));
                *pBusy = TRUE;
                break;
            }
        }
    }

    return status;
}

static VpStatusType
LoadPatch(
    VpDevCtxType *pDevCtx,
    uint16 *pImage,
    uint16 blocks)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;

    /* Select the Code Load Page */
    if (Vp792HbiSelectCLPage(pDevCtx) != VP_STATUS_SUCCESS ) {
        return VP_STATUS_ERR_HBI;
    }

    /* Load the image data to the device a block at a time. */
    VP_INFO(VpDevCtxType, pDevCtx, ("VP792 firmware patch size is %u blocks (= %lu bytes)", blocks, (unsigned long)blocks * VP792_IMAGE_BLOCKS_SIZE_WORDS * 2));
    while (blocks > 0) {
        uint16 cmd = *pImage++;

        if (!VP_HAL_HBI_WRITE(deviceId, cmd, VP792_IMAGE_BLOCKS_SIZE_WORDS - 2, pImage)) {
            return VP_STATUS_ERR_HBI;
        }
        pImage += VP792_IMAGE_BLOCKS_SIZE_WORDS - 1;
        blocks--;
    }

    /* Verify the code load checksum. */
    {
        uint16 checksum[2];
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_CL_CRC_OFFSET, VP792_REG_CL_CRC_LEN, checksum);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }
        if ((checksum[0] != 0xAA55) || (checksum[1] != 0xAA55)) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("VP792 firmware patch download failed (0x%4.4X%4.4X != 0xAA55AA55)", checksum[0], checksum[1]));
            return VP_STATUS_ERR_VERIFY;
        }
    }

    return VP_STATUS_SUCCESS;
} /* LoadPatch() */

#endif /* VP_CC_792_SERIES */
