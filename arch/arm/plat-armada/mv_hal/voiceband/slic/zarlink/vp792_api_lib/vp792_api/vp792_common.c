/** \file vp792_common.c
 * vp792_common.c
 *
 * This file contains support functions used by other functions in the
 * VP792_API library.  If the function appears in this file, it is called from
 * at least two other modules in the VP792_API library.
 *
 * The functions herein are used internally by the API and should not be
 * called directly by the application.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 5264 $
 * $LastChangedDate: 2009-07-06 10:22:54 -0500 (Mon, 06 Jul 2009) $
 */

#include "vp_api.h"

#ifdef VP_CC_792_SERIES  /* Compile only if required */

#define VP_HAL_DEVICE_TYPE VP_DEV_792_SERIES

#include "vp_api_int.h"
#include "vp792_api_int.h"
#include "vp_debug.h"


/* =================================
    Prototypes for Static Functions
   ================================= */

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC_INT)
static void
DisplayMailbox(
    VpDevCtxType *pDevCtx,
    uint16 *pBuf,
    uint16 *pBufEnd);
#endif

static VpStatusType
EnableFilters(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pAcProfile,
    uint8 channelId);

static void
FinishCommand(
    uint16 *pBuf,
    uint16 *pBufEnd,
    bool lastCmd);

static uint16
GetFskChecksumWord(
    Vp792LineObjectType *pLineObj);

static uint16
GetFskDataWord(
    Vp792LineObjectType *pLineObj);

static uint16
GetFskPreambleWord(
    Vp792LineObjectType *pLineObj);

static void
PackBytes(
    uint8p pBytes,
    int numBytes,
    uint16p pWordBuf);

static void
ReadProfileTable(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    int idx,
    VpProfilePtrType *ppProfile);


/* =================
    Local Constants
   ================= */

/* Lookup table for maximum profile size for each type of profile: */
static const uint8 maxProfSize[VP792_NUM_PROFILE_TYPES] = {
    /* Order corresponds to VpProfileType */
    VP792_DEV_PROFILE_LEN,
    VP792_AC_PROFILE_LEN,
    VP792_DC_PROFILE_LEN,
    VP792_RING_PROFILE_LEN,
    VP792_RINGCAD_PROFILE_LEN,
    VP792_TONE_PROFILE_LEN,
    VP792_MTR_PROFILE_LEN,
    VP792_CID_PROFILE_LEN,
    VP792_TONECAD_PROFILE_LEN
};

/* Lookup table for number of profiles of each type in the Profile Table: */
static const uint8 profTableSize[VP792_NUM_PROFILE_TYPES] = {
    /* Order corresponds to VpProfileType */
    VP_792_DEV_PROF_TABLE_SIZE,
    VP_792_AC_PROF_TABLE_SIZE,
    VP_792_DC_PROF_TABLE_SIZE,
    VP_792_RINGING_PROF_TABLE_SIZE,
    VP_792_RING_CADENCE_PROF_TABLE_SIZE,
    VP_792_TONE_PROF_TABLE_SIZE,
    VP_792_METERING_PROF_TABLE_SIZE,
    VP_792_CALLERID_PROF_TABLE_SIZE,
    VP_792_TONE_CADENCE_PROF_TABLE_SIZE
};

/* ===================
    SUPPORT FUNCTIONS
   =================== */

/******************************************************************************
 * Vp792CmdMailboxAcquire()
 * This function attempts to aquires the device mailbox flag from the device
 * specified.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_MAILBOX_BUSY
 ******************************************************************************/
VpStatusType
Vp792CmdMailboxAcquire(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status;
    uint16 flags;

#if (VP792_MAILBOX_SPINWAIT > 0)
    uint32 i;
    for (i = VP792_MAILBOX_SPINWAIT; i; i--) {
#endif

        /* Read mailbox flags register. */
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
                VP792_REG_MB_FLAG_LEN, &flags);
        if (VP_STATUS_SUCCESS != status) {
            return status;
        }

        /* Mailbox flag: 0 => host owns it; 1=> VP792 owns it. */
        if ((flags & VP792_REG_MB_FLAG_CMD_MBOX_MASK) == 0) {
            return VP_STATUS_SUCCESS;
        } else if ((flags & VP792_REG_MB_FLAG_RSP_MBOX_MASK) == 0) {
            uint16 *pCacheData = pDevObj->respMboxCache.data[pDevObj->respMboxCache.count];
            uint16 mbLen;

            /* To do: Check Colby's new congestion flag here. */

            /* We need to empty the response mailbox before we can issue more
               commands. */
            if (pDevObj->respMboxCache.count >= 1) {
                /* The cache is already full; nothing to do but wait. */
                VP_ERROR(VpDevCtxType, pDevCtx, ("Command mailbox busy and cache full (= %u)!", (unsigned)pDevObj->respMboxCache.count));
                continue;
            }

            /* Read mailbox length. */
            status = Vp792HbiPagedRead(pDevCtx, VP792_HBI_RESPONSE_MBOX_PAGE,
                VP792_MBOX_LENGTH_INDEX, 1, &mbLen);
            if (status != VP_STATUS_SUCCESS) {
                return status;
            }

            /* Read the response from the mailbox. */
            pCacheData[VP792_MBOX_LENGTH_INDEX] = mbLen;
            status = Vp792RspMailboxRead(pDevCtx, (uint8)(VP792_MBOX_PAYLOAD_INDEX + mbLen),
                pCacheData);
            if (status != VP_STATUS_SUCCESS) {
                return status;
            }

            pDevObj->respMboxCache.count++;
        }

#if (VP792_MAILBOX_SPINWAIT > 0)
        if (i % (VP792_MAILBOX_SPINWAIT / 10) == 0) {
            VP_WARNING(VpDevCtxType, pDevCtx, ("Waiting for Vp792 command mailbox"));
        }
    }
#endif

    return VP_STATUS_MAILBOX_BUSY;
} /* Vp792CmdMailboxAcquire() */

/******************************************************************************
 * Vp792CmdMailboxRelease()
 * This function releases the device mailbox flag to the device specified.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792CmdMailboxRelease(
    VpDevCtxType *pDevCtx)
{
    /* Release control of the mailbox to the VP792. */
    uint16 flag = VP792_REG_MB_FLAG_CMD_MBOX_MASK;
    return Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
        VP792_REG_MB_FLAG_LEN, &flag);
} /* Vp792CmdMailboxRelease() */

/******************************************************************************
 * This function processes the supplied AC, DC, and Ring profiles.  The
 * sectionType argument specifies what types of sections in the profiles
 * will be processed.
 *
 * If sectionType is VP792_PROF_SECTYPE_ANY, all sections in the profiles will
 * be processed.
 *
 * If sectionType is VP792_PROF_SECTYPE_MBOXCMD, mailbox commands will be
 * extracted from all mailbox sections in the profiles and added to the
 * supplied mailbox buffer (pBuf, ppBufEnd).
 *
 * If sectionType is VP792_PROF_SECTYPE_REGLIST, register settings will be
 * extracted from all register list sections in the profiles and sent to the
 * specified device (pDevCtx), with channel-specific register settings applied
 * to the specified channel (channelId).
 *
 * If sectionType is VP792_PROF_SECTYPE_NONE, the profiles will only be
 * checked for validity -- no other processing will be done.
 *
 * \param[in] pDevCtx       Device context for VP792 device
 * \param[in] channelId     Channel ID for channel-specific register settings
 * \param[in] ppAcProfile   AC profile to be processed, or VP_NULL
 * \param[in] ppDcProfile   DC profile to be processed, or VP_NULL
 * \param[in] ppRingProfile Ring profile to be processed, or VP_NULL
 * \param[in] pBuf          Pointer to start of mailbox buffer
 * \param[in] ppBufEnd      Pointer to calling function's pBufEnd (mailbox end)
 *                          variable
 * \param[in] sectionType   Type(s) of profile section(s) to be processed
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_PROFILE
 * \retval ::VP_STATUS_MAILBOX_BUSY
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792ConfigLineInternal(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpProfilePtrType *ppAcProfile,
    VpProfilePtrType *ppDcProfile,
    VpProfilePtrType *ppRingProfile,
    uint16p pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType sectionType)
{
    VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
    VpStatusType status;

    /* Check the validity of the profile arguments. */
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_AC, ppAcProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Invalid AC profile"));
        return status;
    }
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_DC, ppDcProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Invalid DC profile"));
        return status;
    }
    status = Vp792GetProfileArg(pDevCtx, VP_PROFILE_RING, ppRingProfile);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Invalid ringing profile"));
        return status;
    }

    /* If used just to check profile validity, don't send anything. */
    if (sectionType == VP792_PROF_SECTYPE_NONE) {
        return VP_STATUS_SUCCESS;
    }

    /* Process the AC profile (if any). */
    if (*ppAcProfile != VP_PTABLE_NULL) {
        status = Vp792ProfileProcessSections(pDevCtx, channelId, *ppAcProfile, pBuf,
            ppBufEnd, sectionType);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Failed processing type-%d section in AC profile", sectionType));
            return status;
        }
    }
    /* If updating the AC parameters (or if AC profile is NULL) we need to either
       enable or disable user-defined filter settings. */
    if ((sectionType == VP792_PROF_SECTYPE_MBOXCMD) || (sectionType == VP792_PROF_SECTYPE_ANY)) {
        status = EnableFilters(pDevCtx, *ppAcProfile, channelId);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }
    }

    /* Process the DC profile (if any). */
    if (*ppDcProfile != VP_PTABLE_NULL) {
        status = Vp792ProfileProcessSections(pDevCtx, channelId, *ppDcProfile, pBuf,
            ppBufEnd, sectionType);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Failed processing type-%d section in DC profile", sectionType));
            return status;
        }
    }

    /* Process the ring profile (if any). */
    if (*ppRingProfile != VP_PTABLE_NULL) {
        status = Vp792ProfileProcessSections(pDevCtx, channelId, *ppRingProfile, pBuf,
            ppBufEnd, sectionType);
        if (status != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp792ConfigLineInternal(): Failed processing type-%d section in ringing profile", sectionType));
            return status;
        }

        if ((sectionType == VP792_PROF_SECTYPE_REGLIST) || (sectionType == VP792_PROF_SECTYPE_ANY)) {

            /* Overwrote ring exit state.  Re-apply the option value.  We don't
               need to do this for the case channelId == VP_ALL_LINES, because
               that means this function is being called in VpInitLine(), and
               the option values will be applied after the profiles. */
            if (channelId != VP_ALL_LINES) {
                Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
                status = Vp792SetOption(pLineCtx, VP_NULL,
                    VP_OPTION_ID_RING_CNTRL, &pLineObj->options.ringControl);
            }
        }
    }

    return status;
} /* Vp792ConfigLineInternal() */

/******************************************************************************
 * Vp792GenerateUserEvent()
 * This function generates a USER event with the specified ID.
 *
 * \note This function must be called from within an HBI critical section.
 *
 * \param[in] pDevCtx       Device Context of the VP792 device
 * \param[in] userEventId   ID that determines how the event will be handled by
 *                          the API
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_MAILBOX_BUSY
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792GenerateUserEvent(
    VpDevCtxType *pDevCtx,
    uint16 userEventId,
    uint8 channelId)
{
    uint16 mboxBuf[VP792_CMDSIZE_USER_CTL];
    uint16p pBufEnd = mboxBuf;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp792MailboxAddCmdHeader(mboxBuf, &pBufEnd, VP792_CMD_USER_CTL, channelId);
    *pBufEnd++ = userEventId;
    status = Vp792MailboxSend(pDevCtx, mboxBuf, pBufEnd, VP792_CMDSIZE_USER_CTL);

    return status;
} /* Vp792GenerateUserEvent() */

void
Vp792GetDtmfResources(
    VpDevCtxType *pDevCtx,
    VpOptionDtmfModeType *pDtmfMode)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId;

    /* The following fields are useless for his device, since there full DTMF
       detector coverage (one for each line), but we populate them for back-
       ward compatibility. */
    pDtmfMode->dtmfDetectionSetting = (uint32)pDevObj->options.dtmfEnabled;
    VpMemSet(&pDtmfMode->dtmfDetectionEnabled[0], 0, VP_LINE_FLAG_BYTES);
    pDtmfMode->dtmfDetectionEnabled[0] = (uint8)pDevObj->options.dtmfEnabled;
    pDtmfMode->dtmfResourcesRemaining = VP792_MAX_NUM_CHANNELS;
    for (channelId = 0; channelId < VP792_MAX_NUM_CHANNELS; channelId++) {
        if ((1 << channelId) & pDevObj->options.dtmfEnabled) {
            pDtmfMode->dtmfResourcesRemaining--;
        }
    }
}

VpStatusType
Vp792GetFreeRequestIdx(
    VpDevCtxType *pDevCtx,
    uint16 *pRequestIdx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 idx;

    for (idx = 0; idx < VP792_MAX_OUTSTANDING_RESPONSES; idx++) {
        if (pDevObj->responseRequest[idx].outstanding == FALSE) {
            break;
        }
    }

    if (idx == VP792_MAX_OUTSTANDING_RESPONSES) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Too many outstanding response requests.  You must first call VpGetResults() or VpClearResults(), or increase VP792_MAX_OUTSTANDING_RESPONSES."));
        return VP_STATUS_DEVICE_BUSY;
    }

    *pRequestIdx = idx;
    return VP_STATUS_SUCCESS;
}

/******************************************************************************
 * This function checks the validity of a profile passed into the API via
 * a pointer or an index. If an index is passed, the ppProfile argument
 * is updated to point to the indexed profile in the profile table.
 *
 * \param[in] pDevCtx Device Context for the VP792 device
 * \param[in] profType Type of profile to check
 * \param[in,out] ppProfile Pointer to the calling function's profile pointer
 * variable
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792GetProfileArg(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    VpProfilePtrType *ppProfile)
{
    VpProfilePtrType pTableEntry;

    /* Determine whether a pointer or an index was provided: */
    int profIndex = VpGetProfileIndex(*ppProfile);

    /* First case: NULL is a valid profile pointer, in most cases. */
    if (*ppProfile == VP_PTABLE_NULL) {
        return VP_STATUS_SUCCESS;
    }

    /* Second case: User passed in a pointer to a valid buffer. */
    if (profIndex == -1) {
        uint8 length = (*ppProfile)[VP792_PROF_FLDOFFSET_CONTENT_LEN] + VP792_PROF_FLDOFFSET_CONTENT;

        /* Is the input profile of the expected profile type? */
        if (!VpVerifyProfileType(profType, *ppProfile)) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792GetProfileArg() - profile type mismatch"));
            return VP_STATUS_ERR_PROFILE;
        }
        /* Is the input profile of the expected length? */
        if (length > maxProfSize[profType]) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792GetProfileArg() - profile too large! Adjust #defines in vp792_api.h."));
            return VP_STATUS_ERR_PROFILE;
        }
        return VP_STATUS_SUCCESS;
    }

    /* Remaining case: User passed in a profile index. */
    if (profIndex >= profTableSize[profType]) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Does the profile table contain a profile at the requested index? */
    ReadProfileTable(pDevCtx, profType, profIndex, &pTableEntry);
    if (pTableEntry == VP_NULL) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792GetProfileArg() - profile table entry empty"));
        return VP_STATUS_ERR_PROFILE;
    }

    /* Return a pointer to the a profile from the Profile Table. */
    *ppProfile = pTableEntry;
    return VP_STATUS_SUCCESS;
} /* Vp792GetProfileArg() */

/******************************************************************************
 * Vp792DirectPageRead()
 * This function reads the number of specified words, starting at the specified
 * offset into a buffer from the direct page.
 *
 * \param[in] pDevCtx       Device Context of the VP792 device
 * \param[in] offset        Offset of the requested Page to read from
 * \param[in] numWords      Number of words to read starting from the offset
 * \param[out] pBuf         Pointer to the location to store the read data
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792HbiDirectPageRead(
    VpDevCtxType *pDevCtx,
    uint8 offset,
    uint8 numWords,
    uint16p pBuf)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 slacId = pDevObj->slacId;

    uint16 cmdWrd = HBI_DIRECT_READ(slacId, offset, numWords - (uint8)1);

    if (VP_HAL_HBI_READ(deviceId, cmdWrd, (uint8)(numWords - 1), pBuf)) {
        return VP_STATUS_SUCCESS;
    } else {
        return VP_STATUS_ERR_HBI;
    }
} /* Vp792HbiDirectPageRead() */

/******************************************************************************
 * Vp792DirectPageWrite()
 * This function writes the number of specified words, starting at the specified
 * offset from a source buffer from the direct page.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 * \param[in] offset Offset of the requested Page to read from
 * \param[in] numWords Number of words to read starting from the offset
 * \param[in] pBuf Pointer to the location to store the read data
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792HbiDirectPageWrite(
    VpDevCtxType *pDevCtx,
    uint8 offset,
    uint8 numWords,
    uint16p pSrc)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 slacId = pDevObj->slacId;

    uint16 cmdWrd = HBI_DIRECT_WRITE(slacId, offset, numWords - 1);

    if (VP_HAL_HBI_WRITE(deviceId, cmdWrd, (uint8)(numWords - 1), pSrc) == TRUE) {
        return VP_STATUS_SUCCESS;
    } else {
        return VP_STATUS_ERR_HBI;
    }

} /* Vp792HbiDirectPageWrite() */

/******************************************************************************
 * This function sends the specified number of NO-OPs on the HBI bus of the
 * specified VP792 device (pDevCtx).
 *
 * \param[in] pDevCtx  Device Context of the VP792 device(s)
 * \param[in] numWords Number of NO-OP commands to send
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792HbiNoOp(
    VpDevCtxType *pDevCtx,
    uint8 numWords)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint16 data[255];
    int i;

    for (i = (int)numWords; i > 0; ) {
        data[--i] = HBI_NO_OP;
    }

    if (VP_HAL_HBI_WRITE(deviceId, data[0], numWords - 2, &data[1]) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    return VP_STATUS_SUCCESS;
} /* Vp792HbiNoOp() */

/******************************************************************************
 * Vp792PagedRead()
 * This function selects the specified page, reads the number of specified
 * words, starting at the specified offset into a buffer.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 * \param[in] page Page to select
 * \param[in] offset Offset of the requested Page to read from
 * \param[in] numWords Number of words to read starting from the offset
 * \param[out] pBuf Pointer to the location to store the read data
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792HbiPagedRead(
    VpDevCtxType *pDevCtx,
    uint8 page,
    uint8 offset,
    uint8 numWords,
    uint16p pBuf)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 slacId = pDevObj->slacId;
    uint16 cmdWrd;

    /* Select the correct indirect page */
    if (page == VP792_HBI_CODE_LOAD_PAGE) {
        cmdWrd = HBI_SELECT_CL_PAGE(slacId);
    } else {
        cmdWrd = HBI_SELECT_PAGE(slacId, page);
    }

    /* Send the command word. */
    if (VP_HAL_HBI_CMD(deviceId, cmdWrd) == FALSE) {
       return VP_STATUS_ERR_HBI;
    }

    /* Send the indirect page data */
    cmdWrd = HBI_PAGED_READ(offset, numWords - 1);
    if (VP_HAL_HBI_READ(deviceId, cmdWrd, (uint8)(numWords - 1), pBuf) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    return VP_STATUS_SUCCESS;
} /* Vp792HbiPagedRead() */

/******************************************************************************
 * Vp792PagedWrite()
 * This function selects the specified page, writes the number of specified
 * words, starting at the specified offset from a source buffer.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 * \param[in] page Page to select
 * \param[in] offset Offset of the requested Page to read from
 * \param[in] numWords Number of words to read starting from the offset
 * \param[in] pSrc Pointer to the date to write
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792HbiPagedWrite(
    VpDevCtxType *pDevCtx,
    uint8 page,
    uint8 offset,
    uint8 numWords,
    uint16p pSrc)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 slacId = pDevObj->slacId;

    /* Select the correct indirect page */
    uint16 cmdWrd = HBI_SELECT_PAGE(slacId, page);
    if (VP_HAL_HBI_CMD(deviceId, cmdWrd) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    /* Send the indirect page data */
    cmdWrd = HBI_PAGED_WRITE(offset, numWords - (uint8)1);
    if (VP_HAL_HBI_WRITE(deviceId, cmdWrd, (uint8)(numWords - 1), pSrc) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    return VP_STATUS_SUCCESS;
} /* Vp792HbiPagedWrite() */

/******************************************************************************
 * This function sends a shared-interrupt-read command on the HBI bus of the
 * specified VP792 devices (deviceId).  It saves the interrupt indication
 * register into the supplied uint16 array (pBuf).  If no interrupt occurred,
 * the pBuf array will contain 0 in its first element.
 *
 * \param[in]  deviceId Device ID specifying an HBI bus containing VP792 device(s)
 * \param[out] pBuf     Array of two uint16 elements to receive the interrupt
 *                      indication
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792HbiSharedIntRead(
    VpDeviceIdType deviceId,
    uint16p pBuf)
{
    uint16 cmdWrd = HBI_DIRECT_READ(0, VP792_REG_SHARED_INT_OFFSET,
        VP792_REG_SHARED_INT_LEN - 1);

    if (VP_HAL_HBI_READ(deviceId, cmdWrd, 1, pBuf)) {
        return VP_STATUS_SUCCESS;
    } else {
        return VP_STATUS_ERR_HBI;
    }
} /* Vp792HbiSharedIntRead() */

/******************************************************************************
 * This function sends a select-code-load-page command to the specified VP792
 * device (pDevCtx).
 *
 * \param[in]  pDevCtx Device context of a VP792 device
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792HbiSelectCLPage(
    VpDevCtxType *pDevCtx)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 slacId = pDevObj->slacId;

    uint16 cmdWrd = HBI_SELECT_CL_PAGE(slacId);

    if (VP_HAL_HBI_CMD(deviceId, cmdWrd) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    return VP_STATUS_SUCCESS;
} /* Vp792HbiSelectCLPage() */

VpStatusType
Vp792HbiSync(
    VpDevCtxType *pDevCtx)
{
    VpStatusType status = VP_STATUS_ERR_HBI;
    uint16 clBaseReg[2];
    uint8 sanity = 128;
    bool deSync = FALSE;

    /*
     * Ensure that the SLAC doesn't think there is an ongoing HBI command.
     */
    while (sanity--) {
        clBaseReg[0] = 0x1234;
        clBaseReg[1] = 0x5678;
        status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_CL_BASE_OFFSET,
            VP792_REG_CL_BASE_LEN, clBaseReg);
        if (status != VP_STATUS_SUCCESS) {
            break;
        }
        status = Vp792HbiDirectPageRead(pDevCtx, VP792_REG_CL_BASE_OFFSET,
            VP792_REG_CL_BASE_LEN, clBaseReg);
        if (status != VP_STATUS_SUCCESS) {
            break;
        }
        if ((clBaseReg[0] != 0x1234) || (clBaseReg[1] != 0x5600)) {
            /* Desync detected. */
            deSync = TRUE;
            continue;
        }
        break;
    }

    if (!sanity) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("HBI bus failure"));
        return VP_STATUS_ERR_HBI;
    } else if (deSync) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("HBI bus desynchronization detected"));
        /* Probably due to an invalid VpLowLevelCmd() call. */
    }

    return status;
} /* Vp792HbiSync() */

void
Vp792LoadCidData(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;

    /* Copy FSK data into line object buffer. */
    VpMemCpy(pLineObj->callerId.data.buf, pCidData, length);
    pLineObj->callerId.data.head = (uint8)(length % (VP_SIZEOF_CID_MSG_BUFFER * 2));
    pLineObj->callerId.data.tail = 0;
    pLineObj->callerId.data.nonempty = TRUE;
    pLineObj->callerId.fsk.checksum = 0;

    /* Mark both of the device's FSK buffers as empty. */
    pLineObj->callerId.fsk.bufStatus =
        (VP792_FSKEVENT_BUF1_EMPTY | VP792_FSKEVENT_BUF2_EMPTY);
}

VpStatusType
Vp792InitRelayState(
    VpLineCtxType *pLineCtx)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 ioStReg;

    switch (pLineObj->termType) {
        case VP_TERM_FXS_GENERIC:
            ioStReg = VP792_REG_IO_ST_IO_0_CFG_IN | VP792_REG_IO_ST_IO_1_CFG_IN;
            break;
        case VP_TERM_FXS_TI:
            ioStReg = VP792_REG_IO_ST_IO_0_CFG_OUT | VP792_REG_IO_ST_IO_1_CFG_IN | VP792_REG_IO_ST_IO_0_MASK;
            break;
        default:
            return VP_STATUS_ERR_VTD_CODE;
    }

    return Vp792HbiPagedWrite(pLineCtx->pDevCtx, pLineObj->channelId,
        VP792_REG_IO_ST_OFFSET, VP792_REG_IO_ST_LEN, &ioStReg);
} /* Vp792InitRelayState() */

/******************************************************************************
 * This function adds a string of bytes to the command mailbox buffer.
 * If the length is odd, it pads the string with an extra 0 byte.
 *
 * \param[in,out] ppBufEnd Pointer to the calling function's pBufEnd variable
 * \param[in] pData Bytestring to be added to the mailbox buffer.
 * \param[in] length Length of the bytestring (in bytes)
 *****************************************************************************/
void
Vp792MailboxAddBytes(
    uint16p *ppBufEnd,
    uint8 *pData,
    int length)
{
    /* Add bytes to mailbox buffer (with zero padding if length is odd). */
    PackBytes(pData, length, *ppBufEnd);

    /* Update the calling function's pBufEnd pointer. */
    *ppBufEnd += (length + 1) / 2;
} /* Vp792MailboxAddBytes() */

/******************************************************************************
 * This function adds a two-word command header to the specified command
 * mailbox buffer. It leaves a placeholder for the first word (for the 'more
 * commands' and 'payload length' fields) and fills in the second word (the
 * 'command ID' and 'channel ID' fields). It increases *ppBufEnd by 2.
 *
 * \param[in]     pBuf      Mailbox buffer
 * \param[in,out] ppBufEnd  Pointer to the calling function's pBufEnd variable
 * \param[in]     commandId Command ID to be placed into the command header
 * \param[in]     channelId Channel ID to be placed into the command header
 *****************************************************************************/
void
Vp792MailboxAddCmdHeader(
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792MboxCmdId commandId,
    uint8 channelId)
{
    /* Make a local copy of the calling function's pBufEnd pointer. */
    uint16 *pBufEnd = *ppBufEnd;

    if (pBuf != pBufEnd) {
        /* Mailbox is nonempty. Finalize the header of the previous command. */
        FinishCommand(pBuf, pBufEnd, FALSE);
    }

    /* Add header information for the new command. */
    *pBufEnd++ = 0; /* temporary placeholder */
    *pBufEnd++ = (uint16)(((uint16)commandId << 8) | channelId);

    /* Update the calling function's pBufEnd pointer. */
    *ppBufEnd = pBufEnd;
} /* Vp792MailboxAddCmdHeader() */

/******************************************************************************
 * This function sends the contents of the supplied command mailbox buffer
 * to the VP792 and relinquishes control of the command mailbox to the VP792.
 *
 * \note This function must be called from within an HBI critical section.
 *
 * \param[in] pDevCtx    Device Context of the VP792 device
 * \param[in] pBuf       Mailbox buffer
 * \param[in] pBufEnd    Pointer to the end of the mailbox buffer
 * \param[in] maxBufLen  Total size of mailbox buffer (for array overrun check)
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_MAILBOX_BUSY
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792MailboxSend(
    VpDevCtxType *pDevCtx,
    uint16 *pBuf,
    uint16 *pBufEnd,
    uint16 maxBufLen)
{
    VpStatusType status;
    uint8 numWords = (uint8)(pBufEnd - pBuf);

    if (numWords == 0) {
        /* Mailbox buffer is empty.  Nothing to do. */
        return VP_STATUS_SUCCESS;
    } else if (numWords > maxBufLen) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp729MailboxSend(): Mailbox buffer overflow (%d > %d); stack corrupted!", numWords, maxBufLen));
        return VP_STATUS_FAILURE;
    }

    /* Finalize the header of the last command in the mailbox. */
    FinishCommand(pBuf, pBufEnd, TRUE);

    /* Acquire exclusive access to the command mailbox. */
    status = Vp792CmdMailboxAcquire(pDevCtx);
    if (status != VP_STATUS_SUCCESS) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792MailboxSend() - CmdMailboxAcquire Fail"));
        return status;
    }

    /* Dump the mailbox buffer to the VP792's command mailbox. */
    if (Vp792HbiPagedWrite(pDevCtx, VP792_HBI_COMMAND_MBOX_PAGE, 0, numWords,
        pBuf) != VP_STATUS_SUCCESS) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792MailboxSend() - Vp792PagedWrite Fail "));
        return VP_STATUS_ERR_HBI;
    }

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC_INT)
    DisplayMailbox(pDevCtx, pBuf, pBufEnd);
#endif

    /* Release control of the mailbox to the VP792. */
    return Vp792CmdMailboxRelease(pDevCtx);
} /* Vp792MailboxSend() */

/*******************************************************************************
 * Vp792MapLineState()
 * This function maps a given API line state to the corresponding 792 drive
 * state.
 *
 * \param[in] apiState API Line state
 * \param[out] driveState Drive state value which can be written to the DRIVE_ST
 * register or the RTRIP_ST field of the RING register.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_INVALID_ARG Invalid API line state
 ******************************************************************************/
VpStatusType
Vp792MapLineState(
    VpLineStateType apiState,
    uint16p driveState)
{
    switch (apiState) {
        case VP_LINE_ACTIVE:
        case VP_LINE_TALK:
        case VP_LINE_OHT:
            *driveState = VP792_REG_DRIVE_ST_ST_ACTIVE;
            break;
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT_POLREV:
            *driveState = VP792_REG_DRIVE_ST_ST_ACTIVE |
                          VP792_REG_DRIVE_ST_POL_REVERSE;
            break;
        case VP_LINE_STANDBY:
            *driveState = VP792_REG_DRIVE_ST_ST_STANDBY;
            break;
        case VP_LINE_STANDBY_POLREV:
            *driveState = VP792_REG_DRIVE_ST_ST_STANDBY |
                          VP792_REG_DRIVE_ST_POL_REVERSE;
            break;
        case VP_LINE_RINGING:
            *driveState = VP792_REG_DRIVE_ST_ST_RINGING;
            break;
        case VP_LINE_RINGING_POLREV:
            *driveState = VP792_REG_DRIVE_ST_ST_RINGING |
                          VP792_REG_DRIVE_ST_POL_REVERSE;
            break;
        case VP_LINE_TIP_OPEN:
            *driveState = VP792_REG_DRIVE_ST_ST_LEADOPEN;
            break;
        case VP_LINE_RING_OPEN:
            *driveState = VP792_REG_DRIVE_ST_ST_LEADOPEN |
                          VP792_REG_DRIVE_ST_POL_REVERSE;
            break;
        case VP_LINE_DISCONNECT:
        case VP_LINE_PARK:
            *driveState = VP792_REG_DRIVE_ST_ST_DISCONNECT;
            break;

        case VP_LINE_DISABLED:
            *driveState = VP792_REG_DRIVE_ST_ST_DISABLED;
            break;

        case VP_LINE_NULLFEED:
            *driveState = VP792_REG_DRIVE_ST_ST_NULLFEED;
            break;

        case VP_LINE_HOWLER:
            *driveState = VP792_REG_DRIVE_ST_ST_HOWLER;
            break;

        case VP_LINE_TESTING:
            *driveState = VP792_REG_DRIVE_ST_ST_TESTING;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
            break;
    }
    return VP_STATUS_SUCCESS;
} /* Vp792MapLineState() */

/******************************************************************************
 * This function finds a section of the specified type (pSectionType) in the
 * supplied profile (pProfileStart).  If index is 0, it finds the first such
 * section; otherwise, it finds the (index + 1)th instance of such a section.
 *
 * If pSectionType = VP792_PROF_SECTYPE_ANY, the (index + 1)th section is
 * returned, regardless of section type.
 *
 * This function updates the pSectionType variable to indicate the type of
 * section found.  If no section of the requested type was found, pSectionType
 * = VP792_PROF_SECTYPE_NOT_FOUND.
 *
 * This function updates the ppContent parameter, pointing to the profile
 * section content, and the pLength parameter, indicating the length of the
 * profile content.
 *
 * \param[in]     pProfileStart Profile to be searched
 * \param[in,out] pSectionType  Profile type requested/found
 * \param[in]     index         Instance of the section type to find
 * \param[out]    ppContent     Profile section content pointer
 * \param[out]    pLength       Profile section content length
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792ProfileFindSection(
    const uint8 *pProfileStart,
    Vp792ProfileSectionType *pSectionType,
    uint8 index,
    uint8p *ppContent,
    uint8p pLength)
{
    /* Local temp copies of caller's variables: */
    uint8p pProfile = (uint8p)pProfileStart;
    Vp792ProfileSectionType sectionType = *pSectionType;

    /* Read profile header. */
    uint8 numSections = pProfile[VP792_PROF_FLDOFFSET_NUM_SECS];
    int profContentLen = pProfile[VP792_PROF_FLDOFFSET_CONTENT_LEN];

    /* Go to the start of the profile content. */
    pProfile = &pProfile[VP792_PROF_FLDOFFSET_CONTENT];
    /* Examine each section in the profile. */
    while (numSections-- > 0) {

        /* Get total section length. */
        uint8 sectionLen = (uint8)(VP792_PROF_SEC_FLDOFFSET_CONTENT +
            pProfile[VP792_PROF_SEC_FLDOFFSET_CONTENT_LEN]);

        /* Check for invalid section header to prevent buffer overrun. */
        if (sectionLen > profContentLen) {
            VP_ERROR(None, VP_NULL, ("Vp792ProfileFindSection(): Invalid profile length.  sectionLen %d, profContentLen %d", sectionLen, profContentLen));
            return VP_STATUS_ERR_PROFILE;
        }

        /* Check the section type of the current section. */
        if (
            (sectionType == VP792_PROF_SECTYPE_ANY) ||
            (pProfile[VP792_PROF_SEC_FLDOFFSET_SECTYPE] == sectionType)
        ) {

            /* Section is of the requested type.  Is it of the requested index? */
            if (index-- == 0) {

                /* Found requested section.  Update caller's variables. */
                *pSectionType = (Vp792ProfileSectionType)pProfile[VP792_PROF_SEC_FLDOFFSET_SECTYPE];
                *ppContent = pProfile + VP792_PROF_SEC_FLDOFFSET_CONTENT;
                *pLength = (uint8)(sectionLen - VP792_PROF_SEC_FLDOFFSET_CONTENT);
                /* VP_INFO(None, VP_NULL, ("Vp792ProfileFindSection(): Found type %d section", *pSectionType)); */
                return VP_STATUS_SUCCESS;
            }
        }

        /* Not the section we're looking for.  Go to the next section. */
        pProfile += sectionLen;
        profContentLen -= sectionLen;
    }

    /* The last part of the profile content is unstructured data. */
    if ((*pLength = (uint8)profContentLen) > 0) {
        if (
            ((sectionType == VP792_PROF_SECTYPE_ANY) ||
            (sectionType == VP792_PROF_SECTYPE_UNSTRUCTURED)) && (index == 0)
        ) {

            /* Found requested section.  Update caller's variables. */
            *pSectionType = VP792_PROF_SECTYPE_UNSTRUCTURED;
            *ppContent = pProfile;
            /* VP_INFO(None, VP_NULL, ("Vp792ProfileFindSection(): Found type-%d section", VP792_PROF_SECTYPE_UNSTRUCTURED)); */
            return VP_STATUS_SUCCESS;
        }
    }

    /* Didn't find the requested section. */
    /* VP_INFO(None, VP_NULL, ("Vp792ProfileFindSection(): Section not found")); */
    *pSectionType = VP792_PROF_SECTYPE_NOT_FOUND;
    *ppContent = VP_NULL;
    return VP_STATUS_SUCCESS;
} /* Vp792ProfileFindSection() */

/******************************************************************************
 * This function processes a mailbox command profile section by extracting the
 * mailbox command and arguments from the profile section, and adding them to
 * the specified mailbox buffer (pBuf).  After doing so, it updates the
 * ppBufEnd variable to reflect the new mailbox buffer content length.
 *
 * \param[in]     pDevCtx       Device to send the mailbox command to
 * \param[in]     channelId     Channel to apply the mailbox command to
 * \param[in]     pSequenceSec  Sequencer program section data
 * \param[in]     lengthBytes   Length of the sequencer program section data
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792ProfileProcessMboxCmd(
    uint8 channelId,
    uint8p pMboxCmdSec,
    int lengthBytes,
    uint16 *pBuf,
    uint16p *ppBufEnd)
{
    Vp792MboxCmdId commandId =
        (Vp792MboxCmdId)pMboxCmdSec[VP792_PROF_MBOXCMD_FLDOFFSET_CMDID];
    uint8p pData =
        &pMboxCmdSec[VP792_PROF_MBOXCMD_FLDOFFSET_DATA];

    /* Check for buffer overrun. */
    if (lengthBytes == 0) {
        VP_ERROR(None, VP_NULL, ("Vp792ProfileProcessMboxCmd(): Empty mailbox command section"));
        return VP_STATUS_ERR_PROFILE;
    }

    Vp792MailboxAddCmdHeader(pBuf, ppBufEnd, commandId, channelId);
    Vp792MailboxAddBytes(ppBufEnd, pData, lengthBytes -
        (int)VP792_PROF_MBOXCMD_FLDOFFSET_DATA);

    return VP_STATUS_SUCCESS;
} /* Vp792ProfileProcessMboxCmd() */

/******************************************************************************
 * This function processes a register list profile section (pRegListSec).  It
 * sends an HBI command for each register access in the list.
 *
 * If the register access type is VP792_PROF_REGAXSTYPE_CHANNEL, this function
 * sets the specified register of the channel specified in the channelId
 * parameter.  If channelId == VP_ALL_LINES, the specified register is set for
 * all initialized channels in the device.
 *
 * \param[in]     pDevCtx       Device whose registers will be set
 * \param[in]     channelId     Channel whose registers will be set
 * \param[in]     pRegListSec   Pointer to the register list section content
 * \param[in]     lengthBytes   Length in bytes of the pRegListSec buffer
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792ProfileProcessRegList(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8p pRegListSec,
    int lengthBytes)
{

/* Calculate the maximum data size for a single register access: */
#define VP792_PROF_REGAXS_MAX_DATA_BYTES \
    (0xFF - VP792_PROF_FLDOFFSET_CONTENT - VP792_PROF_SEC_FLDOFFSET_CONTENT \
     - VP792_PROF_REGAXS_FLDOFFSET_DATA)
#define VP792_PROF_REGAXS_MAX_DATA_WORDS \
    ((VP792_PROF_REGAXS_MAX_DATA_BYTES + 1) / 2)

    uint16 wordBuf[VP792_PROF_REGAXS_MAX_DATA_WORDS];
    VpStatusType status = VP_STATUS_SUCCESS;

    while (lengthBytes > 0) {

        /* Read access header. */
        Vp792ProfileRegAccessType accessType =
            (Vp792ProfileRegAccessType)pRegListSec[VP792_PROF_REGAXS_FLDOFFSET_AXSTYPE];
        uint8 offset =
            pRegListSec[VP792_PROF_REGAXS_FLDOFFSET_PAGEOFFSET];
        uint8 numWords =
            pRegListSec[VP792_PROF_REGAXS_FLDOFFSET_DATA_LEN];
        uint8 numBytes = (uint8)(numWords * 2);
        uint8p pData =
            &pRegListSec[VP792_PROF_REGAXS_FLDOFFSET_DATA];

        /* Check for buffer overrun. */
        uint8 accessLength = (uint8)(VP792_PROF_REGAXS_FLDOFFSET_DATA + numBytes);
        lengthBytes -= accessLength;
        if (lengthBytes < 0) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792ProfileProcessRegList(): Invalid profile length"));
            return VP_STATUS_ERR_PROFILE;
        }

        /* Copy register data into a word-aligned buffer. */
        PackBytes(pData, numBytes, wordBuf);

        /* Perform register access. */
        switch (accessType) {
            case VP792_PROF_REGAXSTYPE_DIRECT_PAGE:
                status = Vp792HbiDirectPageWrite(pDevCtx, offset, numWords,
                    wordBuf);
                break;

            case VP792_PROF_REGAXSTYPE_CHANNEL:
            case VP792_PROF_REGAXSTYPE_CHANNEL_FXS:
                status = Vp792WriteMultiplePages(pDevCtx, channelId, offset,
                    numWords, wordBuf);
                break;

            default:
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp792ProfileProcessRegList(): Unsupported access type: %d\n", accessType));
                status = VP_STATUS_ERR_PROFILE;
                break;
        }
        if (status != VP_STATUS_SUCCESS) {
            break;
        }

        /* Move to the next register access. */
        pRegListSec += accessLength;
    }

    return status;
} /* Vp792ProfileProcessRegList() */

/******************************************************************************
 * This function processes a single section from the specified profile
 * (pProfile) for the specified device (pDevCtx).  The choice of section is
 * specified by section type (pSectionType) and index.  If index is 0, the
 * section of the specified type is processed; otherwise, the (index + 1)th
 * instance of such a section is processed.
 *
 * If pSectionType = VP792_PROF_SECTYPE_ANY, the (index + 1)th section is
 * processed, regardless of section type.
 *
 * This function updates the pSectionType variable to indicate the type of
 * section processed.  If no section of the requested type was found,
 * pSectionType = VP792_PROF_SECTYPE_NOT_FOUND.
 *
 * The channelId parameter determines which channel in the device the profile
 * section will be applied to.  If channelId == VP_ALL_LINES, the profile
 * section is applied to all initialized channels in the device.
 *
 * If the selected section is of type VP792_PROF_SECTYPE_MBOXCMD, a mailbox
 * command (and arguments) is added to the specified mailbox buffer (pBuf)
 * and the ppBufEnd variable is updated to reflect the new size of the buffer.
 * If the selected section is of another type, pBuf and ppBufEnd are ignored.
 *
 * \param[in]     pDevCtx       Device to apply the profile section to
 * \param[in]     channelId     Channel to apply the profile section to
 * \param[in]     pProfile      The profile data itself
 * \param[in]     pBuf          Mailbox buffer pointer
 * \param[in]     ppBufEnd      Pointer to caller's pBufEnd variable
 * \param[in]     pSectionType  Type of section to be processed
 * \param[in]     index         Index of section to be processed
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792ProfileProcessSection(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    const uint8 *pProfile,
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType *pSectionType,
    uint8 index)
{
    uint8p pData;
    uint8 length;

    /* Find the section of the specified type and index in the supplied profile. */
    VpStatusType status = Vp792ProfileFindSection(pProfile, pSectionType,
        index, &pData, &length);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    /* Process the section. */
    switch (*pSectionType) {
        case VP792_PROF_SECTYPE_REGLIST:
            status = Vp792ProfileProcessRegList(pDevCtx, channelId, pData,
                length);
            break;

        case VP792_PROF_SECTYPE_MBOXCMD:
            status = Vp792ProfileProcessMboxCmd(channelId, pData, length,
                pBuf, ppBufEnd);
            break;

        case VP792_PROF_SECTYPE_SEQUENCE:
            /* No longer applicable.  Used to program the SEQ register, but now
             * we extract the sequence and put it into a mailbox command. */
            status = VP_STATUS_SUCCESS;
            break;

        case VP792_PROF_SECTYPE_NOT_FOUND:
            /* Section not found. */
            status = VP_STATUS_SUCCESS;
            break;

        case VP792_PROF_SECTYPE_UNSTRUCTURED: {

            status = Vp792ProfileProcessUnstructuredData(pDevCtx, channelId,
                pProfile[VP_PROFILE_TYPE_LSB], pData, length);
            break;
        }

        default:
            /* Not supported. */
            status = VP_STATUS_ERR_PROFILE;
            break;
    }

    return status;
} /* Vp792ProfileProcessSection() */

/******************************************************************************
 * This function processes all sections from the specified profile (pProfile)
 * for the specified device (pDevCtx), of the specified type (sectionType).
 * If sectionType = VP792_PROF_SECTYPE_ANY, all sections are processed,
 * regardless of section type.
 *
 * This function calls Vp792ProfileProcessSection().  Please see the comments
 * for that function for more details about the parameters of this function.
 *
 * \param[in]     pDevCtx       Device to apply the profile sections to
 * \param[in]     channelId     Channel to apply the profile sections to
 * \param[in]     pProfile      The profile data itself
 * \param[in]     pBuf          Mailbox buffer pointer
 * \param[in]     ppBufEnd      Pointer to caller's pBufEnd variable
 * \param[in]     sectionType   Type of sections to be processed
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_ERR_PROFILE
 *****************************************************************************/
VpStatusType
Vp792ProfileProcessSections(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    const uint8 *pProfile,
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType sectionType)
{
    uint8 index = 0;
    VpStatusType status;

    do {
        Vp792ProfileSectionType foundSecType = sectionType;

        /* Find the next section of the specified type in the supplied profile. */
        status = Vp792ProfileProcessSection(pDevCtx, channelId, pProfile, pBuf,
            ppBufEnd, &foundSecType, index++);
        if (status != VP_STATUS_SUCCESS) {
            return status;
        }

        if (foundSecType == VP792_PROF_SECTYPE_NOT_FOUND) {
            /* No more sections of the specified type. */
            return VP_STATUS_SUCCESS;
        }
    } while (1);

} /* Vp792ProfileProcessSections() */

/******************************************************************************
 * This function processes the 'unstructured data' section (specified in the
 * pData and length args) of a profile.  The profileType argument specifies
 * the type of the profile, and thus the format of the data.
 *
 * This function currently does not support channelId = VP_ALL_LINES.
 *
 * \param[in]  pDevCtx      Device to be configured
 * \param[in]  channelId    Channel to be configured
 * \param[in]  profileType  Type of the profile containing the pData buffer
 * \param[in]  pData        Pointer to the unstructured data section
 * \param[in]  length       Length of the unstructured data section (bytes)
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_PROFILE
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792ProfileProcessUnstructuredData(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 profileType,
    uint8p pData,
    uint8 length)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (profileType) {

        /* Tone profile contains a two-byte value in its "unstructured data"
           section that gets programmed into the first word of the SIG_GEN
           register.  It specifies which signal generators are used in the
           tone profile. */
        case VP792_PROFTYPE_TONE: {
            uint16 sigGenCtl;

            /* Prevent seg. faults */
            if (length < 2) {
                return VP_STATUS_ERR_PROFILE;
            }

            /* Update the SIG_GEN register. */
            PackBytes(pData, 2, &sigGenCtl);
            status = Vp792HbiPagedWrite(pDevCtx, channelId,
                VP792_REG_SIG_GEN_OFFSET + VP792_REG_SIG_GEN_CTL_WORD,
                1, &sigGenCtl);
            break;
        }

        /* Caller ID profile contains two two-byte fields in its "unstructured
           data" section.  The first word is the duration (in bit periods) of
           the Channel Seizure period in the FSK data string.  The second word
           is the duration (in bit periods) of the Mark period. */
        case VP792_PROFTYPE_CID: {
            Vp792LineObjectType *pLineObj = pDevCtx->pLineCtx[channelId]->pLineObj;

            /* Prevent seg. faults */
            if (length < 4) {
                return VP_STATUS_ERR_PROFILE;
            }

            /* Save values into line object for later use. */
            PackBytes(&pData[0], 2, &pLineObj->callerId.fsk.seizureBits);
            PackBytes(&pData[2], 2, &pLineObj->callerId.fsk.markBits);
            pLineObj->callerId.fsk.autoChecksum = pData[6];
            pLineObj->callerId.isFsk = !pData[7];
            break;
        }

        /* AC profile contains 1 byte of unstructured data which should always
           be either 0x00 or 0x10 to determine the LRG bit of the VP_CFG2
           register. */
        case VP792_PROFTYPE_AC: {
            uint16 oldvalue, newValue;

            status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
                VP792_REG_VP_CFG2_LEN, &oldvalue);
            if (status != VP_STATUS_SUCCESS) {
                break;
            }
            newValue = (oldvalue & ~VP792_REG_VP_CFG2_LRG_MASK) | *pData;
            if (newValue != oldvalue) {
                status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
                    VP792_REG_VP_CFG2_LEN, &newValue);
            }
            break;
        }

        default:
            /* Nothing to be done for this profile type. */
            break;
    }

    return status;
}  /* Vp792ProfileProcessUnstructuredData() */

void
Vp792ResetLineVars(
    VpLineCtxType *pLineCtx)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;

    pLineObj->lineInit = FALSE;
    pLineObj->eventHandlers = VP792_EH_NONE;
    pLineObj->sequencer.activeSequence = VP792_SEQTYPE_OFF;
    pLineObj->sequencer.aborting = FALSE;
    pLineObj->sendSig.msgWait.readingDcParams = FALSE;
    pLineObj->sendSig.msgWait.aborting = FALSE;
    pLineObj->dtmfDigitDetected = VP_DIG_SENSE_BREAK;
    pLineObj->currentState = VP_LINE_DISCONNECT;

    /* Other option values will be set when the DEV_INIT_CMP or LINE_INIT_CMP
       event arrives, but the DTMF option won't, because the USER event will
       be masked. */
    pLineObj->options.dtmfControlMode = VP_OPTION_DTMF_DECODE_OFF;
}

VpStatusType
Vp792RspMailboxRead(
    VpDevCtxType *pDevCtx,
    uint8 mailboxLen,
    uint16 *pBuf)
{
    uint16 flag;

    /* Read the response from the mailbox, skipping the length field. */
    VpStatusType status = Vp792HbiPagedRead(pDevCtx, VP792_HBI_RESPONSE_MBOX_PAGE,
        VP792_MBOX_ID_CHAN_INDEX, mailboxLen - VP792_MBOX_ID_CHAN_INDEX,
        &pBuf[VP792_MBOX_ID_CHAN_INDEX]);
    if (status |= VP_STATUS_SUCCESS) {
        return status;
    }

    /* Release the SLAC's response mailbox. */
    flag = VP792_REG_MB_FLAG_RSP_MBOX_MASK;
    status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
                VP792_REG_MB_FLAG_LEN, &flag);

    return status;
}

/******************************************************************************
 * Vp792RspMailboxRelease()
 * This function releases the response mailbox flag to the device specified.
 *
 * \param[in] pDevCtx Device Context of the VP792 device
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792RspMailboxRelease(
    VpDevCtxType *pDevCtx)
{
    VpStatusType status = VP_STATUS_SUCCESS;
    Vp792DeviceObjectType * pDevObj = pDevCtx->pDevObj;

    /* Handle cached response, if any. */
    if (pDevObj->respMboxCache.count > 0) {

        /* Remove the response at the head of the cache. */
        pDevObj->respMboxCache.count--;
        if (pDevObj->respMboxCache.count > 0) {
            VpMemCpy(pDevObj->respMboxCache.data[0], pDevObj->respMboxCache.data[1],
                     sizeof(pDevObj->respMboxCache.data));
        }
    }

    /* Otherwise, read the response mailbox. */
    else {

        /* Release control of the mailbox to the VP792. */
        uint16 flag = VP792_REG_MB_FLAG_RSP_MBOX_MASK;
        status = Vp792HbiDirectPageWrite(pDevCtx, VP792_REG_MB_FLAG_OFFSET,
            VP792_REG_MB_FLAG_LEN, &flag);
    }
    return status;
} /* Vp792RspMailboxRelease() */

void
Vp792SaveProfile(
    VpProfilePtrType pSource,
    uint8 *pDest,
    uint16 *pValidFlag,
    uint16 bit)
{
    uint8 length;

    if (pSource == VP_NULL) {
        *pValidFlag &= ~(1 << bit);
        return;
    }

    length = pSource[VP792_PROF_FLDOFFSET_CONTENT_LEN] + VP792_PROF_FLDOFFSET_CONTENT;
    VpMemCpy(pDest, (char *)pSource, length);
    *pValidFlag |= (1 << bit);
} /* Vp792SaveProfile() */

void
Vp792SaveSequence(
    VpLineCtxType *pLineCtx,
    uint8 *pSequence,
    bool endian)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint8 lenBytes;
    uint16 temp;

    /* We will fill the sequencer program buffer in the line object. */
    uint16 *buf = pLineObj->sequencer.program, *pBufEnd = buf;

    /* Extract the sequence length from the control instruction. */
    if (endian) {
        ((uint8 *)(&temp))[0] = pSequence[0];
        ((uint8 *)(&temp))[1] = pSequence[1];
    } else {
        temp = pSequence[1];
    }

    lenBytes = (temp + 1) * 2;

    /* Insert the CTL = START field. */
    temp |= VP792_SEQ_CTL_START;
    if (endian) {
        pSequence[0] = ((uint8 *)(&temp))[0];
        pSequence[1] = ((uint8 *)(&temp))[1];
    } else {
        pSequence[0] = temp >> 8;
        pSequence[1] = temp & 0xFF;
    }

    /* Add command to mailbox buffer. */
    Vp792MailboxAddCmdHeader(buf, &pBufEnd, VP792_CMD_WR_SEQ_CTL, channelId);

    /* Add the instructions to the mailbox buffer (skipping the first two bytes
       of the program, which are always ignored) in the host processor's
       endianness. */
    if (endian) {
        VpMemCpy(pBufEnd, pSequence, lenBytes + 2);
    } else {
        Vp792MailboxAddBytes(&pBufEnd, pSequence, lenBytes + 2);
    }
} /* Vp792SaveSequence() */

/******************************************************************************
 * This function sends the next six bytes of FSK data to the device.  The data
 * may be channel seizure bits, mark bits, or message data, depending on the
 * state of the callerId struct in the line object.
 *
 * \param[in]     pLineCtx      Line context for line on which to transmit
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_FAILURE  The device is not currently transmitting FSK
 * \retval ::VP_STATUS_ERR_HBI
 *****************************************************************************/
VpStatusType
Vp792SendFskData(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    uint16 fskBuf[VP792_CID_BUFSIZE];
    uint16 *pFskBuf = fskBuf;
    uint8 words = VP792_CID_BUFSIZE;
    uint8 offset;

    /* Decide which buffer needs to be filled next. */
    if (pLineObj->callerId.fsk.bufStatus & VP792_FSKEVENT_BUF1_EMPTY) {
        offset = VP792_REG_FSK_BUF_OFFSET;
        pLineObj->callerId.fsk.bufStatus &= ~VP792_FSKEVENT_BUF1_EMPTY;
    } else if (pLineObj->callerId.fsk.bufStatus & VP792_FSKEVENT_BUF2_EMPTY) {
        offset = VP792_REG_FSK_BUF_OFFSET + VP792_CID_BUFSIZE;
        pLineObj->callerId.fsk.bufStatus &= ~VP792_FSKEVENT_BUF2_EMPTY;
    } else {
        /* This function was somehow called when both FSK buffers are full. */
        return VP_STATUS_FAILURE;
    }

    /* FSK message consists of preamble, followed by data. */
    while (words) {
        uint16 word;

        words--;

        /* Get next data word from line object. */
        if (pLineObj->callerId.fsk.seizureBits || pLineObj->callerId.fsk.markBits) {
            word = GetFskPreambleWord(pLineObj);
        } else if (pLineObj->callerId.data.nonempty) {
            word = GetFskDataWord(pLineObj);
        } else if (pLineObj->callerId.fsk.autoChecksum) {
            word = GetFskChecksumWord(pLineObj);
        } else {
            /* No more FSK data to send! */
            return VP_STATUS_SUCCESS;
        }

        /* If last word, exit the loop. */
        *pFskBuf++ = word;
        if (word & VP792_CID_CONTROL_LAST_BYTE)
            break;
    }

    /* Copy the data to the device's FSK buffer. */
    return Vp792HbiPagedWrite(pDevCtx, channelId, offset,
        (uint8)(VP792_CID_BUFSIZE - words), fskBuf);
}

/*******************************************************************************
 * SetOptionDtmfMode()
 * This function implements the DTMF_MODE option.
 *
 * \param[in] pDevCtx Pointer to the Device Context identifying the device to
 * apply the option to.
 * \param[in] pDeviceIo Pointer to the VpOptionLineIoConfigType data structure.
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792SetOptionLineDtmfMode(
    VpLineCtxType *pLineCtx,
    VpOptionDtmfModeType *pDtmfMode)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    if (pDtmfMode->direction != VP_DIRECTION_US) {
        return VP_STATUS_INVALID_ARG;
    }

    switch (pDtmfMode->dtmfControlMode) {
        case VP_OPTION_DTMF_DECODE_OFF:

            /* The SLAC's DTMF detector is always on.  If a digit is still
               ongoing, we may have to generate a fake "break" event.  To
               avoid race conditions, we will request a user event and
               wait until we get the event before updating
               pLineObj->options.dtmfControlMode. */
            status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_DTMF_DISABLE, channelId);
            pDevObj->options.dtmfEnabled &= ~(1 << channelId);
            break;
        case VP_OPTION_DTMF_DECODE_ON:

            /* The SLAC's DTMF detector is always on.  If a digit is already
               ongoing, we may have to generate a fake "make" event.  To
               avoid race conditions, we will request a user event and
               wait until we get the event before updating
               pLineObj->options.dtmfControlMode. */
            status = Vp792GenerateUserEvent(pDevCtx, VP792_UEVID_DTMF_ENABLE, channelId);
            pDevObj->options.dtmfEnabled |= (1 << channelId);
            break;
        case VP_OPTION_DTMF_GET_STATUS:
            /* This feature isn't needed for this device, since there is a DTMF
               detector for each line. */
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    Vp792GetDtmfResources(pDevCtx, pDtmfMode);
    return VP_STATUS_SUCCESS;
}

/*******************************************************************************
 * Vp792SetPcmTxRxMode()
 * This function sets 'Cut off Receive Path' and 'Cut off Transmit Path' as
 * appropriate for the given state and the PCM_TXRX_CNTRL option setting.
 * Both paths are cut off for non-talk states, and the option setting is applied
 * for talk states.
 *
 * \note This function must be called from within an HBI critical section.
 *
 * \param[in] pLineCtx Pointer to the Line Context for the line to modify
 * \param[in] state API Line State used to determine correct PCM mode
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_INVALID_ARG Invalid state or PCM mode
 * \retval ::VP_STATUS_ERR_HBI
 ******************************************************************************/
VpStatusType
Vp792SetPcmTxRxMode(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channelId = pLineObj->channelId;
    uint16 oldValue, newValue;

    status = Vp792HbiPagedRead(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
        VP792_REG_VP_CFG2_LEN, &oldValue);
    newValue = oldValue;
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    switch (pLineObj->options.pcmTxRxCntrl) {
        case VP_OPTION_PCM_BOTH:
            newValue &= ~VP792_REG_VP_CFG2_CTP_MASK;
            newValue &= ~VP792_REG_VP_CFG2_CRP_MASK;
            break;
        case VP_OPTION_PCM_TX_ONLY:
            newValue &= ~VP792_REG_VP_CFG2_CTP_MASK;
            newValue |= VP792_REG_VP_CFG2_CRP_MASK;
            break;
        case VP_OPTION_PCM_RX_ONLY:
            newValue |= VP792_REG_VP_CFG2_CTP_MASK;
            newValue &= ~VP792_REG_VP_CFG2_CRP_MASK;
            break;
        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    switch(state) {
        /* Non-Talk States */
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_RING_OPEN:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
        case VP_LINE_HOWLER:
        case VP_LINE_TESTING:
        case VP_LINE_DISABLED:
        case VP_LINE_PARK:

            newValue |= VP792_REG_VP_CFG2_CTP_MASK;
            newValue |= VP792_REG_VP_CFG2_CRP_MASK;
            break;

        /* Talk States */
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
        case VP_LINE_NULLFEED:
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }
    if ((status != VP_STATUS_SUCCESS) || (newValue == oldValue)) {
        return status;
    }

    status = Vp792HbiPagedWrite(pDevCtx, channelId, VP792_REG_VP_CFG2_OFFSET,
        VP792_REG_VP_CFG2_LEN, &newValue);
    return status;
} /* Vp792SetPcmTxRxMode() */

VpStatusType
Vp792StartSavedSequence(
    VpLineCtxType *pLineCtx,
    uint16 useType)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp792LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 *buf = pLineObj->sequencer.program;
    uint16 programSize = (buf[VP792_MBOX_PAYLOAD_INDEX] & VP792_SEQ_CTL_LENGTH_MASK) + 1;
    uint16 *pBufEnd = &buf[VP792_MBOX_PAYLOAD_INDEX + 1 /* CTL field */ + programSize];
    VpStatusType status;

    VP_ASSERT(pLineObj->sequencer.activeSequence == VP792_SEQTYPE_OFF);

    /* Send the mailbox command to the SLAC. */
    status = Vp792MailboxSend(pDevCtx, buf, pBufEnd, VP792_CMDSIZE_WR_SEQ_CTL);

    if (status == VP_STATUS_SUCCESS) {
        pLineObj->sequencer.activeSequence = useType;
    }

    return status;
} /* Vp792StartSavedSequence() */

/******************************************************************************
 * This function calls Vp792HbiPagedWrite() on the specified channelId.  If
 * channelId = VP_ALL_LINES, it calls Vp792HbiPagedWrite() once for each
 * initialized channel in the device.
 * specified line (pLineCtx).
 *
 * \param[in]  pDevCtx      Device to perform the register write on
 * \param[in]  channelId    Channel(s) to perform the register write on
 * \param[in]  offset       Offset parameter for Vp792HbiPagedWrite()
 * \param[in]  numWords     Numwords parameter for Vp792HbiPagedWrite()
 * \param[in]  pData        pData parameter for Vp792HbiPagedWrite()
 *
 * \retval ::VP_STATUS_SUCCESS
 * \retval ::VP_STATUS_ERR_HBI
 * \retval ::VP_STATUS_MAILBOX_BUSY
 *****************************************************************************/
VpStatusType
Vp792WriteMultiplePages(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 offset,
    uint8 numWords,
    uint16p pData)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 page, lastPage;
    VpStatusType status = VP_STATUS_SUCCESS;

    /* Determine which channel page(s) this operation applies to. */
    if (channelId == VP_ALL_LINES) {
        page = 0;
        lastPage = pDevObj->maxChannels - 1;
    } else {
        page = lastPage = channelId;
    }

    /* Iterate through the selected channel page(s). */
    while (page <= lastPage) {

        /* Make sure the line context for this channel has been created. */
        if (pDevCtx->pLineCtx[page] == VP_NULL) {
            page++;
            continue;
        }

        /* Perform the requested operation, increment to the next page. */
        status = Vp792HbiPagedWrite(pDevCtx, page++, offset, numWords, pData);
        if (status != VP_STATUS_SUCCESS) {
            break;
        }
    }

    return status;
} /* Vp792WriteMultiplePages() */

VpStatusType
Vp792WriteProfileTable(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    int idx,
    VpProfilePtrType pProfile)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    if (idx >= profTableSize[profType]) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Profile table index out of range.  Adjust #defines in vp792_api.h."));
        return VP_STATUS_INVALID_ARG;
    }

#ifdef VP_COMMON_ADDRESS_SPACE
    /* In this mode we can simply save the profile pointer into the device
       object. */

    switch (profType) {
        case VP_PROFILE_DEVICE:
            pDevObj->profTable.pDevice[idx] = pProfile;
            break;
        case VP_PROFILE_AC:
            pDevObj->profTable.pAc[idx] = pProfile;
            break;
        case VP_PROFILE_DC:
            pDevObj->profTable.pDc[idx] = pProfile;
            break;
        case VP_PROFILE_RING:
            pDevObj->profTable.pRinging[idx] = pProfile;
            break;
        case VP_PROFILE_RINGCAD:
            pDevObj->profTable.pRingCad[idx] = pProfile;
            break;
        case VP_PROFILE_TONE:
            pDevObj->profTable.pTone[idx] = pProfile;
            break;
        case VP_PROFILE_METER:
            pDevObj->profTable.pMetering[idx] = pProfile;
            break;
        case VP_PROFILE_CID:
            pDevObj->profTable.pCallerId[idx] = pProfile;
            break;
        case VP_PROFILE_TONECAD:
            pDevObj->profTable.pToneCad[idx] = pProfile;
            break;
        default: /* can't happen */
            break;
    }
#else /* !VP_COMMON_ADDRESS_SPACE */
    /* In this mode we have to copy the profile content into the device
       object. */

    {
        uint8 *pTableEntry = VP_NULL;

        switch (profType) {
            case VP_PROFILE_DEVICE:
                pTableEntry = pDevObj->profTable.device[idx];
                break;
            case VP_PROFILE_AC:
                pTableEntry = pDevObj->profTable.ac[idx];
                break;
            case VP_PROFILE_DC:
                pTableEntry = pDevObj->profTable.dc[idx];
                break;
            case VP_PROFILE_RING:
                pTableEntry = pDevObj->profTable.ringing[idx];
                break;
            case VP_PROFILE_RINGCAD:
                pTableEntry = pDevObj->profTable.ringCad[idx];
                break;
            case VP_PROFILE_TONE:
                pTableEntry = pDevObj->profTable.tone[idx];
                break;
            case VP_PROFILE_METER:
                pTableEntry = pDevObj->profTable.metering[idx];
                break;
            case VP_PROFILE_CID:
                pTableEntry = pDevObj->profTable.callerId[idx];
                break;
            case VP_PROFILE_TONECAD:
                pTableEntry = pDevObj->profTable.toneCad[idx];
                break;
            default: /* can't happen */
                break;
        }

        Vp792SaveProfile(pProfile, pTableEntry, &pDevObj->profTable.valid[profType], idx);
    }
#endif /* !VP_COMMON_ADDRESS_SPACE */

    return VP_STATUS_SUCCESS;
}


/* ==================
    Static Functions
   ================== */

#if (VP_CC_DEBUG_SELECT & VP_DBG_API_FUNC_INT)
static void
DisplayMailbox(
    VpDevCtxType *pDevCtx,
    uint16 *pBuf,
    uint16 *pBufEnd)
{
    uint16 numWords = (uint16)(pBufEnd - pBuf);
    char outBuf[18 + 5 * 128], *pOutBufEnd = outBuf;

    pOutBufEnd += sprintf(pOutBufEnd, "Mailbox buffer [%d] =", numWords);
    while (numWords--) {
        pOutBufEnd += sprintf(pOutBufEnd, " %4.4X", *(pBuf++));
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("%s", outBuf));
}
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_INFO) */

static VpStatusType
EnableFilters(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pAcProfile,
    uint8 channelId)
{
    uint16 vpCfg1;
    if (pAcProfile != VP_PTABLE_NULL) {
        vpCfg1 = VP792_REG_VP_CFG1_FILTERS_USER;
    } else {
        vpCfg1 = VP792_REG_VP_CFG1_FILTERS_DEFAULT;
    }
    return Vp792WriteMultiplePages(pDevCtx, channelId,
        VP792_REG_VP_CFG1_OFFSET, VP792_REG_VP_CFG1_LEN, &vpCfg1);
}

/******************************************************************************
 * This internal function updates the header information for a command in the
 * command mailbox, after all arguments have been added to the buffer. If
 * 'lastCmd' is false, the 'more commands' field for the last command is set to
 * 1, indicating there will be more commands added to the buffer later.
 *
 * Preconditions:
 * There must be at least one command header in the mailbox buffer.
 * The 'more commands' field for the last command in the mailbox must be 0.
 * Postconditions:
 * The 'length' field of the last command in the mailbox is updated, based on
 * the value of ppBufEnd.
 * The 'more commands' field of the last command in the mailbox is set to the
 * logical negation of 'lastCmd'.
 *
 * \param[in] pBuf     Mailbox buffer
 * \param[in] ppBufEnd Pointer to the calling function's pBufEnd variable
 * \param[in] lastCmd  TRUE if no more commands will be added later
 *****************************************************************************/
static void
FinishCommand(
    uint16 *pBuf,
    uint16 *pBufEnd,
    bool lastCmd)
{
    uint8 moreCmds;
    uint8 length;

    /* Skip to the last command in the mailbox. */
    do {
        moreCmds = (uint8)(*pBuf >> 8);
        length = (uint8)(*pBuf & 0xFF);
        pBuf = &pBuf[2 + length];
    } while (moreCmds == VP792_MBOX_MORE_COMMANDS);

    /* Calculate header information for last command. */
    moreCmds = (uint8)(lastCmd ? VP792_MBOX_LAST_COMMAND : VP792_MBOX_MORE_COMMANDS);
    length = (uint8)(pBufEnd - pBuf);

    /* Update header fields in buffer. */
    *(pBuf - 2) = (uint16)(((uint16)moreCmds << 8) | length);
} /* FinishCommand() */

static uint16
GetFskChecksumWord(
    Vp792LineObjectType *pLineObj)
{
    uint8 checksum = (uint8)(~pLineObj->callerId.fsk.checksum + 1);
    uint16 word = (uint16)(VP792_CID_CONTROL_LAST_BYTE | checksum);
    pLineObj->callerId.fsk.autoChecksum = FALSE;
    return word;
}

/******************************************************************************
 * This function gets the next FSK data byte from the callerId buffer in the
 * line object, maintains the running checksum, and sets the LAST_BYTE bit if
 * there are no more data bytes remaining.
 *
 * \param[in]     pLineObj      Line object for line on which to transmit
 *
 * \returns a data word to be written into the FSK_BUFx register in the device
 *****************************************************************************/
static uint16
GetFskDataWord(
    Vp792LineObjectType *pLineObj)
{
    uint16 word;

    /* Get byte from circular buffer, increment tail pointer. */
    uint8 tail = pLineObj->callerId.data.tail;
    word = pLineObj->callerId.data.buf[tail++];
    pLineObj->callerId.data.tail = (uint8)(tail % (VP_SIZEOF_CID_MSG_BUFFER * 2));

    /* Update running checksum caculation. */
    pLineObj->callerId.fsk.checksum += word;

    /* Check for buffer-empty condition. */
    if (pLineObj->callerId.data.tail == pLineObj->callerId.data.head) {
        pLineObj->callerId.data.nonempty = FALSE;
        if (pLineObj->callerId.fsk.autoChecksum == FALSE) {
            word |= VP792_CID_CONTROL_LAST_BYTE;
        }
    }

    return word;
}

/******************************************************************************
 * When in the "Channel Seizure" or "Mark" region of a Caller ID FSK string,
 * this function returns the next data byte to be transmitted, formatted as
 * a word to be written into the device's FSK_BUFx register.
 *
 * \param[in]     pLineObj      Line object for line on which to transmit
 *
 * \returns a data word to be written into the FSK_BUFx register in the device
 *****************************************************************************/
static uint16
GetFskPreambleWord(
    Vp792LineObjectType *pLineObj)
{
    int16 seizureBits = (int16)(pLineObj->callerId.fsk.seizureBits);
    uint8 bitsWanted = 8;

    /* Disable framing bits during FSK preamble. */
    uint16 word = VP792_CID_CONTROL_NO_FRAME;

    /* Channel Seizure (010101...01): */
    if (seizureBits > 0) {
        word |= 0x55;
        if (seizureBits > 8) {
            seizureBits = 8;
        }
        bitsWanted -= seizureBits;
        pLineObj->callerId.fsk.seizureBits -= seizureBits;
    }

    /* Mark (111111...11): */
    if (bitsWanted > 0) {
        uint16 markBits = pLineObj->callerId.fsk.markBits;
        if (bitsWanted > markBits) {
            bitsWanted = (uint8)markBits;
        }
        word |= ((1 << bitsWanted) - 1);
        pLineObj->callerId.fsk.markBits -= bitsWanted;
    }

    return word;
}

/******************************************************************************
 * This internal function packs an array of bytes (pBytes) into an array of
 * uint16s (pWordBuf).  To match the endianness of the VP792 device, it puts
 * the first of each pair of bytes into the high byte, and the second into the
 * low byte.
 *
 * If numBytes is odd, the low byte of the last uint16 element is set to 0.
 *
 * \param[in]     pBytes        Source buffer
 * \param[in]     numBytes      Length of source buffer
 * \param[in]     pWordBuf      Destination buffer
 *****************************************************************************/
static void
PackBytes(
    uint8p pBytes,
    int numBytes,
    uint16p pWordBuf)
{
    uint16 byte0, byte1;

    while (numBytes > 0) {
        byte0 = *(pBytes++);

        /* If length is odd, add zero byte padding. */
        if (numBytes == 1) {
            byte1 = 0;
        } else {
            byte1 = *(pBytes++);
        }

        /* VP792 device is big-endian. Pack bytes accordingly. */
        *pWordBuf++ = (uint16)((byte0 << 8) | byte1);

        numBytes -= 2;
    }
} /* PackBytes() */

static void
ReadProfileTable(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    int idx,
    VpProfilePtrType *ppProfile)
{
    Vp792DeviceObjectType *pDevObj = pDevCtx->pDevObj;

#ifdef VP_COMMON_ADDRESS_SPACE
    switch (profType) {
        case VP_PROFILE_DEVICE:
            *ppProfile = pDevObj->profTable.pDevice[idx];
            break;
        case VP_PROFILE_AC:
            *ppProfile = pDevObj->profTable.pAc[idx];
            break;
        case VP_PROFILE_DC:
            *ppProfile = pDevObj->profTable.pDc[idx];
            break;
        case VP_PROFILE_RING:
            *ppProfile = pDevObj->profTable.pRinging[idx];
            break;
        case VP_PROFILE_RINGCAD:
            *ppProfile = pDevObj->profTable.pRingCad[idx];
            break;
        case VP_PROFILE_TONE:
            *ppProfile = pDevObj->profTable.pTone[idx];
            break;
        case VP_PROFILE_METER:
            *ppProfile = pDevObj->profTable.pMetering[idx];
            break;
        case VP_PROFILE_CID:
            *ppProfile = pDevObj->profTable.pCallerId[idx];
            break;
        case VP_PROFILE_TONECAD:
            *ppProfile = pDevObj->profTable.pToneCad[idx];
            break;
        default: /* can't happen */
            break;
    }
#else /* !VP_COMMON_ADDRESS_SPACE */
    uint16 bit = (1 << idx);

    if ((pDevObj->profTable.valid[profType] & bit) == 0) {
        *ppProfile = VP_NULL;
        return;
    }

    switch (profType) {
        case VP_PROFILE_DEVICE:
            *ppProfile = pDevObj->profTable.device[idx];
            break;
        case VP_PROFILE_AC:
            *ppProfile = pDevObj->profTable.ac[idx];
            break;
        case VP_PROFILE_DC:
            *ppProfile = pDevObj->profTable.dc[idx];
            break;
        case VP_PROFILE_RING:
            *ppProfile = pDevObj->profTable.ringing[idx];
            break;
        case VP_PROFILE_RINGCAD:
            *ppProfile = pDevObj->profTable.ringCad[idx];
            break;
        case VP_PROFILE_TONE:
            *ppProfile = pDevObj->profTable.tone[idx];
            break;
        case VP_PROFILE_METER:
            *ppProfile = pDevObj->profTable.metering[idx];
            break;
        case VP_PROFILE_CID:
            *ppProfile = pDevObj->profTable.callerId[idx];
            break;
        case VP_PROFILE_TONECAD:
            *ppProfile = pDevObj->profTable.toneCad[idx];
            break;
        default: /* can't happen */
            break;
    }
#endif /* !VP_COMMON_ADDRESS_SPACE */

}

#endif /* VP_CC_792_SERIES */
