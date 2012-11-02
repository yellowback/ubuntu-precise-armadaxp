/** \file boot_common.c
 * boot_common.c
 *
 * This file contains all of the VP-API-II declarations that are Voice
 * Termination Device (VTD) family independent. The implementation in this file
 * is applicable to VCP classes of devices.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 6419 $
 * $LastChangedDate: 2010-02-12 16:40:10 -0600 (Fri, 12 Feb 2010) $
 */

/* INCLUDES */
#include "vp_api.h"
#include "vp_debug.h"

/* Compile Further only if required */
#if defined (VP_CC_VCP_SERIES) || defined (VP_CC_VCP2_SERIES)

#include "boot_common.h"

#ifndef VP_NO_COMPRESS
#include "zlib.h"
#ifdef VP_COMPRESS_TEST
#ifndef debug_printf
/* Define the following as necessary to redirect the debug output */
#define debug_printf printf
#endif /* debug_printf */
bool veCompressTestEnable = FALSE;
# endif /* VP_COMPRESS_TEST */
#else /* VP_NO_COMPRESS */
# undef VP_COMPRESS_TEST
#endif /* VP_NO_COMPRESS */


#if defined (VP_CC_VCP_SERIES)
/**
 * VpVcpVppBootLoad()
 *  Loads an executable image into the VTD (VCP).
 *
 * Refer to the VoicePath API User's Guide for details about this function.
 */
VpStatusType
VpVcpVppBootLoad(
    VpDevCtxType *pDevCtx,
    VpBootStateType state,
    VpImagePtrType pImageBuffer,
    uint32 bufferSize,
    VpScratchMemType *pScratchMem,
    VpBootModeType validation
    )
{
    void *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId;
    uint16p pRecCnt;

    switch(pDevCtx->deviceType) {
#if defined (VP_CC_VCP_SERIES)
        case VP_DEV_VCP_SERIES:
            deviceId = ((VpVcpDeviceObjectType *)pDevObj)->deviceId;
            pRecCnt = &((VpVcpDeviceObjectType *)pDevObj)->recCnt;
            break;
#endif
        default:
            return VP_STATUS_INVALID_ARG;
    }

    if (pImageBuffer == VP_NULL) {
        /* Flash booting not supported for these VTDs. */
        return VP_STATUS_INVALID_ARG;
    }

    if (VpHbiVcpVppClearCodeMem(deviceId) == FALSE) {
        return VP_STATUS_ERR_HBI;
    }

    /* Pass VCP specific halt and validate functions to VpBootLoadInternal(). */
    return VpBootLoadInternal(state, pImageBuffer, bufferSize, pScratchMem,
        validation, deviceId, pRecCnt, VpHbiVcpVppHalt,
        VpHbiVcpVppValidate);
}
#endif /* VP_CC_VCP_SERIES */

VpStatusType
VpBootLoadInternal(
    VpBootStateType state,
    VpImagePtrType pImageBuffer,
    uint32 bufferSize,
    VpScratchMemType *pScratchMem,
    VpBootModeType validation,
    VpDeviceIdType deviceId,
    uint16p pRecCnt,
    VpBootSupportFuncPtrType haltFunc,
    VpBootSupportFuncPtrType validateFunc)
{
    int newblocks;					/* number of uncompressed blocks available */
    VpImagePtrType pHbiBlock;       /* points to data that is ready to be transmitted */
    bool firstblock;                /* whether we're processing the first
                                       block in the image */
#ifndef VP_NO_COMPRESS
    VpZlibStreamType *stream = &(pScratchMem->stream);  /* data structure for inflate() I/O */
    uint8p pInflated = pScratchMem->inflated;           /* buffer for holding inflated data */
    int retval = Z_OK;
#endif

    VP_INFO(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): Image length: %ld", bufferSize));

    switch (state) {
        case VP_BOOT_STATE_FIRSTLAST:
        case VP_BOOT_STATE_FIRST:
            firstblock = TRUE;
            break;
        case VP_BOOT_STATE_CONTINUE:
        case VP_BOOT_STATE_LAST:
            firstblock = FALSE;
            break;
        default:
            return VP_STATUS_INVALID_ARG;
    }

    /* User passes scratchMemPtr != VP_NULL for compressed images. */
    if (pScratchMem != VP_NULL) {

#ifdef VP_NO_COMPRESS
        /* If compression isn't compiled in, return with error. */
        return VP_STATUS_INVALID_ARG;
#else
        pHbiBlock = (VpImagePtrType)pInflated;

        if ((state == VP_BOOT_STATE_FIRSTLAST) ||
            (state == VP_BOOT_STATE_FIRST)) {
            /* Initialize data structures for VpZlibInflate().  Ignore return
               value (it's hard-coded to return Z_OK). */
            VpZlibInflateInit2(stream, -1 * VP_COMPRESS_WINDOW_BITS,
                    &pScratchMem->state, pScratchMem->window);

            /* stream->next_in points to the input (deflated) buffer for
             *      inflate().
             * stream->avail_in indicates the number of bytes available there.
             * stream->next_out points to the output (inflated) buffer, which is
             *     filled in by inflate() and must be dumped to VpHbiBootWr().
             * stream->avail_out indicates the amount of space remaining
             *   there. */
            stream->next_out = pInflated;
            stream->avail_out = VP_COMPRESS_OUTBUF_PAGES * 128;
        }
        stream->avail_in = bufferSize;
        stream->next_in = pImageBuffer;
#endif /* !VP_NO_COMPRESS */

    } else {

        /* Uncompressed image.  Make sure length is a multiple of 128 bytes. */
        if ((bufferSize % 128) > 0) {
            VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): Uncompressed length not a multiple of 128: %ld", bufferSize));
            return VP_STATUS_ERR_IMAGE;
        }
    }

    /* As each chunk of pImageBuffer is processed, the length of the chunk will
       be subtracted from bufferSize.  Keep going until bufferSize = 0.  */
#ifdef VP_NO_COMPRESS
    while (bufferSize > 0) {
#else
    while ((bufferSize > 0) && (retval != Z_STREAM_END)) {
        /* If pImageBuffer is compressed... */
        if (pScratchMem != VP_NULL) {

            /* Inflate from pImageBuffer to the pInflated[] buffer. */
            retval = VpZlibInflate(stream, Z_SYNC_FLUSH);

            /* Handle inflate() error conditions. */
            if ((retval != Z_OK) && (retval != Z_STREAM_END)) {
                haltFunc(deviceId); /* halt the VTD */
                VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): inflate() error %d", retval));
                return VP_STATUS_ERR_IMAGE;
            }

            if (stream->avail_out % 128) {
                /* We ran out of input, but didn't produce an integer number
                   of pages of output (i.e. a multiple of 128 bytes). */
                if (
                    (state == VP_BOOT_STATE_LAST) ||
                    (state == VP_BOOT_STATE_FIRSTLAST)
                ) {
                    haltFunc(deviceId); /* halt the VTD */
                    VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): Output not a multiple of 128"));
                    return VP_STATUS_ERR_IMAGE;
                } else {
                    /* Expecting another call to VpBootload() to continue. */
                    return VP_STATUS_SUCCESS;
                }
            }

            /* Prepare to process the newly inflated block(s). */
            newblocks = ((VP_COMPRESS_OUTBUF_PAGES * 128) -
                            stream->avail_out) / 128;
            pHbiBlock = (VpImagePtrType)pInflated;

            /* Reset output pointer to the beginning of the output buffer. */
            stream->next_out = pInflated;
            stream->avail_out = VP_COMPRESS_OUTBUF_PAGES * 128;

            /* Reduce bufferSize to match the amount of input data remaining. */
            bufferSize = stream->avail_in;

        } else
#endif /* !VP_NO_COMPRESS */

        {
            /* Prepare to process the blocks directly from pImageBuffer. */
            pHbiBlock = pImageBuffer;
            newblocks = bufferSize / 128;
            bufferSize = 0;
        }

        /* Process the new block(s). */
        while (newblocks--) {

            /* First block in the application image contains special data. */
            if (firstblock) {

                /* Sanity check for magic numbers at the beginning of the
                 * boot stream. */
                if ((pHbiBlock[0] != 0xFE) || (pHbiBlock[1] != 0xFF)) {
                    VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): FEFF not found at beginning"));
                    return VP_STATUS_ERR_IMAGE;
                }

                /* Extract the record count from the boot stream. */
                *pRecCnt = ((uint16)pHbiBlock[6] << 8) + pHbiBlock[7];
                firstblock = FALSE;
            }

            /* Transmit the block over the HBI bus. */
            if (!VpHalHbiBootWr(deviceId, 63, pHbiBlock)) {
                haltFunc(deviceId); /* halt the VTD */
                VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): Failed VpHalHbiBootWr"));
                return VP_STATUS_ERR_HBI;
            }

            /* Decrement block counter. */
            (*pRecCnt)--;

            /* Increment HBI data pointer to next block. */
            pHbiBlock = &pHbiBlock[128];

        } /* while (newblocks) */
    } /* until we run out of input data */

    /* If we've sent all the blocks, check the hardware checksum and record count. */
    if ((state == VP_BOOT_STATE_FIRSTLAST) || (state == VP_BOOT_STATE_LAST)) {
        if (validation == VP_BOOT_MODE_VERIFY) {

            /* Record count should be 0 after decrementing once for each block sent. */
            if (*pRecCnt != 0) {
                /* debug_printf("recCnt = %lu\n", (unsigned long) recCnt); */
                haltFunc(deviceId);
                VP_ERROR(VpDeviceIdType, &deviceId, ("VpBootLoadInternal(): Record count not 0 at end"));
                return VP_STATUS_ERR_IMAGE;
            }

            /* Verify HBI load checksum equals expected value. */
            if (validateFunc(deviceId) == FALSE) {
                haltFunc(deviceId);
                return VP_STATUS_ERR_VERIFY;
            }
        }
    }

    return VP_STATUS_SUCCESS;
} /* VpBootLoadInternal() */

#endif /* VP_CC_VCP_SERIES || VP_CC_VCP2_SERIES */
