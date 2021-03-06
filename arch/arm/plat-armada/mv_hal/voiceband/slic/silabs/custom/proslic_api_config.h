/*
** Copyright ? 2007 by Silicon Laboratories
**
** $Id: proslic_api_config.h,v 1.2 2008/07/24 21:01:36 lajordan Exp $
**
**
** Author(s):
** laj
**
** Distributed by:
** Silicon Laboratories, Inc
**
** This file contains proprietary information.
** No dissemination allowed without prior written permission from
** Silicon Laboratories, Inc.
**
**
*/

#include "mvOs.h"
#include "mvSysTdmConfig.h"

/* #define DISABLE_DTMF_SETUP  */
/*#define DISABLE_FSK_SETUP  */
/*#define DISABLE_TONE_SETUP  */
/*#define DISABLE_RING_SETUP  */
/*#define DISABLE_AUDIOGAIN_SETUP  */
/*#define DISABLE_DCFEED_SETUP  */
/*#define DISABLE_GPIO_SETUP  */
/*#define DISABLE_PCM_SETUP  */
#define ENABLE_DEBUG
#if defined(SILABS_SLIC_3215)
#define SI321X
#define SI3215_TONE
#elif defined(SILABS_SLIC_3217)
#define SI3217X
#endif
/*#define DISABLE_CI_SETUP  */
/*#define DISABLE_ZSYNTH_SETUP  */
/*#define DISABLE_MALLOC */
/*#define GCI_MODE  */

//#include "stdio.h"
/*#if (PRINT_TO_STRING) */
/*extern char outputBuffer[]; */
/*#define LOGPRINT(...) sprintf(&(outputBuffer[strlen(outputBuffer)]),__VA_ARGS__) */
/*#else */
#define LOGPRINT mvOsPrintf
#define malloc mvOsMalloc
#define free mvOsFree
/*#endif */

#ifndef PROSLIC_API_CFG
#define PROSLIC_API_CFG



#endif
