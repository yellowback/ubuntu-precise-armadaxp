/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	    this list of conditions and the following disclaimer.

    *   Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

    *   Neither the name of Marvell nor the names of its contributors may be
        used to endorse or promote products derived from this software without
        specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "mvCommon.h"
#include "mvOs.h"
#include "ctrlEnv/mvCtrlEnvLib.h"
#include "boardEnv/mvBoardEnvLib.h"
#include "mv_cesa/cesa_if.h"

extern u32 mv_crypto_phys_base_get(u8 chan);
extern u32 mv_crypto_virt_base_get(u8 chan);

/*******************************************************************************
* mvSysCesaInit - Initialize the Cesa subsystem
*
* DESCRIPTION:
*
* INPUT:
*       None
* OUTPUT:
*		None
* RETURN:
*       None
*
*******************************************************************************/
MV_STATUS mvSysCesaInit(int numOfSession, int queueDepth, void *osHandle)
{
	MV_CESA_HAL_DATA halData;
	MV_UNIT_WIN_INFO addrWinMap[MAX_TARGETS + 1];
	MV_STATUS status;
	MV_U8 chan;

	status = mvCtrlAddrWinMapBuild(addrWinMap, MAX_TARGETS + 1);

	if(status == MV_OK) {
		for(chan = 0; chan < MV_CESA_CHANNELS; chan++) {
			status = mvCesaIfTdmaWinInit(chan, addrWinMap);
			
			if(status != MV_OK) {
				mvOsPrintf("Error, unable to initialize CESA windows for channel(%d)\n", chan);
				break;
			}
			halData.sramPhysBase[chan] = (MV_ULONG)mv_crypto_phys_base_get(chan);
			halData.sramVirtBase[chan] = (MV_U8*)mv_crypto_virt_base_get(chan);
			halData.sramOffset[chan] = 0;
		}

		if(status == MV_OK) {
			halData.ctrlModel = mvCtrlModelGet();
			halData.ctrlRev = mvCtrlRevGet();
			status = mvCesaIfInit (numOfSession, queueDepth, osHandle, &halData);
		}
	}

	return status;
}
