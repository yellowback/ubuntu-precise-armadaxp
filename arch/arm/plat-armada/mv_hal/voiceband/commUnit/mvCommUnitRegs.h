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

#ifndef __INCmvCommUnitRegsh
#define __INCmvCommUnitRegsh

#include "mvSysTdmConfig.h"

/****************************************************************/
/*	Time Division Multiplexing Interrupt Controller		*/
/****************************************************************/
#define COMM_UNIT_TOP_CAUSE_REG			(MV_COMM_UNIT_REGS_BASE + 0x8C00)
#define TDM_CAUSE_REG				(MV_COMM_UNIT_REGS_BASE + 0x8C40)
#define COMM_UNIT_TOP_MASK_REG			(MV_COMM_UNIT_REGS_BASE + 0x8C80)
#define VOICE_PERIODICAL_INT_CONTROL_REG	(MV_COMM_UNIT_REGS_BASE + 0x8C90)
#define TDM_MASK_REG				(MV_COMM_UNIT_REGS_BASE + 0x8CC0)

/* COMM_UNIT_TOP_CAUSE_REG bits */
#define TDM_SUM_INT_OFFS			6
#define TDM_SUM_INT_MASK			(1 << TDM_SUM_INT_OFFS)
#define MCSC_SUM_INT_OFFS			28
#define MCSC_SUM_INT_MASK			(1 << MCSC_SUM_INT_OFFS)

/* TDM_CAUSE_REG bits */
#define FLEX_TDM_RX_INT_OFFS			2
#define FLEX_TDM_RX_INT_MASK			(1 << FLEXTDM_RX_INT_OFFS)
#define FLEX_TDM_RX_SYNC_LOSS_OFFS		3
#define FLEX_TDM_RX_SYNC_LOSS_MASK		(1 << FLEX_TDM_RX_SYNC_LOSS_OFFS)
#define FLEX_TDM_TX_INT_OFFS			6
#define FLEX_TDM_TX_INT_MASK			(1 << FLEXTDM_TX_INT_OFFS)
#define FLEX_TDM_TX_SYNC_LOSS_OFFS		7
#define FLEX_TDM_TX_SYNC_LOSS_MASK		(1 << FLEX_TDM_TX_SYNC_LOSS_OFFS)
#define RX_VOICE_INT_PULSE_OFFS			8
#define RX_VOICE_INT_PULSE_MASK			(1 << RX_VOICE_INT_PULSE_OFFS)
#define TX_VOICE_INT_PULSE_OFFS			9
#define TX_VOICE_INT_PULSE_MASK			(1 << TX_VOICE_INT_PULSE_OFFS)
#define EXT_INT_SLIC0_OFFS			10
#define EXT_INT_SLIC0_MASK			(1 << EXT_INT_SLIC0_OFFS)
#define EXT_INT_SLIC1_OFFS			11
#define EXT_INT_SLIC1_MASK			(1 << EXT_INT_SLIC1_OFFS)
#define EXT_INT_SLIC2_OFFS			12
#define EXT_INT_SLIC2_MASK			(1 << EXT_INT_SLIC2_OFFS)
#define EXT_INT_SLIC3_OFFS			13
#define EXT_INT_SLIC3_MASK			(1 << EXT_INT_SLIC3_OFFS)
#define EXT_INT_SLIC4_OFFS			14
#define EXT_INT_SLIC4_MASK			(1 << EXT_INT_SLIC4_OFFS)
#define EXT_INT_SLIC5_OFFS			15
#define EXT_INT_SLIC5_MASK			(1 << EXT_INT_SLIC5_OFFS)
#define EXT_INT_SLIC6_OFFS			16
#define EXT_INT_SLIC6_MASK			(1 << EXT_INT_SLIC6_OFFS)
#define EXT_INT_SLIC7_OFFS			17
#define EXT_INT_SLIC7_MASK			(1 << EXT_INT_SLIC7_OFFS)
#define COMM_UNIT_PAR_ERR_SUM_OFFS		18
#define COMM_UNIT_PAR_ERR_SUM_MASK		(1 << COMM_UNIT_PAR_ERR_SUM_OFFS)
#define TDM_RX_PAR_ERR_SUM_OFFS			19
#define TDM_RX_PAR_ERR_SUM_MASK			(1 << TDM_RX_PAR_ERR_SUM_OFFS)
#define TDM_TX_PAR_ERR_SUM_OFFS			20
#define TDM_TX_PAR_ERR_SUM_MASK			(1 << TDM_TX_PAR_ERR_SUM_OFFS)
#define MCSC_PAR_ERR_SUM_OFFS			21
#define MCSC_PAR_ERR_SUM_MASK			(1 << MCSC_PAR_ERR_SUM_OFFS)
#define MCDMA_PAR_ERR_SUM_OFFS			22
#define MCDMA_PAR_ERR_SUM_MASK			(1 << MCDMA_PAR_ERR_SUM_OFFS)

/*  VOICE_PERIODICAL_INT_CONTROL_REG bits  */
#define RX_VOICE_INT_CNT_REF_OFFS		0
#define RX_VOICE_INT_CNT_REF_MASK		(0xff << RX_VOICE_INT_CNT_REF_OFFS)
#define TX_VOICE_INT_CNT_REF_OFFS		8
#define TX_VOICE_INT_CNT_REF_MASK		(0xff << TX_VOICE_INT_CNT_REF_OFFS)
#define RX_FIRST_DELAY_REF_OFFS			16
#define RX_FIRST_DELAY_REF_MASK			(0xff << RX_FIRST_DELAY_REF_OFFS)
#define TX_FIRST_DELAY_REF_OFFS			24
#define TX_FIRST_DELAY_REF_MASK			(0xff << TX_FIRST_DELAY_REF_OFFS)

/************************************************/
/*	Multi-Channel Serial Controller(MCSC)  	*/
/************************************************/
#define MCSC_CHx_COMM_EXEC_STAT_REG(ch)		(MV_COMM_UNIT_REGS_BASE + (ch<<2))
#define MCSC_CHx_RECEIVE_CONFIG_REG(ch)		((MV_COMM_UNIT_REGS_BASE + 0x400) + (ch<<2))
#define MCSC_CHx_TRANSMIT_CONFIG_REG(ch)	((MV_COMM_UNIT_REGS_BASE + 0x1800) + (ch<<2))
#define MCSC_GLOBAL_CONFIG_REG			(MV_COMM_UNIT_REGS_BASE + 0x2800)
#define MCSC_GLOBAL_INT_CAUSE_REG		(MV_COMM_UNIT_REGS_BASE + 0x2804)
#define MCSC_EXTENDED_INT_CAUSE_REG		(MV_COMM_UNIT_REGS_BASE + 0x2808)
#define MCSC_GLOBAL_INT_MASK_REG		(MV_COMM_UNIT_REGS_BASE + 0x280C)
#define MCSC_EXTENDED_INT_MASK_REG		(MV_COMM_UNIT_REGS_BASE + 0x2810)

/* MCSC_CHx_COMM_EXEC_STAT_REG bits */
#define MCSC_ABR_E_STAT_OFFS			25
#define MCSC_ABR_E_STAT_MASK			(1 << MCSC_ABR_E_STAT_OFFS)
#define MCSC_EH_E_STAT_OFFS			26
#define MCSC_EH_E_STAT_MASK			(1 << MCSC_EH_E_STAT_OFFS)

/* MCSC_RECEIVE_CONFIG_REG(MRCRx) bits */
#define MRCRx_ABORT_OFFS			21
#define MRCRx_ABORT_MASK			(1 << MRCRx_ABORT_OFFS)
#define MRCRx_ENTER_HUNT_OFFS			22
#define MRCRx_ENTER_HUNT_MASK			(1 << MRCRx_ENTER_HUNT_OFFS)
#define MRCRx_ER_OFFS				27
#define MRCRx_ER_MASK				(1 << MRCRx_ER_OFFS)
#define MRCRx_RRVD_OFFS				30
#define MRCRx_RRVD_MASK				(1 << MRCRx_RRVD_OFFS)
#define MRCRx_MODE_OFFS				31
#define MRCRx_MODE_MASK				(1 << MRCRx_MODE_OFFS)

/* MCSC_TRANSMIT_CONFIG_REG(MTCRx) bits */
#define MTCRx_ET_OFFS				27
#define MTCRx_ET_MASK				(1 << MTCRx_ET_OFFS)
#define MTCRx_TRVD_OFFS				30
#define MTCRx_TRVD_MASK				(1 << MTCRx_TRVD_OFFS)
#define MTCRx_MODE_OFFS				31
#define MTCRx_MODE_MASK				(1 << MTCRx_MODE_OFFS)

/* MCSC_GLOBAL_CONFIG_REG bits */
#define MCSC_GLOBAL_CONFIG_ARBM_OFFS		29
#define MCSC_GLOBAL_CONFIG_ARBM_MASK		(1 << MCSC_GLOBAL_CONFIG_ARBM_OFFS)
#define MCSC_GLOBAL_CONFIG_RXEN_OFFS		30
#define MCSC_GLOBAL_CONFIG_RXEN_MASK		(1 << MCSC_GLOBAL_CONFIG_RXEN_OFFS)
#define MCSC_GLOBAL_CONFIG_TXEN_OFFS		31
#define MCSC_GLOBAL_CONFIG_TXEN_MASK		(1 << MCSC_GLOBAL_CONFIG_TXEN_OFFS)

/* MCSC_GLOBAL_INT_CAUSE_REG */
#define  MCSC_GLOBAL_INT_CAUSE_RXOR_OFFS	0
#define  MCSC_GLOBAL_INT_CAUSE_RXOR_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_RXOR_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_TXUR_OFFS	8
#define  MCSC_GLOBAL_INT_CAUSE_TXUR_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_TXUR_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_TXOR_OFFS	16
#define  MCSC_GLOBAL_INT_CAUSE_TXOR_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_TXOR_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_RXBOR_OFFS	24
#define  MCSC_GLOBAL_INT_CAUSE_RXBOR_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_RXBOR_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_INIT_DONE_OFFS	25
#define  MCSC_GLOBAL_INT_CAUSE_INIT_DONE_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_INIT_DONE_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_IQOR_OFFS	26
#define  MCSC_GLOBAL_INT_CAUSE_IQOR_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_IQOR_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_IQNE_OFFS	27
#define  MCSC_GLOBAL_INT_CAUSE_IQNE_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_IQNE_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_MDIL_OFFS	28
#define  MCSC_GLOBAL_INT_CAUSE_MDIL_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_MDIL_OFFS)
#define  MCSC_GLOBAL_INT_CAUSE_MGIS_OFFS	30
#define  MCSC_GLOBAL_INT_CAUSE_MGIS_MASK	(1 << MCSC_GLOBAL_INT_CAUSE_MGIS_OFFS)

/************************************************/
/*	Multi-Channel DMA(MCDMA)       		*/
/************************************************/
#define MCDMA_RECEIVE_CONTROL_REG(ch)		((MV_COMM_UNIT_REGS_BASE + 0x3000) + (ch<<2))
#define MCDMA_RECEIVE_FIFO_MGMT_LL_REG(ch)	((MV_COMM_UNIT_REGS_BASE + 0x3C00) + (ch<<2))
#define MCDMA_CURRENT_RECEIVE_DESC_PTR_REG(ch)	((MV_COMM_UNIT_REGS_BASE + 0x4000) + (ch<<2))
#define MCDMA_GLOBAL_CONTROL_REG		(MV_COMM_UNIT_REGS_BASE + 0x4400)
#define RX_SERVICE_QUEUE_ARBITER_WEIGHT_REG	(MV_COMM_UNIT_REGS_BASE + 0x4408)
#define MCDMA_TRANSMIT_CONTROL_REG(ch)		((MV_COMM_UNIT_REGS_BASE + 0x5000) + (ch<<2))
#define MCDMA_TRANSMIT_FIFO_MGMT_LL_REG(ch)	((MV_COMM_UNIT_REGS_BASE + 0x5C00) + (ch<<2))
#define MCDMA_CURRENT_TRANSMIT_DESC_PTR_REG(ch)	((MV_COMM_UNIT_REGS_BASE + 0x7000) + (ch<<2))
#define TX_SERVICE_QUEUE_ARBITER_WEIGHT_REG	(MV_COMM_UNIT_REGS_BASE + 0x7408)

/* MCDMA_RECEIVE_CONTROL_REG bits */
#define MCDMA_RBSZ_OFFS				0
#define MCDMA_RBSZ_MASK				(0x3 << MCDMA_RBSZ_OFFS)
#define MCDMA_RBSZ_8BYTE			0
#define MCDMA_RBSZ_16BYTE			0x1
#define MCDMA_RBSZ_32BYTE			0x2
#define MCDMA_RBSZ_64BYTE			0x3

#define MCDMA_BLMR_OFFS				2
#define MCDMA_BLMR_MASK				(1 << MCDMA_BLMR_OFFS)
#define MCDMA_ERD_OFFS				6
#define MCDMA_ERD_MASK				(1 << MCDMA_ERD_OFFS)

/* MCDMA_GLOBAL_CONTROL_REG bits */
#define MCDMA_RID_OFFS				1
#define MCDMA_RID_MASK				(1 << MCDMA_RID_OFFS)

/* RX_SERVICE_QUEUE_ARBITER_WEIGHT_REG bits */
#define MCDMA_RSQW_OFFS				24
#define MCDMA_RSQW_MASK				(0x1f << MCDMA_RSQW_OFFS)

/* TX_SERVICE_QUEUE_ARBITER_WEIGHT_REG bits */
#define MCDMA_TSQW_OFFS				24
#define MCDMA_TSQW_MASK				(0x1f << MCDMA_TSQW_OFFS)

/* MCDMA_TRANSMIT_CONTROL_REG bits */
#define MCDMA_FSIZE_OFFS			0
#define MCDMA_FSIZE_MASK			(0xff << MCDMA_FSIZE_OFFS)
#define MCDMA_FSIZE_1BLK			0x1
#define MCDMA_TBSZ_OFFS				8
#define MCDMA_TBSZ_MASK				(0x3 << MCDMA_TBSZ_OFFS)
#define MCDMA_TBSZ_16BYTE			(0x1 << MCDMA_TBSZ_OFFS)
#define MCDMA_TBSZ_32BYTE			(0x2 << MCDMA_TBSZ_OFFS)

#define MCDMA_BLMT_OFFS				10
#define MCDMA_BLMT_MASK				(1 << MCDMA_BLMT_OFFS)
#define MCDMA_NDUR_OFFS				12
#define MCDMA_NDUR_MASK				(1 << MCDMA_NDUR_OFFS)
#define MCDMA_TPC_OFFS				13
#define MCDMA_TPC_MASK				(1 << MCDMA_TPC_OFFS)
#define MCDMA_URPM_OFFS				14
#define MCDMA_URPM_MASK				(1 << MCDMA_URPM_OFFS)
#define MCDMA_STD_OFFS				15
#define MCDMA_STD_MASK				(1 << MCDMA_STD_OFFS)
#define MCDMA_AT_OFFS				16
#define MCDMA_AT_MASK				(1 << MCDMA_AT_OFFS)
#define MCDMA_TXD_OFFS				17
#define MCDMA_TXD_MASK				(1 << MCDMA_TXD_OFFS)

/************************************************/
/*	Time Division Multiplexing(TDM)	  	*/
/************************************************/
#define FLEX_TDM_TDPR_REG(entry)		((MV_COMM_UNIT_REGS_BASE + 0x8000) + (entry<<2))
#define FLEX_TDM_RDPR_REG(entry)		((MV_COMM_UNIT_REGS_BASE + 0x8400) + (entry<<2))
#define FLEX_TDM_TRANSMIT_READ_PTR_REG		(MV_COMM_UNIT_REGS_BASE + 0x8800)
#define FLEX_TDM_RECEIVE_READ_PTR_REG		(MV_COMM_UNIT_REGS_BASE + 0x8804)
#define FLEX_TDM_CONFIG_REG			(MV_COMM_UNIT_REGS_BASE + 0x8808)
#define TDM_CLK_AND_SYNC_CONTROL_REG		(MV_COMM_UNIT_REGS_BASE + 0x881C)
#define TDM_CLK_DIVIDER_CONTROL_REG		(MV_COMM_UNIT_REGS_BASE + 0x8820)
#define TDM_RESV_CLK_DIVIDER_CONTROL_REG	(MV_COMM_UNIT_REGS_BASE + 0x8824)

/* TDM_CLK_AND_SYNC_CONTROL_REG bits */
#define TDM_TX_FSYNC_OUT_ENABLE_OFFS		0
#define TDM_TX_FSYNC_OUT_ENABLE_MASK		(1 << TDM_TX_FSYNC_OUT_ENABLE_OFFS)
#define TDM_RX_FSYNC_OUT_ENABLE_OFFS		1
#define TDM_RX_FSYNC_OUT_ENABLE_MASK		(1 << TDM_RX_FSYNC_OUT_ENABLE_OFFS)
#define TDM_TX_CLK_OUT_ENABLE_OFFS		2
#define TDM_TX_CLK_OUT_ENABLE_MASK		(1 << TDM_TX_CLK_OUT_ENABLE_OFFS)
#define TDM_RX_CLK_OUT_ENABLE_OFFS		3
#define TDM_RX_CLK_OUT_ENABLE_MASK		(1 << TDM_RX_CLK_OUT_ENABLE_OFFS)
#define TDM_TX_REFCLK_DIVIDER_OFFS		4
#define TDM_TX_REFCLK_DIVIDER_MASK		(0xff << TDM_TX_REFCLK_DIVIDER_OFFS)
#define TDM_TX_REFCLK_DIVIDER_2MHZ		BIT7
#define TDM_TX_REFCLK_DIVIDER_4MHZ		BIT8
#define TDM_TX_REFCLK_DIVIDER_8MHZ		BIT9
#define TDM_RX_REFCLK_DIVIDER_OFFS		12
#define TDM_RX_REFCLK_DIVIDER_MASK		(0xff << TDM_RX_REFCLK_DIVIDER_OFFS)
#define TDM_RX_REFCLK_DIVIDER_2MHZ		BIT15
#define TDM_RX_REFCLK_DIVIDER_4MHZ		BIT16
#define TDM_RX_REFCLK_DIVIDER_8MHZ		BIT17
#define TDM_REFCLK_DIVIDER_BYPASS_OFFS		20
#define TDM_REFCLK_DIVIDER_BYPASS_MASK		(3 << TDM_REFCLK_DIVIDER_BYPASS_OFFS)

/* TDM_RX_CLK_DIVIDER_CONTROL_REG bits */
#define TDM_RX_FIXED_DIV_ENABLE_OFFS		31
#define TDM_RX_FIXED_DIV_ENABLE_MASK		(1 << TDM_RX_FIXED_DIV_ENABLE_OFFS)

/* TDM_TX_CLK_DIVIDER_CONTROL_REG bits */
#define TDM_TX_FIXED_DIV_ENABLE_OFFS		31
#define TDM_TX_FIXED_DIV_ENABLE_MASK		(1 << TDM_TX_FIXED_DIV_ENABLE_OFFS)

/* FLEX_TDM_CONFIG_REG bits */
#define TDM_SE_OFFS				20
#define TDM_SE_MASK				(1 << TDM_SE_OFFS)
#define TDM_COMMON_RX_TX_OFFS			23
#define TDM_COMMON_RX_TX_MASK			(1 << TDM_COMMON_RX_TX_OFFS)
#define TSD_OFFS				25
#define TSD_MASK				(0x3 << TSD_OFFS)
#define TSD_NO_DELAY				(0 << TSD_OFFS)
#define TSD_1BIT_DELAY				(1 << TSD_OFFS)
#define TSD_2BIT_DELAY				(2 << TSD_OFFS)
#define TSD_3BIT_DELAY				(3 << TSD_OFFS)
#define RSD_OFFS				27
#define RSD_MASK				(0x3 << RSD_OFFS)
#define RSD_NO_DELAY				(0 << RSD_OFFS)
#define RSD_1BIT_DELAY				(1 << RSD_OFFS)
#define RSD_2BIT_DELAY				(2 << RSD_OFFS)
#define RSD_3BIT_DELAY				(3 << RSD_OFFS)
#define TDM_TEN_OFFS				31
#define TDM_TEN_MASK				(1 << TDM_TEN_OFFS)

/************************************************/
/*	Shared Bus to Crossbar Bridge		*/
/************************************************/
#define COMM_UNIT_MBUS_MAX_WIN			12

#define COMM_UNIT_WIN_CTRL_REG(win)		((MV_COMM_UNIT_REGS_BASE + 0x8A00) + (win<<3))
#define COMM_UNIT_WIN_SIZE_REG(win)		((MV_COMM_UNIT_REGS_BASE + 0x8A04) + (win<<3))
#define COMM_UNIT_WIN_ENABLE_REG(win)		((MV_COMM_UNIT_REGS_BASE + 0x8B04) + (win<<2))
#define COMM_UNIT_OVERRIDE_WIN_CONTROL_REG	(MV_COMM_UNIT_REGS_BASE + 0x8A7C)
#define TIME_OUT_COUNTER_REG			(MV_COMM_UNIT_REGS_BASE + 0x8ADC)
#define COMM_UNIT_WINDOWS_ACCESS_PROTECT_REG	(MV_COMM_UNIT_REGS_BASE + 0x8B00)

/* COMM_UNIT_WIN_CTRL_REG bits */
#define COMM_UNIT_WIN_TARGET_OFFS		0
#define COMM_UNIT_WIN_TARGET_MASK		(0xf << COMM_UNIT_WIN_TARGET_OFFS)
#define COMM_UNIT_WIN_ATTRIB_OFFS		8
#define COMM_UNIT_WIN_ATTRIB_MASK		(0xff << COMM_UNIT_WIN_ATTRIB_OFFS)
#define COMM_UNIT_WIN_BASE_OFFS			16
#define COMM_UNIT_WIN_BASE_MASK			(0xffff << COMM_UNIT_WIN_BASE_OFFS)

/* COMM_UNIT_WIN_SIZE_REG bits */
#define COMM_UNIT_WIN_SIZE_OFFS			16
#define COMM_UNIT_WIN_SIZE_MASK			(0xffff << COMM_UNIT_WIN_SIZE_OFFS)

/* COMM_UNIT_WIN_ENABLE_REG bits */
#define COMM_UNIT_WIN_ENABLE_OFFS		0
#define COMM_UNIT_WIN_ENABLE_MASK		(0xff << COMM_UNIT_WIN_CX_ENABLE_OFFS)
#define COMM_UNIT_WIN_CX_ENABLE_OFFS		0
#define COMM_UNIT_WIN_CX_ENABLE_MASK		(0x1 << COMM_UNIT_WIN_CX_ENABLE_OFFS)
#define COMM_UNIT_WIN_MCDMA_ENABLE_OFFS		3
#define COMM_UNIT_WIN_MCDMA_ENABLE_MASK		(0x1 << COMM_UNIT_WIN_MCDMA_ENABLE_OFFS)
#define COMM_UNIT_WIN_MCSC_ENABLE_OFFS		4
#define COMM_UNIT_WIN_MCSC_ENABLE_MASK		(0x1 << COMM_UNIT_WIN_MCSC_ENABLE_OFFS)
#define COMM_UNIT_WIN_TDM_ENABLE_OFFS		5
#define COMM_UNIT_WIN_TDM_ENABLE_MASK		(0x1 << COMM_UNIT_WIN_TDM_ENABLE_OFFS)
#define COMM_UNIT_WIN_CX_REGFILE_ENABLE_OFFS	7
#define COMM_UNIT_WIN_CX_REGFILE_ENABLE_MASK	(0x1 << COMM_UNIT_WIN_CX_REGFILE_ENABLE_OFFS)

/* TIME_OUT_COUNTER_REG bits */
#define	TIME_OUT_THRESHOLD_COUNT_OFFS		16
#define	TIME_OUT_THRESHOLD_COUNT_MASK		(0xffff << TIME_OUT_THRESHOLD_COUNT_OFFS)

/* COMM_UNIT_WINDOWS_ACCESS_PROTECT_REG bits */
#define COMM_UNIT_WIN_PROTECT_OFFS(win)		(win << 1)
#define COMM_UNIT_WIN_PROTECT_MASK(win)		(0x3 << COMM_UNIT_WIN_PROTECT_OFFS(win))

#define MV_COMM_UNIT_WIN_SIZE_ALIGN	_64K

#endif /*__INCmvCommUnitRegsh*/

