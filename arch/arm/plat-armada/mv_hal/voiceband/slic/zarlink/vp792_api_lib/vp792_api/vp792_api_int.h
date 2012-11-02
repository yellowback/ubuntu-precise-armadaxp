/** \file vp792_api_int.h
 * vp792_api_int.h
 *
 * Header file for the Vp792 series API-II c files.
 *
 * Copyright (c) 2010, Zarlink Semiconductor, Inc.
 *
 * $Revision: 7142 $
 * $LastChangedDate: 2010-05-12 13:09:30 -0500 (Wed, 12 May 2010) $
 */

#ifndef VP792_API_INT_H
#define VP792_API_INT_H

#include "vp_api.h"
#include "vp792_api.h"
#include "vp792_getevent.h"

#ifdef VP792_INCLUDE_TESTLINE_CODE
    #include "vp792pm.h"
#endif

#define VP792_MAX_NUM_CHANNELS  ((VP_MAX_LINES_PER_DEVICE < 8) ? VP_MAX_LINES_PER_DEVICE : 8)


/* =================================
    HBI Transport Layer Definitions
   ================================= */

#define HBI_PAGED_READ(offset,length) \
    ((uint16)(((uint16)(offset) << 8) | (length)))
#define HBI_DIRECT_READ(slacId,offset,length) \
    ((uint16)(0x8000 | ((uint16)(offset) << 8) | ((slacId) << 4) | (length)))
#define HBI_PAGED_WRITE(offset,length) \
    ((uint16)(HBI_PAGED_READ(offset,length) | 0x0080))
#define HBI_DIRECT_WRITE(slacId,offset,length) \
    ((uint16)(HBI_DIRECT_READ(slacId,offset,length) | 0x0080))
#define HBI_GLOBAL_DIRECT_WRITE(offset,length) \
    ((uint16)(0xFC00 | ((offset) << 4) | (length)))
#define HBI_CONFIGURE(pinConfig) \
    ((uint16)(0xFD00 | (pinConfig)))
#define HBI_SELECT_PAGE(slacId,page) \
    ((uint16)(0xFE00 | ((slacId) << 4) | (page)))
#define HBI_SELECT_CL_PAGE(slacId) \
    ((uint16)(0xFE80 | ((slacId) << 4)))
#define HBI_CL_BROADCAST \
    ((uint16)0xFEFF)
#define HBI_NO_OP \
    ((uint16)0xFFFF)


/* =============================
    Global Register Definitions
   ============================= */

/* 0x00 : Interrupt Indication Register */
#define VP792_REG_INTIND_OFFSET             0x00
#define VP792_REG_INTIND_LEN                2
#define VP792_REG_INTIND_EVID_WORD          0
#define VP792_REG_INTIND_PARM_WORD          1
#define VP792_REG_SHARED_INT_OFFSET         0x77
#define VP792_REG_SHARED_INT_LEN            0x02

#define VP792_REG_INTIND_NO_EVENT           0x0000

/* 0x02 : Mailbox Offset */
#define VP792_REG_MB_OFFSET                 0x02
#define VP792_REG_MB_OFFSET_LEN             1
#define VP792_REG_MB_OFFSET_MASK            0x01FF

/* 0x03 : Mailbox Host Semaphore Flags */
#define VP792_REG_MB_FLAG_OFFSET            0x03
#define VP792_REG_MB_FLAG_LEN               1
#define VP792_REG_MB_FLAG_CMD_MBOX_MASK     0x0001
#define VP792_REG_MB_FLAG_RSP_MBOX_MASK     0x0002

/* 0x04 : Code Load CRC */
#define VP792_REG_CL_CRC_OFFSET             0x04
#define VP792_REG_CL_CRC_LEN                2
#define VP792_REG_CL_CRC_WORD_0             0x00
#define VP792_REG_CL_CRC_SEED_UPPER_MASK    0xFFFF
#define VP792_REG_CL_CRC_WORD_1             0x01
#define VP792_REG_CL_CRC_SEED_LOWER_MASK    0xFFFF

/* 0x06 : Code Load Base Address */
#define VP792_REG_CL_BASE_OFFSET            0x06
#define VP792_REG_CL_BASE_LEN               2
#define VP792_REG_CL_BASE_WORD_0            0x00
#define VP792_REG_CL_BASE_CLBA0_MASK        0x3fff
#define VP792_REG_CL_BASE_WORD_1            0x01
#define VP792_REG_CL_BASE_CLBA1_MASK        0xff00

/* 0x09 : PCM Configuration */
#define VP792_REG_PCM_CFG_OFFSET            0x09
#define VP792_REG_PCM_CFG_LEN               1
#define VP792_REG_PCM_CFG_TCS_MASK          0x0007
#define VP792_REG_PCM_CFG_RCS_MASK          0x0038
#define VP792_REG_PCM_CFG_XE_MASK           0x0040
#define VP792_REG_PCM_CFG_TAB_MASK          0x0080
#define VP792_REG_PCM_CFG_32K_CH_MASK       0x0700
#define VP792_REG_PCM_CFG_32K_CH7           0x0700
#define VP792_REG_PCM_CFG_32K_CH6           0x0600
#define VP792_REG_PCM_CFG_32K_CH5           0x0500
#define VP792_REG_PCM_CFG_32K_CH4           0x0400
#define VP792_REG_PCM_CFG_32K_CH3           0x0300
#define VP792_REG_PCM_CFG_32K_CH2           0x0200
#define VP792_REG_PCM_CFG_32K_CH1           0x0100
#define VP792_REG_PCM_CFG_32K_CH0           0x0000
#define VP792_REG_PCM_CFG_32K_EN_MASK       0x0800

/* 0x0A : Clock Status Register */
#define VP792_REG_CLK_STATUS_OFFSET         0x0A
#define VP792_REG_CLK_STATUS_LEN            1
#define VP792_REG_CLK_STATUS_HW_RST_MASK    0x0001
#define VP792_REG_CLK_STATUS_PCLK_FAIL_MASK 0x0100
#define VP792_REG_CLK_STATUS_FS_FAIL_MASK   0x0200
#define VP792_REG_CLK_STATUS_CFAIL_MASK     0x0400

/* 0x0B : Register Clock Configuration */
#define VP792_REG_CLK_CFG_OFFSET            0x0B
#define VP792_REG_CLK_CFG_LEN               1
#define VP792_REG_CLK_CFG_CSEL_MASK         0x000F
#define VP792_REG_CLK_CFG_CSEL_1536_KHZ     0x0000
#define VP792_REG_CLK_CFG_CSEL_1544_KHZ     0x0001
#define VP792_REG_CLK_CFG_CSEL_2048_KHZ     0x0002
#define VP792_REG_CLK_CFG_CSEL_3072_KHZ     0x0004
#define VP792_REG_CLK_CFG_CSEL_3088_KHZ     0x0005
#define VP792_REG_CLK_CFG_CSEL_4096_KHZ     0x0006
#define VP792_REG_CLK_CFG_CSEL_6144_KHZ     0x0008
#define VP792_REG_CLK_CFG_CSEL_6176_KHZ     0x0009
#define VP792_REG_CLK_CFG_CSEL_8192_KHZ     0x000A

/* 0x0C : System Status Register */
#define VP792_REG_SYS_STAT_OFFSET           0x0C
#define VP792_REG_SYS_STAT_LEN              1
#define VP792_REG_SYS_STAT_SYS_RESET_MASK   0x0001
#define VP792_REG_SYS_STAT_SYS_PREB         0x0002
#define VP792_REG_SYS_STAT_HBF_MASK         0x0010
#define VP792_REG_SYS_STAT_LBF_MASK         0x0020
#define VP792_REG_SYS_STAT_PBF_MASK         0x0040
#define VP792_REG_SYS_STAT_CFAIL_MASK       0x0200
#define VP792_REG_SYS_STAT_WDT_MASK         0x0400
#define VP792_REG_SYS_STAT_OVL_MASK         0x0800
#define VP792_REG_SYS_STAT_BF_MASK          0x0070
#define VP792_REG_SYS_STAT_BF_SHIFT         4

/* 0x0D : Register System Interupt Mask */
#define VP792_REG_SYS_MASK_OFFSET           0x0D
#define VP792_REG_SYS_MASK_LEN              1
#define VP792_REG_SYS_MASK_SYS_PREB_MASK    0x0002
#define VP792_REG_SYS_MASK_BATF_MASK        0x0070
#define VP792_REG_SYS_MASK_HBF_MASK         0x0010
#define VP792_REG_SYS_MASK_LBF_MASK         0x0020
#define VP792_REG_SYS_MASK_PBF_MASK         0x0040
#define VP792_REG_SYS_MASK_CFAIL_MASK       0x0200
#define VP792_REG_SYS_MASK_WDT_MASK         0x0400
#define VP792_REG_SYS_MASK_OVL_MASK         0x0800
#define VP792_REG_SYS_MASK_UNMASK_ALL       0x0000

/* 0x20 : Global Hook Status Register */
#define VP792_REG_GHOOK_OFFSET              0x20
#define VP792_REG_GHOOK_LEN                 1

/* 0x21 : Global Ground-Key Status Register */
#define VP792_REG_GGKEY_OFFSET              0x21
#define VP792_REG_GGKEY_LEN                 1

/* 0x22 : Global Raw Hook Status Register */
#define VP792_REG_GRHOOK_OFFSET             0x22
#define VP792_REG_GRHOOK_LEN                1

/* 0x23 : Reserved */

/* 0x24 : Global Fault Status Register */
#define VP792_REG_GFLT_OFFSET               0x24
#define VP792_REG_GFLT_LEN                  1

/* 0x26 : Revision Code of the device */
#define VP792_REG_REVCODE_OFFSET            0x26
#define VP792_REG_REVCODE_LEN               1
#define VP792_REG_REVCODE_MAJOR_MASK        0x00FF
#define VP792_REG_REVCODE_MINOR_MASK        0xFF00

/* 0x27 : Device Code of the device */
#define VP792_REG_DEVCODE_OFFSET            0x27
#define VP792_REG_DEVCODE_LEN               1
#define VP792_REG_DEVCODE_MINOR_MASK        0x00FF
#define VP792_REG_DEVCODE_MINOR_EXT_RING    0x0001
#define VP792_REG_DEVCODE_MAJOR_MASK        0xEF00
#define VP792_REG_DEVCODE_MAJOR_SDP         0x0100
#define VP792_REG_DEVCODE_MAJOR_EMU         0x0200
#define VP792_REG_DEVCODE_MAJOR_ARCTURUS    0x0300
#define VP792_REG_DEVCODE_MAJOR_NGSLAC      0x0100
#define VP792_REG_DEVCODE_INT_MASK          0x8000

/* 0x28 : Revision Code of the device */
#define VP792_REG_ICODE_OFFSET              0x28
#define VP792_REG_ICODE_LEN                 0x01

/* 0x2A : Timestamp */
#define VP792_REG_TSTAMP_OFFSET             0x2A
#define VP792_REG_TSTAMP_LEN                0x01

/* 0x2D : Sensed high battery voltage */
#define VP792_REG_HBAT_OFFSET               0x2D
#define VP792_REG_HBAT_LEN                  1

/* 0x2E : Sensed low battery voltage */
#define VP792_REG_LBAT_OFFSET               0x2E
#define VP792_REG_LBAT_LEN                  1

/* 0x2F : Sensed positive battery voltage */
#define VP792_REG_PBAT_OFFSET               0x2F
#define VP792_REG_PBAT_LEN                  1

/* All battery voltages in a single read */
#define VP792_REG_BAT_OFFSET                0x2D
#define VP792_REG_BAT_LEN                   3
#define VP792_REG_BAT_H_WORD                0
#define VP792_REG_BAT_L_WORD                1
#define VP792_REG_BAT_P_WORD                2

/* 0x31 : Global Event Mask Register */
#define VP792_REG_GMASK_OFFSET              0x31
#define VP792_REG_GMASK_LEN                 1
#define VP792_REG_GMASK_CMD_M_MASK          0x0001
#define VP792_REG_GMASK_RSP_M_MASK          0x0002
#define VP792_REG_GMASK_ERR_M_MASK          0x0004
#define VP792_REG_GMASK_TMR_M_MASK          0x0010
#define VP792_REG_GMASK_TS_M_MASK           0x0020
#define VP792_REG_GMASK_CAL_M_MASK          0x0040
#define VP792_REG_GMASK_UNMASK_ALL          0x0000

/* 0x58 : Read DCA       */
#define VP792_REG_DCA_OFFSET                                0x58
#define VP792_REG_DCA_LEN                                   1

/* 0x59 : Read DCB       */
#define VP792_REG_DCB_OFFSET                                0x59
#define VP792_REG_DCB_LEN                                   1


/* 0x32 : High battery voltage adjustment */
#define VP792_REG_HBAT_ADJ_OFFSET           0x32
#define VP792_REG_HBAT_ADJ_LEN              1

/* VP792 VP_CFG2 Register field defines */
#define VP792_REG_VP_CFG2_CTP_INACTIVE                      (0)
#define VP792_REG_VP_CFG2_CTP_ACTIVE                        (1)
#define VP792_REG_VP_CFG2_CRP_INACTIVE                      (0)
#define VP792_REG_VP_CFG2_CRP_ACTIVE                        (1)
#define VP792_REG_VP_CFG2_DHP_INACTIVE                      (0)
#define VP792_REG_VP_CFG2_DHP_ACTIVE                        (1)
#define VP792_REG_VP_CFG2_LRG_INACTIVE                      (0)
#define VP792_REG_VP_CFG2_LRG_ACTIVE                        (1)
#define VP792_REG_VP_CFG2_TSLB_INACTIVE                     (0)
#define VP792_REG_VP_CFG2_TSLB_ACTIVE                       (1)
#define VP792_REG_VP_CFG2_TONE_INACTIVE                     (0)
#define VP792_REG_VP_CFG2_TONE_ACTIVE                       (1)

/* 0x39 - 0x3F : Pulse, Hook, and Flash Decoder Configuration Register */
#define VP792_REG_PULSE_OFFSET              0x39
#define VP792_REG_PULSE_LEN                 7
#define VP792_REG_PULSE_MAX_MAKE_OFFSET     0x39
#define VP792_REG_PULSE_MIN_MAKE_OFFSET     0x3A
#define VP792_REG_PULSE_MAX_BREAK_OFFSET    0x3B
#define VP792_REG_PULSE_MIN_BREAK_OFFSET    0x3C
#define VP792_REG_PULSE_MAX_FLASH_OFFSET    0x3D
#define VP792_REG_PULSE_MIN_FLASH_OFFSET    0x3E
#define VP792_REG_PULSE_MIN_INDIG_OFFSET    0x3F
#define VP792_REG_PULSE_MAX_MAKE            0
#define VP792_REG_PULSE_MIN_MAKE            1
#define VP792_REG_PULSE_MAX_BREAK           2
#define VP792_REG_PULSE_MIN_BREAK           3
#define VP792_REG_PULSE_MAX_FLASH           4
#define VP792_REG_PULSE_MIN_FLASH           5
#define VP792_REG_PULSE_MIN_INDIG           6

/* Command mailbox */
#define VP792_HBI_COMMAND_MBOX_PAGE         0x08
#define VP792_HBI_COMMAND_MBOX_SIZE         0x80

/* Response mailbox */
#define VP792_HBI_RESPONSE_MBOX_PAGE        0x09
#define VP792_HBI_RESPONSE_MBOX_SIZE        0x80

#define VP792_HBI_MAX_MBOX_PAGE             0x0F

/* Code Load Page */
#define VP792_HBI_CODE_LOAD_PAGE            0xFF

/* Hardware Rev Code (AMBA) */
#define VP792_AMBA_REG_HW_REV_ADDR_UPPER    0x3FFF
#define VP792_AMBA_REG_HW_REV_ADDR_LOWER    0xFC00
#define VP792_AMBA_REG_HW_REV_OFFSET        0x20
#define VP792_AMBA_REG_HW_REV_LEN           0x4
#define VP792_AMBA_REG_HW_REV_MJR_WORD_0    0x0
#define VP792_AMBA_REG_HW_REV_MJR_WORD_1    0x1
#define VP792_AMBA_REG_HW_REV_MJR_WORD_1_REV_MASK 0x00F8
#define VP792_AMBA_REG_HW_REV_MNR_WORD_0    0x2
#define VP792_AMBA_REG_HW_REV_MNR_WORD_1    0x3


/* ==============================
    Channel Register Definitions
   ============================== */

/* 0x02 - 0x0C : Signal Generator Parameters */
#define VP792_REG_SIG_GEN_OFFSET            0x02
#define VP792_REG_SIG_GEN_LEN               11
#define VP792_REG_SIG_GEN_BIAS_OFFSET       0x02
#define VP792_REG_SIG_GEN_FRQA_HIGH_OFFSET  0x03
#define VP792_REG_SIG_GEN_FRQA_OFFSET       0x04
#define VP792_REG_SIG_GEN_AMPA_OFFSET       0x05
#define VP792_REG_SIG_GEN_FRQB_OFFSET       0x06
#define VP792_REG_SIG_GEN_AMPB_OFFSET       0x07
#define VP792_REG_SIG_GEN_FRQC_OFFSET       0x08
#define VP792_REG_SIG_GEN_AMPC_OFFSET       0x09
#define VP792_REG_SIG_GEN_FRQD_OFFSET       0x0A
#define VP792_REG_SIG_GEN_AMPD_OFFSET       0x0B
#define VP792_REG_SIG_GEN_CTL_WORD          10
#define VP792_REG_SIG_GEN_CTL_DISABLE       0x0000
#define VP792_REG_SIG_GEN_CTL_DCGA_VP       0x0000
#define VP792_REG_SIG_GEN_CTL_DCGA_TIP      0x0100
#define VP792_REG_SIG_GEN_CTL_DCGA_RING     0x0200
#define VP792_REG_SIG_GEN_CTL_ETA           0x0001
#define VP792_REG_SIG_GEN_CTL_ETB           0x0002
#define VP792_REG_SIG_GEN_CTL_ETC           0x0004
#define VP792_REG_SIG_GEN_CTL_ETD           0x0008

/* 0x10 - 0x1F : FSK Generation Parameters */
#define VP792_REG_FSK_GEN_OFFSET            0x10
#define VP792_REG_FSK_BUF_OFFSET            0x13
#define VP792_REG_FSK_CTL_OFFSET            0x1F
#define VP792_REG_FSK_CTL_LEN               1
#define VP792_REG_FSK_CTL_EFSK_MASK         0x0001
#define VP792_REG_FSK_BUF_LEN               (VP792_CID_BUFSIZE * 2)

/* 0x22 - 0x2C: Ring Configuration Register */
#define VP792_REG_RING_OFFSET               0x22
#define VP792_REG_RING_LEN                  11
#define VP792_REG_RING_RTRIP_ST_WORD        5
#define VP792_REG_RING_EBR_WORD             5
#define VP792_REG_RING_EXR_WORD             5
#define VP792_REG_RING_EBR_MASK             0x0600
#define VP792_REG_RING_EXR_MASK             0x0100
#define VP792_REG_RING_RTRIP_ST_MASK        0x001F
#define VP792_REG_RING_RTRIP_ST_POL         0x0010
#define VP792_REG_RING_RTRIP_ST_DISABLED    VP792_REG_DRIVE_ST_ST_DISABLED
#define VP792_REG_RING_RTRIP_ST_DISCONNECT  VP792_REG_DRIVE_ST_ST_DISCONNECT
#define VP792_REG_RING_RTRIP_ST_NULLFEED    VP792_REG_DRIVE_ST_ST_NULLFEED
#define VP792_REG_RING_RTRIP_ST_STANDBY     VP792_REG_DRIVE_ST_ST_STANDBY
#define VP792_REG_RING_RTRIP_ST_LEADOPEN    VP792_REG_DRIVE_ST_ST_LEADOPEN
#define VP792_REG_RING_RTRIP_ST_ACTIVE      VP792_REG_DRIVE_ST_ST_ACTIVE
#define VP792_REG_RING_RTRIP_ST_HOWLER      VP792_REG_DRIVE_ST_ST_HOWLER
#define VP792_REG_RING_RTRIP_ST_RINGING     VP792_REG_DRIVE_ST_ST_RINGING
#define VP792_REG_RING_RTRIP_ST_TESTING     VP792_REG_DRIVE_ST_ST_TESTING

/* 0x32 - 0x39 : Loop Supervision Configuration Register */
#define VP792_REG_LOOP_SUP_OFFSET           0x32
#define VP792_REG_LOOP_SUP_LEN              11
#define VP792_REG_LOOP_SUP_DTMF_REG_MASK    0x0070
#define VP792_REG_LOOP_SUP_DTMF_REG_ATT     0x0000
#define VP792_REG_LOOP_SUP_DTMF_REG_NTT     0x0010
#define VP792_REG_LOOP_SUP_DTMF_REG_AUS     0x0020
#define VP792_REG_LOOP_SUP_DTMF_REG_BRZL    0x0030
#define VP792_REG_LOOP_SUP_DTMF_REG_ETSI    0x0040
#define VP792_REG_LOOP_SUP_RSH_BIT          0x0080
#define VP792_REG_LOOP_SUP_TGK_WORD         1
#define VP792_REG_LOOP_SUP_PGK_WORD         2
#define VP792_REG_LOOP_SUP_TSH_WORD         3
#define VP792_REG_LOOP_SUP_DSH_WORD         4
#define VP792_REG_LOOP_SUP_IFTA_WORD        5
#define VP792_REG_LOOP_SUP_IFTD_WORD        6
#define VP792_REG_LOOP_SUP_BOSH_WORD        7

#define VP792_REG_INTR_TEST_OFFSET                          0x3E
#define VP792_REG_INTR_TEST_OFFSET_LEN                      6
#define	VP792_REG_INTR_TEST_VABG_MASK			            (0x0030)
#define	VP792_REG_INTR_TEST_VACB_MASK			            (0x0004)
#define	VP792_REG_INTR_TEST_DCBW_MASK			            (0x0003)
#define	VP792_REG_INTR_TEST_VA_ADC_SEL_MASK                 (0xc000)
#define	VP792_REG_INTR_TEST_VB_ADC_SEL_MASK                 (0x3000)
#define	VP792_REG_INTR_TEST_ILG_ADC_SEL_MASK	            (0x0070)
#define	VP792_REG_INTR_TEST_BSEL_MASK                       (0x3000)
#define	VP792_REG_INTR_TEST_FXS_ST_MASK                     (0x0007)

#define	VP792_REG_INTR_TEST_VABG_SHIFT                      (4)
#define	VP792_REG_INTR_TEST_VACB_SHIFT                      (2)
#define	VP792_REG_INTR_TEST_DCBW_SHIFT                      (0)
#define	VP792_REG_INTR_TEST_VA_ADC_SEL_SHIFT                (14)
#define	VP792_REG_INTR_TEST_VB_ADC_SEL_SHIFT                (12)
#define	VP792_REG_INTR_TEST_ILG_ADC_SEL_SHIFT               (4)
#define	VP792_REG_INTR_TEST_BSEL_SHIFT                      (12)
#define	VP792_REG_INTR_TEST_FXS_ST_SHIFT                    (0)

#define VP792_REG_INTR_TEST_VABG_1X                         (0x0000)
#define VP792_REG_INTR_TEST_VABG_25X                        (0x0001)
#define VP792_REG_INTR_TEST_VABG_1_3X                       (0x0002)

#define VP792_REG_INTR_TEST_DCBW_LOW                        (0x0000)
#define VP792_REG_INTR_TEST_DCBW_MED                        (0x0001)
#define VP792_REG_INTR_TEST_DCBW_HI                         (0x0003)
#define VP792_REG_INTR_TEST_ADC_Vx_SEL_SVx                  (0x0000)
#define VP792_REG_INTR_TEST_ADC_Vx_SEL_Ix                   (0x0001)
#define VP792_REG_INTR_TEST_ADC_Vx_SEL_0uA                  (0x0002)
#define VP792_REG_INTR_TEST_ILG_ADC_SEL_VILG                (0x0000)
#define VP792_REG_INTR_TEST_ILG_ADC_SEL_IA                  (0x0001)
#define VP792_REG_INTR_TEST_ILG_ADC_SEL_IB                  (0x0002)
#define VP792_REG_INTR_TEST_ILG_ADC_SEL_0uA                 (0x0004)
#define VP792_REG_INTR_TEST_ILG_ADC_SEL_XSB                 (0x0007)
#define	VP792_REG_INTR_TEST_BSEL_GND_LBAT                   (0x0000)
#define	VP792_REG_INTR_TEST_BSEL_GND_HBAT                   (0x0001)
#define	VP792_REG_INTR_TEST_BSEL_PBAT_LBAT                  (0x0002)
#define	VP792_REG_INTR_TEST_BSEL_PBAT_HBAT                  (0x0003)

#define VP792_REG_INTR_TEST_BS0_MASK                        0x1000
#define VP792_REG_INTR_TEST_BS1_MASK                        0x2000
#define VP792_REG_INTR_TEST_BS0_SHIFT                       12
#define VP792_REG_INTR_TEST_BS1_SHIFT                       13

/* 0x46 : Read Metering Cancel Amplitude */
#define VP792_REG_MGEN_OFFSET               0x46
#define VP792_REG_MGEN_LEN                  2

/* 0x4A : DC Feed Register */
#define VP792_REG_DC_FEED_SLOPE_OFFSET      0x4A
#define VP792_REG_DC_FEED_SLOPE_LEN         1
#define VP792_REG_DC_FEED_OFFSET            0x4B
#define VP792_REG_DC_FEED_LEN               1
#define VP792_REG_DC_FEED_POLRR_MASK        0x0004
#define VP792_REG_DC_FEED_POLRR_SMOOTH      0x0000
#define VP792_REG_DC_FEED_POLRR_ABRUPT      0x0004
#define VP792_REG_DC_FEED_BSET_MASK         0x0003
#define VP792_REG_DC_FEED_BSET_SHIFT        (0)
#define VP792_REG_DC_FEED_BSET_LBAT         0x0000
#define VP792_REG_DC_FEED_BSET_HBAT         0x0001
#define VP792_REG_DC_FEED_BSET_AUTO         0x0002
#define VP792_REG_DC_FEED_BSET_BOOST        0x0003

/* 0x4C : Channel Mask 1 Register */
#define VP792_REG_CH_MASKS_OFFSET           0x4C
#define VP792_REG_CH_MASKS_LEN              1
#define VP792_REG_CH_MASKS_HK_M_MASK        0x0001
#define VP792_REG_CH_MASKS_GK_M_MASK        0x0002
#define VP792_REG_CH_MASKS_HKR_M_MASK       0x0004
#define VP792_REG_CH_MASKS_DG_M_MASK        0x0008
#define VP792_REG_CH_MASKS_TFT_M_MASK       0x0010
#define VP792_REG_CH_MASKS_DCFT_M_MASK      0x0020
#define VP792_REG_CH_MASKS_ACFT_M_MASK      0x0040
#define VP792_REG_CH_MASKS_RCAD_M_MASK      0x0080
#define VP792_REG_CH_MASKS_CB_M_MASK        0x0100
#define VP792_REG_CH_MASKS_TN_M_MASK        0x0200
#define VP792_REG_CH_MASKS_USER_M_MASK      0x0400
#define VP792_REG_CH_MASKS_CHKS_M_MASK      0x0800
#define VP792_REG_CH_MASKS_FLASH_M_MASK     0x1000
#define VP792_REG_CH_MASKS_PDG_M_MASK       0x2000
#define VP792_REG_CH_MASKS_SEQ_M_MASK       0x4000
#define VP792_REG_CH_MASKS_PREQ_HK_M_MASK   0x8000
#define VP792_REG_CH_MASKS_UNMASK_ALL       0x0000

/* 0x4D : Channel Mask 2 Register */
#define VP792_REG_CH_MASKS2_OFFSET          0x4D
#define VP792_REG_CH_MASKS2_LEN             1
#define VP792_REG_CH_MASKS2_MTR_CAD_M_MASK  0x0001
#define VP792_REG_CH_MASKS2_MTR_CMP_M_MASK  0x0002
#define VP792_REG_CH_MASKS2_PCM_DATAFRAME_M_MASK 0x0008
#define VP792_REG_CH_MASKS2_UNMASK_ALL      0x0000

/* 0x4F : Read Channel Supervision Status */
#define VP792_REG_CH_SUP_OFFSET             0x4F
#define VP792_REG_CH_SUP_LEN                1
#define VP792_REG_CH_SUP_HK_ST_MASK         0x0001
#define VP792_REG_CH_SUP_GK_ST_MASK         0x0002
#define VP792_REG_CH_SUP_HKR_ST_MASK        0x0004
#define VP792_REG_CH_SUP_TFT_ST_MASK        0x0010
#define VP792_REG_CH_SUP_DCFT_ST_MASK       0x0020
#define VP792_REG_CH_SUP_ACFT_ST_MASK       0x0040

/* 0x50 : Read Channel Status Register */
#define VP792_REG_CH_STAT_OFFSET            0x50
#define VP792_REG_CH_STAT_LEN               1
#define VP792_REG_CH_STAT_CB_ST_MASK        0x0400
#define VP792_REG_CH_STAT_DCFR_MASK         0x0300
#define VP792_REG_CH_STAT_DCFR_UNKNOWN      0x0000
#define VP792_REG_CH_STAT_DCFR_VAS          0x0100
#define VP792_REG_CH_STAT_DCFR_IL           0x0200
#define VP792_REG_CH_STAT_DCFR_RF           0x0300
#define VP792_REG_CH_STAT_IO_ST_MASK        0x00C0
#define VP792_REG_CH_STAT_IO1_ST_MASK       0x0080
#define VP792_REG_CH_STAT_IO0_ST_MASK       0x0040
#define VP792_REG_CH_STAT_IO_ST_SHIFT       6
#define VP792_REG_CH_STAT_TIP_MASK          0x0020
#define VP792_REG_CH_STAT_TMODE_MASK        0x0010
#define VP792_REG_CH_STAT_SEQ_MASK          0x0008
#define VP792_REG_CH_STAT_SB_BUSY_MASK      0x0004
#define VP792_REG_CH_STAT_BSTAT_MASK        0x0003
#define VP792_REG_CH_STAT_BSTAT_LBAT        0x0000
#define VP792_REG_CH_STAT_BSTAT_HBAT        0x0001
#define VP792_REG_CH_STAT_BSTAT_PBAT        0x0003

/* 0x53 : Read VSAB */
#define VP792_REG_VSAB_OFFSET               0x53
#define VP792_REG_VSAB_LEN                  1

/* 0x54 : Read Metallic Loop Current */
#define VP792_REG_IMT_OFFSET                0x54
#define VP792_REG_IMT_LEN                   1

/* 0x55 : Read VSAB */
#define VP792_REG_ILG_OFFSET                0x55
#define VP792_REG_ILG_LEN                   1

/* 0x56 : Read Loop Resistance */
#define VP792_REG_RLOOP_OFFSET              0x56
#define VP792_REG_RLOOP_LEN                 1

/* All loop conditions registers in a single read */
#define VP792_REG_LOOP_COND_OFFSET          0x53
#define VP792_REG_LOOP_COND_LEN             4
#define VP792_REG_LOOP_COND_VSAB_WORD       0
#define VP792_REG_LOOP_COND_IMT_WORD        1
#define VP792_REG_LOOP_COND_ILG_WORD        2
#define VP792_REG_LOOP_COND_RLOOP_WORD      3

/* 0x5A : Drive State Register */
#define VP792_REG_DRIVE_ST_OFFSET           0x5A
#define VP792_REG_DRIVE_ST_LEN              1
#define VP792_REG_DRIVE_ST_FULL_ST_MASK     0x001F
#define VP792_REG_DRIVE_ST_POL_MASK         0x0010
#define VP792_REG_DRIVE_ST_POL_NORMAL       0x0000
#define VP792_REG_DRIVE_ST_POL_REVERSE      0x0010
#define VP792_REG_DRIVE_ST_ST_MASK          0x000F
#define VP792_REG_DRIVE_ST_ST_DISABLED      0x0000
#define VP792_REG_DRIVE_ST_ST_DISCONNECT    0x0001
#define VP792_REG_DRIVE_ST_ST_NULLFEED      0x0002
#define VP792_REG_DRIVE_ST_ST_STANDBY       0x0003
#define VP792_REG_DRIVE_ST_ST_LEADOPEN      0x0004
#define VP792_REG_DRIVE_ST_ST_ACTIVE        0x0005
#define VP792_REG_DRIVE_ST_ST_RINGING       0x0006
#define VP792_REG_DRIVE_ST_ST_HOWLER        0x0007
#define VP792_REG_DRIVE_ST_ST_TESTING       0x0008

/* 0x5B : I/O State Register */
#define VP792_REG_IO_ST_OFFSET              0x5B
#define VP792_REG_IO_ST_LEN                 1
#define VP792_REG_IO_ST_TSW_MASK            0x0200
#define VP792_REG_IO_ST_IO_0_MODE_MASK      0x0100
#define VP792_REG_IO_ST_IO_0_MODE_SHIFT     8
#define VP792_REG_IO_ST_ELLD_MASK           0x0080
#define VP792_REG_IO_ST_LCAS_MASK           0x0070
#define VP792_REG_IO_ST_IO_X_CFG_MASK       0x000C
#define VP792_REG_IO_ST_IO_X_CFG_SHIFT      2

#define VP792_REG_IO_ST_IO_1_CFG_MASK       0x0008
#define VP792_REG_IO_ST_IO_1_CFG_IN         0x0000
#define VP792_REG_IO_ST_IO_1_CFG_OUT        0x0008

#define VP792_REG_IO_ST_IO_0_CFG_MASK       0x0004
#define VP792_REG_IO_ST_IO_0_CFG_IN         0x0000
#define VP792_REG_IO_ST_IO_0_CFG_OUT        0x0004

#define VP792_REG_IO_ST_IO_X_MASK           0x0003
#define VP792_REG_IO_ST_IO_1_MASK           0x0002
#define VP792_REG_IO_ST_IO_0_MASK           0x0001

/* 0x5C : Voice Path Config 1 Register */
#define VP792_REG_VP_CFG1_OFFSET            0x5C
#define VP792_REG_VP_CFG1_LEN               1
#define VP792_REG_VP_CFG1_CODEC_MASK        0x00C0
#define VP792_REG_VP_CFG1_CODEC_SHIFT       6
#define VP792_REG_VP_CFG1_CODEC_ALAW        0x0000
#define VP792_REG_VP_CFG1_CODEC_ULAW        0x0040
#define VP792_REG_VP_CFG1_CODEC_LINEAR      0x0080
#define VP792_REG_VP_CFG1_EGR_MASK          0x0020
#define VP792_REG_VP_CFG1_EGX_MASK          0x0010
#define VP792_REG_VP_CFG1_EX_MASK           0x0008
#define VP792_REG_VP_CFG1_ER_MASK           0x0004
#define VP792_REG_VP_CFG1_EZ_MASK           0x0002
#define VP792_REG_VP_CFG1_EB_MASK           0x0001
#define VP792_REG_VP_CFG1_FILTERS_MASK      0x003F
#define VP792_REG_VP_CFG1_FILTERS_DEFAULT   0x0000
#define VP792_REG_VP_CFG1_FILTERS_USER      0x003F

/* 0x5D : Voice Path Config 2 Register */
#define VP792_REG_VP_CFG2_OFFSET            0x5D
#define VP792_REG_VP_CFG2_LEN               1
#define VP792_REG_VP_CFG2_CTP_MASK          0x0080
#define VP792_REG_VP_CFG2_CRP_MASK          0x0040
#define VP792_REG_VP_CFG2_DHP_MASK          0x0020
#define VP792_REG_VP_CFG2_LRG_MASK          0x0010
#define VP792_REG_VP_CFG2_TSLB_MASK         0x0004
#define VP792_REG_VP_CFG2_TONE_MASK         0x0001

/* 0x5E : PCM Transmit Timeslot Config Register */
#define VP792_REG_TXTS_CFG_OFFSET           0x5E
#define VP792_REG_TXTS_CFG_LEN              1
#define VP792_REG_TXTS_CFG_THWY_MASK        0x0300
#define VP792_REG_TXTS_CFG_THWY_A           0x0100
#define VP792_REG_TXTS_CFG_THWY_B           0x0200
#define VP792_REG_TXTS_CFG_THWY_AB          0x0300
#define VP792_REG_TXTS_CFG_TTS_MASK         0x007F

/* 0x5F : PCM Receive Timeslot Config Register */
#define VP792_REG_RXTS_CFG_OFFSET           0x5F
#define VP792_REG_RXTS_CFG_LEN              1
#define VP792_REG_RXTS_CFG_RHWY_MASK        0x0100
#define VP792_REG_RXTS_CFG_RHWY_A           0x0000
#define VP792_REG_RXTS_CFG_RHWY_B           0x0100
#define VP792_REG_RXTS_CFG_RTS_MASK         0x007F

/* 0x60 : Calibration coefficients */
#define VP792_REG_CAL_COEFS                 0x60
#define VP792_REG_CAL_COEFS_LEN             32

/* =============================
    VP792 Sequencer Definitions
   ============================= */

/* Sequencer control values */
#define VP792_SEQ_CTL_STOP              0x0000
#define VP792_SEQ_CTL_START             0x0100
#define VP792_SEQ_CTL_RESTART           0x0200
#define VP792_SEQ_CTL_RESUME            0x0300
#define VP792_SEQ_CTL_BREAK             0x0500
#define VP792_SEQ_CTL_BREAK_AND_RESUME  0x0600  /* API internal value */
#define VP792_SEQ_CTL_LENGTH_MASK       0x001F

/* Command type masks */
#define VP792_SEQ_STD_CMD_MASK          0xFF00
#define VP792_SEQ_LONG_CMD_MASK         0xF000
#define VP792_SEQ_VLONG_CMD_MASK        0xC000

/* Data type masks */
#define VP792_SEQ_STD_DATA_MASK         0x00FF
#define VP792_SEQ_LONG_DATA_MASK        0x0FFF
#define VP792_SEQ_VLONG_DATA_MASK       0x3FFF

/* Standard commands */
#define VP792_SEQ_CMD_USER_EVENT        0x0100
#define VP792_SEQ_CMD_DRIVE_ST          0x0200
#define VP792_SEQ_CMD_POLREV            0x0300
#define VP792_SEQ_CMD_IO_ST             0x0400
#define VP792_SEQ_CMD_GEN_CTRL          0x0500
#define VP792_SEQ_CMD_METER             0x0600
#define VP792_SEQ_CMD_VP1_CTRL          0x0700
#define VP792_SEQ_CMD_VP2_CTRL          0x0800
#define VP792_SEQ_CMD_PAUSE             0x0900
#define VP792_SEQ_CMD_PAUSE_UE          0x0980
#define VP792_SEQ_CMD_TIMESTAMP         0x0A00
#define VP792_SEQ_CMD_EN_FSK            0x0B00

/* Long data commands */
#define VP792_SEQ_CMD_BRANCH            0x1000

/* Very long data commands */
#define VP792_SEQ_CMD_DELAY             0x4000
#define VP792_SEQ_CMD_GENB_AMP          0x8000

/* Only the last 4 bits of the drive state apply */
#define VP792_SEQ_DRIVE_ST_DATA_MASK    0x000F

/* Branch data:  Bits 0-4 are destination, 5-11 are number of iterations */
#define VP792_SEQ_BRANCH_TARGET_MASK    0x001F
#define VP792_SEQ_BRANCH_ITERATE_MASK   0x0FE0
#define VP792_SEQ_BRANCH_ITERATE_POS    5
#define VP792_SEQ_BRANCH_ITERATE_MAX    (VP792_SEQ_BRANCH_ITERATE_MASK >> VP792_SEQ_BRANCH_ITERATE_POS)

/* Delay data: Most significant bit determines relative/absolute mode.  Rest of
 * data is time in ms up to 8192. */
#define VP792_SEQ_DELAY_MODE_MASK       0x2000
#define VP792_SEQ_DELAY_MODE_ABSOLUTE   0x0000
#define VP792_SEQ_DELAY_MODE_RELATIVE   0x2000
#define VP792_SEQ_DELAY_DATA_MASK       0x1FFF


/* =============================
    Mailbox Content Definitions
   ============================= */

/* Mailbox field indexes */
#define VP792_MBOX_LENGTH_INDEX     0
#define VP792_MBOX_ID_CHAN_INDEX    1
#define VP792_MBOX_PAYLOAD_INDEX    2

/* Mailbox header bitfields */
#define VP792_MBOX_IDCHAN_CHAN_MASK 0x000F
#define VP792_MBOX_IDCHAN_CHAN_POS  0
#define VP792_MBOX_IDCHAN_CMD_MASK  0xFF00
#define VP792_MBOX_IDCHAN_CMD_POS   8

#define VP792_MBOX_MORE_COMMANDS    1
#define VP792_MBOX_LAST_COMMAND     0

/* Mailbox command IDs */
typedef enum {
    VP792_CMD_TIMER_CTL = 0x05,
    VP792_CMD_CAL_CTL = 0x07,
    VP792_CMD_RESET_CTL = 0x0A,
    VP792_CMD_USER_CTL = 0x0C,
    VP792_CMD_WR_AFE_CONFIG = 0x1C,
    VP792_CMD_RD_AFE_CONFIG = 0x1D,
    VP792_CMD_FLUSH_EVENTS = 0x1E,
    VP792_CMD_HMI_PASSTHRU = 0x1F,
    VP792_CMD_TST_RESERVE = 0x20,
    VP792_CMD_TST_RELEASE = 0x21,
    VP792_CMD_TST_FLT_CFG = 0x22,
    VP792_CMD_TST_SG_CFG = 0x24,
    VP792_CMD_TST_MSRMT = 0x26,
    VP792_CMD_WR_BAT_OVR = 0x28,
    VP792_CMD_RD_BAT_OVR = 0x29,
    VP792_CMD_WR_AC_PARAM = 0x2A,
    VP792_CMD_RD_AC_PARAM = 0x2B,
    VP792_CMD_WR_DC_PARAM = 0x2C,
    VP792_CMD_RD_DC_PARAM = 0x2D,
    VP792_CMD_WR_REL_GAIN = 0x2E,
    VP792_CMD_RD_REL_GAIN = 0x2F,
    VP792_CMD_WR_SEQ_CTL = 0x30,
    VP792_CMD_RD_SEQ = 0x31,
    VP792_CMD_PATCH_CTL = 0x32,
    VP792_CMD_RD_PULSE_STATS = 0x33,
    VP792_CMD_WR_PULSE_PARAMS = 0x34,
    VP792_CMD_RD_PULSE_PARAMS = 0x35,
    VP792_CMD_WR_MTR_CTL = 0x36,
    VP792_CMD_RD_MTR_CTL = 0x37,
    VP792_CMD_WR_MTR_SHADOW = 0x38,
    VP792_CMD_RD_MTR_SHAWDOW = 0x39,
    VP792_CMD_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792MboxCmdId;

/* Size (in words) of the various mailbox fields: */
#define VP792_HEADER_FLD   (2)
#define VP792_CONTEXT_FLD  (1)
#define VP792_ARG_FLD      (1)
#define VP792_HANDLE_FLD   (1)
#define VP792_TST_FILTER_FLD   (5)

/* Mailbox buffer size requirement for each mailbox command: */
#define VP792_CMDSIZE_TIMER_CTL         (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 2 * VP792_ARG_FLD)
#define VP792_CMDSIZE_CAL_CTL           (VP792_HEADER_FLD)
#define VP792_CMDSIZE_RESET_CTL         (VP792_HEADER_FLD)
#define VP792_CMDSIZE_USER_CTL          (VP792_HEADER_FLD + VP792_ARG_FLD)
#define VP792_CMDSIZE_WR_AFE_CONFIG     (VP792_HEADER_FLD + VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_AFE_CONFIG     (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_FLUSH_EVENTS      (VP792_HEADER_FLD)
#define VP792_CMDSIZE_HMI_PASSTHRU      (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 6)
#define VP792_CMDSIZE_TST_RESERVE       (VP792_HEADER_FLD)
#define VP792_CMDSIZE_TST_RELEASE       (VP792_HEADER_FLD)
#define VP792_CMDSIZE_TST_FLT_CFG       (VP792_HEADER_FLD + VP792_ARG_FLD + (2 * VP792_TST_FILTER_FLD))
#define VP792_CMDSIZE_TST_SG_CFG        (VP792_HEADER_FLD + 11 * VP792_ARG_FLD)
#define VP792_CMDSIZE_TST_MSRMT         (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 9 * VP792_ARG_FLD)
#define VP792_CMDSIZE_WR_BAT_OVR        (VP792_HEADER_FLD + 4 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_BAT_OVR        (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_AC_PARAM       (VP792_HEADER_FLD + 40 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_AC_PARAM       (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_DC_PARAM       (VP792_HEADER_FLD + 6 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_DC_PARAM       (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_REL_GAIN       (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 2 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_REL_GAIN       (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_SEQ_CTL        (VP792_HEADER_FLD + 33 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_SEQ            (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_PATCH_CTL         (VP792_HEADER_FLD + 4 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_PULSE_STATS    (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_PULSE_PARAMS   (VP792_HEADER_FLD + 9 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_PULSE_PARAMS   (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_MTR_CTL        (VP792_HEADER_FLD + 11 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_MTR_CTL        (VP792_HEADER_FLD + VP792_CONTEXT_FLD)
#define VP792_CMDSIZE_WR_MTR_SHADOW     (VP792_HEADER_FLD + 4 * VP792_ARG_FLD)
#define VP792_CMDSIZE_RD_MTR_SHAWDOW    (VP792_HEADER_FLD + VP792_CONTEXT_FLD)

#if (_VP792_CMDSIZE_WR_SEQ_CTL != VP792_CMDSIZE_WR_SEQ_CTL)
#error
#endif

/* Mailbox buffer size requirement for each mailbox response: */
#define VP792_RSPSIZE_RD_AFE_CONFIG     (VP792_HEADER_FLD + VP792_CONTEXT_FLD + VP792_ARG_FLD)
#define VP792_RSPSIZE_TST_MSRMT         (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 4 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_BAT_OVR        (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 4 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_AC_PARAM       (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 39 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_DC_PARAM       (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 6 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_REL_GAIN       (VP792_HEADER_FLD + VP792_CONTEXT_FLD + (2 + 5) * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_SEQ            (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 32 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_PULSE_STATS    (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 8 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_PULSE_PARAMS   (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 9 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_MTR_CTL        (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 10 * VP792_ARG_FLD)
#define VP792_RSPSIZE_RD_MTR_SHADOW     (VP792_HEADER_FLD + VP792_CONTEXT_FLD + 3 * VP792_ARG_FLD)

/* Calculate the maximum size needed for a response mailbox buffer: */
#define VP792_MAX_RSPSIZE_CALCULATED MAX12( \
    VP792_RSPSIZE_RD_AFE_CONFIG, VP792_RSPSIZE_TST_MSRMT, VP792_RSPSIZE_RD_BAT_OVR, \
    VP792_RSPSIZE_RD_AC_PARAM, VP792_RSPSIZE_RD_DC_PARAM, 0, \
    VP792_RSPSIZE_RD_REL_GAIN, VP792_RSPSIZE_RD_SEQ, VP792_RSPSIZE_RD_PULSE_STATS, \
    VP792_RSPSIZE_RD_PULSE_PARAMS, VP792_RSPSIZE_RD_MTR_CTL, VP792_RSPSIZE_RD_MTR_SHADOW)
#if (VP792_MAX_RSPSIZE != VP792_MAX_RSPSIZE_CALCULATED)
#error "VP792_MAX_RSPSIZE definition needs to be updated."
#endif

/* WR_MTR_CTL command payload values */
#define VP792_MTR_CTL_STOP          0x0000
#define VP792_MTR_CTL_START         0x0001
#define VP792_MTR_CTL_RESET         0x0002

#define VP792_MTR_TYPE_POLREV_PULSE  0x00
#define VP792_MTR_TYPE_RESERVED      0x01
#define VP792_MTR_TYPE_12KHZ_TONE    0x02
#define VP792_MTR_TYPE_16KHZ_TONE    0x03

/* WR_CID_BUF command payload values */
#define VP792_CID_BUFSIZE           6
#define VP792_CID_CTL_STOP          0x0000

/* Bit fields for CID buffer words */
#define VP792_CID_DATA              0x00FF
#define VP792_CID_CONTROL           0xFF00
#define VP792_CID_CONTROL_NO_FRAME  0x0100
#define VP792_CID_CONTROL_LAST_BYTE 0x0200

/* WR_AFE_CONFIG command payload values */
#define VP792_MB_AFE_CONFIG_FDLB_MASK       1

/* WR_AC_PARAM command AC_MASK values */
#define VP792_MB_WR_AC_PARAM_AC_MASK_BFIL   0x0008

/* PATCH_CTL command payload values */
#define VP792_CMD_PATCH_CTL_CMD_KEY_HIGH     (0xA5A5)
#define VP792_CMD_PATCH_CTL_CMD_KEY_LOW		 (0xC030)

/* TIMER_CTL command payload values */
#define VP792_CMD_GEN_TIMER_STOP             0x0000
#define VP792_CMD_GEN_TIMER_START            0x0001
#define VP792_CMD_GEN_TIMER_DELAY_MAX        0xFFFF


/* =============================
    Profile Content Definitions
   ============================= */

/* Types of profile supported for VP792 devices: */
typedef enum {
  VP792_PROFTYPE_AC      = 0x00, /* = VP_PRFWZ_PROFILE_AC */
  VP792_PROFTYPE_DC      = 0x01, /* = VP_PRFWZ_PROFILE_DC */
  VP792_PROFTYPE_TONE    = 0x02, /* = VP_PRFWZ_PROFILE_TONE */
  VP792_PROFTYPE_TONECAD = 0x03, /* = VP_PRFWZ_PROFILE_TONECAD */
  VP792_PROFTYPE_RING    = 0x04, /* = VP_PRFWZ_PROFILE_RING */
  VP792_PROFTYPE_CID     = 0x05, /* = VP_PRFWZ_PROFILE_CID_TYPE1 */
  VP792_PROFTYPE_METER   = 0x07, /* = VP_PRFWZ_PROFILE_METER */
  VP792_PROFTYPE_RINGCAD = 0x08, /* = VP_PRFWZ_PROFILE_RINGCAD */
  VP792_PROFTYPE_DEVICE  = 0xFF, /* = VP_PRFWZ_PROFILE_DEVICE */
  VP792_PROFTYPE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileType;

/* Top-level VP792 profile fields */
typedef enum
{
  VP792_PROF_FLDOFFSET_VERSION = 0,
  VP792_PROF_FLDOFFSET_PROFTYPE = 1,
  VP792_PROF_FLDOFFSET_NUM_SECS = 2,
  VP792_PROF_FLDOFFSET_CONTENT_LEN = 3,
  VP792_PROF_FLDOFFSET_CONTENT = 4,
  VP792_PROF_FLDOFFSET_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileFieldOffset;

/* Section types appearing in VP792 profiles */
typedef enum {
  VP792_PROF_SECTYPE_REGLIST = 0x00,
  VP792_PROF_SECTYPE_MBOXCMD = 0x01,
  VP792_PROF_SECTYPE_SEQUENCE = 0x02,

  /* The following values don't actually appear in VP792 device profiles,
     but are used internally by the API. */
  VP792_PROF_SECTYPE_NONE = 0xFD,
  VP792_PROF_SECTYPE_UNSTRUCTURED = 0xFE,
  VP792_PROF_SECTYPE_ANY = 0xFF,
  VP792_PROF_SECTYPE_NOT_FOUND = 0xFF,

  VP792_PROF_SECTYPE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileSectionType;

/* Section header fields */
typedef enum {
  VP792_PROF_SEC_FLDOFFSET_SECTYPE = 0,
  VP792_PROF_SEC_FLDOFFSET_CONTENT_LEN = 1,
  VP792_PROF_SEC_FLDOFFSET_CONTENT = 2,
  VP792_PROF_SEC_FLDOFFSET_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileSectionFieldOffset;

/* Register access section fields */
typedef enum {
  VP792_PROF_REGAXS_FLDOFFSET_AXSTYPE = 0,
  VP792_PROF_REGAXS_FLDOFFSET_PAGEOFFSET = 1,
  VP792_PROF_REGAXS_FLDOFFSET_DATA_LEN = 2,
  VP792_PROF_REGAXS_FLDOFFSET_DATA = 3,
  VP792_PROF_REGAXS_FLDOFFSET_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileRegAccessFieldOffset;

/* Register access types */
typedef enum {
  VP792_PROF_REGAXSTYPE_DIRECT_PAGE = 0x00,
  VP792_PROF_REGAXSTYPE_CHANNEL = 0x01,
  VP792_PROF_REGAXSTYPE_CHANNEL_FXS = 0x02,
  VP792_PROF_REGAXSTYPE_CHANNEL_FXO = 0x03,
  VP792_PROF_REGAXSTYPE_INDIRECT_PAGE = 0x10,
  VP792_PROF_REGAXSTYPE_CL_PAGE = 0xFF,
  VP792_PROF_REGAXSTYPE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileRegAccessType;

/* Mailbox command section fields */
typedef enum {
  VP792_PROF_MBOXCMD_FLDOFFSET_CMDID = 0,
  VP792_PROF_MBOXCMD_FLDOFFSET_DATA = 1,
  VP792_PROF_MBOXCMD_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileMboxCmdFieldOffset;

/* Sequencer section fields */
typedef enum {
  VP792_PROF_SEQ_FLDOFFSET_LENGTH = 1,
  VP792_PROF_SEQ_FLDOFFSET_INSTRUCTIONS = 2,
  VP792_PROF_SEQ_FLDOFFSET_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE
} Vp792ProfileSequencerFieldOffset;

/* Offsets in the unstructured data section of a CID profile */
#define VP792_CID_PROF_DETECT_TONES    4
#define VP792_CID_PROF_CHECKSUM        6

/* Register list sections in the device profile: */
#define VP792_DEV_PROF_SEC_PREBOOT  0
#define VP792_DEV_PROF_SEC_POSTBOOT 1


/* =====================
    VP792 API Constants
   ===================== */

/* Various types of sequence that the API may send to the SLAC: */
#define VP792_SEQTYPE_OFF                  0
#define VP792_SEQTYPE_RING                 1
#define VP792_SEQTYPE_TONE                 2
#define VP792_SEQTYPE_CAL                  3
#define VP792_SEQTYPE_PARK                 4
#define VP792_SEQTYPE_CID                  5
#define VP792_SEQTYPE_SENDSIG              6


/* ====================================
    Prototypes for Top-Level Functions
   ==================================== */

/* vp792_config.c: */

EXTERN VpStatusType
Vp792MakeDeviceObject(
    VpDevCtxType *pDevCtx,
    Vp792DeviceObjectType *pDevObj);

EXTERN VpStatusType
Vp792MakeDeviceCtx(
    VpDevCtxType *pDevCtx,
    Vp792DeviceObjectType *pDevObj);

EXTERN VpStatusType
Vp792MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pLineObj,
    VpDevCtxType *pDevCtx);

/* vp792_init.c: */

EXTERN VpStatusType
Vp792CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode);

EXTERN VpStatusType
Vp792CalLine(
    VpLineCtxType *pLineCtx);

EXTERN VpStatusType
Vp792ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile);

EXTERN VpStatusType
Vp792InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

EXTERN VpStatusType
Vp792InitDevice(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pDevProfile,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcProfile,
    VpProfilePtrType pRingProfile,
    VpProfilePtrType pFxoAcProfile,
    VpProfilePtrType pFxoCfgProfile);

EXTERN VpStatusType
Vp792InitLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcOrFxoProfile,
    VpProfilePtrType pRingProfile);

EXTERN VpStatusType
Vp792InitMeter(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pMeterProfile);

EXTERN VpStatusType
Vp792InitProfile(
    VpDevCtxType *pDevCtx,
    VpProfileType type,
    VpProfilePtrType pProfileIndex,
    VpProfilePtrType pProfile);

EXTERN VpStatusType
Vp792InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpProfilePtrType pCidProfile);

VpStatusType
Vp792SetBatteries(
    VpLineCtxType *pLineCtx,
    VpBatteryModeType battMode,
    VpBatteryValuesType *battVal);

/* vp792_control.c: */

EXTERN VpStatusType
Vp792ContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

EXTERN VpStatusType
Vp792DeviceIoAccessExt(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessExtType *pDeviceIoAccess);

EXTERN VpStatusType
Vp792GenTimerCtrl(
    VpLineCtxType *pLineCtx,
    VpGenTimerCtrlType timerCtrl,
    uint32 duration,
    uint16 handle);

EXTERN VpStatusType
Vp792LineIoAccess(
    VpLineCtxType *pLineCtx,
    VpLineIoAccessType *pLineIoData,
    uint16 handle);

EXTERN VpStatusType
Vp792LowLevelCmd16(
    VpLineCtxType *pLineCtx,
    VpLowLevelCmdType cmdType,
    uint16 *writeWords,
    uint8 numWriteWords,
    uint8 numReadWords,
    uint16 handle);

EXTERN VpStatusType
Vp792SendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData);

EXTERN VpStatusType
Vp792SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType signalType,
    void *pSignalData);

EXTERN VpStatusType
Vp792SetBFilter(
    VpLineCtxType *pLineCtx,
    VpBFilterModeType bFiltMode,
    VpProfilePtrType pAcProfile);

EXTERN VpStatusType
Vp792SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

EXTERN VpStatusType
Vp792SetLineStateInt(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    bool criticalSection);

EXTERN VpStatusType
Vp792SetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl);

EXTERN VpStatusType
Vp792SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue);

EXTERN VpStatusType
Vp792SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState);

EXTERN VpStatusType
Vp792SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle);

EXTERN VpStatusType
Vp792StartMeter32Q(
    VpLineCtxType *pLineCtx,
    uint32 minDelay,
    uint32 onTime,
    uint32 offTime,
    uint16 numMeters,
    uint16 eventRate);

/* vp792_query.c: */

EXTERN VpStatusType
Vp792ClearResults(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792ClearResultsInt(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792FlushEvents(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp792GetDeviceStatusExt(
    VpDevCtxType *pDevCtx,
    VpDeviceStatusType *pDeviceStatus);

EXTERN bool
Vp792GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

EXTERN VpStatusType
Vp792GetLineStatus(
    VpLineCtxType *pLineCtx,
    VpInputType input,
    bool *pStatus);

EXTERN VpStatusType
Vp792GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle);

EXTERN VpStatusType
Vp792GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle);

EXTERN VpStatusType
Vp792GetResults(
    VpEventType *pEvent,
    void *pResults);

EXTERN VpStatusType
Vp792GetResultsInt(
    VpEventType *pEvent,
    void *pResults);

EXTERN bool
Vp792IdentifyResponseEvent(
    VpEventType *pEvent,
    uint16 eventIdWord,
    uint16 idx);

EXTERN VpStatusType
Vp792NewSequence(
    VpLineCtxType *pLineCtx,
    uint16 eventHandler,
    uint16 *pCallerBuf,
    uint8 callerBufLen);

EXTERN VpStatusType
Vp792ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx);


/* ================================
    Prototypes for Other Functions
   ================================ */

/* vp792_firmware.c: */

EXTERN VpStatusType
Vp792SelectPatch(
    VpDevCtxType *pDevCtx,
    uint16p *ppImage,
    uint32 *pLength,
    uint32 *pAddress);

/* vp792_common.c: */

EXTERN VpStatusType
Vp792CmdMailboxAcquire(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792CmdMailboxRelease(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792ConfigLineInternal(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpProfilePtrType *ppAcProfile,
    VpProfilePtrType *ppDcProfile,
    VpProfilePtrType *ppRingProfile,
    uint16p pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType sectionType);

EXTERN VpStatusType
Vp792GenerateUserEvent(
    VpDevCtxType *pDevCtx,
    uint16 userEventId,
    uint8 channelId);

EXTERN void
Vp792GetDtmfResources(
    VpDevCtxType *pDevCtx,
    VpOptionDtmfModeType *pDtmfMode);

EXTERN VpStatusType
Vp792GetFreeRequestIdx(
    VpDevCtxType *pDevCtx,
    uint16 *pRequestIdx);

EXTERN VpStatusType
Vp792GetProfileArg(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    VpProfilePtrType *ppProfile);

EXTERN VpProfilePtrType *
Vp792GetProfileTable(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    uint8 *pTableSize,
    uint8 *pProfSize);

EXTERN VpStatusType
Vp792HbiDirectPageRead(
    VpDevCtxType *pDevCtx,
    uint8 offset,
    uint8 numWords,
    uint16p pBuf);

EXTERN VpStatusType
Vp792HbiDirectPageWrite(
    VpDevCtxType *pDevCtx,
    uint8 offset,
    uint8 numWords,
    uint16p pSrc);

EXTERN VpStatusType
Vp792HbiNoOp(
    VpDevCtxType *pDevCtx,
    uint8 numWords);

EXTERN VpStatusType
Vp792HbiPagedRead(
    VpDevCtxType *pDevCtx,
    uint8 page,
    uint8 offset,
    uint8 numWords,
    uint16p pBuf);

EXTERN VpStatusType
Vp792HbiPagedWrite(
    VpDevCtxType *pDevCtx,
    uint8 page,
    uint8 offset,
    uint8 numWords,
    uint16p pSrc);

EXTERN VpStatusType
Vp792HbiSelectCLPage(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792HbiSharedIntRead(
    VpDeviceIdType deviceId,
    uint16p pBuf);

EXTERN VpStatusType
Vp792HbiSync(
    VpDevCtxType *pDevCtx);

EXTERN VpStatusType
Vp792InitRelayState(
    VpLineCtxType * pLineCtx);

EXTERN void
Vp792LoadCidData(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

EXTERN VpStatusType
Vp792LoadImage(
    VpDevCtxType *pDevCtx,
    const unsigned char *pImage,
    unsigned long imageLength);

EXTERN void
Vp792MailboxAddBytes(
    uint16p *ppBufEnd,
    uint8 *pData,
    int length);

EXTERN void
Vp792MailboxAddCmdHeader(
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792MboxCmdId commandId,
    uint8 channelId);

EXTERN VpStatusType
Vp792MailboxSend(
    VpDevCtxType *pDevCtx,
    uint16 *pBuf,
    uint16 *pBufEnd,
    uint16 maxBufLen);

EXTERN VpStatusType
Vp792MapLineState(
    VpLineStateType apiState,
    uint16p driveState);

EXTERN VpStatusType
Vp792ProfileFindSection(
    const uint8 *pProfile,
    Vp792ProfileSectionType *pSectionType,
    uint8 index,
    uint8p *ppContent,
    uint8p pLength);

EXTERN VpStatusType
Vp792ProfileProcessMboxCmd(
    uint8 channelId,
    uint8p pMboxCmdSec,
    int lengthBytes,
    uint16 *pBuf,
    uint16p *ppBufEnd);

EXTERN VpStatusType
Vp792ProfileProcessRegList(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8p pRegList,
    int lengthBytes);

EXTERN VpStatusType
Vp792ProfileProcessSection(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    const uint8 *pProfile,
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType *pSectionType,
    uint8 index);

EXTERN VpStatusType
Vp792ProfileProcessSections(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    const uint8 *pProfile,
    uint16 *pBuf,
    uint16p *ppBufEnd,
    Vp792ProfileSectionType sectionType);

EXTERN VpStatusType
Vp792ProfileProcessUnstructuredData(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 profileType,
    uint8p pData,
    uint8 length);

EXTERN void
Vp792ResetLineVars(
    VpLineCtxType *pLineCtx);

EXTERN void
Vp792ResetLineVars(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp792RspMailboxRead(
    VpDevCtxType *pDevCtx,
    uint8 mailboxLen,
    uint16 *pBuf);

EXTERN VpStatusType
Vp792RspMailboxRelease(
    VpDevCtxType *pDevCtx);

EXTERN void
Vp792SaveProfile(
    VpProfilePtrType pSource,
    uint8 *pDest,
    uint16 *pValidFlag,
    uint16 bit);

EXTERN void
Vp792SaveSequence(
    VpLineCtxType *pLineCtx,
    uint8 *pSequence,
    bool endian);

EXTERN VpStatusType
Vp792SendFskData(
    VpLineCtxType *pLineCtx);

EXTERN VpStatusType
Vp792SetOptionLineDtmfMode(
    VpLineCtxType *pLineCtx,
    VpOptionDtmfModeType *pDtmfMode);

EXTERN VpStatusType
Vp792SetPcmTxRxMode(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

EXTERN VpStatusType
Vp792StartMeterInternalExt(
    VpLineCtxType *pLineCtx,
    uint32 minDelay,
    uint32 onTime,
    uint32 offTime,
    uint16 numMeters,
    uint16 mtrEventRate);

EXTERN VpStatusType
Vp792StartSavedSequence(
    VpLineCtxType *pLineCtx,
    uint16 useType);

EXTERN VpStatusType
Vp792WriteMultiplePages(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 offset,
    uint8 numWords,
    uint16p pData);

VpStatusType
Vp792WriteProfileTable(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    int idx,
    VpProfilePtrType pProfile);

#ifdef VP792_INCLUDE_TESTLINE_CODE
/* VP-API Line Tests Structs */

EXTERN VpStatusType
Vp792TestLine(
    VpLineCtxType *pLineCtx,
    VpTestIdType test,
    const void *pArgs,
    uint16 handle);

#endif

#endif /* VP792_API_INT_H */
