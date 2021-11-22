/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2015 Arm Limited (or its affiliates). All 
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        18. December 2015
 * $Revision:    V1.0
 *
 * Project:      CAN Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __CAN_LPC17XX_H
#define __CAN_LPC17XX_H

#include <string.h>

#include "Driver_CAN.h"

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
  #include "PIN_LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
  #include "PIN_LPC177x_8x.h"
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"

#ifndef RTE_CAN_CAN1
#define RTE_CAN_CAN1                    (0U)
#endif
#ifndef RTE_CAN_CAN2
#define RTE_CAN_CAN2                    (0U)
#endif

#if   ((RTE_CAN_CAN1 == 0U) && (RTE_CAN_CAN2 == 0U))
#error "No CAN is enabled in the RTE_Device.h!"
#endif

#if    (RTE_CAN_CAN2 == 1U)
#define CAN_CTRL_NUM                    (2U)
#else
#define CAN_CTRL_NUM                    (1U)
#endif

// CAN Acceptance Filter Registers bit definitions
#define CANAF_AFMR_AccOff               (1U            )
#define CANAF_AFMR_AccBP                (1U      <<  1U)
#define CANAF_AFMR_eFCANf               (1U      <<  2U)

#define CANAF_SFF_sa_SFF_sa_Pos         (            2U)
#define CANAF_SFF_sa_SFF_sa_Msk         (0x1FFU  <<  CANAF_SFF_sa_SFF_sa_Pos)
#define CANAF_SFF_sa_SFF_sa(x)          (((x)    <<  CANAF_SFF_sa_SFF_sa_Pos) & CANAF_SFF_sa_SFF_sa_Msk)

#define CANAF_SFF_GRP_sa_SFF_GRP_sa_Pos (            2U)
#define CANAF_SFF_GRP_sa_SFF_GRP_sa_Msk (0x3FFU  <<  CANAF_SFF_GRP_sa_SFF_GRP_sa_Pos)
#define CANAF_SFF_GRP_sa_SFF_GRP_sa(x)  (((x)    <<  CANAF_SFF_GRP_sa_SFF_GRP_sa_Pos) & CANAF_SFF_GRP_sa_SFF_GRP_sa_Msk)

#define CANAF_EFF_sa_EFF_sa_Pos         (            2U)
#define CANAF_EFF_sa_EFF_sa_Msk         (0x1FFU  <<  CANAF_EFF_sa_EFF_sa_Pos)
#define CANAF_EFF_sa_EFF_sa(x)          (((x)    <<  CANAF_EFF_sa_EFF_sa_Pos) & CANAF_EFF_sa_EFF_sa_Msk)

#define CANAF_EFF_GRP_sa_Eff_GRP_sa_Pos (            2U)
#define CANAF_EFF_GRP_sa_Eff_GRP_sa_Msk (0x3FFU  <<  CANAF_EFF_GRP_sa_Eff_GRP_sa_Pos)
#define CANAF_EFF_GRP_sa_Eff_GRP_sa(x)  (((x)    <<  CANAF_EFF_GRP_sa_Eff_GRP_sa_Pos) & CANAF_EFF_GRP_sa_Eff_GRP_sa_Msk)

#define CANAF_ENDofTable_ENDofTable_Pos (            2U)
#define CANAF_ENDofTable_ENDofTable_Msk (0x3FFU  <<  CANAF_ENDofTable_ENDofTable_Pos)
#define CANAF_ENDofTable_ENDofTable(x)  (((x)    <<  CANAF_ENDofTable_ENDofTable_Pos) & CANAF_ENDofTable_ENDofTable_Msk)

#define CANAF_LUTerrAd_LUTerrAd_Pos     (            2U)
#define CANAF_LUTerrAd_LUTerrAd_Msk     (0x1FFU  <<  CANAF_LUTerrAd_LUTerrAd_Pos)
#define CANAF_LUTerrAd_LUTerrAd(x)      (((x)    <<  CANAF_LUTerrAd_LUTerrAd_Pos) & CANAF_LUTerrAd_LUTerrAd_Msk)

#define CANAF_LUTerr_LUTerr             (1U            )

#define CANAF_FCANIE_FCANIE             (1U            )

#define CANAF_FCANIC0_IntPnd(x)         (1U      <<  (x)     )

#define CANAF_FCANIC1_IntPnd(x)         (1U      << ((x)-32U))

// CAN Central Registers bit definitions
#define CANCR_CANTxSR_TS1               (1U            )
#define CANCR_CANTxSR_TS2               (1U      <<  1U)
#define CANCR_CANTxSR_TBS1              (1U      <<  8U)
#define CANCR_CANTxSR_TBS2              (1U      <<  9U)
#define CANCR_CANTxSR_TCS1              (1U      << 16U)
#define CANCR_CANTxSR_TCS2              (1U      << 17U)

#define CANCR_CANRxSR_RS1               (1U            )
#define CANCR_CANRxSR_RS2               (1U      <<  1U)
#define CANCR_CANRxSR_RB1               (1U      <<  8U)
#define CANCR_CANRxSR_RB2               (1U      <<  9U)
#define CANCR_CANRxSR_DOS1              (1U      << 16U)
#define CANCR_CANRxSR_DOS2              (1U      << 17U)

#define CANCR_CANMSR_E1                 (1U            )
#define CANCR_CANMSR_E2                 (1U      <<  1U)
#define CANCR_CANMSR_BS1                (1U      <<  8U)
#define CANCR_CANMSR_BS2                (1U      <<  9U)

// CAN Registers bit definitions
#define CAN_MOD_RM                      (1U            )
#define CAN_MOD_LOM                     (1U      <<  1U)
#define CAN_MOD_STM                     (1U      <<  2U)
#define CAN_MOD_TPM                     (1U      <<  3U)
#define CAN_MOD_SM                      (1U      <<  4U)
#define CAN_MOD_RPM                     (1U      <<  5U)
#define CAN_MOD_TM                      (1U      <<  7U)

#define CAN_CMR_TR                      (1U            )
#define CAN_CMR_AT                      (1U      <<  1U)
#define CAN_CMR_RRB                     (1U      <<  2U)
#define CAN_CMR_CDO                     (1U      <<  3U)
#define CAN_CMR_SRR                     (1U      <<  4U)
#define CAN_CMR_STB1                    (1U      <<  5U)
#define CAN_CMR_STB2                    (1U      <<  6U)
#define CAN_CMR_STB3                    (1U      <<  7U)

#define CAN_GSR_RBS                     (1U            )
#define CAN_GSR_DOS                     (1U      <<  1U)
#define CAN_GSR_TBS                     (1U      <<  2U)
#define CAN_GSR_TCS                     (1U      <<  3U)
#define CAN_GSR_RS                      (1U      <<  4U)
#define CAN_GSR_TS                      (1U      <<  5U)
#define CAN_GSR_ES                      (1U      <<  6U)
#define CAN_GSR_BS                      (1U      <<  7U)
#define CAN_GSR_RXERR_Pos               (           16U)
#define CAN_GSR_RXERR_Msk               (0xFFU   << CAN_GSR_RXERR_Pos)
#define CAN_GSR_TXERR_Pos               (           24U)
#define CAN_GSR_TXERR_Msk               (0xFFU   << CAN_GSR_TXERR_Pos)

#define CAN_ICR_RI                      (1U            )
#define CAN_ICR_TI1                     (1U      <<  1U)
#define CAN_ICR_EI                      (1U      <<  2U)
#define CAN_ICR_DOI                     (1U      <<  3U)
#define CAN_ICR_WUI                     (1U      <<  4U)
#define CAN_ICR_EPI                     (1U      <<  5U)
#define CAN_ICR_ALI                     (1U      <<  6U)
#define CAN_ICR_BEI                     (1U      <<  7U)
#define CAN_ICR_IDI                     (1U      <<  8U)
#define CAN_ICR_TI2                     (1U      <<  9U)
#define CAN_ICR_TI3                     (1U      << 10U)
#define CAN_ICR_ERRBIT_Pos              (           16U)
#define CAN_ICR_ERRBIT_Msk              (0x1FU   << CAN_ICR_ERRBIT_Pos)
#define CAN_ICR_ERRDIR                  (1U      << 21U)
#define CAN_ICR_ERRC1_0_Pos             (           22U)
#define CAN_ICR_ERRC1_0_Msk             (3U      << CAN_ICR_ERRC1_0_Pos)
#define CAN_ICR_ALCBIT_Pos              (           24U)
#define CAN_ICR_ALCBIT_Msk              (0xFFU   << CAN_ICR_ALCBIT_Pos)

#define CAN_IER_RIE                     (1U            )
#define CAN_IER_TIE1                    (1U      <<  1U)
#define CAN_IER_EIE                     (1U      <<  2U)
#define CAN_IER_DOIE                    (1U      <<  3U)
#define CAN_IER_WUIE                    (1U      <<  4U)
#define CAN_IER_EPIE                    (1U      <<  5U)
#define CAN_IER_ALIE                    (1U      <<  6U)
#define CAN_IER_BEIE                    (1U      <<  7U)
#define CAN_IER_IDIE                    (1U      <<  8U)
#define CAN_IER_TIE2                    (1U      <<  9U)
#define CAN_IER_TIE3                    (1U      << 10U)

#define CAN_BTR_BRP_Pos                 (            0U)
#define CAN_BTR_BRP_Msk                 (0x3FFU        )
#define CAN_BTR_BRP(x)                  (0x3FFU &   (x))
#define CAN_BTR_SJW                     (3U      << 14U)
#define CAN_BTR_TSEG1_Pos               (           16U)
#define CAN_BTR_TSEG1_Msk               (0x0FU   << CAN_BTR_TSEG1_Pos)
#define CAN_BTR_TSEG1(x)                (((x)    << CAN_BTR_TSEG1_Pos) & CAN_BTR_TSEG1_Msk)
#define CAN_BTR_TSEG2_Pos               (           20U)
#define CAN_BTR_TSEG2_Msk               (7U      << CAN_BTR_TSEG2_Pos)
#define CAN_BTR_TSEG2(x)                (((x)    << CAN_BTR_TSEG2_Pos) & CAN_BTR_TSEG2_Msk)
#define CAN_BTR_SAM                     (1U      << 23U)

#define CAN_EWL_EWL_Pos                 (            0U)
#define CAN_EWL_EWL_Msk                 (0xFFU         )

#define CAN_SR_RBS                      (1U            )
#define CAN_SR_DOS                      (1U      <<  1U)
#define CAN_SR_TBS1                     (1U      <<  2U)
#define CAN_SR_TCS1                     (1U      <<  3U)
#define CAN_SR_RS                       (1U      <<  4U)
#define CAN_SR_TS1                      (1U      <<  5U)
#define CAN_SR_ES                       (1U      <<  6U)
#define CAN_SR_BS                       (1U      <<  7U)
#define CAN_SR_RBS2                     (1U      <<  8U)
#define CAN_SR_DOS2                     (1U      <<  9U)
#define CAN_SR_TBS2                     (1U      << 10U)
#define CAN_SR_TCS2                     (1U      << 11U)
#define CAN_SR_RS2                      (1U      << 12U)
#define CAN_SR_TS2                      (1U      << 13U)
#define CAN_SR_ES2                      (1U      << 14U)
#define CAN_SR_BS2                      (1U      << 15U)
#define CAN_SR_RBS3                     (1U      << 16U)
#define CAN_SR_DOS3                     (1U      << 17U)
#define CAN_SR_TBS3                     (1U      << 18U)
#define CAN_SR_TCS3                     (1U      << 19U)
#define CAN_SR_RS3                      (1U      << 20U)
#define CAN_SR_TS3                      (1U      << 21U)
#define CAN_SR_ES3                      (1U      << 22U)
#define CAN_SR_BS3                      (1U      << 23U)

#define CAN_RFS_ID_INDEX_Pos            (            0U)
#define CAN_RFS_ID_INDEX_Msk            (0x3FFU        )
#define CAN_RFS_BP                      (1U      <<  1U)
#define CAN_RFS_DLC_Pos                 (           16U)
#define CAN_RFS_DLC_Msk                 (0xFU    << CAN_RFS_DLC_Pos)
#define CAN_RFS_RTR_Pos                 (           30U)
#define CAN_RFS_RTR                     (1U      << 30U)
#define CAN_RFS_FF                      (1U      << 31U)

#define CAN_RID_ID_A                    (0x7FFU        )
#define CAN_RID_ID_B                    (0x1FFFFFFFU   )

#define CAN_RDA_DATA0_Pos               (            0U)
#define CAN_RDA_DATA0_Msk               (0xFFU         )
#define CAN_RDA_DATA0(x)                (((x)    << CAN_RDA_DATA0_Pos) & CAN_RDA_DATA0_Msk)
#define CAN_RDA_DATA1_Pos               (            8U)
#define CAN_RDA_DATA1_Msk               (0xFFU   << CAN_RDA_DATA1_Pos)
#define CAN_RDA_DATA1(x)                (((x)    << CAN_RDA_DATA1_Pos) & CAN_RDA_DATA1_Msk)
#define CAN_RDA_DATA2_Pos               (           16U)
#define CAN_RDA_DATA2_Msk               (0xFFU   << CAN_RDA_DATA2_Pos)
#define CAN_RDA_DATA2(x)                (((x)    << CAN_RDA_DATA2_Pos) & CAN_RDA_DATA2_Msk)
#define CAN_RDA_DATA3_Pos               (           24U)
#define CAN_RDA_DATA3_Msk               (0xFFU   << CAN_RDA_DATA3_Pos)
#define CAN_RDA_DATA3(x)                (((x)    << CAN_RDA_DATA3_Pos) & CAN_RDA_DATA3_Msk)

#define CAN_RDA_DATA5_Pos               (            0U)
#define CAN_RDA_DATA5_Msk               (0xFFU         )
#define CAN_RDA_DATA5(x)                (((x)    << CAN_RDA_DATA5_Pos) & CAN_RDA_DATA5_Msk)
#define CAN_RDA_DATA6_Pos               (            8U)
#define CAN_RDA_DATA6_Msk               (0xFFU   << CAN_RDA_DATA6_Pos)
#define CAN_RDA_DATA6(x)                (((x)    << CAN_RDA_DATA6_Pos) & CAN_RDA_DATA6_Msk)
#define CAN_RDA_DATA7_Pos               (           16U)
#define CAN_RDA_DATA7_Msk               (0xFFU   << CAN_RDA_DATA7_Pos)
#define CAN_RDA_DATA7(x)                (((x)    << CAN_RDA_DATA7_Pos) & CAN_RDA_DATA7_Msk)
#define CAN_RDA_DATA8_Pos               (           24U)
#define CAN_RDA_DATA8_Msk               (0xFFU   << CAN_RDA_DATA8_Pos)
#define CAN_RDA_DATA8(x)                (((x)    << CAN_RDA_DATA8_Pos) & CAN_RDA_DATA8_Msk)

#define CAN_TFI1_PRIO                   (1U            )
#define CAN_TFI1_DLC_Pos                (           16U)
#define CAN_TFI1_DLC_Msk                (0xFU    << CAN_TFI1_DLC_Pos)
#define CAN_TFI1_DLC(x)                 (((x)    << CAN_TFI1_DLC_Pos) & CAN_TFI1_DLC_Msk)
#define CAN_TFI1_RTR                    (1U      << 30U)
#define CAN_TFI1_FF                     (1U      << 31U)

#define CAN_TID1_ID_A                   (0x7FFU        )
#define CAN_TID1_ID_B                   (0x1FFFFFFFU   )

#define CAN_TDA1_DATA0_Pos              (            0U)
#define CAN_TDA1_DATA0_Msk              (0xFFU         )
#define CAN_TDA1_DATA0(x)               (((x)    << CAN_TDA1_DATA0_Pos) & CAN_TDA1_DATA0_Msk)
#define CAN_TDA1_DATA1_Pos              (            8U)
#define CAN_TDA1_DATA1_Msk              (0xFFU   << CAN_TDA1_DATA1_Pos)
#define CAN_TDA1_DATA1(x)               (((x)    << CAN_TDA1_DATA1_Pos) & CAN_TDA1_DATA1_Msk)
#define CAN_TDA1_DATA2_Pos              (           16U)
#define CAN_TDA1_DATA2_Msk              (0xFFU   << CAN_TDA1_DATA2_Pos)
#define CAN_TDA1_DATA2(x)               (((x)    << CAN_TDA1_DATA2_Pos) & CAN_TDA1_DATA2_Msk)
#define CAN_TDA1_DATA3_Pos              (           24U)
#define CAN_TDA1_DATA3_Msk              (0xFFU   << CAN_TDA1_DATA3_Pos)
#define CAN_TDA1_DATA3(x)               (((x)    << CAN_TDA1_DATA3_Pos) & CAN_TDA1_DATA3_Msk)

#define CAN_TDB1_DATA5_Pos              (            0U)
#define CAN_TDB1_DATA5_Msk              (0xFFU         )
#define CAN_TDB1_DATA5(x)               (((x)    << CAN_TDB1_DATA5_Pos) & CAN_TDB1_DATA5_Msk)
#define CAN_TDB1_DATA6_Pos              (            8U)
#define CAN_TDB1_DATA6_Msk              (0xFFU   << CAN_TDB1_DATA6_Pos)
#define CAN_TDB1_DATA6(x)               (((x)    << CAN_TDB1_DATA6_Pos) & CAN_TDB1_DATA6_Msk)
#define CAN_TDB1_DATA7_Pos              (           16U)
#define CAN_TDB1_DATA7_Msk              (0xFFU   << CAN_TDB1_DATA7_Pos)
#define CAN_TDB1_DATA7(x)               (((x)    << CAN_TDB1_DATA7_Pos) & CAN_TDB1_DATA7_Msk)
#define CAN_TDB1_DATA8_Pos              (           24U)
#define CAN_TDB1_DATA8_Msk              (0xFFU   << CAN_TDB1_DATA8_Pos)
#define CAN_TDB1_DATA8(x)               (((x)    << CAN_TDB1_DATA8_Pos) & CAN_TDB1_DATA8_Msk)

#define CAN_TFI2_PRIO                   (1U            )
#define CAN_TFI2_DLC_Pos                (           16U)
#define CAN_TFI2_DLC_Msk                (0xFU    << CAN_TFI2_DLC_Pos)
#define CAN_TFI2_DLC(x)                 (((x)    << CAN_TFI2_DLC_Pos) & CAN_TFI2_DLC_Msk)
#define CAN_TFI2_RTR                    (1U      << 30U)
#define CAN_TFI2_FF                     (1U      << 31U)

#define CAN_TID2_ID_A                   (0x7FFU        )
#define CAN_TID2_ID_B                   (0x1FFFFFFFU   )

#define CAN_TDA2_DATA0_Pos              (            0U)
#define CAN_TDA2_DATA0_Msk              (0xFFU         )
#define CAN_TDA2_DATA0(x)               (((x)    << CAN_TDA2_DATA0_Pos) & CAN_TDA2_DATA0_Msk)
#define CAN_TDA2_DATA1_Pos              (            8U)
#define CAN_TDA2_DATA1_Msk              (0xFFU   << CAN_TDA2_DATA1_Pos)
#define CAN_TDA2_DATA1(x)               (((x)    << CAN_TDA2_DATA1_Pos) & CAN_TDA2_DATA1_Msk)
#define CAN_TDA2_DATA2_Pos              (           16U)
#define CAN_TDA2_DATA2_Msk              (0xFFU   << CAN_TDA2_DATA2_Pos)
#define CAN_TDA2_DATA2(x)               (((x)    << CAN_TDA2_DATA2_Pos) & CAN_TDA2_DATA2_Msk)
#define CAN_TDA2_DATA3_Pos              (           24U)
#define CAN_TDA2_DATA3_Msk              (0xFFU   << CAN_TDA2_DATA3_Pos)
#define CAN_TDA2_DATA3(x)               (((x)    << CAN_TDA2_DATA3_Pos) & CAN_TDA2_DATA3_Msk)

#define CAN_TDB2_DATA5_Pos              (            0U)
#define CAN_TDB2_DATA5_Msk              (0xFFU         )
#define CAN_TDB2_DATA5(x)               (((x)    << CAN_TDB2_DATA5_Pos) & CAN_TDB2_DATA5_Msk)
#define CAN_TDB2_DATA6_Pos              (            8U)
#define CAN_TDB2_DATA6_Msk              (0xFFU   << CAN_TDB2_DATA6_Pos)
#define CAN_TDB2_DATA6(x)               (((x)    << CAN_TDB2_DATA6_Pos) & CAN_TDB2_DATA6_Msk)
#define CAN_TDB2_DATA7_Pos              (           16U)
#define CAN_TDB2_DATA7_Msk              (0xFFU   << CAN_TDB2_DATA7_Pos)
#define CAN_TDB2_DATA7(x)               (((x)    << CAN_TDB2_DATA7_Pos) & CAN_TDB2_DATA7_Msk)
#define CAN_TDB2_DATA8_Pos              (           24U)
#define CAN_TDB2_DATA8_Msk              (0xFFU   << CAN_TDB2_DATA8_Pos)
#define CAN_TDB2_DATA8(x)               (((x)    << CAN_TDB2_DATA8_Pos) & CAN_TDB2_DATA8_Msk)

#define CAN_TFI3_PRIO                   (1U            )
#define CAN_TFI3_DLC_Pos                (           16U)
#define CAN_TFI3_DLC_Msk                (0xFU    << CAN_TFI3_DLC_Pos)
#define CAN_TFI3_DLC(x)                 (((x)    << CAN_TFI3_DLC_Pos) & CAN_TFI3_DLC_Msk)
#define CAN_TFI3_RTR                    (1U      << 30U)
#define CAN_TFI3_FF                     (1U      << 31U)

#define CAN_TID3_ID_A                   (0x7FFU        )
#define CAN_TID3_ID_B                   (0x1FFFFFFFU   )

#define CAN_TDA3_DATA0_Pos              (            0U)
#define CAN_TDA3_DATA0_Msk              (0xFFU         )
#define CAN_TDA3_DATA0(x)               (((x)    << CAN_TDA3_DATA0_Pos) & CAN_TDA3_DATA0_Msk)
#define CAN_TDA3_DATA1_Pos              (            8U)
#define CAN_TDA3_DATA1_Msk              (0xFFU   << CAN_TDA3_DATA1_Pos)
#define CAN_TDA3_DATA1(x)               (((x)    << CAN_TDA3_DATA1_Pos) & CAN_TDA3_DATA1_Msk)
#define CAN_TDA3_DATA2_Pos              (           16U)
#define CAN_TDA3_DATA2_Msk              (0xFFU   << CAN_TDA3_DATA2_Pos)
#define CAN_TDA3_DATA2(x)               (((x)    << CAN_TDA3_DATA2_Pos) & CAN_TDA3_DATA2_Msk)
#define CAN_TDA3_DATA3_Pos              (           24U)
#define CAN_TDA3_DATA3_Msk              (0xFFU   << CAN_TDA3_DATA3_Pos)
#define CAN_TDA3_DATA3(x)               (((x)    << CAN_TDA3_DATA3_Pos) & CAN_TDA3_DATA3_Msk)

#define CAN_TDB3_DATA5_Pos              (            0U)
#define CAN_TDB3_DATA5_Msk              (0xFFU         )
#define CAN_TDB3_DATA5(x)               (((x)    << CAN_TDB3_DATA5_Pos) & CAN_TDB3_DATA5_Msk)
#define CAN_TDB3_DATA6_Pos              (            8U)
#define CAN_TDB3_DATA6_Msk              (0xFFU   << CAN_TDB3_DATA6_Pos)
#define CAN_TDB3_DATA6(x)               (((x)    << CAN_TDB3_DATA6_Pos) & CAN_TDB3_DATA6_Msk)
#define CAN_TDB3_DATA7_Pos              (           16U)
#define CAN_TDB3_DATA7_Msk              (0xFFU   << CAN_TDB3_DATA7_Pos)
#define CAN_TDB3_DATA7(x)               (((x)    << CAN_TDB3_DATA7_Pos) & CAN_TDB3_DATA7_Msk)
#define CAN_TDB3_DATA8_Pos              (           24U)
#define CAN_TDB3_DATA8_Msk              (0xFFU   << CAN_TDB3_DATA8_Pos)
#define CAN_TDB3_DATA8(x)               (((x)    << CAN_TDB3_DATA8_Pos) & CAN_TDB3_DATA8_Msk)

#endif // __CAN_LPC17XX_H
