/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All 
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
 * $Date:        10. Januar 2020
 * $Revision:    V1.4
 *
 * Project:      MCI Driver Definitions for NXP LPC177x/8x
 * -------------------------------------------------------------------------- */

#ifndef __MCI_LPC177X8X_H
#define __MCI_LPC177X8X_H

#include <string.h>

#include "Driver_MCI.h"

#include "LPC177x_8x.h"
#include "PIN_LPC177x_8x.h"
#include "GPIO_LPC17xx.h"
#include "GPDMA_LPC17xx.h"

#include "RTE_Device.h"

#if (defined(RTE_Drivers_MCI0) && (RTE_SDIO == 0))
  #error "SDIO not configured in RTE_Device.h!"
#endif


#if (RTE_SD_CD_PIN_EN == 0)
#define MCI_CD_PIN            0U
#else
#define MCI_CD_PIN            1U
#endif

#if (RTE_SD_WP_PIN_EN == 0)
#define MCI_WP_PIN            0U
#else
#define MCI_WP_PIN            1U
#endif

#if (RTE_SD_PWR_PIN_EN == 0)
#define MCI_PWR_PIN           0U
#else
#define MCI_PWR_PIN           1U
#endif

#if (RTE_SD_BUS_WIDTH_4 == 0)
#define MCI_BUS_WIDTH_4       0U
#else
#define MCI_BUS_WIDTH_4       1U
#endif

/* SCS register bit definitions */
#define SC_SCS_MCIPWRAL                   ((uint8_t)0x08)

/* PWR register bit definitions */
#define MCI_PWR_CTRL                      ((uint8_t)0x03)
#define MCI_PWR_CTRL_0                    ((uint8_t)0x01)
#define MCI_PWR_CTRL_1                    ((uint8_t)0x02)
#define MCI_PWR_OPENDRAIN                 ((uint8_t)0x04)
#define MCI_PWR_ROD                       ((uint8_t)0x08)

/* CLOCK register bit definitions */
#define MCI_CLOCK_CLKDIV                  ((uint16_t)0x00FF)
#define MCI_CLOCK_ENABLE                  ((uint16_t)0x0100)
#define MCI_CLOCK_PWRSAVE                 ((uint16_t)0x0200)
#define MCI_CLOCK_BYPASS                  ((uint16_t)0x0400)
#define MCI_CLOCK_WIDEBUS                 ((uint16_t)0x0800)

/* COMMAND register bit definitions */
#define MCI_COMMAND_CMDINDEX              ((uint16_t)0x003F)
#define MCI_COMMAND_RESPONSE              ((uint16_t)0x0040)
#define MCI_COMMAND_LONGRSP               ((uint16_t)0x0080)
#define MCI_COMMAND_INTERRUPT             ((uint16_t)0x0100)
#define MCI_COMMAND_PENDING               ((uint16_t)0x0200)
#define MCI_COMMAND_ENABLE                ((uint16_t)0x0400)

/* DATACTRL register bit definitions */
#define MCI_DATACTRL_ENABLE               ((uint16_t)0x0001)
#define MCI_DATACTRL_DIRECTION            ((uint16_t)0x0002)
#define MCI_DATACTRL_MODE                 ((uint16_t)0x0004)
#define MCI_DATACTRL_DMAENABLE            ((uint16_t)0x0008)
#define MCI_DATACTRL_BLOCKSIZE            ((uint16_t)0x00F0)

/* STATUS register bit definitions */
#define MCI_STATUS_CMDCRCFAIL             ((uint32_t)0x00000001)
#define MCI_STATUS_DATACRCFAIL            ((uint32_t)0x00000002)
#define MCI_STATUS_CMDTIMEOUT             ((uint32_t)0x00000004)
#define MCI_STATUS_DATATIMEOUT            ((uint32_t)0x00000008)
#define MCI_STATUS_TXUNDERRUN             ((uint32_t)0x00000010)
#define MCI_STATUS_RXOVERRUN              ((uint32_t)0x00000020)
#define MCI_STATUS_CMDRESPEND             ((uint32_t)0x00000040)
#define MCI_STATUS_CMDSENT                ((uint32_t)0x00000080)
#define MCI_STATUS_DATAEND                ((uint32_t)0x00000100)
#define MCI_STATUS_STARTBITERR            ((uint32_t)0x00000200)
#define MCI_STATUS_DATABLOCKEND           ((uint32_t)0x00000400)
#define MCI_STATUS_CMDACTIVE              ((uint32_t)0x00000800)
#define MCI_STATUS_TXACTIVE               ((uint32_t)0x00001000)
#define MCI_STATUS_RXACTIVE               ((uint32_t)0x00002000)
#define MCI_STATUS_TXFIFOHALFEMPTY        ((uint32_t)0x00004000)
#define MCI_STATUS_RXFIFOHSLFFULL         ((uint32_t)0x00008000)
#define MCI_STATUS_TXFIFOFULL             ((uint32_t)0x00010000)
#define MCI_STATUS_RXFIFOFULL             ((uint32_t)0x00020000)
#define MCI_STATUS_TXFIFOEMPTY            ((uint32_t)0x00040000)
#define MCI_STATUS_RXFIFOEMPTY            ((uint32_t)0x00080000)
#define MCI_STATUS_TXDATAAVLBL            ((uint32_t)0x00100000)
#define MCI_STATUS_RXDATAAVLBL            ((uint32_t)0x00200000)

/* CLEAR register bit definitions */
#define MCI_CLEAR_CMDCRCFAILCLR           ((uint32_t)0x00000001)
#define MCI_CLEAR_DATACRCFAILCLR          ((uint32_t)0x00000002)
#define MCI_CLEAR_CMDTIMEOUTCLR           ((uint32_t)0x00000004)
#define MCI_CLEAR_DATATIMEOUTCLR          ((uint32_t)0x00000008)
#define MCI_CLEAR_TXUNDERRUNCLR           ((uint32_t)0x00000010)
#define MCI_CLEAR_RXOVERRUNCLR            ((uint32_t)0x00000020)
#define MCI_CLEAR_CMDRESPENDCLR           ((uint32_t)0x00000040)
#define MCI_CLEAR_CMDSENTCLR              ((uint32_t)0x00000080)
#define MCI_CLEAR_DATAENDCLR              ((uint32_t)0x00000100)
#define MCI_CLEAR_STARTBITERRCLR          ((uint32_t)0x00000200)
#define MCI_CLEAR_DATABLOCKENDCLR         ((uint32_t)0x00000400)

/* MASK0 register bit definitions */
#define MCI_MASK0_CMDCRCFAIL              ((uint32_t)0x00000001)
#define MCI_MASK0_DATACRCFAIL             ((uint32_t)0x00000002)
#define MCI_MASK0_CMDTIMEOUT              ((uint32_t)0x00000004)
#define MCI_MASK0_DATATIMEOUT             ((uint32_t)0x00000008)
#define MCI_MASK0_TXUNDERRUN              ((uint32_t)0x00000010)
#define MCI_MASK0_RXOVERRUN               ((uint32_t)0x00000020)
#define MCI_MASK0_CMDRESPEND              ((uint32_t)0x00000040)
#define MCI_MASK0_CMDSENT                 ((uint32_t)0x00000080)
#define MCI_MASK0_DATAEND                 ((uint32_t)0x00000100)
#define MCI_MASK0_STARTBITERR             ((uint32_t)0x00000200)
#define MCI_MASK0_DATABLOCKEND            ((uint32_t)0x00000400)
#define MCI_MASK0_CMDACTIVE               ((uint32_t)0x00000800)
#define MCI_MASK0_TXACTIVE                ((uint32_t)0x00001000)
#define MCI_MASK0_RXACTIVE                ((uint32_t)0x00002000)
#define MCI_MASK0_TXFIFOHALFEMPTY         ((uint32_t)0x00004000)
#define MCI_MASK0_RXFIFOHALFFULL          ((uint32_t)0x00008000)
#define MCI_MASK0_TXFIFOFULL              ((uint32_t)0x00010000)
#define MCI_MASK0_RXFIFOFULL              ((uint32_t)0x00020000)
#define MCI_MASK0_TXFIFOEMPTY             ((uint32_t)0x00040000)
#define MCI_MASK0_RXFIFOEMPTY             ((uint32_t)0x00080000)
#define MCI_MASK0_TXDATAAVLBL             ((uint32_t)0x00100000)
#define MCI_MASK0_RXDATAAVLBL             ((uint32_t)0x00200000)

/* Interrupt clear Mask */
#define MCI_CLEAR_Msk             (MCI_CLEAR_CMDCRCFAILCLR  | \
                                   MCI_CLEAR_DATACRCFAILCLR | \
                                   MCI_CLEAR_CMDTIMEOUTCLR  | \
                                   MCI_CLEAR_DATATIMEOUTCLR | \
                                   MCI_CLEAR_TXUNDERRUNCLR  | \
                                   MCI_CLEAR_RXOVERRUNCLR   | \
                                   MCI_CLEAR_CMDRESPENDCLR  | \
                                   MCI_CLEAR_CMDSENTCLR     | \
                                   MCI_CLEAR_DATAENDCLR     | \
                                   MCI_CLEAR_STARTBITERRCLR | \
                                   MCI_CLEAR_DATABLOCKENDCLR)

/* Error interrupt mask */
#define MCI_STA_ERROR_Msk         (MCI_STATUS_CMDCRCFAIL  | \
                                   MCI_STATUS_DATACRCFAIL | \
                                   MCI_STATUS_CMDTIMEOUT  | \
                                   MCI_STATUS_DATATIMEOUT | \
                                   MCI_STATUS_STARTBITERR)

/* Transfer event mask */
#define MCI_TRANSFER_EVENT_Msk    (ARM_MCI_EVENT_TRANSFER_ERROR   | \
                                   ARM_MCI_EVENT_TRANSFER_TIMEOUT | \
                                   ARM_MCI_EVENT_TRANSFER_COMPLETE)

/* Command event mask */
#define MCI_COMMAND_EVENT_Msk     (ARM_MCI_EVENT_COMMAND_ERROR   | \
                                   ARM_MCI_EVENT_COMMAND_TIMEOUT | \
                                   ARM_MCI_EVENT_COMMAND_COMPLETE)

#define MCI_RESPONSE_EXPECTED_Msk (ARM_MCI_RESPONSE_SHORT      | \
                                   ARM_MCI_RESPONSE_SHORT_BUSY | \
                                   ARM_MCI_RESPONSE_LONG)

/* Driver flag definitions */
#define MCI_INIT      ((uint8_t)0x01)   /* MCI initialized           */
#define MCI_POWER     ((uint8_t)0x02)   /* MCI powered on            */
#define MCI_SETUP     ((uint8_t)0x04)   /* MCI configured            */
#define MCI_RESP_LONG ((uint8_t)0x08)   /* Long response expected    */
#define MCI_RESP_CRC  ((uint8_t)0x10)   /* Check response CRC error  */
#define MCI_DATA_XFER ((uint8_t)0x20)   /* Transfer data             */
#define MCI_DATA_READ ((uint8_t)0x40)   /* Read transfer             */

/* MCI Transfer Information Definition */
typedef struct _MCI_XFER {
  uint8_t *buf;                         /* Data buffer                        */
  uint32_t cnt;                         /* Data bytes to transfer             */
} MCI_XFER;

/* MCI Driver State Definition */
typedef struct _MCI_INFO {
  ARM_MCI_SignalEvent_t cb_event;       /* Driver event callback function     */
  ARM_MCI_STATUS        status;         /* Driver status                      */
  uint32_t             *response;       /* Pointer to response buffer         */
  MCI_XFER              xfer;           /* Data transfer description          */
  uint8_t volatile      flags;          /* Driver state flags                 */
  uint8_t               reserved[3];    /* Reserved                           */
  uint32_t              dctrl;          /* Data control register value        */
  uint32_t              dlen;           /* Data length register value         */
} MCI_INFO;
#endif /* __MCI_LPC177X8X_H */
