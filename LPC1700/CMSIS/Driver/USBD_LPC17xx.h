/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2016 Arm Limited (or its affiliates). All 
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
 * $Date:        12. January 2016
 * $Revision:    V2.2.1
 *
 * Project:      USB Device Driver definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __USBD_LPC17XX_H
#define __USBD_LPC17XX_H

#include <stdint.h>

#include "RTE_Device.h"
#include "RTE_Components.h"

/* Device Interrupt Bit Definitions */
#define USBD_FRAME_INT          (1U     )
#define USBD_EP_FAST_INT        (1U << 1)
#define USBD_EP_SLOW_INT        (1U << 2)
#define USBD_DEV_STAT_INT       (1U << 3)
#define USBD_CCEMTY_INT         (1U << 4)
#define USBD_CDFULL_INT         (1U << 5)
#define USBD_RxENDPKT_INT       (1U << 6)
#define USBD_TxENDPKT_INT       (1U << 7)
#define USBD_EP_RLZED_INT       (1U << 8)
#define USBD_ERR_INT            (1U << 9)

/* Rx & Tx Packet Length Definitions */
#define USBD_PKT_LNGTH_MASK     (0x03FFU)
#define USBD_PKT_DV             (1U << 10)
#define USBD_PKT_RDY            (1U << 11)

/* USB Control Definitions */
#define USBD_CTRL_RD_EN         (1U     )
#define USBD_CTRL_WR_EN         (1U << 1)

/* Command Codes */
#define USBD_CMD_SET_ADDR       (0x00D00500UL)
#define USBD_CMD_CFG_DEV        (0x00D80500UL)
#define USBD_CMD_SET_MODE       (0x00F30500UL)
#define USBD_CMD_RD_FRAME       (0x00F50500UL)
#define USBD_DAT_RD_FRAME       (0x00F50200UL)
#define USBD_CMD_RD_TEST        (0x00FD0500UL)
#define USBD_DAT_RD_TEST        (0x00FD0200UL)
#define USBD_CMD_SET_DEV_STAT   (0x00FE0500UL)
#define USBD_CMD_GET_DEV_STAT   (0x00FE0500UL)
#define USBD_DAT_GET_DEV_STAT   (0x00FE0200UL)
#define USBD_CMD_GET_ERR_CODE   (0x00FF0500UL)
#define USBD_DAT_GET_ERR_CODE   (0x00FF0200UL)
#define USBD_CMD_RD_ERR_STAT    (0x00FB0500UL)
#define USBD_DAT_RD_ERR_STAT    (0x00FB0200UL)
#define USBD_DAT_WR_BYTE(x)     (0x00000100UL | ((x) << 16))
#define USBD_CMD_SEL_EP(x)      (0x00000500UL | ((x) << 16))
#define USBD_DAT_SEL_EP(x)      (0x00000200UL | ((x) << 16))
#define USBD_CMD_SEL_EP_CLRI(x) (0x00400500UL | ((x) << 16))
#define USBD_DAT_SEL_EP_CLRI(x) (0x00400200UL | ((x) << 16))
#define USBD_CMD_SET_EP_STAT(x) (0x00400500UL | ((x) << 16))
#define USBD_CMD_CLR_BUF        (0x00F20500UL)
#define USBD_DAT_CLR_BUF        (0x00F20200UL)
#define USBD_CMD_VALID_BUF      (0x00FA0500UL)

/* Device Address Register Definitions */
#define USBD_DEV_ADDR_MASK      (0x7FU)
#define USBD_DEV_EN             (0x80U)

/* Device Configure Register Definitions */
#define USBD_CONF_DVICE         (1U    )

/* Device Mode Register Definitions */
#define USBD_AP_CLK             (1U     )
#define USBD_INAK_CI            (1U << 1)
#define USBD_INAK_CO            (1U << 2)
#define USBD_INAK_II            (1U << 3)
#define USBD_INAK_IO            (1U << 4)
#define USBD_INAK_BI            (1U << 5)
#define USBD_INAK_BO            (1U << 6)

/* Device Status Register Definitions */
#define USBD_DEV_CON            (1U     )
#define USBD_DEV_CON_CH         (1U << 1)
#define USBD_DEV_SUS            (1U << 2)
#define USBD_DEV_SUS_CH         (1U << 3)
#define USBD_DEV_RST            (1U << 4)

/* Error Code Register Definitions */
#define USBD_ERR_EC_MASK        (0x0FU)
#define USBD_ERR_EA             (0x10U)

/* Error Status Register Definitions */
#define USBD_ERR_PID            (1U     )
#define USBD_ERR_UEPKT          (1U << 1)
#define USBD_ERR_DCRC           (1U << 2)
#define USBD_ERR_TIMOUT         (1U << 3)
#define USBD_ERR_EOP            (1U << 4)
#define USBD_ERR_B_OVRN         (1U << 5)
#define USBD_ERR_BTSTF          (1U << 6)
#define USBD_ERR_TGL            (1U << 7)

/* Endpoint Select Register Definitions */
#define USBD_EP_SEL_F           (1U     )
#define USBD_EP_SEL_ST          (1U << 1)
#define USBD_EP_SEL_STP         (1U << 2)
#define USBD_EP_SEL_PO          (1U << 3)
#define USBD_EP_SEL_EPN         (1U << 4)
#define USBD_EP_SEL_B_1_FULL    (1U << 5)
#define USBD_EP_SEL_B_2_FULL    (1U << 6)

/* Endpoint Status Register Definitions */
#define USBD_EP_STAT_ST         (1U     )
#define USBD_EP_STAT_DA         (1U << 1)
#define USBD_EP_STAT_RF_MO      (1U << 2)
#define USBD_EP_STAT_CND_ST     (1U << 3)

/* Clear Buffer Register Definitions */
#define USBD_CLR_BUF_PO         (1U     )

/* DMA Interrupt Bit Definitions */
#define USBD_EOT_INT            (1U     )
#define USBD_NDD_REQ_INT        (1U << 1)
#define USBD_SYS_ERR_INT        (1U << 2)

#if defined (LPC175x_6x)

typedef struct
{
  __I  uint32_t Revision;               // USB Host Registers
  __IO uint32_t Control;
  __IO uint32_t CommandStatus;
  __IO uint32_t InterruptStatus;
  __IO uint32_t InterruptEnable;
  __IO uint32_t InterruptDisable;
  __IO uint32_t HCCA;
  __I  uint32_t PeriodCurrentED;
  __IO uint32_t ControlHeadED;
  __IO uint32_t ControlCurrentED;
  __IO uint32_t BulkHeadED;
  __IO uint32_t BulkCurrentED;
  __I  uint32_t DoneHead;
  __IO uint32_t FmInterval;
  __I  uint32_t FmRemaining;
  __I  uint32_t FmNumber;
  __IO uint32_t PeriodicStart;
  __IO uint32_t LSTreshold;
  __IO uint32_t RhDescriptorA;
  __IO uint32_t RhDescriptorB;
  __IO uint32_t RhStatus;
  __IO uint32_t RhPortStatus1;
  __IO uint32_t RhPortStatus2;
       uint32_t RESERVED0[40];
  __I  uint32_t Module_ID;

  __I  uint32_t IntSt;                  // USB On-The-Go Registers
  __IO uint32_t IntEn;
  __O  uint32_t IntSet;
  __O  uint32_t IntClr;
  __IO uint32_t StCtrl;
  __IO uint32_t Tmr;
       uint32_t RESERVED1[58];

  __I  uint32_t DevIntSt;               // USB Device Interrupt Registers
  __IO uint32_t DevIntEn;
  __O  uint32_t DevIntClr;
  __O  uint32_t DevIntSet;

  __O  uint32_t CmdCode;                // USB Device SIE Command Registers
  __I  uint32_t CmdData;

  __I  uint32_t RxData;                 // USB Device Transfer Registers
  __O  uint32_t TxData;
  __I  uint32_t RxPLen;
  __O  uint32_t TxPLen;
  __IO uint32_t Ctrl;
  __O  uint32_t DevIntPri;

  __I  uint32_t EpIntSt;                // USB Device Endpoint Interrupt Registers
  __IO uint32_t EpIntEn;
  __O  uint32_t EpIntClr;
  __O  uint32_t EpIntSet;
  __O  uint32_t EpIntPri;

  __IO uint32_t ReEp;                   // USB Device Endpoint Realization Registers
  __O  uint32_t EpInd;
  __IO uint32_t MaxPSize;

  __I  uint32_t DMARSt;                 // USB Device DMA Registers
  __O  uint32_t DMARClr;
  __O  uint32_t DMARSet;
       uint32_t RESERVED2[9];
  __IO uint32_t UDCAH;
  __I  uint32_t EpDMASt;
  __O  uint32_t EpDMAEn;
  __O  uint32_t EpDMADis;
  __I  uint32_t DMAIntSt;
  __IO uint32_t DMAIntEn;
       uint32_t RESERVED3[2];
  __I  uint32_t EoTIntSt;
  __O  uint32_t EoTIntClr;
  __O  uint32_t EoTIntSet;
  __I  uint32_t NDDRIntSt;
  __O  uint32_t NDDRIntClr;
  __O  uint32_t NDDRIntSet;
  __I  uint32_t SysErrIntSt;
  __O  uint32_t SysErrIntClr;
  __O  uint32_t SysErrIntSet;
       uint32_t RESERVED4[15];

  __IO uint32_t I2C_RX;                 // USB OTG I2C RX/TX Register

  __IO uint32_t I2C_STS;
  __IO uint32_t I2C_CTL;
  __IO uint32_t I2C_CLKHI;
  __O  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  __IO uint32_t USBClkCtrl;             // OTG/USB Clock Control Register
  __I  uint32_t USBClkSt;
} _LPC_USB_TypeDef;

#define I2C_TX                  I2C_RX
#define OTGClkCtrl              USBClkCtrl
#define OTGClkSt                USBClkSt

#ifdef  LPC_USB
#undef  LPC_USB
#endif

#define LPC_USB               ((_LPC_USB_TypeDef       *) LPC_USB_BASE      )

#endif

#endif /* __USBD_LPC17XX_H */
