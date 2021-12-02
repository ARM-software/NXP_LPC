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
 * $Date:        4. January 2016
 * $Revision:    V1.0
 *
 * Project:      USB Device Driver definitions for NXP LPC40xx
 * -------------------------------------------------------------------------- */

#ifndef __USBD_LPC40XX_H
#define __USBD_LPC40XX_H

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

#endif /* __USBD_LPC40XX_H */
