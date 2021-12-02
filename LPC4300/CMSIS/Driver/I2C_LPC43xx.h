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
 * $Date:        20. Januar 2020
 * $Revision:    V2.3
 *
 * Project:      I2C Driver Definitions for NXP LPC43xx
 * -------------------------------------------------------------------------- */

#ifndef __I2C_LPC43XX_H
#define __I2C_LPC43XX_H

#include <string.h>
#include "LPC43xx.h"

#include "Driver_I2C.h"
#include "SCU_LPC43xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

/* Clock Control Unit register */
#define CCU_CLK_CFG_RUN     (1U << 0)
#define CCU_CLK_CFG_AUTO    (1U << 1)
#define CCU_CLK_STAT_RUN    (1U << 0)

#define CLK_SRC_PLL1        0x09U           // I2C clock source

/* I2C reset value for RGU */
#define RGU_RESET_I2C0      (1U << 16)      // I2C0 reset
#define RGU_RESET_I2C1      (1U << 17)      // I2C1 reset

/* I2C Driver state flags */
#define I2C_FLAG_INIT       (1U << 0)       // Driver initialized
#define I2C_FLAG_POWER      (1U << 1)       // Driver power on
#define I2C_FLAG_SETUP      (1U << 2)       // Master configured, clock set
#define I2C_FLAG_SLAVE_RX   (1U << 3)       // Slave receive registered

/* I2C Common Control flags */
#define I2C_CON_AA          (1U << 2)       // Assert acknowledge bit
#define I2C_CON_SI          (1U << 3)       // I2C interrupt bit
#define I2C_CON_STO         (1U << 4)       // STOP bit
#define I2C_CON_STA         (1U << 5)       // START bit
#define I2C_CON_I2EN        (1U << 6)       // I2C interface enable
#define I2C_CON_FLAGS       (I2C_CON_AA | I2C_CON_SI | I2C_CON_STO | I2C_CON_STA)

/* I2C Stalled Status flags */
#define I2C_MASTER          (1U << 0)       // Master stalled
#define I2C_SLAVE_TX        (1U << 1)       // Slave stalled on transmit
#define I2C_SLAVE_RX        (1U << 2)       // Slave stalled on receive
#define I2C_SLAVE           (I2C_SLAVE_TX | I2C_SLAVE_RX)

/* I2C Status Miscellaneous states */
#define I2C_STAT_BUSERR      0x00U          // I2C Bus error

/* I2C Status Master mode */
#define I2C_STAT_MA_START    0x08U          // START transmitted
#define I2C_STAT_MA_RSTART   0x10U          // Repeated START transmitted
#define I2C_STAT_MA_SLAW_A   0x18U          // SLA+W transmitted, ACK received
#define I2C_STAT_MA_SLAW_NA  0x20U          // SLA+W transmitted, no ACK recvd
#define I2C_STAT_MA_DT_A     0x28U          // Data transmitted, ACK received
#define I2C_STAT_MA_DT_NA    0x30U          // Data transmitted, no ACK recvd
#define I2C_STAT_MA_ALOST    0x38U          // Arbitration lost SLA+W or data
#define I2C_STAT_MA_SLAR_A   0x40U          // SLA+R transmitted, ACK received
#define I2C_STAT_MA_SLAR_NA  0x48U          // SLA+R transmitted, no ACK recvd
#define I2C_STAT_MA_DR_A     0x50U          // Data received, ACK returned
#define I2C_STAT_MA_DR_NA    0x58U          // Data received, no ACK returned

/* I2C Status Slave mode */
#define I2C_STAT_SL_SLAW_A   0x60U          // SLA+W received, ACK returned
#define I2C_STAT_SL_ALOST_MW 0x68U          // Arbitration lost SLA+W in Master mode
#define I2C_STAT_SL_GCA_A    0x70U          // General address recvd, ACK returned
#define I2C_STAT_SL_ALOST_GC 0x78U          // Arbitration lost in General call
#define I2C_STAT_SL_DR_A     0x80U          // Data received, ACK returned
#define I2C_STAT_SL_DR_NA    0x88U          // Data received, no ACK returned
#define I2C_STAT_SL_DRGC_A   0x90U          // Data recvd General call, ACK returned
#define I2C_STAT_SL_DRGC_NA  0x98U          // Data recvd General call, no ACK returned
#define I2C_STAT_SL_STOP     0xA0U          // STOP received while addressed
#define I2C_STAT_SL_SLAR_A   0xA8U          // SLA+R received, ACK returned
#define I2C_STAT_SL_ALOST_MR 0xB0U          // Arbitration lost SLA+R in Master mode
#define I2C_STAT_SL_DT_A     0xB8U          // Data transmitted, ACK received
#define I2C_STAT_SL_DT_NA    0xC0U          // Data transmitted, no ACK received
#define I2C_STAT_SL_LDT_A    0xC8U          // Last data transmitted, ACK received

/* I2C Control Information */
typedef struct {
  ARM_I2C_SignalEvent_t cb_event;           // Event callback
  ARM_I2C_STATUS        status;             // Status flags
  uint8_t               flags;              // Control and state flags
  uint8_t               sla_rw;             // Slave address and RW bit
  uint8_t               stalled;            // Stall mode status flags
  uint8_t               con_aa;             // I2C slave CON flag
  uint32_t              pending;            // Transfer pending (no STOP)
  int32_t               cnt;                // Master transfer count
  uint8_t              *data;               // Master data to transfer
  uint32_t              num;                // Number of bytes to transfer
  uint8_t              *sdata;              // Slave data to transfer
  uint32_t              snum;               // Number of bytes to transfer
} I2C_CTRL;

/* I2C Resource Configuration */
typedef struct {
  LPC_I2Cn_Type        *reg;                // I2C register interface
  int32_t               i2c_ev_irq;         // I2C Event IRQ Number
  volatile uint32_t    *base_clk_reg;       // Base clock register
  volatile uint32_t    *pclk_cfg_reg;       // Peripheral clock config register
  const volatile uint32_t *pclk_stat_reg;   // Peripheral clock status register
  uint32_t              rgu_val;            // Peripheral reset value
  I2C_CTRL             *ctrl;               // Run-Time control information
} const I2C_RESOURCES;

// Global functions and variables exported by driver .c module
#if (RTE_I2C0)
extern ARM_DRIVER_I2C Driver_I2C0;
#endif
#if (RTE_I2C1)
extern ARM_DRIVER_I2C Driver_I2C1;
#endif

#endif /* __I2C_LPC43XX_H */
