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
 * $Revision:    V2.13
 *
 * Driver:       Driver_ETH_MAC0
 * Configured:   via RTE_Device.h configuration file
 * Project:      Ethernet Media Access (MAC) Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                     Value
 *   ---------------------                     -----
 *   Connect to hardware via Driver_ETH_MAC# = 0
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.13
 *    Removed minor compiler warnings
 *  Version 2.12
 *    Added support for ARM Compiler 6
 *  Version 2.11
      - Corrected timeout implementation for RTOS2
 *  Version 2.10
 *    - Added support for CMSIS-RTOS2
 *  Version 2.9
 *    Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 2.8
 *    - Fixed PHY device addressing when using software controlled MDI
 *    - Corrected return value of the ReadFrame function
 *  Version 2.7
 *    - Updated initialization, uninitialization and power procedures
 *  Version 2.6
 *    - Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *  Version 2.5
 *    - Corrected return value of PHY_Read function on timeout
 *  Version 2.4
 *    - GetMacAddress function implemented in Ethernet driver
 *  Version 2.3
 *    - Corrected MDC clock divider setting for ethernet PHY
 *  Version 2.2
 *    - Corrected problem (no communication after cable reconnected)
 *  Version 2.1
 *    - Improved robustness and error control
 *  Version 2.0
 *    - Based on API V2.00
 *    - Added multicast MAC address filtering
 *  Version 1.1
 *    - Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.0
 *    - Initial release
 */

#include "EMAC_LPC17xx.h"

extern ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;

#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,13) /* driver version */

/* Interrupt Handler Prototype */
void ENET_IRQHandler (void);

/* Timeouts */
#define PHY_TIMEOUT         500U        /* PHY Register access timeout in us  */

/* EMAC Memory Buffer configuration for 16K Ethernet RAM */
#define NUM_RX_BUF          4U          /* 0x1800 for Rx (4*1536=6.0K)        */
#define NUM_TX_BUF          3U          /* 0x1200 for Tx (3*1536=4.6K)        */
#define ETH_BUF_SIZE        1536U       /* ETH Receive/Transmit buffer size   */

/* Ethernet Pin definitions */
static PIN eth_pins[] = {
  { RTE_ENET_MDI_MDC_PORT,     RTE_ENET_MDI_MDC_PIN      },
  { RTE_ENET_MDI_MDIO_PORT,    RTE_ENET_MDI_MDIO_PIN     },
#if (RTE_ENET_RMII)
  { RTE_ENET_RMII_TXD0_PORT,   RTE_ENET_RMII_TXD0_PIN    },
  { RTE_ENET_RMII_TXD1_PORT,   RTE_ENET_RMII_TXD1_PIN    },
  { RTE_ENET_RMII_TX_EN_PORT,  RTE_ENET_RMII_TX_EN_PIN   },
  { RTE_ENET_RMII_CRS_PORT,    RTE_ENET_RMII_CRS_PIN     },
  { RTE_ENET_RMII_REF_CLK_PORT,RTE_ENET_RMII_REF_CLK_PIN },
  { RTE_ENET_RMII_RXD0_PORT,   RTE_ENET_RMII_RXD0_PIN    },
  { RTE_ENET_RMII_RXD1_PORT,   RTE_ENET_RMII_RXD1_PIN    },
  { RTE_ENET_RMII_RX_ER_PORT,  RTE_ENET_RMII_RX_ER_PIN   },
#endif
#ifdef RTE_ENET_MII
  { RTE_ENET_MII_TXD0_PORT,    RTE_ENET_MII_TXD0_PIN     },
  { RTE_ENET_MII_TXD1_PORT,    RTE_ENET_MII_TXD1_PIN     },
  { RTE_ENET_MII_TXD2_PORT,    RTE_ENET_MII_TXD2_PIN     },
  { RTE_ENET_MII_TXD3_PORT,    RTE_ENET_MII_TXD3_PIN     },
  { RTE_ENET_MII_TX_EN_PORT,   RTE_ENET_MII_TX_EN_PIN    },
  { RTE_ENET_MII_TX_CLK_PORT,  RTE_ENET_MII_TX_CLK_PIN   },
  { RTE_ENET_MII_RXD0_PORT,    RTE_ENET_MII_RXD0_PIN     },
  { RTE_ENET_MII_RXD1_PORT,    RTE_ENET_MII_RXD1_PIN     },
  { RTE_ENET_MII_RXD2_PORT,    RTE_ENET_MII_RXD2_PIN     },
  { RTE_ENET_MII_RXD3_PORT,    RTE_ENET_MII_RXD3_PIN     },
  { RTE_ENET_MII_RX_DV_PORT,   RTE_ENET_MII_RX_DV_PIN    },
  { RTE_ENET_MII_RX_CLK_PORT,  RTE_ENET_MII_RX_CLK_PIN   },
  { RTE_ENET_MII_RX_ER_PORT,   RTE_ENET_MII_RX_ER_PIN    },
  { RTE_ENET_MII_COL_PORT,     RTE_ENET_MII_COL_PIN      },
  { RTE_ENET_MII_CRS_PORT,     RTE_ENET_MII_CRS_PIN      },
  #if (RTE_ENET_MII_TX_ER_PIN_EN)
  { RTE_ENET_MII_TX_ER_PORT,   RTE_ENET_MII_TX_ER_PIN    },
  #endif
#endif
};

#define MDIO_MASK           0x00000200U
#define MDC_MASK            0x00000100U


#define EMAC_MDC_PIN        (&eth_pins[0])
#define EMAC_MDIO_PIN       (&eth_pins[1])

#if (RTE_ENET_RMII)
#define EMAC_TXD0_PIN       (&eth_pins[2])
#define EMAC_TXD1_PIN       (&eth_pins[3])
#define EMAC_TX_EN_PIN      (&eth_pins[4])
#define EMAC_CRS_PIN        (&eth_pins[5])
#define EMAC_REF_PIN        (&eth_pins[6])
#define EMAC_RXD0_PIN       (&eth_pins[7])
#define EMAC_RXD1_PIN       (&eth_pins[8])
#define EMAC_RX_ER_PIN      (&eth_pins[9])
#endif

#if (RTE_ENET_MII)
#define EMAC_TXD0_PIN       (&eth_pins[2]) 
#define EMAC_TXD1_PIN       (&eth_pins[3])
#define EMAC_TXD2_PIN       (&eth_pins[4]) 
#define EMAC_TXD3_PIN       (&eth_pins[5])
#define EMAC_TX_EN_PIN      (&eth_pins[6])
#define EMAC_TX_CLK_PIN     (&eth_pins[7])
#define EMAC_RXD0_PIN       (&eth_pins[8])
#define EMAC_RXD1_PIN       (&eth_pins[9])
#define EMAC_RXD2_PIN       (&eth_pins[10])
#define EMAC_RXD3_PIN       (&eth_pins[11])
#define EMAC_RX_DV_PIN      (&eth_pins[12])
#define EMAC_REF_PIN        (&eth_pins[13])
#define EMAC_RX_ER_PIN      (&eth_pins[14])
#define EMAC_COL_PIN        (&eth_pins[15])
#define EMAC_CRS_PIN        (&eth_pins[16])
#endif

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_ETH_MAC_API_VERSION,
  ARM_ETH_MAC_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES DriverCapabilities = {
  0U,                               /* checksum_offload_rx_ip4  */
  0U,                               /* checksum_offload_rx_ip6  */
  0U,                               /* checksum_offload_rx_udp  */
  0U,                               /* checksum_offload_rx_tcp  */
  0U,                               /* checksum_offload_rx_icmp */
  0U,                               /* checksum_offload_tx_ip4  */
  0U,                               /* checksum_offload_tx_ip6  */
  0U,                               /* checksum_offload_tx_udp  */
  0U,                               /* checksum_offload_tx_tcp  */
  0U,                               /* checksum_offload_tx_icmp */
  ARM_ETH_INTERFACE_RMII,           /* media_interface          */
  0U,                               /* mac_address              */
  1U,                               /* event_rx_frame           */
  1U,                               /* event_tx_frame           */
  1U,                               /* event_wakeup             */
  0U,                               /* precision_timer          */
  0U                                /* reserved                 */
};

/* EMAC local DMA Descriptors. */
static            RX_Desc Rx_Desc[NUM_RX_BUF];
static __ALIGNED(8) RX_Stat Rx_Stat[NUM_RX_BUF]; /* Must be 8-Byte aligned   */
static            TX_Desc Tx_Desc[NUM_TX_BUF];
static            TX_Stat Tx_Stat[NUM_TX_BUF];

/* EMAC local DMA buffers. */
static uint32_t rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static uint32_t tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

/* Local variables */
static EMAC_CTRL  emac_control = { 0 };
#define emac     (emac_control)

typedef void (*IAP)(uint32_t *cmd, uint32_t *res);
static IAP iap_entry = (IAP)0x1FFF1FF1;

/* Local functions */
static void init_rx_desc (void);
static void init_tx_desc (void);
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val);
static uint32_t crc32_data (const uint8_t *data, uint32_t len);

/**
  \fn          void output_MDIO (uint32_t val, uint32_t n)
  \brief       Output a value to the MII PHY management interface.
  \param[in]   val  Pointer to buffer containing the data
  \param[in]   num  Data length in bytes
  \return      none.
*/
static void output_MDIO (uint32_t val, uint32_t num) {
  for (val <<= (32U - num); num; val <<= 1, num--) {
    GPIO_PinWrite (EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, ((val & 0x80000000U) ? 1U : 0U));
    GPIO_PinWrite (EMAC_MDC_PIN->Portnum, EMAC_MDC_PIN->Pinnum, 1U);
    GPIO_PinWrite (EMAC_MDC_PIN->Portnum, EMAC_MDC_PIN->Pinnum, 0U);
  }
}

/**
  \fn          void turnaround_MDIO (void)
  \brief       Turnaround MDO is tristated.
  \return      none.
*/
static void turnaround_MDIO (void) {
  GPIO_SetDir   (EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, GPIO_DIR_INPUT);
  GPIO_PinWrite (EMAC_MDC_PIN->Portnum,  EMAC_MDC_PIN->Pinnum, 1U);
  GPIO_PinWrite (EMAC_MDC_PIN->Portnum,  EMAC_MDC_PIN->Pinnum, 0U);
}

/**
  \fn          vuint32_t input_MDIO (void)
  \brief       Input a value from the MII PHY management interface.
  \return      none.
*/
static uint32_t input_MDIO (void) {
  uint32_t i,val = 0U;

  for (i = 0U; i < 16U; i++) {
    val <<= 1;
    GPIO_PinWrite (EMAC_MDC_PIN->Portnum, EMAC_MDC_PIN->Pinnum, 1U);
    GPIO_PinWrite (EMAC_MDC_PIN->Portnum, EMAC_MDC_PIN->Pinnum, 0U);
    if (GPIO_PortRead(EMAC_MDIO_PIN->Portnum) & MDIO_MASK) {
      val |= 1U;
    }
  }
  return (val);
}

/**
  \fn          void init_rx_desc (void)
  \brief       Initialize Rx DMA descriptors.
  \return      none.
*/
static void init_rx_desc (void) {
  uint32_t i;

  for (i = 0U; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Packet  = (uint8_t *)&rx_buf[i];
    Rx_Desc[i].Ctrl    = RCTRL_INT | (ETH_BUF_SIZE-1);
    Rx_Stat[i].Info    = 0U;
    Rx_Stat[i].HashCRC = 0U;
  }

  /* Set EMAC Receive Descriptor Registers */
  LPC_EMAC->RxDescriptor       = (uint32_t)&Rx_Desc[0];
  LPC_EMAC->RxStatus           = (uint32_t)&Rx_Stat[0];
  LPC_EMAC->RxDescriptorNumber = NUM_RX_BUF-1U;

  /* Rx Descriptors Point to 0 */
  LPC_EMAC->RxConsumeIndex  = 0U;
}

/**
  \fn          void init_tx_desc (void)
  \brief       Initialize Tx DMA descriptors.
  \return      none.
*/
static void init_tx_desc (void) {
  uint32_t i;

  for (i = 0U; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].Packet = (uint8_t *)&tx_buf[i];
    Tx_Desc[i].Ctrl   = 0U;
    Tx_Stat[i].Info   = 0U;
  }

  /* Set EMAC Transmit Descriptor Registers */
  LPC_EMAC->TxDescriptor       = (uint32_t)&Tx_Desc[0];
  LPC_EMAC->TxStatus           = (uint32_t)&Tx_Stat[0];
  LPC_EMAC->TxDescriptorNumber = NUM_TX_BUF-1;

  /* Tx Descriptors Point to 0 */
  LPC_EMAC->TxProduceIndex  = 0U;
}

/**
  \fn          uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val)
  \brief       Calculate 32-bit CRC (Polynom: 0x04C11DB7, data bit-reversed).
  \param[in]   crc32  CRC initial value
  \param[in]   val    Input value
  \return      Calculated CRC value
*/
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val) {
  uint32_t n;

  crc32 ^= __RBIT(val);
  for (n = 8U; n; n--) {
    if (crc32 & 0x80000000U) {
      crc32 <<= 1;
      crc32  ^= 0x04C11DB7U;
    } else {
      crc32 <<= 1;
    }
  }
  return (crc32);
}

/**
  \fn          uint32_t crc32_data (const uint8_t *data, uint32_t len)
  \brief       Calculate standard 32-bit Ethernet CRC.
  \param[in]   data  Pointer to buffer containing the data
  \param[in]   len   Data length in bytes
  \return      Calculated CRC value
*/
static uint32_t crc32_data (const uint8_t *data, uint32_t len) {
  uint32_t crc;

  for (crc = 0xFFFFFFFFU; len; len--) {
    crc = crc32_8bit_rev (crc, *data++);
  }
  return (crc);
}

/* Ethernet Driver functions */

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event) {
  uint32_t pb[2];
  bool     dev_175x;
#if defined (LPC177x_8x)
  uint32_t cfg_val;
#endif

  if (emac.flags & EMAC_FLAG_INIT) { return ARM_DRIVER_OK; }

  /* Read device ID with IAP */
  pb[0] = 54;
  iap_entry (&pb[0], &pb[0]);
  if ((pb[1] >> 24) == 0x25U) {
    /* Use software RMII management routines */
    dev_175x = true;
  }
  else {
    dev_175x = false;
  }
#if defined (LPC175x_6x)
  /* Enable Ethernet Pins. */
  PIN_Configure (EMAC_TXD0_PIN->Portnum,   EMAC_TXD0_PIN->Pinnum,    RTE_ENET_RMII_TXD0_FUNC,   0U,   0U);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,   EMAC_TXD1_PIN->Pinnum,    RTE_ENET_RMII_TXD1_FUNC,   0U,   0U);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum,  EMAC_TX_EN_PIN->Pinnum,   RTE_ENET_RMII_TX_EN_FUNC,  0U,   0U);
  PIN_Configure (EMAC_CRS_PIN->Portnum,    EMAC_CRS_PIN->Pinnum,     RTE_ENET_RMII_CRS_FUNC,    0U,   0U);
  PIN_Configure (EMAC_REF_PIN->Portnum,    EMAC_REF_PIN->Pinnum,     RTE_ENET_RMII_REF_CLK_FUNC,0U,   0U);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,   EMAC_RXD0_PIN->Pinnum,    RTE_ENET_RMII_RXD0_FUNC,   0U,   0U);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,   EMAC_RXD1_PIN->Pinnum,    RTE_ENET_RMII_RXD1_FUNC,   0U,   0U);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum,  EMAC_RX_ER_PIN->Pinnum,   RTE_ENET_RMII_RX_ER_FUNC,  0U,   0U);

  if (dev_175x == false) {
    /* LPC176x devices, no MDIO, MDC remap. */
    PIN_Configure (RTE_ENET_MDI_MDC_PORT,  RTE_ENET_MDI_MDC_PIN,     RTE_ENET_MDI_MDC_FUNC,     0U,   0U);
    PIN_Configure (RTE_ENET_MDI_MDIO_PORT, RTE_ENET_MDI_MDIO_PIN,    RTE_ENET_MDI_MDIO_FUNC,    0U,   0U);
  }
  else {
    /* LPC175x devices, use software MII management. */  
    PIN_Configure (RTE_ENET_MDI_MDC_PORT,  RTE_ENET_MDI_MDC_PIN,     0U,                         0U,   0U);
    PIN_Configure (RTE_ENET_MDI_MDIO_PORT, RTE_ENET_MDI_MDIO_PIN,    0U,                         0U,   0U);
    GPIO_SetDir(EMAC_MDC_PIN->Portnum,  EMAC_MDC_PIN->Pinnum,  GPIO_DIR_OUTPUT);
  }
  /* Enable P1 Ethernet Pins. */
  LPC_PINCON->PINSEL2 = 0x50150105U;
  if (dev_175x == false) {
    /* LPC176x devices, no MDIO, MDC remap. */
    LPC_PINCON->PINSEL3 = (LPC_PINCON->PINSEL3 & ~0x0000000FU) | 0x00000005U;
  }
  else {
    /* LPC175x devices, use software MII management. */  
    LPC_PINCON->PINSEL4 &= ~0x000F0000U;
    LPC_GPIO2->FIODIR   |= MDC_MASK;
  }
#elif defined (LPC177x_8x)
  cfg_val = IOCON_MODE_PULLUP | IOCON_HYS_ENABLE;
  /* Enable Ethernet Pins. */
  #if (RTE_ENET_RMII) 
  PIN_Configure (EMAC_TXD0_PIN->Portnum,   EMAC_TXD0_PIN->Pinnum,    cfg_val | RTE_ENET_RMII_TXD0_FUNC);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,   EMAC_TXD1_PIN->Pinnum,    cfg_val | RTE_ENET_RMII_TXD1_FUNC);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum,  EMAC_TX_EN_PIN->Pinnum,   cfg_val | RTE_ENET_RMII_TX_EN_FUNC);
  PIN_Configure (EMAC_CRS_PIN->Portnum,    EMAC_CRS_PIN->Pinnum,     cfg_val | RTE_ENET_RMII_CRS_FUNC);
  PIN_Configure (EMAC_REF_PIN->Portnum,    EMAC_REF_PIN->Pinnum,     cfg_val | RTE_ENET_RMII_REF_CLK_FUNC);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,   EMAC_RXD0_PIN->Pinnum,    cfg_val | RTE_ENET_RMII_RXD0_FUNC);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,   EMAC_RXD1_PIN->Pinnum,    cfg_val | RTE_ENET_RMII_RXD1_FUNC);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum,  EMAC_RX_ER_PIN->Pinnum,   cfg_val | RTE_ENET_RMII_RX_ER_FUNC);
  #endif
  #if (RTE_ENET_MII)
  PIN_Configure (EMAC_TXD0_PIN->Portnum,   EMAC_TXD0_PIN->Pinnum,    cfg_val | RTE_ENET_MII_TXD0_FUNC);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,   EMAC_TXD1_PIN->Pinnum,    cfg_val | RTE_ENET_MII_TXD1_FUNC);
  PIN_Configure (EMAC_TXD2_PIN->Portnum,   EMAC_TXD2_PIN->Pinnum,    cfg_val | RTE_ENET_MII_TXD2_FUNC);
  PIN_Configure (EMAC_TXD3_PIN->Portnum,   EMAC_TXD3_PIN->Pinnum,    cfg_val | RTE_ENET_MII_TXD3_FUNC);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum,  EMAC_TX_EN_PIN->Pinnum,   cfg_val | RTE_ENET_MII_TX_EN_FUNC);
  PIN_Configure (EMAC_TX_CLK_PIN->Portnum, EMAC_TX_CLK_PIN->Pinnum,  cfg_val | RTE_ENET_MII_TX_CLK_FUNC);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,   EMAC_RXD0_PIN->Pinnum,    cfg_val | RTE_ENET_MII_RXD0_FUNC);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,   EMAC_RXD1_PIN->Pinnum,    cfg_val | RTE_ENET_MII_RXD1_FUNC);
  PIN_Configure (EMAC_RXD2_PIN->Portnum,   EMAC_RXD2_PIN->Pinnum,    cfg_val | RTE_ENET_MII_RXD2_FUNC);
  PIN_Configure (EMAC_RXD3_PIN->Portnum,   EMAC_RXD3_PIN->Pinnum,    cfg_val | RTE_ENET_MII_RXD3_FUNC);
  PIN_Configure (EMAC_RX_DV_PIN->Portnum,  EMAC_RX_DV_PIN->Pinnum,   cfg_val | RTE_ENET_MII_RX_DV_FUNC);
  PIN_Configure (EMAC_REF_PIN->Portnum,    EMAC_REF_PIN->Pinnum,     cfg_val | RTE_ENET_MII_RX_CLK_FUNC);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum,  EMAC_RX_ER_PIN->Pinnum,   cfg_val | RTE_ENET_MII_RX_ER_FUNC);
  PIN_Configure (EMAC_COL_PIN->Portnum,    EMAC_COL_PIN->Pinnum,     cfg_val | RTE_ENET_MII_COL_FUNC);
  PIN_Configure (EMAC_CRS_PIN->Portnum,    EMAC_CRS_PIN->Pinnum,     cfg_val | RTE_ENET_MII_CRS_FUNC);
  #endif
  PIN_Configure (RTE_ENET_MDI_MDC_PORT,     RTE_ENET_MDI_MDC_PIN,    cfg_val | RTE_ENET_MDI_MDC_FUNC);
  PIN_Configure (RTE_ENET_MDI_MDIO_PORT,    RTE_ENET_MDI_MDIO_PIN,   cfg_val | RTE_ENET_MDI_MDIO_FUNC);
#endif

  /* Clear control structure */
  memset (&emac, 0, sizeof (EMAC_CTRL));

  emac.cb_event = cb_event;
  emac.dev_175x = dev_175x;
  emac.flags    = EMAC_FLAG_INIT;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t Uninitialize (void) {
#if defined (LPC177x_8x)
  uint32_t cfg_val;
#endif 

  emac.flags = 0U;

#if defined (LPC175x_6x)
  /* Disable Ethernet Pins. */
  PIN_Configure (EMAC_TXD0_PIN->Portnum,  EMAC_TXD0_PIN->Pinnum,  0U, 0U, 0U);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,  EMAC_TXD1_PIN->Pinnum,  0U, 0U, 0U);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum, EMAC_TX_EN_PIN->Pinnum, 0U, 0U, 0U);
  PIN_Configure (EMAC_CRS_PIN->Portnum,   EMAC_CRS_PIN->Pinnum,   0U, 0U, 0U);
  PIN_Configure (EMAC_REF_PIN->Portnum,   EMAC_REF_PIN->Pinnum,   0U, 0U, 0U);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,  EMAC_RXD0_PIN->Pinnum,  0U, 0U, 0U);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,  EMAC_RXD1_PIN->Pinnum,  0U, 0U, 0U);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum, EMAC_RX_ER_PIN->Pinnum, 0U, 0U, 0U);

  if (emac.dev_175x == false) {
    /* LPC176x devices, no MDIO, MDC remap. */
    PIN_Configure (EMAC_MDC_PIN->Portnum,  EMAC_MDC_PIN->Pinnum,  0U, 0U, 0U);
    PIN_Configure (EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, 0U, 0U, 0U);
  }
  else {
    /* LPC175x devices, use software MII management. */  
    PIN_Configure (EMAC_MDC_PIN->Portnum,  EMAC_MDC_PIN->Pinnum,  0U, 0U, 0U);
    PIN_Configure (EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, 0U, 0U, 0U);

    GPIO_SetDir (EMAC_MDC_PIN->Portnum, EMAC_MDC_PIN->Pinnum,  0U);
  }
#elif defined (LPC177x_8x)
  cfg_val = IOCON_MODE_PULLUP | IOCON_HYS_ENABLE;
  /* Enable Ethernet Pins. */
  #if (RTE_ENET_RMII) 
  PIN_Configure (EMAC_TXD0_PIN->Portnum,   EMAC_TXD0_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,   EMAC_TXD1_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum,  EMAC_TX_EN_PIN->Pinnum,   cfg_val);
  PIN_Configure (EMAC_CRS_PIN->Portnum,    EMAC_CRS_PIN->Pinnum,     cfg_val);
  PIN_Configure (EMAC_REF_PIN->Portnum,    EMAC_REF_PIN->Pinnum,     cfg_val);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,   EMAC_RXD0_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,   EMAC_RXD1_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum,  EMAC_RX_ER_PIN->Pinnum,   cfg_val);
  #endif
  #if (RTE_ENET_MII)
  PIN_Configure (EMAC_TXD0_PIN->Portnum,   EMAC_TXD0_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TXD1_PIN->Portnum,   EMAC_TXD1_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TXD2_PIN->Portnum,   EMAC_TXD2_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TXD3_PIN->Portnum,   EMAC_TXD3_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_TX_EN_PIN->Portnum,  EMAC_TX_EN_PIN->Pinnum,   cfg_val);
  PIN_Configure (EMAC_TX_CLK_PIN->Portnum, EMAC_TX_CLK_PIN->Pinnum,  cfg_val);
  PIN_Configure (EMAC_RXD0_PIN->Portnum,   EMAC_RXD0_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RXD1_PIN->Portnum,   EMAC_RXD1_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RXD2_PIN->Portnum,   EMAC_RXD2_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RXD3_PIN->Portnum,   EMAC_RXD3_PIN->Pinnum,    cfg_val);
  PIN_Configure (EMAC_RX_DV_PIN->Portnum,  EMAC_RX_DV_PIN->Pinnum,   cfg_val);
  PIN_Configure (EMAC_REF_PIN->Portnum,    EMAC_REF_PIN->Pinnum,     cfg_val);
  PIN_Configure (EMAC_RX_ER_PIN->Portnum,  EMAC_RX_ER_PIN->Pinnum,   cfg_val);
  PIN_Configure (EMAC_COL_PIN->Portnum,    EMAC_COL_PIN->Pinnum,     cfg_val);
  PIN_Configure (EMAC_CRS_PIN->Portnum,    EMAC_CRS_PIN->Pinnum,     cfg_val);
  #endif
  PIN_Configure (RTE_ENET_MDI_MDC_PORT,    RTE_ENET_MDI_MDC_PIN,     cfg_val);
  PIN_Configure (RTE_ENET_MDI_MDIO_PORT,   RTE_ENET_MDI_MDIO_PIN,    cfg_val);
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  uint32_t tout,hclk,div;

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      /* Disable EMAC interrupts */
      NVIC_DisableIRQ(ENET_IRQn);

      /* Power Up the EMAC controller. */
      LPC_SC->PCONP |= 0x40000000U;

      /* Reset all EMAC internal modules. */
      LPC_EMAC->MAC1 = MAC1_SOFT_RES;

      /* A short delay after reset. */
      for (tout = 10U; tout; tout--);

      /* Power Off the EMAC controller. */
      LPC_SC->PCONP &= ~(0x40000000U);

      emac.flags &= ~EMAC_FLAG_POWER;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((emac.flags & EMAC_FLAG_INIT)  == 0U) { return ARM_DRIVER_ERROR; }
      if ((emac.flags & EMAC_FLAG_POWER) != 0U) { return ARM_DRIVER_OK; }

      /* Power Up the EMAC controller. */
      LPC_SC->PCONP |= 0x40000000U;

      /* Reset all EMAC internal modules. */
      LPC_EMAC->MAC1    = MAC1_RES_TX     | MAC1_RES_MCS_TX | MAC1_RES_RX   |
                          MAC1_RES_MCS_RX | MAC1_SIM_RES    | MAC1_SOFT_RES;
#if defined (LPC175x_6x)
      LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM;
#elif defined (LPC177x_8x)
  #if (RTE_ENET_RMII)
      LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM | CR_RMII;
  #else
      LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM;
  #endif
#endif      
      /* A short delay after reset. */
      for (tout = 10U; tout; tout--);

      /* Initialize MAC control registers. */
      LPC_EMAC->MAC1 = MAC1_PASS_ALL;
      LPC_EMAC->MAC2 = MAC2_CRC_EN | MAC2_PAD_EN | MAC2_ADET_PAD_EN;
      LPC_EMAC->MAXF = ETH_BUF_SIZE;
      LPC_EMAC->CLRT = CLRT_DEF;
      LPC_EMAC->IPGR = IPGR_DEF;

      LPC_EMAC->MCFG = MCFG_CLK_SEL | MCFG_RES_MII;
      for (tout = 10U; tout; tout--);

      /* MDC clock range selection */
      hclk = SystemCoreClock;
      if      (hclk > 150000000U) div = MCFG_CS_Div64;
      else if (hclk > 100000000U) div = MCFG_CS_Div60;
      else if (hclk >  60000000U) div = MCFG_CS_Div40;
      else if (hclk >  35000000U) div = MCFG_CS_Div28;
      else if (hclk >  20000000U) div = MCFG_CS_Div14;
      else if (hclk >  10000000U) div = MCFG_CS_Div8;
      else                       div = MCFG_CS_Div4;
      LPC_EMAC->MCFG = div;

      /* Enable Reduced MII interface. */
      LPC_EMAC->Command = CR_RMII | CR_PASS_RUNT_FRM;
      
      /* Reset Reduced MII Logic. */
      LPC_EMAC->SUPP = SUPP_RES_RMII;
      for (tout = 10U; tout; tout--);
      LPC_EMAC->SUPP = 0U;

      /* Initilaize Ethernet MAC Address registers */
      LPC_EMAC->SA0 = 0x00000000U;
      LPC_EMAC->SA1 = 0x00000000U;
      LPC_EMAC->SA2 = 0x00000000U;

      /* Initialize Tx and Rx DMA Descriptors */
      init_rx_desc ();
      init_tx_desc ();
  
      /* Receive Perfect Match Packets */
      LPC_EMAC->RxFilterCtrl = RFC_PERFECT_EN;

      /* Enable EMAC interrupts */
      LPC_EMAC->IntClear  = 0xFFFFU;
      LPC_EMAC->IntEnable = INT_RX_DONE | INT_TX_DONE;

      /* Enable ethernet interrupts */
      NVIC_ClearPendingIRQ(ENET_IRQn);
      NVIC_EnableIRQ(ENET_IRQn);

      emac.frame_end = NULL;
      emac.flags    |= EMAC_FLAG_POWER;
      break;
  }
  
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr) {
  uint32_t val;

  if (!ptr_addr) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return ARM_DRIVER_ERROR;
  }

  val = LPC_EMAC->SA0;
  ptr_addr->b[5] = (uint8_t)(val >> 8);
  ptr_addr->b[4] = (uint8_t)val;
  val = LPC_EMAC->SA1;
  ptr_addr->b[3] = (uint8_t)(val >> 8);
  ptr_addr->b[2] = (uint8_t)val;
  val = LPC_EMAC->SA2;
  ptr_addr->b[1] = (uint8_t)(val >> 8);
  ptr_addr->b[0] = (uint8_t)val;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr) {

  if (!ptr_addr) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return ARM_DRIVER_ERROR;
  }

  /* Set Ethernet MAC Address registers */
  LPC_EMAC->SA0 = (uint32_t)(ptr_addr->b[5] << 8) | ptr_addr->b[4];
  LPC_EMAC->SA1 = (uint32_t)(ptr_addr->b[3] << 8) | ptr_addr->b[2];
  LPC_EMAC->SA2 = (uint32_t)(ptr_addr->b[1] << 8) | ptr_addr->b[0];

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr) {
  uint32_t crc;

  if (!ptr_addr && num_addr) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return ARM_DRIVER_ERROR;
  }

  LPC_EMAC->RxFilterCtrl &= ~(RFC_UCAST_HASH_EN | RFC_MCAST_HASH_EN);
  LPC_EMAC->HashFilterH = 0x00000000U;
  LPC_EMAC->HashFilterL = 0x00000000U;
  
  if (num_addr == 0U) {
    return ARM_DRIVER_OK;
  }

  /* Calculate 64-bit Hash table for MAC addresses */
  for ( ; num_addr; ptr_addr++, num_addr--) {
    crc = crc32_data(&ptr_addr->b[0], 6U) >> 23;
    if (crc & 0x20) {
      LPC_EMAC->HashFilterH |= (1U << (crc & 0x1FU));
    }
    else {
      LPC_EMAC->HashFilterL |= (1U << (crc & 0x1FU));
    }
  }

  /* Enable Rx Filter */
  LPC_EMAC->RxFilterCtrl |= (RFC_UCAST_HASH_EN | RFC_MCAST_HASH_EN);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags) {
  uint8_t *dst;
  uint32_t idx;

  if (!frame || !len) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return ARM_DRIVER_ERROR;
  }

  dst = emac.frame_end;
  idx = LPC_EMAC->TxProduceIndex;
  if (dst == NULL) {
    dst = Tx_Desc[idx].Packet;
    emac.frame_len = len;
  }
  else {
    /* Sending data fragments in progress */
    emac.frame_len += len;
  }
  /* Fast-copy data fragments to EMAC-DMA buffer */
  for ( ; len > 7U; dst += 8U, frame += 8U, len -= 8U) {
    __UNALIGNED_UINT32_WRITE(&dst[0], __UNALIGNED_UINT32_READ(&frame[0]));
    __UNALIGNED_UINT32_WRITE(&dst[4], __UNALIGNED_UINT32_READ(&frame[4]));
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1U; dst += 2U, frame += 2U, len -= 2U) {
    __UNALIGNED_UINT16_WRITE(&dst[0], __UNALIGNED_UINT16_READ(&frame[0]));
  }
  if (len > 0U) dst++[0] = frame++[0];

  if (flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) {
    /* More data to come, remember current write position */
    emac.frame_end = dst;
    return ARM_DRIVER_OK;
  }

  Tx_Desc[idx].Ctrl = (emac.frame_len-1U) | (TCTRL_INT | TCTRL_LAST);

  emac.frame_end = NULL;
  emac.frame_len = 0U;

  /* Start frame transmission. */
  if (++idx == NUM_TX_BUF) idx = 0U;
  LPC_EMAC->TxProduceIndex = idx;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref execution_status 
*/
static int32_t ReadFrame (uint8_t *frame, uint32_t len) {
  uint8_t const *src;
  uint32_t idx;
  int32_t cnt = (int32_t)len;

  if (!frame && len) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return ARM_DRIVER_ERROR;
  }

  idx = LPC_EMAC->RxConsumeIndex;
  src = (uint8_t const *)Rx_Desc[idx].Packet;
  /* Fast-copy data to packet buffer */
  for ( ; len > 7U; frame += 8U, src += 8U, len -= 8U) {
    __UNALIGNED_UINT32_WRITE(&frame[0], __UNALIGNED_UINT32_READ(&src[0]));
    __UNALIGNED_UINT32_WRITE(&frame[4], __UNALIGNED_UINT32_READ(&src[4]));
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1U; frame += 2U, src += 2U, len -= 2U) {
    __UNALIGNED_UINT16_WRITE(&frame[0], __UNALIGNED_UINT16_READ(&src[0]));
  }
  if (len > 0U) frame[0] = src[0];

  if (++idx == NUM_RX_BUF) idx = 0U;
  /* Release frame from EMAC buffer */
  LPC_EMAC->RxConsumeIndex = idx;

  return (cnt);
}

/**
  \fn          uint32_t GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t GetRxFrameSize (void) {
  uint32_t info,idx;

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not yet powered */
    return (0U);
  }

  idx = LPC_EMAC->RxConsumeIndex;
  if (idx == LPC_EMAC->RxProduceIndex) {
    /* No packet received */
    return (0U);
  }

  info = Rx_Stat[idx].Info;
  if (!(info & RINFO_LAST_FLAG) || (info & RINFO_ERR_MASK)) {
    /* Error, this block is invalid */
    return (0xFFFFFFFFU);
  }

  return ((info & RINFO_SIZE) - 3U);
}

/* Ethernet IRQ Handler */
void ENET_IRQHandler (void) {
  /* EMAC Ethernet Controller Interrupt function. */
  uint32_t int_stat;
  uint32_t event = 0U;

  int_stat = (LPC_EMAC->IntStatus & LPC_EMAC->IntEnable);
  LPC_EMAC->IntClear = int_stat;
  
  if (int_stat & INT_RX_DONE) {
    /* Packet received, check if packet is valid. */
    event |= ARM_ETH_MAC_EVENT_RX_FRAME;
  }
  if (int_stat & INT_TX_DONE) {
    /* Frame transmit completed. */
    event |= ARM_ETH_MAC_EVENT_TX_FRAME;
  }
  /* Callback event notification */
  if (event && emac.cb_event) {
    emac.cb_event (event);
  }
}

/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time) {
  (void)time;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time) {
  (void)time;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  uint32_t rxfilterctrl;
  uint32_t command, mac1, mac2, igpt, supp;

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not powered */
    return ARM_DRIVER_ERROR;
  }
  
  switch (control) {
    case ARM_ETH_MAC_CONFIGURE:
      mac2    = LPC_EMAC->MAC2    & ~(MAC2_FULL_DUP | MAC2_FULL_DUP);
      command = LPC_EMAC->Command & ~CR_FULL_DUP;
      igpt    = LPC_EMAC->IPGT    & ~IPGT_FULL_DUP;
      mac1    = LPC_EMAC->MAC1    & ~MAC1_LOOPB;
      supp    = LPC_EMAC->SUPP    & ~SUPP_SPEED;
      /* Configure 100MBit/10MBit mode */
      switch (arg & ARM_ETH_MAC_SPEED_Msk) {
        case ARM_ETH_MAC_SPEED_10M:
          break;
        case ARM_ETH_SPEED_100M:
#if (RTE_ENET_RMII)
          supp |= SUPP_SPEED;
#endif
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      /* Configure Half/Full duplex mode */
      switch (arg & ARM_ETH_MAC_DUPLEX_Msk) {
        case ARM_ETH_MAC_DUPLEX_FULL:
          mac2    |= MAC2_FULL_DUP;
          command |= CR_FULL_DUP;
          igpt    |= IPGT_FULL_DUP;
          break;
      }

      /* Configure loopback mode */
      if (arg & ARM_ETH_MAC_LOOPBACK) {
        mac1 |= MAC1_LOOPB;
      }

      if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) ||
          (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)) {
        /* Checksum offload is disabled in the driver */
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      LPC_EMAC->SUPP    = supp;
      LPC_EMAC->MAC2    = mac2;
      LPC_EMAC->Command = command;
      LPC_EMAC->IPGT    = igpt;
      LPC_EMAC->MAC1    = mac1;

      rxfilterctrl = LPC_EMAC->RxFilterCtrl & ~(RFC_UCAST_EN | RFC_BCAST_EN | RFC_MCAST_EN);
      /* Enable broadcast frame receive */
      if (arg & ARM_ETH_MAC_ADDRESS_BROADCAST) {
        rxfilterctrl |= RFC_BCAST_EN;
      }

      /* Enable all multicast frame receive */
      if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST) {
        rxfilterctrl |= RFC_MCAST_EN;
      }

      /* Enable promiscuous mode (no filtering) */
      if (arg & ARM_ETH_MAC_ADDRESS_ALL) {
        rxfilterctrl |= (RFC_BCAST_EN | RFC_UCAST_EN | RFC_MCAST_EN);
      }
      LPC_EMAC->RxFilterCtrl = rxfilterctrl;
      break;

    case ARM_ETH_MAC_CONTROL_TX:
      /* Enable/disable MAC transmitter */
      command = LPC_EMAC->Command & ~CR_TX_EN;

      if (arg != 0U) {
        command |= CR_TX_EN;
      }
      LPC_EMAC->Command = command;
      break;

    case ARM_ETH_MAC_CONTROL_RX:
      /* Enable/disable MAC receiver */
      command = LPC_EMAC->Command & ~CR_RX_EN;
      mac1    = LPC_EMAC->MAC1    & ~MAC1_REC_EN;
      if (arg != 0U) {
        command |= CR_RX_EN;
        mac1    |= MAC1_REC_EN;
      }
      LPC_EMAC->Command = command;
      LPC_EMAC->MAC1    = mac1;
      break;

    case ARM_ETH_MAC_FLUSH:
      /* Flush Tx and Rx buffers */
      if (arg & ARM_ETH_MAC_FLUSH_RX) {
        /* Stop/Start DMA Receive */
        command = LPC_EMAC->Command;
        LPC_EMAC->Command &= ~CR_RX_EN;
        init_rx_desc ();
        LPC_EMAC->Command = command;
      }
      if (arg & ARM_ETH_MAC_FLUSH_TX) {
        /* Stop/Start DMA Transmit */
        command = LPC_EMAC->Command;
        LPC_EMAC->Command &= ~CR_TX_EN;
        init_tx_desc ();
        LPC_EMAC->Command = command;
      }
      break;

    case ARM_ETH_MAC_VLAN_FILTER:
      /* Configure VLAN filter */
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  Operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time) {
  (void)time;
  (void)control;

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data) {
  uint32_t tick;

  if (!data) {
    /* Invalid parameter */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not powered */
    return ARM_DRIVER_ERROR;
  }

  if (emac.dev_175x == true) {
    /* Software MII Management for LPC175x. */
    /* Remapped MDC on P2.8 and MDIO on P2.9 does not work. */
    GPIO_SetDir(EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, GPIO_DIR_OUTPUT);
    /* 32 consecutive ones on MDO to establish sync */
    output_MDIO (0xFFFFFFFFU, 32U);

    /* start code (01), read command (10) */
    output_MDIO (0x06U, 4U);

    /* write PHY address */
    output_MDIO (phy_addr, 5U);

    /* write the PHY register to write */
    output_MDIO (reg_addr, 5U);

    /* turnaround MDO is tristated */
    turnaround_MDIO ();

    /* read the data value */
    *data = (uint16_t)input_MDIO ();

    /* turnaround MDIO is tristated */
    turnaround_MDIO ();

    return ARM_DRIVER_OK;
  }
  else {
    LPC_EMAC->MADR = (uint32_t)(phy_addr << 8) | reg_addr;
    LPC_EMAC->MCMD = MCMD_READ;

    /* Wait until operation completed */
#if   defined(RTE_CMSIS_RTOS2)
    tick = osKernelGetSysTimerCount();
#elif defined(RTE_CMSIS_RTOS)
    tick = osKernelSysTick();
#endif
    do {
      if ((LPC_EMAC->MIND & MIND_BUSY) == 0U) break;
#if   defined(RTE_CMSIS_RTOS2)
    } while ((osKernelGetSysTimerCount() - tick) < (((uint64_t)PHY_TIMEOUT * osKernelGetSysTimerFreq()) / 1000000U));
#elif defined(RTE_CMSIS_RTOS)
    } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(PHY_TIMEOUT));
#endif

    if ((LPC_EMAC->MIND & MIND_BUSY) == 0U) {
      LPC_EMAC->MCMD = 0U;
      *data = (uint16_t)LPC_EMAC->MRDD;
      return ARM_DRIVER_OK;
    }
  }
  return ARM_DRIVER_ERROR_TIMEOUT;
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data) {
  uint32_t tick;

  if (!(emac.flags & EMAC_FLAG_POWER)) {
    /* Driver not powered */
    return ARM_DRIVER_ERROR;
  }

  if (emac.dev_175x == true) {
    /* Software MII Management for LPC175x. */
    /* Remapped MDC on P2.8 and MDIO on P2.9 do not work. */
    GPIO_SetDir(EMAC_MDIO_PIN->Portnum, EMAC_MDIO_PIN->Pinnum, GPIO_DIR_OUTPUT);
    /* 32 consecutive ones on MDO to establish sync */
    output_MDIO (0xFFFFFFFFU, 32U);

    /* start code (01), write command (01) */
    output_MDIO (0x05U, 4U);

    /* write PHY address */
    output_MDIO (phy_addr, 5U);

    /* write the PHY register to write */
    output_MDIO (reg_addr, 5U);

    /* turnaround MDIO (1,0)*/
    output_MDIO (0x02U, 2U);

    /* write the data value */
    output_MDIO (data, 16U);

    /* turnaround MDO is tristated */
    turnaround_MDIO ();

    return ARM_DRIVER_OK;
  }
  else {
    /* Hardware MII Management for LPC176x devices. */
    LPC_EMAC->MADR = (uint32_t)(phy_addr << 8) | reg_addr;
    LPC_EMAC->MWTD = data;

    /* Wait until operation completed */
#if   defined(RTE_CMSIS_RTOS2)
    tick = osKernelGetSysTimerCount();
#elif defined(RTE_CMSIS_RTOS)
    tick = osKernelSysTick();
#endif
    do {
      if ((LPC_EMAC->MIND & MIND_BUSY) == 0U) break;
#if   defined(RTE_CMSIS_RTOS2)
    } while ((osKernelGetSysTimerCount() - tick) < (((uint64_t)PHY_TIMEOUT * osKernelGetSysTimerFreq()) / 1000000U));
#elif defined(RTE_CMSIS_RTOS)
    } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(PHY_TIMEOUT));
#endif
    
    if ((LPC_EMAC->MIND & MIND_BUSY) == 0U) {
      return ARM_DRIVER_OK;
    }
  }
  return ARM_DRIVER_ERROR_TIMEOUT;
}

/* MAC Driver Control Block */
ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  GetMacAddress,
  SetMacAddress,
  SetAddressFilter,
  SendFrame,
  ReadFrame,
  GetRxFrameSize,
  GetRxFrameTime,
  GetTxFrameTime,
  ControlTimer,
  Control,
  PHY_Read,
  PHY_Write
};
