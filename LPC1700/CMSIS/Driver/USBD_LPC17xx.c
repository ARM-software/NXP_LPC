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
 * $Revision:    V2.14
 *
 * Driver:       Driver_USBD0
 * Configured:   via RTE_Device.h configuration file
 * Project:      USB Device Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                  Value
 *   ---------------------                  -----
 *   Connect to hardware via Driver_USBD# = 0
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   USBD_MAX_ENDPOINT_NUM:  defines maximum number of IN/OUT Endpoint pairs
 *                           that driver will support with Control Endpoint 0
 *                           not included, this value impacts driver memory
 *                           requirements
 *     - default value: 15
 *     - maximum value: 15
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.14
 *    Removed minor compiler warnings
 *  Version 2.13
 *    Added support for ARM Compiler 6
 *  Version 2.12
 *    Fixed USBD_HW_ReadEP/USBD_HW_WriteEP function behavior for data parameter equal to NULL
 *  Version 2.11
 *    Corrected EndpointUnconfigure function for isochronous endpoints
 *    (unconfiguring one isochronous endpoint prevented further functionality
 *     of any other configured isochronous endpoints)
 *  Version 2.10
 *    Removed access to OTG registers for non OTG peripheral
 *  Version 2.9
 *    Removed unnecessary __packed specifier for structure ENDPOINT_t
 *  Version 2.8
 *    Corrected functionality in case of SETUP followed by OUT packet
 *    (occurring in SetLineCoding request on CDC ACM device)
 *  Version 2.7
 *    Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 2.6
 *    Added check for USBD_EndpointConfigure function: allow only endpoint
 *    configuration supported by hardware
 *  Version 2.5
 *    Corrected initialization failure if Port1 not enabled in RTE_Device.h
 *  Version 2.4
 *    Corrected reading data from stalled endpoint causing hardfault
 *  Version 2.3
 *    PowerControl for Power OFF and Uninitialize functions made unconditional
 *  Version 2.2
 *    Added support for U1 and U2 for LPC177x/8x
 *    Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *  Version 2.1
 *    Added support for isochronous endpoints
 *  Version 2.0
 *    Initial release for CMSIS Drivers v2.01 API
 */

#include <stdint.h>
#include <string.h>

#include "Driver_USBD.h"

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
#endif
#include "USBD_LPC17xx.h"

#if      (RTE_USB_USB0 == 0)
#error   "USB is not enabled in the RTE_Device.h!"
#endif

#ifndef USBD_MAX_ENDPOINT_NUM
#define USBD_MAX_ENDPOINT_NUM           15U
#endif
#if    (USBD_MAX_ENDPOINT_NUM > 15)
#error  Too many Endpoints, maximum IN/OUT Endpoint pairs that this driver supports is 15 !!!
#endif

#ifndef USBD_CTRL_ENDPOINT_MASK
#define USBD_CTRL_ENDPOINT_MASK         (0x00000003U)
#endif
#ifndef USBD_INT_ENDPOINT_MASK
#define USBD_INT_ENDPOINT_MASK          (0x0C30C30CU)
#endif
#ifndef USBD_BULK_ENDPOINT_MASK
#define USBD_BULK_ENDPOINT_MASK         (0xF0C30C30U)
#endif
#ifndef USBD_ISO_ENDPOINT_MASK
#define USBD_ISO_ENDPOINT_MASK          (0x030C30C0U)
#endif

#define USBD_DRIVER_INITIALIZED         (1U     )
#define USBD_DRIVER_POWERED             (1U << 1)

extern ARM_DRIVER_USBD Driver_USBD0;

extern uint8_t usb_role;
extern uint8_t usb_state;

#if defined (LPC177x_8x)
extern int32_t USB_I2C_Initialize       (void);
extern int32_t USB_I2C_Uninitialize     (void);
extern int32_t USB_I2C_RegisterRead     (uint8_t i2c_addr, uint8_t reg_addr);
extern int32_t USB_I2C_RegisterWrite    (uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_val);
extern int32_t USB_I2C_DpPullUp         (bool enable);
extern int32_t USB_I2C_ControlDmPullUp  (bool enable);
extern int32_t USB_I2C_ControlPullDowns (bool enable);
#endif
extern int32_t USB_PinsConfigure        (void);
extern int32_t USB_PinsUnconfigure      (void);


// USBD Driver *****************************************************************

#define ARM_USBD_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,14)

// Driver Version
static const ARM_DRIVER_VERSION usbd_driver_version = { ARM_USBD_API_VERSION, ARM_USBD_DRV_VERSION };

// Driver Capabilities
static const ARM_USBD_CAPABILITIES usbd_driver_capabilities = {
  1U,   // VBUS Detection
  1U,   // Event VBUS On
  1U    // Event VBUS Off
#if (defined(ARM_USBD_API_VERSION) && (ARM_USBD_API_VERSION >= 0x202U))
, 0U
#endif
};

#define EP_PHY(ep_addr)                 ((((ep_addr) & 0x0FU) << 1) + (((ep_addr) >> 7) & 1U))

typedef struct {                        // Endpoint information structure
  uint8_t  *data;
  uint32_t  num;
  uint32_t  num_transferred_total;
  uint16_t  num_transferring;
  uint16_t  max_packet_size;
  uint8_t   type;
  uint8_t   active;
  uint8_t   out_irq_pending;
  uint8_t   reserved;
} ENDPOINT_t;

static ARM_USBD_SignalDeviceEvent_t   SignalDeviceEvent;
static ARM_USBD_SignalEndpointEvent_t SignalEndpointEvent;

static ARM_USBD_STATE      usbd_state;

static uint32_t            ep_iso_cfg_mask;
static uint32_t            ep_iso_act_mask;
static volatile uint8_t    usbd_int_active;
static volatile uint32_t   usbd_sie_busy;
static volatile uint8_t    setup_received;

static ENDPOINT_t          ep[(USBD_MAX_ENDPOINT_NUM + 1U) * 2U];

// Function prototypes
static int32_t USBD_EndpointConfigure (uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size);
void USBD_IRQ (void);

// Auxiliary functions

// Function check_and_set_var tries to set variable (uint32_t) to 1 if it was
// interrupted or var was already 1 it returns false, if it succeeded it
// returns true (it uses load/store exclusive to achieve atomicity)
__inline static bool check_and_set_var (volatile uint32_t *addr) {
  bool val;
  do {
    val = __LDREXW (addr);
    if ((val & 1U) != 0U) {
      __CLREX ();
      return false;
    }
  } while (__STREXW (1, addr));
  return true;
}

/**
  \fn          void SIE_WrCmd (uint32_t cmd)
  \brief       Write command to Serial Interface Engine (SIE).
  \param[in]   cmd     Command
*/
static void SIE_WrCmd (uint32_t cmd) {

  LPC_USB->DevIntClr = USBD_CCEMTY_INT;
  LPC_USB->CmdCode   = cmd;
  while ((LPC_USB->DevIntSt & USBD_CCEMTY_INT) == 0U);
}

/**
  \fn          void SIE_WrCmdData (uint32_t cmd, uint32_t data)
  \brief       Write command data to Serial Interface Engine (SIE).
  \param[in]   cmd     Command
  \param[in]   data    Data
*/
static void SIE_WrCmdData (uint32_t cmd, uint32_t data) {

  LPC_USB->DevIntClr = USBD_CCEMTY_INT;
  LPC_USB->CmdCode   = cmd;
  while ((LPC_USB->DevIntSt & USBD_CCEMTY_INT) == 0U);
  LPC_USB->DevIntClr = USBD_CCEMTY_INT;
  LPC_USB->CmdCode   = data;
  while ((LPC_USB->DevIntSt & USBD_CCEMTY_INT) == 0U);
}

/**
  \fn          void SIE_EP_WrCmd (uint32_t ep_phy, uint32_t cmd)
  \brief       Write Endpoint command to Serial Interface Engine (SIE).
  \param[in]   ep_phy  Physical Endpoint index
  \param[in]   cmd     Command
*/
static void SIE_EP_WrCmd (uint32_t ep_phy, uint32_t cmd) {

  LPC_USB->DevIntClr = USBD_CCEMTY_INT;
  LPC_USB->CmdCode   = USBD_CMD_SEL_EP(ep_phy);
  while ((LPC_USB->DevIntSt & USBD_CCEMTY_INT) == 0U);
  LPC_USB->DevIntClr = USBD_CCEMTY_INT;
  LPC_USB->CmdCode   = cmd;
}

/**
  \fn          uint32_t SIE_RdCmdData (uint32_t cmd)
  \brief       Read command data from Serial Interface Engine (SIE).
  \param[in]   cmd  Command
  \return      Read command data
*/
static uint32_t SIE_RdCmdData (uint32_t cmd) {
  uint32_t CmdData;

  LPC_USB->DevIntClr = USBD_CCEMTY_INT | USBD_CDFULL_INT;
  LPC_USB->CmdCode   = cmd;
  while ((LPC_USB->DevIntSt & USBD_CDFULL_INT) == 0U);
  CmdData            = LPC_USB->CmdData;

  return CmdData;
}

/**
  \fn          void USBD_HW_BusReset (void)
  \brief       USB Bus Reset.
*/
static void USBD_HW_Reset (void) {

  // Disable Endpoint interrupts and clear all interrupts
  LPC_USB->EpIntEn   = 0U;
  LPC_USB->EpIntClr  = 0xFFFFFFFFU;
  LPC_USB->DevIntClr = 0xFFFFFFFFU;

  // Reset global variables
  setup_received       = 0U;
  memset((void *)&usbd_state, 0, sizeof(usbd_state));
  memset((void *)ep,          0, sizeof(ep));
  ep_iso_cfg_mask      = 0U;
  ep_iso_act_mask      = 0U;

  // Default Initialize Control Endpoint 0
  USBD_EndpointConfigure (0x00U, ARM_USB_ENDPOINT_CONTROL,  8U);
  USBD_EndpointConfigure (0x80U, ARM_USB_ENDPOINT_CONTROL,  8U);
}

/**
  \fn          void USBD_HW_ConfigEP (uint32_t ep_phy, uint32_t ep_max_packet_size)
  \brief       Configure Endpoint.
  \param[in]   ep_phy             Physical Endpoint index
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
*/
static void USBD_HW_ConfigEP (uint32_t ep_phy, uint32_t ep_max_packet_size) {

  LPC_USB->EpInd     = ep_phy;
  LPC_USB->MaxPSize  = ep_max_packet_size;
  while ((LPC_USB->DevIntSt & USBD_EP_RLZED_INT) == 0U);
  LPC_USB->DevIntClr = USBD_EP_RLZED_INT;
}

/**
  \fn          void USBD_HW_EnableEP (uint32_t ep_phy)
  \brief       Enable Endpoint.
  \param[in]   ep_phy Physical Endpoint index
*/
static void USBD_HW_EnableEP (uint32_t ep_phy) {

  LPC_USB->ReEp     |= 1U << ep_phy;    // Realize endpoint
  while ((LPC_USB->DevIntSt & USBD_EP_RLZED_INT) == 0U);
  LPC_USB->DevIntClr = USBD_EP_RLZED_INT;

  LPC_USB->EpIntEn  |= 1U << ep_phy;    // Enable Endpoint interrupts
}

/**
  \fn          USBD_HW_DisableEP (uint32_t ep_phy)
  \brief       Disable Endpoint.
  \param[in]   ep_phy  Physical Endpoint index
*/
static void USBD_HW_DisableEP (uint32_t ep_phy) {

  LPC_USB->EpIntEn  &= ~(1U << ep_phy); // Disable Endpoint interrupts

  LPC_USB->ReEp     &= ~(1U << ep_phy); // De-realize endpoint
  while ((LPC_USB->DevIntSt & USBD_EP_RLZED_INT) == 0U);
  LPC_USB->DevIntClr = USBD_EP_RLZED_INT;
}

/**
  \fn          uint32_t USBD_HW_ReadEP (uint32_t ep_log, uint8_t *data)
  \brief       Read data received on Endpoint.
  \param[in]   ep_log Logical Endpoint
  \param[out]  data   Pointer to buffer for data to read
  \return      Number of bytes read
*/
static uint32_t USBD_HW_ReadEP (uint32_t ep_log, uint8_t *data) {
  uint32_t ep_phy;
  uint32_t num, i, val;

  ep_phy = EP_PHY(ep_log);

  LPC_USB->Ctrl = ((ep_log & 0x0FU) << 2) | USBD_CTRL_RD_EN;

  do {
    num = LPC_USB->RxPLen;
  } while ((num & USBD_PKT_RDY) == 0U);
  num &= USBD_PKT_LNGTH_MASK;

  for (i = 0U; (i + 4U) <= num; i += 4U) {
    if (data != NULL) {
      __UNALIGNED_UINT32_WRITE(data, LPC_USB->RxData);
      data += 4U;
    } else {
      LPC_USB->RxData;
    }
  }
  if (i < num) {
    val = LPC_USB->RxData;
    for (; i < num; i++) {
      if (data != NULL) {
        *(data++) = (uint8_t)val;
        val >>= 8;
      }
    }
  }

  LPC_USB->Ctrl = 0U;

  if (ep[ep_phy].type != ARM_USB_ENDPOINT_ISOCHRONOUS) {
    // Non-Isochronous Endpoint
    SIE_EP_WrCmd(ep_phy, USBD_CMD_CLR_BUF);
  }

  return num;
}

/**
  \fn          uint32_t USBD_HW_WriteEP (uint32_t ep_log, uint8_t *data, uint32_t num)
  \brief       Write data to be transferred on Endpoint.
  \param[in]   ep_log Logical Endpoint
  \param[out]  data   Pointer to buffer with data to write
  \param[in]   num    Number of data bytes to transfer
  \return      Number of bytes written
*/
static uint32_t USBD_HW_WriteEP (uint32_t ep_log, uint8_t *data, uint32_t num) {
  uint32_t ep_phy;
  uint32_t i;

  if ((data == NULL) && (num != 0U)) { return 0U; }

  ep_phy = EP_PHY(ep_log | 0x80U);

  if (num > ep[ep_phy].max_packet_size) {
    num = ep[ep_phy].max_packet_size;
  }

  LPC_USB->Ctrl = ((ep_log & 0x0FU) << 2) | USBD_CTRL_WR_EN;

  LPC_USB->TxPLen = num;

  for (i = 0U; i < (num + 3U) / 4U; i++) {
     LPC_USB->TxData = __UNALIGNED_UINT32_READ(data);
    data += 4U;
  }

  LPC_USB->Ctrl = 0U;

  SIE_EP_WrCmd(ep_phy, USBD_CMD_VALID_BUF);

  return num;
}


// USBD Driver functions

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion (void) { return usbd_driver_version; }

/**
  \fn          ARM_USBD_CAPABILITIES USBD_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBD_GetCapabilities (void) { return usbd_driver_capabilities; }

/**
  \fn          int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                        ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBD_Initialize (ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) {

  if ((usb_state & USBD_DRIVER_INITIALIZED) != 0U) { return ARM_DRIVER_OK; }

  SignalDeviceEvent   = cb_device_event;
  SignalEndpointEvent = cb_endpoint_event;

  usb_role   =  ARM_USB_ROLE_DEVICE;

  if (USB_PinsConfigure () == -1) {
    usb_role = ARM_USB_ROLE_NONE;
    return ARM_DRIVER_ERROR;
  }

  usb_state |=  USBD_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_Uninitialize (void)
  \brief       De-initialize USB Device Interface.
  \return      \ref execution_status
*/
static int32_t USBD_Uninitialize (void) {

  if (USB_PinsUnconfigure () == -1) { return ARM_DRIVER_ERROR; }
  usb_role   =  ARM_USB_ROLE_NONE;
  usb_state  =  0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Device Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USBD_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ      (USB_IRQn);                  // Disable interrupt
      NVIC_ClearPendingIRQ (USB_IRQn);                  // Clear pending interrupt
      usb_state &= ~USBD_DRIVER_POWERED;                // Clear powered flag
                                                        // Reset variables
      setup_received =  0U;
      memset((void *)&usbd_state, 0, sizeof(usbd_state));

      LPC_SC->PCONP |=  (1UL << 31);                    // USB PCLK -> enable USB Peripheral
#if   defined (RTE_Drivers_USBH0)
      LPC_USB->OTGClkCtrl |= 0x1AU;                     // Enable AHB, OTG and Device clocks
      while ((LPC_USB->OTGClkSt & 0x1AU) != 0x1AU);     // Wait for AHB, OTG and Device clocks
#else
      LPC_USB->OTGClkCtrl |= 0x12U;                     // Enable AHB and Device clocks
      while ((LPC_USB->OTGClkSt & 0x12U) != 0x12U);     // Wait for AHB and Device clocks
#endif
      LPC_USB->USBClkCtrl |= 0x12U;                     // Enable AHB and Device clocks
      while ((LPC_USB->USBClkSt & 0x12U) != 0x12U);

      LPC_USB->DevIntEn = 0U;                           // Disable USB Controller Device interrupts

      LPC_USB->USBClkCtrl &= ~0x02U;                    // Disable Device clock
      while ((LPC_USB->USBClkSt & 0x02U) != 0U);

      if ((usb_state & 0xF0U) == 0U) {                  // If Host is not enabled
        LPC_USB->USBClkCtrl &= ~0x10U;                  // Disable AHB clock
        while ((LPC_USB->USBClkSt & 0x10U) != 0U);
#if     defined (RTE_Drivers_USBH0)
        LPC_USB->OTGClkCtrl &= ~0x1AU;                  // Disable AHB, OTG and Device clocks
        while ((LPC_USB->OTGClkSt & 0x1AU) != 0U);      // Wait for AHB, OTG and Device clocks disabled
#else
        LPC_USB->OTGClkCtrl &= ~0x12U;                  // Disable AHB and Device clock
        while ((LPC_USB->OTGClkSt & 0x12U) != 0U);      // Wait for AHB and Device clock disabled
#endif
        LPC_SC->PCONP &= ~(1UL << 31);                  // USB PCLK -> disable USB Peripheral
      } else {                                          // If Host is enabled
        NVIC_EnableIRQ (USB_IRQn);                      // Enable interrupt
      }
      break;

    case ARM_POWER_FULL:
      if ((usb_state & USBD_DRIVER_INITIALIZED) == 0U) { return ARM_DRIVER_ERROR; }
      if ((usb_state & USBD_DRIVER_POWERED)     != 0U) { return ARM_DRIVER_OK; }

      LPC_SC->PCONP |=  (1UL << 31);                    // USB PCLK -> enable USB Peripheral
#if   defined (RTE_Drivers_USBH0)
      LPC_USB->OTGClkCtrl |= 0x1AU;                     // Enable AHB, OTG and Device clocks
      while ((LPC_USB->OTGClkSt & 0x1AU) != 0x1AU);     // Wait for AHB, OTG and Device clocks
#else
      LPC_USB->OTGClkCtrl |= 0x12U;                     // Enable AHB and Device clocks
      while ((LPC_USB->OTGClkSt & 0x12U) != 0x12U);     // Wait for AHB and Device clocks
#endif
      LPC_USB->USBClkCtrl |= 0x12U;                     // Enable AHB and Device clocks
      while ((LPC_USB->USBClkSt & 0x12U) != 0x12U);     // Wait for AHB and Device clocks




#if  (defined (LPC175x_6x) && defined (RTE_Drivers_USBH0))
      LPC_USB->StCtrl &= ~0x01U;                        // Reset port function
#elif defined (LPC177x_8x)
      LPC_USB->USBClkCtrl |= 0x08U;                     // Enable port select register clocks
      while ((LPC_USB->USBClkSt & 0x08U) == 0U);
      LPC_USB->StCtrl &= ~0x03U;                        // Reset port function
      if (LPC_USB->StCtrl != RTE_USB_PORT_CFG) {
        LPC_USB->StCtrl |=  RTE_USB_PORT_CFG;           // Select port function
      }
#if ((RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
      USB_I2C_Initialize ();                            // Initialize I2C for OTG Transceiver
#endif
#endif
      LPC_USB->DevIntEn  = USBD_DEV_STAT_INT | USBD_EP_SLOW_INT;

      usb_state |=  USBD_DRIVER_POWERED;                // Set powered flag
      NVIC_EnableIRQ   (USB_IRQn);                      // Enable interrupt
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceConnect (void)
  \brief       Connect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceConnect (void) {

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  SIE_WrCmdData (USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(USBD_DEV_CON));

#if (defined (LPC177x_8x) && (RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
  USB_I2C_DpPullUp (true);
#endif

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceDisconnect (void)
  \brief       Disconnect USB Device.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceDisconnect (void) {

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

#if (defined (LPC177x_8x) && (RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG == 0))
  USB_I2C_DpPullUp (false);
#endif

  SIE_WrCmdData (USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(0));

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBD_DeviceGetState (void)
  \brief       Get current USB Device State.
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBD_DeviceGetState (void) {
  return usbd_state;
}

/**
  \fn          int32_t USBD_DeviceRemoteWakeup (void)
  \brief       Trigger USB Remote Wakeup.
  \return      \ref execution_status
*/
static int32_t USBD_DeviceRemoteWakeup (void) {

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  USBD_PowerControl (ARM_POWER_FULL);

  SIE_WrCmdData (USBD_CMD_SET_DEV_STAT, USBD_DAT_WR_BYTE(SIE_RdCmdData(USBD_CMD_GET_DEV_STAT) & (~USBD_DEV_SUS)));

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_DeviceSetAddress (uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBD_DeviceSetAddress (uint8_t dev_addr) {

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  SIE_WrCmdData (USBD_CMD_SET_ADDR, USBD_DAT_WR_BYTE(USBD_DEV_EN | dev_addr));
  SIE_WrCmdData (USBD_CMD_SET_ADDR, USBD_DAT_WR_BYTE(USBD_DEV_EN | dev_addr));

  // Set device configured
  SIE_WrCmdData (USBD_CMD_CFG_DEV,  USBD_DAT_WR_BYTE(USBD_CONF_DVICE));

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_ReadSetupPacket (uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBD_ReadSetupPacket (uint8_t *setup) {

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return ARM_DRIVER_ERROR;      }
  if (setup_received                    == 0U) { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  setup_received = 0U;
  USBD_HW_ReadEP (0, setup);

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  if (setup_received != 0U) {           // If new setup packet was received while this was being read
    return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                               uint8_t  ep_type,
                                               uint16_t ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBD_EndpointConfigure (uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size) {
  ENDPOINT_t *ptr_ep;
  uint32_t    ep_phy;

  ep_phy = EP_PHY(ep_addr);
  ptr_ep = &ep[ep_phy];
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U))             { return ARM_DRIVER_ERROR;      }
  switch (ep_type) {
    case ARM_USB_ENDPOINT_CONTROL:
      if ((USBD_CTRL_ENDPOINT_MASK & (1U << ep_phy)) == 0U) { return ARM_DRIVER_ERROR;      }
      break;
    case ARM_USB_ENDPOINT_ISOCHRONOUS:
      if ((USBD_ISO_ENDPOINT_MASK  & (1U << ep_phy)) == 0U) { return ARM_DRIVER_ERROR;      }
      break;
    case ARM_USB_ENDPOINT_BULK:
      if ((USBD_BULK_ENDPOINT_MASK & (1U << ep_phy)) == 0U) { return ARM_DRIVER_ERROR;      }
      break;
    case ARM_USB_ENDPOINT_INTERRUPT:
      if ((USBD_INT_ENDPOINT_MASK  & (1U << ep_phy)) == 0U) { return ARM_DRIVER_ERROR;      }
      break;
    default:
      return ARM_DRIVER_ERROR;
  }
  if ((usb_state & USBD_DRIVER_POWERED) == 0U)              { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U)              { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  // Clear Endpoint settings in memory
  memset((void *)ptr_ep, 0, sizeof(ENDPOINT_t));

  // Store Endpoint settings in memory
  ptr_ep->type            = ep_type;
  ptr_ep->max_packet_size = ep_max_packet_size;
  if (ep_type == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    // For isochronous endpoints transfers are handled on SOF
    ep_iso_cfg_mask |= (1U << ep_phy);
    if (ep_iso_cfg_mask != 0U) {
      LPC_USB->DevIntEn |= USBD_FRAME_INT;
    }
  }

  USBD_HW_ConfigEP (ep_phy, ep_max_packet_size);
  SIE_WrCmdData (USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(0));
  USBD_HW_EnableEP (ep_phy);

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointUnconfigure (uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointUnconfigure (uint8_t ep_addr) {
  ENDPOINT_t *ptr_ep;
  uint32_t    ep_phy;

  ep_phy = EP_PHY(ep_addr);
  ptr_ep = &ep[ep_phy];
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U)) { return ARM_DRIVER_ERROR;      }
  if ((usb_state & USBD_DRIVER_POWERED) == 0U)  { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U)  { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  USBD_HW_DisableEP (ep_phy);

  if (ptr_ep->type == ARM_USB_ENDPOINT_ISOCHRONOUS) {
    ep_iso_cfg_mask &= ~(1U << ep_phy);
    if (ep_iso_cfg_mask == 0U) {
      LPC_USB->DevIntEn &= ~USBD_FRAME_INT;
    }
  }

  // Clear Endpoint settings in memory
  memset((void *)ptr_ep, 0, sizeof(ENDPOINT_t));

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBD_EndpointStall (uint8_t ep_addr, bool stall) {
  uint32_t ep_phy;

  ep_phy = EP_PHY(ep_addr);
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U)) { return ARM_DRIVER_ERROR;      }
  if ((usb_state & USBD_DRIVER_POWERED) == 0U)  { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U)  { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  if (stall != 0U) {
    SIE_WrCmdData (USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(USBD_EP_STAT_ST));
  } else {
    SIE_WrCmdData (USBD_CMD_SET_EP_STAT(ep_phy), USBD_DAT_WR_BYTE(0));
  }

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransfer (uint8_t ep_addr, uint8_t *data, uint32_t num) {
  ENDPOINT_t *ptr_ep;
  uint32_t    ep_phy, ep_msk;

  ep_phy = EP_PHY(ep_addr);
  ptr_ep = &ep[ep_phy];
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U)) { return ARM_DRIVER_ERROR;      }
  if ((usb_state & USBD_DRIVER_POWERED) == 0U)  { return ARM_DRIVER_ERROR;      }
  if (ptr_ep->active != 0U)                     { return ARM_DRIVER_ERROR_BUSY; }
  if (check_and_set_var(&usbd_sie_busy) == 0U)  { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  ep_msk = 1U << ep_phy;

  ptr_ep->data                  = data;
  ptr_ep->num                   = num;
  ptr_ep->num_transferred_total = 0U;
  ptr_ep->num_transferring      = 0U;

  if ((ep_iso_cfg_mask & ep_msk) != 0U) {
    ep_iso_act_mask |= ep_msk;
  }

  // Ignore OUT ZLP request as it is automatically received by hardware and next SETUP
  // can over-write it before it was activated for transfer
  if ((ep_phy != 0U) || 
     ((ep_phy == 0U) && (num != 0U))) {
    if ((ep_addr & 0x80U) != 0U) {                                        // for IN Endpoint
      ptr_ep->active = 1U;
      ptr_ep->num_transferring = (uint16_t)USBD_HW_WriteEP (ep_phy >> 1, ptr_ep->data, ptr_ep->num);
    } else {                                                              // for OUT Endpoint
      ptr_ep->active = 1U;
      if (ptr_ep->out_irq_pending != 0U) {                                // if something was already received
        ptr_ep->out_irq_pending = 0U;
        LPC_USB->EpIntSet = ep_msk;                                       // force interrupt to handle already received data
      }
    }
  }

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBD_EndpointTransferGetResult (uint8_t ep_addr) {
  uint32_t    ep_phy;

  ep_phy = EP_PHY(ep_addr);
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U)) { return 0U; }

  return (ep[ep_phy].num_transferred_total);
}

/**
  \fn          int32_t USBD_EndpointTransferAbort (uint8_t ep_addr)
  \brief       Abort current USB Endpoint transfer.
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBD_EndpointTransferAbort (uint8_t ep_addr) {
  uint32_t ep_phy;

  ep_phy = EP_PHY(ep_addr);
  if (ep_phy > ((USBD_MAX_ENDPOINT_NUM+1U)*2U)) { return ARM_DRIVER_ERROR;      }
  if ((usb_state & USBD_DRIVER_POWERED) == 0U)  { return ARM_DRIVER_ERROR;      }
  if (check_and_set_var(&usbd_sie_busy) == 0U)  { return ARM_DRIVER_ERROR_BUSY; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  ep[ep_phy].active = 0U;
  if ((ep_iso_act_mask & (1U << ep_phy)) != 0U) {
    if ((ep_addr & 0x80) != 0U) { USBD_HW_WriteEP (ep_phy >> 1, NULL, 0); }
    ep_iso_act_mask &= ~(1U << ep_phy);
  }

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBD_GetFrameNumber (void)
  \brief       Get current USB Frame Number.
  \return      Frame Number
*/
static uint16_t USBD_GetFrameNumber (void) {
  uint16_t val;

  if ((usb_state & USBD_DRIVER_POWERED) == 0U) { return 0U; }
  if (check_and_set_var(&usbd_sie_busy) == 0U) { return 0U; }

  if (usbd_int_active == 0U) { NVIC_DisableIRQ(USB_IRQn); }

  SIE_WrCmd (USBD_CMD_RD_FRAME);
  val  = (uint16_t)SIE_RdCmdData(USBD_DAT_RD_FRAME);
  val |= SIE_RdCmdData(USBD_DAT_RD_FRAME) << 8;

  usbd_sie_busy = 0U;
  if (usbd_int_active == 0U) { NVIC_EnableIRQ(USB_IRQn); }

  return val;
}

/**
  \fn          void USBD_IRQ (void)
  \brief       USB Device Interrupt Routine (IRQ).
*/
void USBD_IRQ (void) {
  ENDPOINT_t *ptr_ep;
  uint32_t    dev_isr, ep_isr, ep_msk, ep_log, ep_phy, val, len;
  uint32_t    evt_ep_in  = 0U;
  uint32_t    evt_ep_out = 0U;
  uint8_t     evt_stp    = 0U;
  uint8_t     cnt        = 0U;

  usbd_int_active  = 1U;

  dev_isr = LPC_USB->DevIntSt & LPC_USB->DevIntEn;      // Device Int Mask Stat

  // Device Status Interrupt (Reset, Connect change, Suspend/Resume)
  if ((dev_isr & USBD_DEV_STAT_INT) != 0U) {
    LPC_USB->DevIntClr = USBD_DEV_STAT_INT;
    SIE_WrCmd (USBD_CMD_GET_DEV_STAT);
    val = SIE_RdCmdData (USBD_DAT_GET_DEV_STAT);        // Get device status
    if ((val & USBD_DEV_CON_CH) != 0U) {                // Connect change
      if ((val & USBD_DEV_CON) != 0U) {                 // Connect
        usbd_state.vbus   = 1U;
        usbd_state.speed  = ARM_USB_SPEED_FULL;
        usbd_state.active = 1U;
        SignalDeviceEvent(ARM_USBD_EVENT_VBUS_ON);
      } else {                                          // Disconnect
        usbd_state.vbus   = 0U;
        usbd_state.speed  = 0U;
        usbd_state.active = 0U;
        SignalDeviceEvent(ARM_USBD_EVENT_VBUS_OFF);
      }
    }
    if ((val & USBD_DEV_SUS_CH) != 0U) {                // Suspend/Resume
      if ((val & USBD_DEV_SUS) != 0U) {                 // Suspend
        usbd_state.active = 0U;
        SignalDeviceEvent(ARM_USBD_EVENT_SUSPEND);
      } else {                                          // Resume
        usbd_state.active = true;
        SignalDeviceEvent(ARM_USBD_EVENT_RESUME);
      }
    }
    if ((val & USBD_DEV_RST) != 0U) {                   // Device Reset
      USBD_HW_Reset();
      SignalDeviceEvent(ARM_USBD_EVENT_RESET);
    }

    goto isr_end;
  }

  if ((dev_isr & USBD_FRAME_INT) != 0U) {               // Start of Frame Interrupt
    LPC_USB->DevIntClr = USBD_FRAME_INT;
    if ((ep_iso_cfg_mask & ep_iso_act_mask) != 0U) {
      for (ep_phy = 0U; ep_phy < ((USBD_MAX_ENDPOINT_NUM + 1U) * 2U); ep_phy++) {
        if ((ep_iso_cfg_mask & ep_iso_act_mask & (1U << ep_phy)) != 0U) {
          ptr_ep = &ep[ep_phy];
          if ((ep_phy & 1U) == 0U) {                    // ISO OUT Endpoint
            len = USBD_HW_ReadEP (ep_phy >> 1, ptr_ep->data + ptr_ep->num_transferred_total);
            ptr_ep->num_transferred_total += len;
            if ((ptr_ep->num_transferred_total == ptr_ep->num) ||       // if all data received or
                (len < ptr_ep->max_packet_size)) {                      // if short packet received
              ptr_ep->active = 0U;
              ep_iso_act_mask  &= ~(1U << ep_phy);
              evt_ep_out |= (1U << (ep_phy >> 1U));     // OUT event
            }
          } else {                                      // ISO IN  Endpoint
            ptr_ep->num_transferred_total += ptr_ep->num_transferring;
            if (ptr_ep->num_transferred_total == ptr_ep->num) {
              ptr_ep->active = 0U;
              ep_iso_act_mask  &= ~(1U << ep_phy);
              evt_ep_in  |= (1U << (ep_phy >> 1U));     // IN event
            } else {
              // Write data to send
              ptr_ep->num_transferring = (uint16_t)USBD_HW_WriteEP (ep_phy >> 1, ptr_ep->data + ptr_ep->num_transferred_total, ptr_ep->num - ptr_ep->num_transferred_total);
            }
          }
        }
      }
    }
  }

  if ((dev_isr & USBD_EP_SLOW_INT) != 0U) {             // Endpoint's Slow Interrupt
    ep_isr = LPC_USB->EpIntSt & LPC_USB->EpIntEn;
    for (ep_phy = 0U; ep_isr; ep_phy++) {
      ep_msk = 1U << ep_phy;
      if ((ep_isr & ep_msk) != 0U) {
        ep_log = ep_phy >> 1;
        ptr_ep = &ep[ep_phy];
        ep_isr &= ~ep_msk;
        LPC_USB->EpIntClr = ep_msk;
        while ((LPC_USB->DevIntSt & USBD_CDFULL_INT) == 0U);
        val = LPC_USB->CmdData;
        if ((ep_phy & 1U) == 0U) {                      // OUT Endpoint
          if (val & USBD_EP_SEL_STP) {
            setup_received      = 1U;
            evt_stp = 1U;                               // SETUP event
          } else if (ptr_ep->active != 0U) {            // if transfer is active
            if (val & USBD_EP_SEL_ST) {
              cnt = 0U;                                 // if endpoint is stalled do not read data
            } else {
              cnt = ((val>>5) & 1U) + ((val >> 6) & 1U);// number of full buffers
            }
            if (cnt != 0U) {
              while (cnt != 0U) {                       // while received data
                len = USBD_HW_ReadEP (ep_log, ptr_ep->data + ptr_ep->num_transferred_total);
                ptr_ep->num_transferred_total += len;
                if ((ptr_ep->num_transferred_total == ptr_ep->num) ||   // if all data received or
                    (len < ptr_ep->max_packet_size)) {                  // if short packet received
                  ptr_ep->active = 0U;
                  evt_ep_out |= (1U << ep_log);         // OUT event
                  break;
                }
                cnt--;
              }
            }
          } else {                                      // if transfer not yet active
            // read OUT ZLP if it was received on Control 0 Endpoint
            if (val & USBD_EP_SEL_ST) {
              cnt = 0U;                                 // if endpoint is stalled do not read data
            } else {
              cnt = ((val>>5) & 1U) + ((val >> 6) & 1U);// number of full buffers
            }
            if (cnt != 0U) {
              ptr_ep->out_irq_pending = 1U;
            }
          }
        } else {                                        // IN Endpoint
          if (ptr_ep->active != 0U) {                   // if transfer is active
            ptr_ep->num_transferred_total += ptr_ep->num_transferring;
            if (ptr_ep->num_transferred_total == ptr_ep->num) {
              ptr_ep->active = 0U;
              evt_ep_in |= (1U << ep_log);              // IN event
            } else {
              // Write data to send
              ptr_ep->num_transferring = (uint16_t)USBD_HW_WriteEP (ep_log, ptr_ep->data + ptr_ep->num_transferred_total, ptr_ep->num - ptr_ep->num_transferred_total);
            }
          }
        }
      }
    }
    LPC_USB->DevIntClr = USBD_EP_SLOW_INT;
  }

  if (evt_ep_in != 0U) {                // If IN event should be signalled
    ep_msk = 1U;
    for (ep_log = 0U; ep_log <= USBD_MAX_ENDPOINT_NUM; ep_log++) {
      if ((evt_ep_in & ep_msk) != 0U) {
        evt_ep_in &= ~ep_msk;
        SignalEndpointEvent((uint8_t)ep_log | 0x80U, ARM_USBD_EVENT_IN);
      }
      if (evt_ep_in == 0U) { break; }
      ep_msk <<= 1U;
    }
  }
  if (evt_ep_out != 0U) {               // If OUT event should be signalled
    ep_msk = 1U;
    for (ep_log = 0U; ep_log <= USBD_MAX_ENDPOINT_NUM; ep_log++) {
      if ((evt_ep_out & ep_msk) != 0U) {
        evt_ep_out &= ~ep_msk;
        SignalEndpointEvent((uint8_t)ep_log, ARM_USBD_EVENT_OUT);
      }
      if (evt_ep_out == 0U) { break; }
      ep_msk <<= 1U;
    }
  }
  if (evt_stp) { SignalEndpointEvent(0U, ARM_USBD_EVENT_SETUP); }

isr_end:
  usbd_int_active = 0U;
}

ARM_DRIVER_USBD Driver_USBD0 = {
  USBD_GetVersion,
  USBD_GetCapabilities,
  USBD_Initialize,
  USBD_Uninitialize,
  USBD_PowerControl,
  USBD_DeviceConnect,
  USBD_DeviceDisconnect,
  USBD_DeviceGetState,
  USBD_DeviceRemoteWakeup,
  USBD_DeviceSetAddress,
  USBD_ReadSetupPacket,
  USBD_EndpointConfigure,
  USBD_EndpointUnconfigure,
  USBD_EndpointStall,
  USBD_EndpointTransfer,
  USBD_EndpointTransferGetResult,
  USBD_EndpointTransferAbort,
  USBD_GetFrameNumber
};
