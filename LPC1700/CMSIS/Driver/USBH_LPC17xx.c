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
 * $Revision:    V2.6
 *
 * Driver:       Driver_USBH0_HCI
 * Configured:   via RTE_Device.h configuration file
 * Project:      USB Host 0 HCI Controller (OHCI) Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                  Value
 *   ---------------------                  -----
 *   Connect to hardware via Driver_USBH# = 0
 *   USB Host controller interface        = OHCI
 * -------------------------------------------------------------------------- */

/* History:
*  Version 2.6
 *    Removed minor compiler warnings
*  Version 2.5
 *    Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 2.4
 *    Fixed PowerControl for Power OFF newer finishing
 *  Version 2.3
 *    Corrected initialization failure if Port1 not enabled in RTE_Device.h
 *  Version 2.2
 *    PowerControl for Power OFF and Uninitialize functions made unconditional
 *  Version 2.1
 *    Added support for U1 and U2 on LPC177x/8x
 *    Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *  Version 2.0
 *    Initial release for USB Host OHCI Driver API v2.0
 *  Version 1.0
 *    Initial release
 */

#include "Driver_USBH.h"

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"

#if      (RTE_USB_USB0 == 0)
#error   "USB is not enabled in the RTE_Device.h!"
#endif

#if     defined (LPC177x_8x)
#if     ((RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1))
#if     ((RTE_USB_OVRCR1_ID == 1) && (RTE_USB_INT1_ID == 1))
#error "Invalid P1_27 Pin Configuration (USB_OVRCR1 and USB_INT1 selected)!"
#endif
#if     ((RTE_USB_PPWR1_ID == 1) && (RTE_USB_TX_E1_ID == 1))
#error "Invalid P1_19 Pin Configuration (USB_PPWR1 and USB_TX_E1 selected)!"
#endif
#if     ((RTE_USB_PWRD1_ID == 1) && (RTE_USB_RCV1_ID == 1))
#error "Invalid P1_22 Pin Configuration (USB_PWRD1 and USB_RCV1 selected)!"
#endif
#endif
#if     ((RTE_USB_PORT1_EN == 2))
#if     ((RTE_USB_PWRD2_ID == 1) && (RTE_USB_VBUS_ID == 1))
#error "Invalid P1_30 Pin Configuration (USB_PWRD2 and USB_VBUS selected)!"
#endif
#endif
#endif

#define USBH_DRIVER_INITIALIZED         (1U << 4)
#define USBH_DRIVER_POWERED             (1U << 5)

extern ARM_DRIVER_USBH_HCI Driver_USBH0_HCI;

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


// USBH OHCI Driver ************************************************************

#define ARM_USBH_OHCI_DRIVER_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,6)

/* Interrupt Handler Prototype */
void USBH_IRQ (void);

// Driver Version
static const ARM_DRIVER_VERSION usbh_ohci_driver_version = { ARM_USBH_API_VERSION, ARM_USBH_OHCI_DRIVER_VERSION };

// Driver Capabilities
static const ARM_USBH_HCI_CAPABILITIES usbh_ohci_driver_capabilities = {
#if defined (LPC175x_6x)
  0x0001U,       // Root HUB available Ports Mask
  0U,
#elif defined (LPC177x_8x)
#if   (RTE_USB_PORT_CFG == 0)
  0x0002U,       // Root HUB available Ports Mask
#elif (RTE_USB_PORT_CFG == 1)
  0x0003U,       // Root HUB available Ports Mask
#elif (RTE_USB_PORT_CFG == 3)
  0x0001U,       // Root HUB available Ports Mask
#else
  0x0000U,       // Root HUB available Ports Mask
#endif
  0U,
#endif
};

static ARM_USBH_HCI_Interrupt_t OHCI_IRQ;

// USBH OHCI Driver functions

/**
  \fn          ARM_DRIVER_VERSION USBH_HCI_GetVersion (void)
  \brief       Get USB Host HCI (OHCI/EHCI) driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBH_HCI_GetVersion (void) { return usbh_ohci_driver_version; }

/**
  \fn          ARM_USBH_HCI_CAPABILITIES USBH_HCI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_USBH_HCI_CAPABILITIES
*/
static ARM_USBH_HCI_CAPABILITIES USBH_HCI_GetCapabilities (void) { return usbh_ohci_driver_capabilities; }

/**
  \fn          int32_t USBH_HCI_Initialize (ARM_USBH_HCI_Interrupt_t *cb_interrupt)
  \brief       Initialize USB Host HCI (OHCI/EHCI) Interface.
  \param[in]   cb_interrupt Pointer to Interrupt Handler Routine
  \return      \ref execution_status
*/
static int32_t USBH_HCI_Initialize (ARM_USBH_HCI_Interrupt_t cb_interrupt) {

  if ((usb_state & USBH_DRIVER_INITIALIZED) != 0U) { return ARM_DRIVER_OK; }

  OHCI_IRQ = cb_interrupt;

  usb_role   =  ARM_USB_ROLE_HOST;
  if (USB_PinsConfigure () == -1) {
    usb_role = ARM_USB_ROLE_NONE;
    return ARM_DRIVER_ERROR;
  }
  usb_state |=  USBH_DRIVER_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBH_HCI_Uninitialize (void)
  \brief       De-initialize USB Host HCI (OHCI/EHCI) Interface.
  \return      \ref execution_status
*/
static int32_t USBH_HCI_Uninitialize (void) {

  USB_PinsUnconfigure ();
  usb_role   =  ARM_USB_ROLE_NONE;
  usb_state  = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBH_HCI_PowerControl (ARM_POWER_STATE state)
  \brief       Control USB Host HCI (OHCI/EHCI) Interface Power.
  \param[in]   state Power state
  \return      \ref execution_status
*/
static int32_t USBH_HCI_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ      (USB_IRQn);                  // Disable interrupt
      NVIC_ClearPendingIRQ (USB_IRQn);                  // Clear pending interrupt
      usb_state &= ~USBH_DRIVER_POWERED;                // Clear powered flag

      LPC_USB->OTGClkCtrl &= ~0x01U;                    // Disable Host clock
      if ((usb_state & 0x0FU) == 0U) {                  // If Device is not enabled
        LPC_USB->OTGClkCtrl &= ~0x0AU;                  // Disable OTG and Device clocks
        LPC_USB->OTGClkCtrl &= ~0x10U;                  // Disable AHB clocks
        LPC_SC->PCONP &= ~(1UL << 31);                  // USB PCLK -> disable USB Peripheral
      } else {                                          // If Device is enabled
        NVIC_EnableIRQ (USB_IRQn);                      // Enable interrupt
      }
      break;

    case ARM_POWER_FULL:
      if ((usb_state & USBH_DRIVER_INITIALIZED) == 0U) { return ARM_DRIVER_ERROR; }
      if ((usb_state & USBH_DRIVER_POWERED)     != 0U) { return ARM_DRIVER_OK; }

      LPC_SC->PCONP |=  (1UL << 31);                    // USB PCLK -> enable USB Peripheral
      LPC_USB->OTGClkCtrl |= 0x19U;                     // Enable AHB, OTG and Host clocks
      while ((LPC_USB->OTGClkSt & 0x19) != 0x19U);
      LPC_USB->USBClkCtrl |= 0x10U;                     // Enable AHB clock
      while ((LPC_USB->USBClkSt & 0x10) != 0x10U);
#if   defined (LPC175x_6x)
      LPC_USB->OTGStCtrl  |= 0x01U;                     // Set port function
#elif defined (LPC177x_8x)
      LPC_USB->USBClkCtrl |= 0x08U;                     // Enable port select register clocks
      while ((LPC_USB->USBClkSt & 0x08U) == 0U);
      LPC_USB->StCtrl &= ~0x03U;                        // Reset port function
      if (LPC_USB->StCtrl != RTE_USB_PORT_CFG) {
        LPC_USB->StCtrl |=  RTE_USB_PORT_CFG;           // Select port function
      }
#if ((RTE_USB_PORT1_EN == 1) && (RTE_USB_PORT1_OTG_EN == 1) && (RTE_USB_PORT_CFG != 0))
      USB_I2C_Initialize ();                            // Initialize I2C for OTG Transceiver
      USB_I2C_ControlPullDowns (true);
#endif
#endif

      usb_state |=  USBH_DRIVER_POWERED;                // Set powered flag
      NVIC_EnableIRQ   (USB_IRQn);                      // Enable interrupt
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBH_HCI_PortVbusOnOff (uint8_t port, bool vbus)
  \brief       USB Host HCI (OHCI/EHCI) Root HUB Port VBUS on/off.
  \param[in]   port  Root HUB Port Number
  \param[in]   vbus
                - \b false VBUS off
                - \b true  VBUS on
  \return      \ref execution_status
*/
static int32_t USBH_HCI_PortVbusOnOff (uint8_t port, bool power) {
  (void)power;
  // No GPIO pins used for VBUS control it is controlled by OHCI Controller

  if (((1U << port) & usbh_ohci_driver_capabilities.port_mask) == 0U) { return ARM_DRIVER_ERROR; }
  return ARM_DRIVER_OK;
}

/**
  \fn          void USBH_IRQ (void)
  \brief       USB0 Host Interrupt Routine (IRQ).
*/
void USBH_IRQ (void) {
  OHCI_IRQ();
}

ARM_DRIVER_USBH_HCI Driver_USBH0_HCI = {
  USBH_HCI_GetVersion,
  USBH_HCI_GetCapabilities,
  USBH_HCI_Initialize,
  USBH_HCI_Uninitialize,
  USBH_HCI_PowerControl,
  USBH_HCI_PortVbusOnOff
};
