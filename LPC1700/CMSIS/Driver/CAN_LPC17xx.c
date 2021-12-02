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
 * $Revision:    V1.6
 *
 * Driver:       Driver_CAN1/2
 * Configured:   via RTE_Device.h configuration file
 * Project:      CAN Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   CAN Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via Driver_CAN# = 1       use CAN1
 *   Connect to hardware via Driver_CAN# = 2       use CAN2
 * --------------------------------------------------------------------------
 * Defines used for driver configuration (at compile time):
 *
 *   CAN_CLOCK_TOLERANCE:  defines maximum allowed clock tolerance in 1/1024 steps
 *     - default value:    15 (approx. 1.5 %)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.6
 *    Removed minor compiler warnings
 *    Updated CAN Capabilities structures
 *  Version 1.5
 *    Updated  CANx_PowerControl function
 *  Version 1.4
 *    Corrected SetBitrate function
 *  Version 1.3
 *    Corrected receive overrun signaling
 *  Version 1.2
 *    Corrected functionality when NULL pointer is provided for one or both 
 *    signal callbacks in Initialize function call
 *  Version 1.1
 *    Minor improvements and typo corrections
 *  Version 1.0
 *    Initial release
 */

#include "CAN_LPC17xx.h"

// Externally overridable configuration definitions

// Maximum allowed clock tolerance in 1/1024 steps
#ifndef CAN_CLOCK_TOLERANCE
#define CAN_CLOCK_TOLERANCE             (15U)   // 15/1024 approx. 1.5 %
#endif

// Interrupt Handler Prototypes
void CAN_IRQHandler (void);

extern ARM_DRIVER_CAN Driver_CAN1;
extern ARM_DRIVER_CAN Driver_CAN2;

// CAN Driver ******************************************************************

#define ARM_CAN_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,6) // CAN driver version

// Driver Version
static const ARM_DRIVER_VERSION can_driver_version = { ARM_CAN_API_VERSION, ARM_CAN_DRV_VERSION };

// Driver Capabilities
static const ARM_CAN_CAPABILITIES can_driver_capabilities = {
  2U,                   // Number of CAN Objects available
  1U,                   // Supports reentrant calls to ARM_CAN_MessageSend, ARM_CAN_MessageRead, ARM_CAN_ObjectConfigure and abort message sending used by ARM_CAN_Control.
  0U,                   // Does not support CAN with flexible data-rate mode (CAN_FD)
  0U,                   // Does not support restricted operation mode
  1U,                   // Supports bus monitoring mode
  1U,                   // Supports internal loopback mode
  1U,                   // Supports external loopback mode
  0U                    // Reserved
};

// Object Capabilities
static const ARM_CAN_OBJ_CAPABILITIES can_object_capabilities_rx = {
  0U,                   // Object does not support transmission
  1U,                   // Object supports reception
  0U,                   // Object does not support RTR reception and automatic Data transmission
  0U,                   // Object does not support RTR transmission and automatic Data reception
  1U,                   // Object allows assignment of multiple filters to it
  1U,                   // Object supports exact identifier filtering
  1U,                   // Object supports range identifier filtering
  0U,                   // Object does not support mask identifier filtering
  2U,                   // Object can buffer 2 messages
  0U                    // Reserved
};
static const ARM_CAN_OBJ_CAPABILITIES can_object_capabilities_tx = {
  1U,                   // Object supports transmission
  0U,                   // Object does not support reception
  0U,                   // Object does not support RTR reception and automatic Data transmission
  0U,                   // Object does not support RTR transmission and automatic Data reception
  0U,                   // Object does not allow assignment of multiple filters to it
  0U,                   // Object does not support exact identifier filtering
  0U,                   // Object does not support range identifier filtering
  0U,                   // Object does not support mask identifier filtering
  3U,                   // Object can buffer 3 message
  0U                    // Reserved
};


typedef enum _CAN_FILTER_TYPE {
  CAN_FILTER_TYPE_EXACT_ID = 0U,
  CAN_FILTER_TYPE_RANGE_ID = 1U
} CAN_FILTER_TYPE;

static LPC_CAN_TypeDef * const ptr_CANx[2] = { LPC_CAN1, LPC_CAN2 };

// Local variables and structures
static uint8_t                     can_driver_powered    [CAN_CTRL_NUM];
static uint8_t                     can_driver_initialized[CAN_CTRL_NUM];
static uint8_t                     can_obj_tx_alloc      [CAN_CTRL_NUM][3];
static uint8_t                     can_obj_tx_alloc_lock [CAN_CTRL_NUM];
static uint8_t                     can_no_retransmission [CAN_CTRL_NUM];
static uint8_t                     can_loopback          [CAN_CTRL_NUM];
static uint8_t                     can_obj_cfg_msk       [CAN_CTRL_NUM];
static uint8_t                     can_unit_state        [CAN_CTRL_NUM];
static uint8_t                     can_last_error_code   [CAN_CTRL_NUM];
static ARM_CAN_SignalUnitEvent_t   CAN_SignalUnitEvent   [CAN_CTRL_NUM];
static ARM_CAN_SignalObjectEvent_t CAN_SignalObjectEvent [CAN_CTRL_NUM];


// Helper Functions

/**
  \fn          void CANx_ResetRuntimeInfo (uint8_t x)
  \brief       Reset runtime information.
*/
static void CANx_ResetRuntimeInfo (uint8_t x) {

  if (x >= CAN_CTRL_NUM) { return; }

  can_obj_tx_alloc      [x][0] =  0U;
  can_obj_tx_alloc      [x][1] =  0U;
  can_obj_tx_alloc      [x][2] =  0U;
  can_obj_tx_alloc_lock [x]    =  0U;
  can_no_retransmission [x]    =  0U;
  can_loopback          [x]    =  0U;
  can_obj_cfg_msk       [x]    =  0U;
  can_unit_state        [x]    =  0x10U;        // Initialize state to unexisting to force initial event
  can_last_error_code   [x]    =  0U;
}

/**
  \fn          int32_t CANx_AddFilter (CAN_FILTER_TYPE filter_type, uint32_t id, uint32_t id_range_end, uint8_t x)
  \brief       Add receive filter for specified id or id range.
  \param[in]   filter_type  Type of filter to add
                 - CAN_FILTER_TYPE_EXACT_ID: exact id filter (id only)
                 - CAN_FILTER_TYPE_RANGE_ID: range id filter (id of range start and id of range end)
  \param[in]   id           Exact identifier or identifier of range start
  \param[in]   id_range_end Identifier of range end
  \param[in]   x            Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_AddFilter (CAN_FILTER_TYPE filter_type, uint32_t id, uint32_t id_range_end, uint8_t x) {
  uint32_t  i, i_end, j, j_end;
  uint32_t  entry;

  if (x >= CAN_CTRL_NUM) { return ARM_DRIVER_ERROR; }

  if ((id & ARM_CAN_ID_IDE_Msk) == 0U) {                        // Standard Identifier (11 bit)
    switch (filter_type) {
      case CAN_FILTER_TYPE_EXACT_ID:                            // Exact
        // Entry in this section is 16 bits large
        if (((LPC_CANAF->ENDofTable & CANAF_ENDofTable_ENDofTable_Msk) >= 0x800U) && 
            ((LPC_CANAF->SFF_GRP_sa == 0U) || ((LPC_CANAF->SFF_GRP_sa != 0U) && ((LPC_CANAF_RAM->mask[(LPC_CANAF->SFF_GRP_sa/4U)-1U] & (1U << 12)) == 0U)))) {
          // If table is full and no disabled entries are available
          return ARM_DRIVER_ERROR;
        }
        if ((LPC_CANAF->SFF_GRP_sa == 0U) || ((LPC_CANAF_RAM->mask[(LPC_CANAF->SFF_GRP_sa/4U)-1U] & (1U << 12)) == 0U)) {
          // If we should enlarge this section (contains no entries yet or last entry is not disabled)

          // Increment affected pointers
          LPC_CANAF->ENDofTable += 4U;
          LPC_CANAF->EFF_GRP_sa += 4U;
          LPC_CANAF->EFF_sa     += 4U;
          LPC_CANAF->SFF_GRP_sa += 4U;

          // Move all existing entries 32 bits towards end of table
          i     =  LPC_CANAF->ENDofTable / 4U;
          i_end = (LPC_CANAF->SFF_GRP_sa / 4U) - 1U;
          for (; i > i_end; i--) {
            LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i-1U];
          }

          // Disable 2 newly added entries in this section
          LPC_CANAF_RAM->mask[i] = (1U << 29) | (1U << 28) | (0x7FFU << 16) | (1U << 13) | (1U << 12) | (0x7FFU);
        }

        // Add new entry sorted into this section
        entry = ((uint32_t)x << 13) | (id & 0x7FFU);
        i     =  LPC_CANAF->SFF_sa     / 4U;
        i_end = (LPC_CANAF->SFF_GRP_sa / 4U) - 1U;
        for (; i <= i_end; i++) {
          if ((LPC_CANAF_RAM->mask[i] >> 16) > entry) {         // If this is place to insert new entry (high 16 bits)
            // Move all remaining entries in this section 16 bits towards end of table
            j     = i_end;
            j_end = i;
            for (; j > j_end; j--) {
              LPC_CANAF_RAM->mask[j] = (LPC_CANAF_RAM->mask[j] >> 16) | (LPC_CANAF_RAM->mask[j-1U] << 16);
            }
            // Move entry to be replaced to low 16 bits and replace it with new entry in high 16 bits
            LPC_CANAF_RAM->mask[i] = (LPC_CANAF_RAM->mask[i] >> 16) | (entry << 16);
            break;
          } else if ((LPC_CANAF_RAM->mask[i] & 0xFFFFU) > entry) {    // If this is place to insert new entry (in low 16 bits)
            // Move all remaining entries in this section 16 bits towards end of table
            j     = i_end;
            j_end = i;
            for (; j > j_end; j--) {
              LPC_CANAF_RAM->mask[j] = (LPC_CANAF_RAM->mask[j] >> 16) | (LPC_CANAF_RAM->mask[j-1U] << 16);
            }
            // Insert new entry into low 16 bits and keep high 16 bits
            LPC_CANAF_RAM->mask[i] &= 0xFFFF0000U;
            LPC_CANAF_RAM->mask[i] |= entry;
            break;
          }
        }
        break;
      case CAN_FILTER_TYPE_RANGE_ID:                            // Range
        // Entry in this section is 32 bits large
        if ((LPC_CANAF->ENDofTable & CANAF_ENDofTable_ENDofTable_Msk) >= 0x800U) {
          // If table is full
          return ARM_DRIVER_ERROR;
        }
        // Enlarge this section

        // Increment affected pointers
        LPC_CANAF->ENDofTable += 4U;
        LPC_CANAF->EFF_GRP_sa += 4U;
        LPC_CANAF->EFF_sa     += 4U;

        // Move all existing entries 32 bits towards end of table
        i     =  LPC_CANAF->ENDofTable / 4U;
        i_end = (LPC_CANAF->EFF_sa     / 4U) - 1U;
        for (; i > i_end; i--) {
          LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i-1U];
        }
        // Disable newly added entry in this section
        LPC_CANAF_RAM->mask[i] = (1U << 29) | (1U << 28) | (0x7FFU << 16) | (1U << 13) | (1U << 12) | (0x7FFU);

        // Add new entry sorted into this section
        entry = ((uint32_t)x << 29) | ((id & 0x7FFU) << 16) | ((uint32_t)x << 13) | (id_range_end & 0x7FFU);
        i     =  LPC_CANAF->SFF_GRP_sa / 4U;
        i_end = (LPC_CANAF->EFF_sa     / 4U) - 1U;
        for (; i <= i_end; i++) {
          if (LPC_CANAF_RAM->mask[i] > entry) {                 // If this is place to insert new entry
            // Move all remaining entries 32 bits towards end of table
            j     = i_end;
            j_end = i;
            for (; j > j_end; j--) {
              LPC_CANAF_RAM->mask[j] = LPC_CANAF_RAM->mask[j-1U];
            }
            // Insert new entry
            LPC_CANAF_RAM->mask[i] = entry;
            break;
          }
        }
        break;
    }
  } else {                                                      // Extended Identifier (29 bit)
    switch (filter_type) {
      case CAN_FILTER_TYPE_EXACT_ID:                            // Exact
        // Entry in this section is 32 bits large
        if ((LPC_CANAF->ENDofTable & CANAF_ENDofTable_ENDofTable_Msk) >= 0x800U) {
          // If table is full
          return ARM_DRIVER_ERROR;
        }
        // Enlarge this section

        // Increment affected pointers
        LPC_CANAF->ENDofTable += 4U;
        LPC_CANAF->EFF_GRP_sa += 4U;

        // Move all entries 32 bits towards end of table
        i     =  LPC_CANAF->ENDofTable / 4U;
        i_end = (LPC_CANAF->EFF_GRP_sa / 4U - 1);
        for (; i > i_end; i--) {
          LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i-1U];
        }
        // Disable newly added entry in this section
        LPC_CANAF_RAM->mask[i] = (7U << 29) | (0x1FFFFFFFU);

        // Add new entry sorted into this section
        entry = ((uint32_t)x << 29) | (id & 0x1FFFFFFFU);
        i     =  LPC_CANAF->EFF_sa     / 4U;
        i_end = (LPC_CANAF->EFF_GRP_sa / 4U) - 1U;
        for (; i <= i_end; i++) {
          if (LPC_CANAF_RAM->mask[i] > entry) {                 // If this is place to insert new entry
            // Move all remaining entries in this section 32 bits towards end of table
            j     = i_end;
            j_end = i;
            for (; j > j_end; j--) {
              LPC_CANAF_RAM->mask[j] = LPC_CANAF_RAM->mask[j-1U];
            }
            // Insert new entry
            LPC_CANAF_RAM->mask[i] = entry;
            break;
          }
        }
        break;
      case CAN_FILTER_TYPE_RANGE_ID:                            // Range
        // Entry in this section is 64 bits large
        if ((LPC_CANAF->ENDofTable & CANAF_ENDofTable_ENDofTable_Msk) >= 0x800U) {
          // If table is full
          return ARM_DRIVER_ERROR;
        }
        // Enlarge this section

        // Increment end of table pointer
        LPC_CANAF->ENDofTable += 8U;

        // Disable newly added entry in this section
        LPC_CANAF_RAM->mask[(LPC_CANAF->ENDofTable/4U)-2U] = (7U << 29) | (0x1FFFFFFFU);
        LPC_CANAF_RAM->mask[(LPC_CANAF->ENDofTable/4U)-1U] = (7U << 29) | (0x1FFFFFFFU);

        // Add new entry sorted into this section
        entry = ((uint32_t)x << 29) | (id & 0x1FFFFFFFU);
        i     =  LPC_CANAF->EFF_GRP_sa / 4U;
        i_end = (LPC_CANAF->ENDofTable / 4U) - 2U;
        for (; i <= i_end; i += 2U) {
          if (LPC_CANAF_RAM->mask[i] > entry) {             // If this is place to insert new entry
            // Move all remaining entries in this section 64 bits towards end of table
            j     = i_end;
            j_end = i;
            for (; j > j_end; j -= 2U) {
              LPC_CANAF_RAM->mask[j   ] = LPC_CANAF_RAM->mask[j-2U];
              LPC_CANAF_RAM->mask[j+1U] = LPC_CANAF_RAM->mask[j-1U];
            }
            // Insert new entry
            LPC_CANAF_RAM->mask[i   ] = entry;
            LPC_CANAF_RAM->mask[i+1U] = ((uint32_t)x << 29) | (id_range_end & 0x1FFFFFFFU);
            break;
          }
        }
        break;
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t CANx_RemoveFilter (CAN_FILTER_TYPE filter_type, uint32_t id, uint32_t id_range_end, uint8_t x)
  \brief       Remove receive filter for specified id or id range.
  \param[in]   filter_type  Type of filter to remove
                 - CAN_FILTER_TYPE_EXACT_ID: exact id filter (id only)
                 - CAN_FILTER_TYPE_RANGE_ID: range id filter (id of range start and id of range end)
  \param[in]   id           Exact identifier or identifier of range start
  \param[in]   id_range_end Identifier of range end
  \param[in]   x            Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_RemoveFilter (CAN_FILTER_TYPE filter_type, uint32_t id, uint32_t id_range_end, uint8_t x) {
  uint32_t  i, i_end;
  uint32_t  entry;

  if (x >= CAN_CTRL_NUM) { return ARM_DRIVER_ERROR; }

  if ((id & ARM_CAN_ID_IDE_Msk) == 0U) {                        // Standard Identifier (11 bit)
    switch (filter_type) {
      case CAN_FILTER_TYPE_EXACT_ID:                            // Exact
        // Entry in this section is 16 bits large
        if (LPC_CANAF->SFF_sa >= LPC_CANAF->SFF_GRP_sa) {
          // If this section does not contain any entries
          return ARM_DRIVER_ERROR;
        }

        // Find entry to be removed
        entry = ((uint32_t)x << 13) | (id & 0x7FFU);
        i     =  LPC_CANAF->SFF_sa     / 4U;
        i_end = (LPC_CANAF->SFF_GRP_sa / 4U) - 1U;
        for (; i <= i_end; i++) {
          if ((LPC_CANAF_RAM->mask[i] >> 16) == entry) {        // If this is entry to be removed (in high 16 bits)
            // Move all remaining entries in this section 16 bits towards start of table
            for (; i < i_end; i++) {
              LPC_CANAF_RAM->mask[i] = (LPC_CANAF_RAM->mask[i] << 16) | (LPC_CANAF_RAM->mask[i+1U] >> 16);
            }
            // Insert disabled entry into lower 16 bits of last entry
            LPC_CANAF_RAM->mask[i] = (LPC_CANAF_RAM->mask[i] << 16) | (1U << 13) | (1U << 12) | (0x7FFU);
            break;
          } else if ((LPC_CANAF_RAM->mask[i] & 0xFFFFU) == entry) {   // If this is entry to be removed (in low 16 bits)
            // Move all remaining entries in this section 16 bits towards start of table
            if (i == i_end) {                                   // Corner case when requested entry is in lower 16 bits of last entry
              LPC_CANAF_RAM->mask[i]= (LPC_CANAF_RAM->mask[i] & 0xFFFF0000U) | (1U << 13) | (1U << 12) | (0x7FFU);
            } else {
              LPC_CANAF_RAM->mask[i]= (LPC_CANAF_RAM->mask[i] & 0xFFFF0000U) | (LPC_CANAF_RAM->mask[i+1U] >> 16);
              i++;
              for (; i < i_end; i++) {
                LPC_CANAF_RAM->mask[i] = (LPC_CANAF_RAM->mask[i] << 16) | (LPC_CANAF_RAM->mask[i+1U] >> 16);
              }
              // Disable lower 16 bits entry in last location in this section
              LPC_CANAF_RAM->mask[i] = (LPC_CANAF_RAM->mask[i] << 16) | (1U << 13) | (1U << 12) | (0x7FFU);
            }
            break;
          }
        }

        if (LPC_CANAF_RAM->mask[i] == ((1U << 29) | (1U << 28) | (0x7FFU << 16) | (1U << 13) | (1U << 12) | (0x7FFU))) {
          // If we should reduce this section (last 2 entries in 32 bits are both disabled)

          // Move all remaining entries in remaining sections 32 bits towards start of table
          i     = (LPC_CANAF->SFF_GRP_sa / 4U) - 1U;
          i_end =  LPC_CANAF->ENDofTable / 4U;
          for (; i < i_end; i++) {
            LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i+1U];
          }

          // Clear last entry in table
          LPC_CANAF_RAM->mask[i+1U] = 0U;

          // Decrement affected pointers
          LPC_CANAF->ENDofTable -= 4U;
          LPC_CANAF->EFF_GRP_sa -= 4U;
          LPC_CANAF->EFF_sa     -= 4U;
          LPC_CANAF->SFF_GRP_sa -= 4U;
        }
        break;
      case CAN_FILTER_TYPE_RANGE_ID:                            // Range
        // Entry in this section is 32 bits large
        if (LPC_CANAF->SFF_GRP_sa >= LPC_CANAF->EFF_sa) {
          // If this section does not contain any entries
          return ARM_DRIVER_ERROR;
        }

        // Find entry to be removed
        entry = ((uint32_t)x << 29) | ((id & 0x7FFU) << 16) | ((uint32_t)x << 13) | (id_range_end & 0x7FFU);
        i     =  LPC_CANAF->SFF_GRP_sa / 4U;
        i_end = (LPC_CANAF->EFF_sa     / 4U) - 1U;
        for (; i <= i_end; i++) {
          if (LPC_CANAF_RAM->mask[i] == entry) {                // If this is entry to be removed
            // Move all remaining entries in this section 32 bits towards start of table
            for (; i < i_end; i++) {
              LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i+1U];
            }
            // Move all remaining entries in remaining sections 32 bits towards start of table
            i     = (LPC_CANAF->EFF_sa     / 4U) - 1U;
            i_end =  LPC_CANAF->ENDofTable / 4U;
            for (; i < i_end; i++) {
              LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i+1U];
            }

            // Clear last entry in table
            LPC_CANAF_RAM->mask[i+1U] = 0U;

            // Decrement affected pointers
            LPC_CANAF->ENDofTable -= 4U;
            LPC_CANAF->EFF_GRP_sa -= 4U;
            LPC_CANAF->EFF_sa     -= 4U;
            break;
          }
        }
        break;
    }
  } else {                                                      // Extended Identifier (29 bit)
    switch (filter_type) {
      case CAN_FILTER_TYPE_EXACT_ID:                            // Exact
        // Entry in this section is 32 bits large
        if (LPC_CANAF->EFF_sa >= LPC_CANAF->EFF_GRP_sa) {
          // If this section does not contain any entries
          return ARM_DRIVER_ERROR;
        }

        // Find entry to be removed
        entry = ((uint32_t)x << 29) | (id & 0x1FFFFFFFU);
        i     =  LPC_CANAF->EFF_sa     / 4U;
        i_end = (LPC_CANAF->EFF_GRP_sa / 4U) - 1U;
        for (; i <= i_end; i++) {
          if (LPC_CANAF_RAM->mask[i] == entry) {                // If this is entry to be removed
            // Move all remaining entries in this section 32 bits towards start of table
            for (; i < i_end; i++) {
              LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i+1U];
            }
            // Move all remaining entries in remaining sections 32 bit towards start of table
            i     = (LPC_CANAF->EFF_GRP_sa / 4U) - 1U;
            i_end =  LPC_CANAF->ENDofTable / 4U;
            for (; i < i_end; i++) {
              LPC_CANAF_RAM->mask[i] = LPC_CANAF_RAM->mask[i+1U];
            }

            // Clear last entry in table
            LPC_CANAF_RAM->mask[i+1U] = 0U;

            // Decrement affected pointers
            LPC_CANAF->ENDofTable -= 4U;
            LPC_CANAF->EFF_GRP_sa -= 4U;
            break;
          }
        }
        break;
      case CAN_FILTER_TYPE_RANGE_ID:                            // Range
        // Entry in this section is 64 bits large
        if (LPC_CANAF->EFF_GRP_sa >= LPC_CANAF->ENDofTable) {
          // If this section does not contain any entries
          return ARM_DRIVER_ERROR;
        }

        // Find entry to be removed
        entry = ((uint32_t)x << 29) | (id & 0x1FFFFFFFU);
        i     =  LPC_CANAF->EFF_GRP_sa / 4U;
        i_end = (LPC_CANAF->ENDofTable / 4U) - 2U;
        for (; i <= i_end; i += 2U) {
          if ((LPC_CANAF_RAM->mask[i] == entry) &&              // If this is entry to be removed
              (LPC_CANAF_RAM->mask[i+1U] == (((uint32_t)x << 29) | (id_range_end & 0x1FFFFFFFU)))) {
            // Move all remaining entries in this section 64 bits towards start of table
            for (; i < i_end; i += 2U) {
              LPC_CANAF_RAM->mask[i   ] = LPC_CANAF_RAM->mask[i+2U];
              LPC_CANAF_RAM->mask[i+1U] = LPC_CANAF_RAM->mask[i+3U];
            }

            // Clear last entry in table
            LPC_CANAF_RAM->mask[i_end   ] = 0U;
            LPC_CANAF_RAM->mask[i_end+1U] = 0U;

            // Decrement end of table table pointer
            LPC_CANAF->ENDofTable -= 8U;
            break;
          }
        }
        break;
    }
  }

  return ARM_DRIVER_OK;
}


// CAN Driver Functions

/**
  \fn          ARM_DRIVER_VERSION CAN_GetVersion (void)
  \brief       Get driver version.
  \return      ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION CAN_GetVersion (void) { return can_driver_version; }

/**
  \fn          ARM_CAN_CAPABILITIES CAN_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      ARM_CAN_CAPABILITIES
*/
static ARM_CAN_CAPABILITIES CAN_GetCapabilities (void) { return can_driver_capabilities; }

/**
  \fn          int32_t CANx_Initialize (ARM_CAN_SignalUnitEvent_t   cb_unit_event,
                                        ARM_CAN_SignalObjectEvent_t cb_object_event,
                                        uint8_t                     x)
  \brief       Initialize CAN interface and register signal (callback) functions.
  \param[in]   cb_unit_event   Pointer to ARM_CAN_SignalUnitEvent callback function
  \param[in]   cb_object_event Pointer to ARM_CAN_SignalObjectEvent callback function
  \param[in]   x               Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_Initialize (ARM_CAN_SignalUnitEvent_t   cb_unit_event,
                                ARM_CAN_SignalObjectEvent_t cb_object_event,
                                uint8_t                     x) {

  if (x >= CAN_CTRL_NUM)               { return ARM_DRIVER_ERROR; }
  if (can_driver_initialized[x] != 0U) { return ARM_DRIVER_OK;    }

  CAN_SignalUnitEvent  [x] = cb_unit_event;
  CAN_SignalObjectEvent[x] = cb_object_event;

  if (x == 0U) {
#if (RTE_CAN1_RD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN1_RD_PORT, RTE_CAN1_RD_BIT, RTE_CAN1_RD_FUNC, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN1_RD_PORT, RTE_CAN1_RD_BIT, RTE_CAN1_RD_FUNC | IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
#endif
#endif
#if (RTE_CAN1_TD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN1_TD_PORT, RTE_CAN1_TD_BIT, RTE_CAN1_TD_FUNC, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN1_TD_PORT, RTE_CAN1_TD_BIT, RTE_CAN1_TD_FUNC | IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
#endif
#endif
  } else {
#if (RTE_CAN2_RD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN2_RD_PORT, RTE_CAN2_RD_BIT, RTE_CAN2_RD_FUNC, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN2_RD_PORT, RTE_CAN2_RD_BIT, RTE_CAN2_RD_FUNC | IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
#endif
#endif
#if (RTE_CAN2_TD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN2_TD_PORT, RTE_CAN2_TD_BIT, RTE_CAN2_TD_FUNC, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN2_TD_PORT, RTE_CAN2_TD_BIT, RTE_CAN2_TD_FUNC | IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
#endif
#endif
  }

  can_driver_initialized[x] = 1U;

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_Initialize (ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event) { return CANx_Initialize (cb_unit_event, cb_object_event, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_Initialize (ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event) { return CANx_Initialize (cb_unit_event, cb_object_event, 1U); }
#endif

/**
  \fn          int32_t CANx_Uninitialize (uint8_t x)
  \brief       De-initialize CAN interface.
  \param[in]   x      Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_Uninitialize (uint8_t x) {

  if (x >= CAN_CTRL_NUM) { return ARM_DRIVER_ERROR; }

  if (x == 0U) {
#if (RTE_CAN1_RD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN1_RD_PORT, RTE_CAN1_RD_BIT, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN1_RD_PORT, RTE_CAN1_RD_BIT, IOCON_MODE_PULLUP | IOCON_HYS_ENABLE);
#endif
#endif
#if (RTE_CAN1_TD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN1_TD_PORT, RTE_CAN1_TD_BIT, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN1_TD_PORT, RTE_CAN1_TD_BIT, IOCON_MODE_PULLUP | IOCON_HYS_ENABLE);
#endif
#endif
  } else {
#if (RTE_CAN2_RD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN2_RD_PORT, RTE_CAN2_RD_BIT, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN2_RD_PORT, RTE_CAN2_RD_BIT, IOCON_MODE_PULLUP | IOCON_HYS_ENABLE);
#endif
#endif
#if (RTE_CAN2_TD_PIN_EN == 1)
#if defined (LPC175x_6x)
    PIN_Configure (RTE_CAN2_TD_PORT, RTE_CAN2_TD_BIT, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (RTE_CAN2_TD_PORT, RTE_CAN2_TD_BIT, IOCON_MODE_PULLUP | IOCON_HYS_ENABLE);
#endif
#endif
  }

  can_driver_initialized[x] = 0U;

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_Uninitialize (void) { return CANx_Uninitialize (0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_Uninitialize (void) { return CANx_Uninitialize (1U); }
#endif

/**
  \fn          int32_t CANx_PowerControl (ARM_POWER_STATE state, uint8_t x)
  \brief       Control CAN interface power.
  \param[in]   state  Power state
                 - ARM_POWER_OFF :  power off: no operation possible
                 - ARM_POWER_LOW :  low power mode: retain state, detect and signal wake-up events
                 - ARM_POWER_FULL : power on: full operation at maximum performance
  \param[in]   x      Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_PowerControl (ARM_POWER_STATE state, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;

  if (x >= CAN_CTRL_NUM) { return ARM_DRIVER_ERROR; }
  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  ptr_CAN = ptr_CANx[x];

  switch (state) {
    case ARM_POWER_OFF:
      can_driver_powered[x] = 0U;
      NVIC_DisableIRQ (CAN_IRQn);

      LPC_SC->PCONP     |=  1U << (x + 13U);    // Enable power to CANx block
      ptr_CAN->IER       =  0U;                 // Disable interrupts
      ptr_CAN->MOD       =  CAN_MOD_RM;         // Enter reset mode

      CANx_ResetRuntimeInfo (x);                // Reset runtime information

      LPC_SC->PCONP     &=~(1U << (x + 13U));   // Disable power to CANx block
      break;

    case ARM_POWER_FULL:
      if (can_driver_initialized[x] == 0U) { return ARM_DRIVER_ERROR; }
      if (can_driver_powered[x]     != 0U) { return ARM_DRIVER_OK;    }

      LPC_SC->PCONP     |=  1U << (x + 13U);    // Enable power to CANx block

      LPC_CANAF->AFMR    =  CANAF_AFMR_AccOff;  // Disable filter
      ptr_CAN->MOD       =  CAN_MOD_RM;         // Enter reset mode

      CANx_ResetRuntimeInfo (x);                // Reset runtime information

      ptr_CAN->IER       =  CAN_IER_RIE  |      // Enable interrupts
                            CAN_IER_TIE1 |
                            CAN_IER_EIE  |
                            CAN_IER_DOIE |
                            CAN_IER_EPIE |
                            CAN_IER_ALIE |
                            CAN_IER_BEIE |
                            CAN_IER_TIE2 |
                            CAN_IER_TIE3 ;

      can_driver_powered[x] = 1U;

      NVIC_ClearPendingIRQ (CAN_IRQn);
      NVIC_EnableIRQ       (CAN_IRQn);
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_PowerControl (ARM_POWER_STATE state) { return CANx_PowerControl (state, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_PowerControl (ARM_POWER_STATE state) { return CANx_PowerControl (state, 1U); }
#endif

/**
  \fn          uint32_t CANx_GetClock (uint8_t x)
  \brief       Retrieve CAN base clock frequency.
  \param[in]   x      Controller number (0..1)
  \return      base clock frequency
*/
static uint32_t CANx_GetClock (uint8_t x) {

  if (x >= CAN_CTRL_NUM) { return 0U; }

#if defined (LPC175x_6x)
  if ((LPC_SC->PCONP & (1U << (x + 13U))) == 0U) {      // If clock to peripheral is not enabled
    return 0U;
  }

  if (x == 0U) {
    switch ((LPC_SC->PCLKSEL0 >> 26) & 3U) {
      case 0U:
        return (SystemCoreClock / 4U);
      case 1U:
        return (SystemCoreClock);
      case 2U:
        return (SystemCoreClock / 2U);
      case 3U:
        return (SystemCoreClock / 6U);
    }
  } else {
    switch ((LPC_SC->PCLKSEL0 >> 28) & 3U) {
      case 0U:
        return (SystemCoreClock / 4U);
      case 1U:
        return (SystemCoreClock);     
      case 2U:
        return (SystemCoreClock / 2U);
      case 3U:
        return (SystemCoreClock / 6U);
    }
  }
#elif defined (LPC177x_8x)
  if ((LPC_SC->PCONP & (1U << (x + 13U))) == 0U) {      // If clock to peripheral is not enabled
    return 0U;
  }

  if ((LPC_SC->PCLKSEL > 0U) && (LPC_SC->PCLKSEL <= 4U)) {
    return (SystemCoreClock / LPC_SC->PCLKSEL);
  }
#endif
  return 0U;
}
#if (RTE_CAN_CAN1 == 1U)
static uint32_t CAN1_GetClock (void) { return CANx_GetClock (0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static uint32_t CAN2_GetClock (void) { return CANx_GetClock (1U); }
#endif

/**
  \fn          int32_t CANx_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments, uint8_t x)
  \brief       Set bitrate for CAN interface.
  \param[in]   select       Bitrate selection
                 - ARM_CAN_BITRATE_NOMINAL : nominal (flexible data-rate arbitration) bitrate
                 - ARM_CAN_BITRATE_FD_DATA : flexible data-rate data bitrate
  \param[in]   bitrate      Bitrate
  \param[in]   bit_segments Bit segments settings
  \param[in]   x            Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;
  uint32_t         mod, sjw, prop_seg, phase_seg1, phase_seg2, pclk, brp, tq_num;

  if (x >= CAN_CTRL_NUM)                 { return ARM_DRIVER_ERROR;               }
  if (select != ARM_CAN_BITRATE_NOMINAL) { return ARM_CAN_INVALID_BITRATE_SELECT; }
  if (can_driver_powered[x] == 0U)       { return ARM_DRIVER_ERROR;               }

  prop_seg   = (bit_segments & ARM_CAN_BIT_PROP_SEG_Msk  ) >> ARM_CAN_BIT_PROP_SEG_Pos;
  phase_seg1 = (bit_segments & ARM_CAN_BIT_PHASE_SEG1_Msk) >> ARM_CAN_BIT_PHASE_SEG1_Pos;
  phase_seg2 = (bit_segments & ARM_CAN_BIT_PHASE_SEG2_Msk) >> ARM_CAN_BIT_PHASE_SEG2_Pos;
  sjw        = (bit_segments & ARM_CAN_BIT_SJW_Msk       ) >> ARM_CAN_BIT_SJW_Pos;

  if (((prop_seg + phase_seg1) < 1U) || ((prop_seg + phase_seg1) > 16U)) { return ARM_CAN_INVALID_BIT_PROP_SEG;   }
  if (( phase_seg2             < 1U) || ( phase_seg2             >  8U)) { return ARM_CAN_INVALID_BIT_PHASE_SEG2; }
  if (( sjw                    < 1U) || ( sjw                    >  4U)) { return ARM_CAN_INVALID_BIT_SJW;        }

  ptr_CAN = ptr_CANx[x];

  tq_num = 1U + prop_seg + phase_seg1 + phase_seg2;
  pclk   = CANx_GetClock(x);          if (pclk == 0U)  { return ARM_DRIVER_ERROR;        }
  brp    = pclk / (tq_num * bitrate); if (brp > 1024U) { return ARM_CAN_INVALID_BITRATE; }
  if (pclk > (brp * tq_num * bitrate)) {
    if ((((pclk - (brp * tq_num * bitrate)) * 1024U) / pclk) > CAN_CLOCK_TOLERANCE) { return ARM_CAN_INVALID_BITRATE; }
  } else if (pclk < (brp * tq_num * bitrate)) {
    if (((((brp * tq_num * bitrate) - pclk) * 1024U) / pclk) > CAN_CLOCK_TOLERANCE) { return ARM_CAN_INVALID_BITRATE; }
  }

  mod = ptr_CAN->MOD;                   // Store current mode
  ptr_CAN->MOD =  CAN_MOD_RM;           // Enter reset mode
  ptr_CAN->BTR = ((brp - 1U) & CAN_BTR_BRP_Msk) | ((sjw - 1U) << 14) | ((phase_seg2 - 1U) << 20) | ((prop_seg + phase_seg1 - 1U) << 16);
  ptr_CAN->MOD =  mod;                  // Restore mode active before BTR register was accessed

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments) { return CANx_SetBitrate (select, bitrate, bit_segments, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments) { return CANx_SetBitrate (select, bitrate, bit_segments, 1U); }
#endif

/**
  \fn          int32_t CANx_SetMode (ARM_CAN_MODE mode, uint8_t x)
  \brief       Set operating mode for CAN interface.
  \param[in]   mode   Operating mode
                 - ARM_CAN_MODE_INITIALIZATION :    initialization mode
                 - ARM_CAN_MODE_NORMAL :            normal operation mode
                 - ARM_CAN_MODE_RESTRICTED :        restricted operation mode
                 - ARM_CAN_MODE_MONITOR :           bus monitoring mode
                 - ARM_CAN_MODE_LOOPBACK_INTERNAL : loopback internal mode
                 - ARM_CAN_MODE_LOOPBACK_EXTERNAL : loopback external mode
  \param[in]   x      Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_SetMode (ARM_CAN_MODE mode, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;
  uint32_t         event;

  if (x >= CAN_CTRL_NUM)           { return ARM_DRIVER_ERROR; }
  if (can_driver_powered[x] == 0U) { return ARM_DRIVER_ERROR; }

  ptr_CAN = ptr_CANx[x];

  event = 0U;
  switch (mode) {
    case ARM_CAN_MODE_INITIALIZATION:
      ptr_CAN->MOD    =  CAN_MOD_RM;            // Enter reset mode
                                                // Disable filter and allow table RAM access
      LPC_CANAF->AFMR =  CANAF_AFMR_AccBP | CANAF_AFMR_AccOff;
      if (can_unit_state[x] != ARM_CAN_UNIT_STATE_INACTIVE) {
        can_unit_state[x]    = ARM_CAN_UNIT_STATE_INACTIVE;
        event                = ARM_CAN_EVENT_UNIT_BUS_OFF;
      }
      break;
    case ARM_CAN_MODE_NORMAL:
      LPC_CANAF->AFMR =  0U;                    // Enable filter
      ptr_CAN->MOD   &=~(CAN_MOD_STM |          // Deactivate Self Test Mode
                         CAN_MOD_LOM |          // Deactivate Listen Only Mode
                         CAN_MOD_RM) ;          // Enter operating mode
      can_loopback[x] =  0U;                    // Loopback not active
      if (can_unit_state[x] != ARM_CAN_UNIT_STATE_ACTIVE) {
        can_unit_state[x]    = ARM_CAN_UNIT_STATE_ACTIVE;
        event                = ARM_CAN_EVENT_UNIT_ACTIVE;
      }
      break;
    case ARM_CAN_MODE_RESTRICTED:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_CAN_MODE_MONITOR:
      LPC_CANAF->AFMR =  0U;                    // Enable filter
      ptr_CAN->MOD   |=  CAN_MOD_LOM;           // Activate Listen Only Mode
      ptr_CAN->MOD   &=~(CAN_MOD_STM |          // Deactivate Self Test Mode
                         CAN_MOD_RM) ;          // Enter operating mode
      can_loopback[x] =  0U;                    // Loopback not active
      if (can_unit_state[x] == ARM_CAN_UNIT_STATE_ACTIVE) {
        can_unit_state[x]    = ARM_CAN_UNIT_STATE_PASSIVE;
        event                = ARM_CAN_EVENT_UNIT_PASSIVE;
      }
      break;
    case ARM_CAN_MODE_LOOPBACK_INTERNAL:
      LPC_CANAF->AFMR =  0U;                    // Enable filter
      ptr_CAN->MOD   |=  CAN_MOD_STM |          // Activate Self Test Mode
                         CAN_MOD_LOM ;          // Activate Listen Only Mode
      ptr_CAN->MOD   &= ~CAN_MOD_RM  ;          // Enter operating mode
      can_loopback[x] =  1U;                    // Loopback active
      if (can_unit_state[x] == ARM_CAN_UNIT_STATE_ACTIVE) {
        can_unit_state[x]    = ARM_CAN_UNIT_STATE_PASSIVE;
        event                = ARM_CAN_EVENT_UNIT_PASSIVE;
      }
      break;
    case ARM_CAN_MODE_LOOPBACK_EXTERNAL:
      LPC_CANAF->AFMR =  0U;                    // Enable filter
      ptr_CAN->MOD   |=  CAN_MOD_STM;           // Activate Self Test Mode
      ptr_CAN->MOD   &=~(CAN_MOD_LOM |          // Deactivate Listen Only Mode
                         CAN_MOD_RM) ;          // Enter operating mode
      can_loopback[x] =  1U;                    // Loopback active
      if (can_unit_state[x] == ARM_CAN_UNIT_STATE_PASSIVE) {
        can_unit_state[x]    = ARM_CAN_UNIT_STATE_ACTIVE;
        event                = ARM_CAN_EVENT_UNIT_ACTIVE;
      }
      break;
  }
  if ((CAN_SignalUnitEvent[x] != NULL) && (event != 0U)) { CAN_SignalUnitEvent[x](event); }

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_SetMode (ARM_CAN_MODE mode) { return CANx_SetMode (mode, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_SetMode (ARM_CAN_MODE mode) { return CANx_SetMode (mode, 1U); }
#endif

/**
  \fn          ARM_CAN_OBJ_CAPABILITIES CANx_ObjectGetCapabilities (uint32_t obj_idx, uint8_t x)
  \brief       Retrieve capabilities of an object.
  \param[in]   obj_idx  Object index
  \param[in]   x        Controller number (0..1)
  \return      ARM_CAN_OBJ_CAPABILITIES
*/
static ARM_CAN_OBJ_CAPABILITIES CANx_ObjectGetCapabilities (uint32_t obj_idx, uint8_t x) {
  ARM_CAN_OBJ_CAPABILITIES obj_cap_null;

  if ((x >= CAN_CTRL_NUM) || (obj_idx > 1U)) {
    memset (&obj_cap_null, 0U, sizeof(ARM_CAN_OBJ_CAPABILITIES));
    return obj_cap_null;
  }

  if (obj_idx == 0U) {
    return can_object_capabilities_rx;
  } else {
    return can_object_capabilities_tx;
  }
}
#if (RTE_CAN_CAN1 == 1U)
static ARM_CAN_OBJ_CAPABILITIES CAN1_ObjectGetCapabilities (uint32_t obj_idx) { return CANx_ObjectGetCapabilities (obj_idx, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static ARM_CAN_OBJ_CAPABILITIES CAN2_ObjectGetCapabilities (uint32_t obj_idx) { return CANx_ObjectGetCapabilities (obj_idx, 1U); }
#endif

/**
  \fn          int32_t CANx_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg, uint8_t x)
  \brief       Add or remove filter for message reception.
  \param[in]   obj_idx      Object index of object that filter should be or is assigned to
  \param[in]   operation    Operation on filter
                 - ARM_CAN_FILTER_ID_EXACT_ADD :       add    exact id filter
                 - ARM_CAN_FILTER_ID_EXACT_REMOVE :    remove exact id filter
                 - ARM_CAN_FILTER_ID_RANGE_ADD :       add    range id filter
                 - ARM_CAN_FILTER_ID_RANGE_REMOVE :    remove range id filter
                 - ARM_CAN_FILTER_ID_MASKABLE_ADD :    add    maskable id filter
                 - ARM_CAN_FILTER_ID_MASKABLE_REMOVE : remove maskable id filter
  \param[in]   id           ID or start of ID range (depending on filter type)
  \param[in]   arg          Mask or end of ID range (depending on filter type)
  \param[in]   x            Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg, uint8_t x) {
  uint32_t afmr;
  int32_t  status;

  if (x >= CAN_CTRL_NUM)           { return ARM_DRIVER_ERROR;           }
  if (obj_idx != 0U)               { return ARM_DRIVER_ERROR_PARAMETER; }
  if (can_driver_powered[x] == 0U) { return ARM_DRIVER_ERROR;           }

  afmr = LPC_CANAF->AFMR;                                       // Store current acceptance filter configuration
  LPC_CANAF->AFMR = CANAF_AFMR_AccBP | CANAF_AFMR_AccOff;       // Disable filter and allow table RAM access

  switch (operation) {
    case ARM_CAN_FILTER_ID_EXACT_ADD:
      status = CANx_AddFilter   (CAN_FILTER_TYPE_EXACT_ID, id,  0U, x);
      break;
    case ARM_CAN_FILTER_ID_RANGE_ADD:
      status = CANx_AddFilter   (CAN_FILTER_TYPE_RANGE_ID, id, arg, x);
      break;
    case ARM_CAN_FILTER_ID_EXACT_REMOVE:
      status = CANx_RemoveFilter(CAN_FILTER_TYPE_EXACT_ID, id,  0U, x);
      break;
    case ARM_CAN_FILTER_ID_RANGE_REMOVE:
      status = CANx_RemoveFilter(CAN_FILTER_TYPE_RANGE_ID, id, arg, x);
      break;
    case ARM_CAN_FILTER_ID_MASKABLE_ADD:
    case ARM_CAN_FILTER_ID_MASKABLE_REMOVE:
      status = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
  }
  LPC_CANAF->AFMR = afmr;                                       // Restore acceptance filter configuration

  return status;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg) { return CANx_ObjectSetFilter (obj_idx, operation, id, arg, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg) { return CANx_ObjectSetFilter (obj_idx, operation, id, arg, 1U); }
#endif

/**
  \fn          int32_t CANx_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg, uint8_t x)
  \brief       Configure object.
  \param[in]   obj_idx  Object index
  \param[in]   obj_cfg  Object configuration state
                 - ARM_CAN_OBJ_INACTIVE :       deactivate object
                 - ARM_CAN_OBJ_RX :             configure object for reception
                 - ARM_CAN_OBJ_TX :             configure object for transmission
                 - ARM_CAN_OBJ_RX_RTR_TX_DATA : configure object that on RTR reception automatically transmits Data Frame
                 - ARM_CAN_OBJ_TX_RTR_RX_DATA : configure object that transmits RTR and automatically receives Data Frame
  \param[in]   x        Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg, uint8_t x) {

  if (x >= CAN_CTRL_NUM)           { return ARM_DRIVER_ERROR;           }
  if (obj_idx > 1U)                { return ARM_DRIVER_ERROR_PARAMETER; }
  if (can_driver_powered[x] == 0U) { return ARM_DRIVER_ERROR;           }

  switch (obj_cfg) {
    case ARM_CAN_OBJ_INACTIVE:
      can_obj_cfg_msk[x] &= ~(1U << obj_idx);
      break;
    case ARM_CAN_OBJ_RX_RTR_TX_DATA:
    case ARM_CAN_OBJ_TX_RTR_RX_DATA:
      can_obj_cfg_msk[x] &= ~(1U << obj_idx);
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    case ARM_CAN_OBJ_TX:
      if (obj_idx != 1U) { return ARM_DRIVER_ERROR_PARAMETER; }
      can_obj_cfg_msk[x] = 2U;
      break;
    case ARM_CAN_OBJ_RX:
      if (obj_idx != 0U) { return ARM_DRIVER_ERROR_PARAMETER; }
      can_obj_cfg_msk[x] = 1U;
      break;
  }

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg) { return CANx_ObjectConfigure (obj_idx, obj_cfg, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg) { return CANx_ObjectConfigure (obj_idx, obj_cfg, 1U); }
#endif

/**
  \fn          int32_t CANx_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size, uint8_t x)
  \brief       Send message on CAN bus.
  \param[in]   obj_idx  Object index
  \param[in]   msg_info Pointer to CAN message information
  \param[in]   data     Pointer to data buffer
  \param[in]   size     Number of data bytes to send
  \param[in]   x        Controller number (0..1)
  \return      value >= 0  number of data bytes accepted to send
  \return      value < 0   execution status
*/
static int32_t CANx_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;
  uint32_t         tfi, tid, cmr;
  uint8_t          tx_obj_alloc;

  if (x >= CAN_CTRL_NUM)              { return ARM_DRIVER_ERROR;           }
  if (obj_idx != 1U)                  { return ARM_DRIVER_ERROR_PARAMETER; }
  if (can_driver_powered[x]    == 0U) { return ARM_DRIVER_ERROR;           }
  if (can_obj_tx_alloc_lock[x] != 0U) { return ARM_DRIVER_ERROR_BUSY;      }

  ptr_CAN = ptr_CANx[x];

  // Protect simultaneous accesses
  can_obj_tx_alloc_lock[x] = 1U;        // Block access to allocate
  if        (can_obj_tx_alloc [x][0] == 0U) {
    can_obj_tx_alloc [x][0]  = 1U;      // Allocated Transmit Buffer 0
    can_obj_tx_alloc_lock[x] = 0U;
    tx_obj_alloc             = 0U;
  } else if (can_obj_tx_alloc [x][1] == 0U) {
    can_obj_tx_alloc [x][1]  = 1U;      // Allocated Transmit Buffer 1
    can_obj_tx_alloc_lock[x] = 0U;
    tx_obj_alloc             = 1U;
  } else if (can_obj_tx_alloc [x][2] == 0U) {
    can_obj_tx_alloc [x][2]  = 1U;      // Allocated Transmit Buffer 2
    can_obj_tx_alloc_lock[x] = 0U;
    tx_obj_alloc             = 2U;
  } else {
    can_obj_tx_alloc_lock[x] = 0U;      // Unblock access to allocate
    return ARM_DRIVER_ERROR_BUSY;
  }

  if (size > 8U) { size = 8U; }

  tfi = (((msg_info->id & ARM_CAN_ID_IDE_Msk) == ARM_CAN_ID_IDE_Msk) ? (1U << 31) : 0U) | ((uint32_t)msg_info->rtr << 30) | ((uint32_t)size << 16);
  tid = msg_info->id & ~ARM_CAN_ID_IDE_Msk;
  cmr = ((can_loopback[x] != 0U) ? CAN_CMR_SRR : CAN_CMR_TR) | ((can_no_retransmission[x] != 0U) ? CAN_CMR_AT : 0U);

  switch (tx_obj_alloc) {
    case 0U:
      ptr_CAN->TFI1 = tfi;
      ptr_CAN->TID1 = tid;
      if (data != NULL) {
        ptr_CAN->TDA1 = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        ptr_CAN->TDB1 = (uint32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
      }
      ptr_CAN->CMR = CAN_CMR_STB1 | cmr;
      break;
    case 1U:
      ptr_CAN->TFI2 = tfi;
      ptr_CAN->TID2 = tid;
      if (data != NULL) {
        ptr_CAN->TDA2 = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        ptr_CAN->TDB2 = (uint32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
      }
      ptr_CAN->CMR = CAN_CMR_STB2 | cmr;
      break;
    case 2U:
      ptr_CAN->TFI3 = tfi;
      ptr_CAN->TID3 = tid;
      if (data != NULL) {
        ptr_CAN->TDA3 = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        ptr_CAN->TDB3 = (uint32_t)(data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24));
      }
      ptr_CAN->CMR = CAN_CMR_STB3 | cmr;
      break;
    default:
      return ARM_DRIVER_ERROR;
  }

  return ((int32_t)size);
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size) { return CANx_MessageSend (obj_idx, msg_info, data, size, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size) { return CANx_MessageSend (obj_idx, msg_info, data, size, 1U); }
#endif

/**
  \fn          int32_t CANx_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size, uint8_t x)
  \brief       Read message received on CAN bus.
  \param[in]   obj_idx  Object index
  \param[out]  msg_info Pointer to read CAN message information
  \param[out]  data     Pointer to data buffer for read data
  \param[in]   size     Maximum number of data bytes to read
  \param[in]   x        Controller number (0..1)
  \return      value >= 0  number of data bytes read
  \return      value < 0   execution status
*/
static int32_t CANx_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;
  uint32_t         rfs;
  uint32_t         data_rx[2];

  if (x >= CAN_CTRL_NUM)           { return ARM_DRIVER_ERROR;           }
  if (obj_idx != 0U)               { return ARM_DRIVER_ERROR_PARAMETER; }
  if (can_driver_powered[x] == 0U) { return ARM_DRIVER_ERROR;           }

  ptr_CAN = ptr_CANx[x];

  if (size > 8U) { size = 8U; }

  rfs = ptr_CAN->RFS;

  if ((rfs & CAN_RFS_FF) == 0U) {       // Standard Identifier (11 bit)
    msg_info->id =  ptr_CAN->RID & 0x7FFU;
  } else {                              // Extended Identifier (29 bit)
    msg_info->id = (ptr_CAN->RID & 0x1FFFFFFFU) | ARM_CAN_ID_IDE_Msk;
  }
  msg_info->rtr = (rfs & CAN_RFS_RTR) >> CAN_RFS_RTR_Pos;
  if (msg_info->rtr != 0U) { size = 0U; }

  msg_info->dlc = (rfs & CAN_RFS_DLC_Msk) >> CAN_RFS_DLC_Pos;
  if (msg_info->dlc > 8U) { msg_info->dlc = 8U; }

  if (size > 0U){
    data_rx[0] = ptr_CAN->RDA;
    data_rx[1] = ptr_CAN->RDB;
    memcpy(data, (uint8_t *)(&data_rx[0]), size);
  }

  return ((int32_t)size);
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size) { return CANx_MessageRead (obj_idx, msg_info, data, size, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size) { return CANx_MessageRead (obj_idx, msg_info, data, size, 1U); }
#endif

/**
  \fn          int32_t CANx_Control (uint32_t control, uint32_t arg, uint8_t x)
  \brief       Control CAN interface.
  \param[in]   control  Operation
                 - ARM_CAN_SET_FD_MODE :            set FD operation mode
                 - ARM_CAN_ABORT_MESSAGE_SEND :     abort sending of CAN message
                 - ARM_CAN_CONTROL_RETRANSMISSION : enable/disable automatic retransmission
                 - ARM_CAN_SET_TRANSCEIVER_DELAY :  set transceiver delay
  \param[in]   arg      Argument of operation
  \param[in]   x        Controller number (0..1)
  \return      execution status
*/
static int32_t CANx_Control (uint32_t control, uint32_t arg, uint8_t x) {
  LPC_CAN_TypeDef *ptr_CAN;

  if (x >= CAN_CTRL_NUM)           { return ARM_DRIVER_ERROR; }
  if (can_driver_powered[x] == 0U) { return ARM_DRIVER_ERROR; }

  ptr_CAN = ptr_CANx[x];

  switch (control & ARM_CAN_CONTROL_Msk) {
    case ARM_CAN_ABORT_MESSAGE_SEND:
      if (arg != 1U) { return ARM_DRIVER_ERROR_PARAMETER; }
      ptr_CAN->CMR = CAN_CMR_AT;
      break;
    case ARM_CAN_CONTROL_RETRANSMISSION:
      switch (arg) {
        case 0U:
          can_no_retransmission[x] = 1U;
          break;
        case 1U:
          can_no_retransmission[x] = 0U;
          break;
        default:
          return ARM_DRIVER_ERROR_PARAMETER;
      }
      break;
    case ARM_CAN_SET_FD_MODE:
    case ARM_CAN_SET_TRANSCEIVER_DELAY:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}
#if (RTE_CAN_CAN1 == 1U)
static int32_t CAN1_Control (uint32_t control, uint32_t arg) { return CANx_Control (control, arg, 0U); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static int32_t CAN2_Control (uint32_t control, uint32_t arg) { return CANx_Control (control, arg, 1U); }
#endif

/**
  \fn          ARM_CAN_STATUS CANx_GetStatus (uint8_t x)
  \brief       Get CAN status.
  \param[in]   x      Controller number (0..1)
  \return      CAN status ARM_CAN_STATUS
*/
static ARM_CAN_STATUS CANx_GetStatus (uint8_t x) {
  ARM_CAN_STATUS can_status;
  uint32_t       gsr;

  if ((x >= CAN_CTRL_NUM) || (can_driver_powered[x] == 0U)) {
    memset((void *)&can_status, 0U, sizeof(ARM_CAN_STATUS));
    return can_status;
  }

  gsr = ptr_CANx[x]->GSR;

  can_status.unit_state      = can_unit_state[x] & 3U;

  can_status.last_error_code = can_last_error_code[x];
  can_status.tx_error_count  = (uint8_t)((gsr & CAN_GSR_TXERR_Msk) >> CAN_GSR_TXERR_Pos);
  can_status.rx_error_count  = (uint8_t)((gsr & CAN_GSR_RXERR_Msk) >> CAN_GSR_RXERR_Pos);

  return can_status;
}
#if (RTE_CAN_CAN1 == 1U)
static ARM_CAN_STATUS CAN1_GetStatus (void) { return CANx_GetStatus (0); }
#endif
#if (RTE_CAN_CAN2 == 1U)
static ARM_CAN_STATUS CAN2_GetStatus (void) { return CANx_GetStatus (1); }
#endif


/**
  \fn          void CAN_IRQHandler (void)
  \brief       CAN Interrupt Routine (IRQ).
*/
void CAN_IRQHandler (void) {
  LPC_CAN_TypeDef *ptr_CAN;
  uint32_t         icr, gsr;
  uint32_t         x;

#if (RTE_CAN_CAN1 == 1U)
  x = 0U;
#else
  x = 1U;
#endif

#if ((RTE_CAN_CAN1 == 1U) && (RTE_CAN_CAN2 == 1U))
  for (x = 0U; x < 2U; x++) {
    if (can_driver_powered[x] == 0U) { continue; }
#endif

  ptr_CAN = ptr_CANx[x];

  icr = ptr_CAN->ICR;
  gsr = ptr_CAN->GSR;
  if ((icr & CAN_ICR_DOI) != 0U) {    // If Data Overrun Interrupt is active
    if (CAN_SignalObjectEvent[x] != NULL) { CAN_SignalObjectEvent[x](0U, ARM_CAN_EVENT_RECEIVE | ARM_CAN_EVENT_RECEIVE_OVERRUN); }
    ptr_CAN->CMR = CAN_CMR_CDO;       // Clear Data Overrun if active
  } else if ((icr&CAN_ICR_RI)!=0U) {  // If Receive Interrupt is active
    if (CAN_SignalObjectEvent[x] != NULL) { CAN_SignalObjectEvent[x](0U, ARM_CAN_EVENT_RECEIVE); }
    ptr_CAN->CMR = CAN_CMR_RRB;       // Release Receive Buffer
  }
  if ((icr & CAN_ICR_TI1) != 0U) {    // If Transmit Interrupt 1 is active
    can_obj_tx_alloc[x][0] = 0U;
    if (CAN_SignalObjectEvent[x] != NULL) { CAN_SignalObjectEvent[x](1U, ARM_CAN_EVENT_SEND_COMPLETE); }
  }
  if ((icr & CAN_ICR_TI2) != 0U) {    // If Transmit Interrupt 2 is active
    can_obj_tx_alloc[x][1] = 0U;
    if (CAN_SignalObjectEvent[x] != NULL) { CAN_SignalObjectEvent[x](1U, ARM_CAN_EVENT_SEND_COMPLETE); }
  }
  if ((icr & CAN_ICR_TI3) != 0U) {    // If Transmit Interrupt 3 is active
    can_obj_tx_alloc[x][2] = 0U;
    if (CAN_SignalObjectEvent[x] != NULL) { CAN_SignalObjectEvent[x](1U, ARM_CAN_EVENT_SEND_COMPLETE); }
  }
  if ((icr & CAN_ICR_EI) != 0U) {     // If Error Warning Interrupt is active
    if ((gsr & CAN_GSR_ES) != 0U) {
      if (CAN_SignalUnitEvent[x] != NULL) { CAN_SignalUnitEvent[x](ARM_CAN_EVENT_UNIT_WARNING); }
    }
  }
  if ((icr & CAN_ICR_EPI) != 0U) {    // If Error Passive Interrupt is active
    if ((((gsr & CAN_GSR_RXERR_Msk) >> CAN_GSR_RXERR_Pos) > 127U) ||
        (((gsr & CAN_GSR_TXERR_Msk) >> CAN_GSR_TXERR_Pos) > 127U)) {
      can_unit_state[x] = ARM_CAN_UNIT_STATE_PASSIVE;
      if (CAN_SignalUnitEvent[x] != NULL) { CAN_SignalUnitEvent[x](ARM_CAN_EVENT_UNIT_PASSIVE); }
    } else {
      can_unit_state[x] = ARM_CAN_UNIT_STATE_ACTIVE;
      if (CAN_SignalUnitEvent[x] != NULL) { CAN_SignalUnitEvent[x](ARM_CAN_EVENT_UNIT_ACTIVE); }
    }
  }
  if ((icr & CAN_ICR_BEI) != 0U) {    // If Bus Error Interrupt is active
    switch ((icr & CAN_ICR_ERRC1_0_Msk) >> CAN_ICR_ERRC1_0_Pos) {
      case 0U:
        can_last_error_code[x] = ARM_CAN_LEC_BIT_ERROR;
        break;
      case 1U:
        can_last_error_code[x] = ARM_CAN_LEC_FORM_ERROR;
        break;
      case 2U:
        can_last_error_code[x] = ARM_CAN_LEC_STUFF_ERROR;
        break;
      case 3U:
        switch ((icr & CAN_ICR_ERRBIT_Msk) >> CAN_ICR_ERRBIT_Pos) {
          case 8U:
            can_last_error_code[x] = ARM_CAN_LEC_CRC_ERROR;
            break;
          case 25U:
            can_last_error_code[x] = ARM_CAN_LEC_ACK_ERROR;
            break;
          default:
            break;
        }
        break;
    }
    if ((gsr & CAN_GSR_BS) != 0U) {
      can_unit_state[x] = ARM_CAN_UNIT_STATE_INACTIVE;
      if (CAN_SignalUnitEvent[x] != NULL) { CAN_SignalUnitEvent[x](ARM_CAN_EVENT_UNIT_BUS_OFF); }
    } else if (can_unit_state[x] == ARM_CAN_UNIT_STATE_INACTIVE) {
      can_unit_state[x] = ARM_CAN_UNIT_STATE_ACTIVE;
      if (CAN_SignalUnitEvent[x] != NULL) { CAN_SignalUnitEvent[x](ARM_CAN_EVENT_UNIT_ACTIVE); }
    }
  }
#if ((RTE_CAN_CAN1 == 1U) && (RTE_CAN_CAN2 == 1U))
  }
#endif
}


#if (RTE_CAN_CAN1 == 1U)
ARM_DRIVER_CAN Driver_CAN1 = {
  CAN_GetVersion,
  CAN_GetCapabilities,
  CAN1_Initialize,
  CAN1_Uninitialize,
  CAN1_PowerControl,
  CAN1_GetClock,
  CAN1_SetBitrate,
  CAN1_SetMode,
  CAN1_ObjectGetCapabilities,
  CAN1_ObjectSetFilter,
  CAN1_ObjectConfigure,
  CAN1_MessageSend,
  CAN1_MessageRead,
  CAN1_Control,
  CAN1_GetStatus
};
#endif

#if (RTE_CAN_CAN2 == 1U)
ARM_DRIVER_CAN Driver_CAN2 = {
  CAN_GetVersion,
  CAN_GetCapabilities,
  CAN2_Initialize,
  CAN2_Uninitialize,
  CAN2_PowerControl,
  CAN2_GetClock,
  CAN2_SetBitrate,
  CAN2_SetMode,
  CAN2_ObjectGetCapabilities,
  CAN2_ObjectSetFilter,
  CAN2_ObjectConfigure,
  CAN2_MessageSend,
  CAN2_MessageRead,
  CAN2_Control,
  CAN2_GetStatus
};
#endif
