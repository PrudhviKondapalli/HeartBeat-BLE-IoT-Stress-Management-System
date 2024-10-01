/***************************************************************************//**
 * @file
 * @brief Template for Application Properties
 *******************************************************************************
 * # License
 * <b>Copyright 2017 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <api/application_properties.h>

#if defined(__IAR_SYSTEMS_ICC__)
/* IAR ICC */
  #define KEEP_SYMBOL           __root
#elif defined(__GNUC__)
/* GNU GCC */
  #define KEEP_SYMBOL           __attribute__((used))
#else
  #define KEEP_SYMBOL
#endif

/// Version number for this application (uint32_t)
#define APP_PROPERTIES_VERSION 1
/// Unique ID (e.g. UUID or GUID) for the product this application is built for (uint8_t[16])
#define APP_PROPERTIES_ID { 0 }

KEEP_SYMBOL const ApplicationProperties_t sl_app_properties = {
  /// @brief Magic value indicating that this is an ApplicationProperties_t struct.
  /// Must equal @ref APPLICATION_PROPERTIES_MAGIC
  .magic = APPLICATION_PROPERTIES_MAGIC,
  /// Version number of this struct
  .structVersion = APPLICATION_PROPERTIES_VERSION,
  /// Type of signature this application is signed with
  .signatureType = APPLICATION_SIGNATURE_NONE,
  /// Location of the signature. Typically a pointer to the end of the application
  .signatureLocation = 0,
  /// Information about the application
  .app = {
    /// Bitfield representing type of application, e.g. @ref APPLICATION_TYPE_BLUETOOTH_APP
    .type = APPLICATION_TYPE_BLUETOOTH_APP,
    /// Version number for this application
    .version = APP_PROPERTIES_VERSION,
    /// Capabilities of this application
    .capabilities = 0,
    /// Unique ID (e.g. UUID or GUID) for the product this application is built for
    .productId = APP_PROPERTIES_ID,
  },
};
