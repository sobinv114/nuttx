/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_sdmmc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO host for operation on the specified slot (0 or 1).
 *
 * Input Parameters:
 *   slotno - Slot number to initialize (0 = Slot 0, 1 = Slot 1).
 *
 * Returned Value:
 *   A reference to an SDIO interface structure. NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno);

void esp32p4_sdmmc_dump_details(struct sdio_dev_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H */