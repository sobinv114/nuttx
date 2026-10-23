/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>

#include "riscv_internal.h"
#include "rom/rom_layout.h"
#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
#  include "esp_retentionheap.h"
#endif
#if defined(CONFIG_ESPRESSIF_SPIRAM)
#  include "esp_psram.h"
#  include "esp_private/esp_psram_extram.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

uintptr_t _heap_start;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel builds (CONFIG_BUILD_PROTECTED=y or
 *   CONFIG_BUILD_KERNEL=y) there may be both kernel- and user-space heaps
 *   as determined by CONFIG_MM_KERNEL_HEAP=y.  This function allocates (and
 *   protects) the kernel-space heap.
 *
 *   For Flat build (CONFIG_BUILD_FLAT=y), this function enables a separate
 *   (although unprotected) heap for the kernel.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  uintptr_t kbase = g_idle_topstack;
  uintptr_t ktop;

#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
  uintptr_t rstart;
  uintptr_t rend;

  esp_retentionheap_find_region(&rstart, &rend);
  ktop = rstart;
#elif defined(CONFIG_ESP32P4_SELECTS_REV_LESS_V3)
  /* Rev 1.3 sram_low ends at 0x4ff2cbb0. Stop here to avoid the memory hole! */
  ktop = 0x4ff2cbb0;
#else
  ktop = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#endif

  DEBUGASSERT(ktop > kbase);

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)kbase;
  *heap_size  = ktop - kbase;

}
#endif


/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   Region 1: Primary heap in clean 384 KB SRAM_HIGH.
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  board_autoled_on(LED_HEAPALLOCATE);

  /* Start primary heap in fast internal SRAM (384 KB) */
  *heap_start = (void *)0x4ff40000;
  *heap_size  = (size_t)(384 * 1024);

  memset(*heap_start, 0, *heap_size);
  _heap_start = (uintptr_t)*heap_start;

}

/****************************************************************************
 * Name: riscv_addregion
 *
 * Description:
 *   Region 2: Add the full 32 MB PSRAM to the unified heap pool.
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void)
{
  if (esp_psram_is_initialized())
    {
      uintptr_t psram_start = esp_psram_extram_vaddr_start(); // 0x48000000
      uintptr_t psram_end   = esp_psram_extram_vaddr_end();   // 0x4a000000
      size_t    psram_size  = psram_end - psram_start;        // 32 MB

      if (psram_size > 0)
        {
          /* Add 32 MB PSRAM to the heap */
          #ifdef CONFIG_MM_KERNEL_HEAP
              kumm_addregion((void *)psram_start, psram_size);
          #else
              umm_addregion((void *)psram_start, psram_size);
          #endif

        }
    }
}
#endif
