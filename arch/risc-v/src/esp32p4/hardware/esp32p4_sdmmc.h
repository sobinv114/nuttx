/****************************************************************************
 * arch/risc-v/src/esp32p4/hardware/esp32p4_sdmmc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_HARDWARE_ESP32P4_SDMMC_H
#define __ARCH_RISCV_SRC_ESP32P4_HARDWARE_ESP32P4_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-P4 SDMMC Base Address */
#undef  ESP32P4_SDMMC_BASE
#define ESP32P4_SDMMC_BASE                 0x50083000U  //0x50083000

#define ESP32P4_TXFIFO_DEPTH               32
#define ESP32P4_TXFIFO_WIDTH               4
#define ESP32P4_RXFIFO_DEPTH               32
#define ESP32P4_RXFIFO_WIDTH               4

/* Register offsets */

#define ESP32P4_SDMMC_CTRL_OFFSET          0x0000
#define ESP32P4_SDMMC_PWREN_OFFSET         0x0004
#define ESP32P4_SDMMC_CLKDIV_OFFSET        0x0008
#define ESP32P4_SDMMC_CLKSRC_OFFSET        0x000c
#define ESP32P4_SDMMC_CLKENA_OFFSET        0x0010
#define ESP32P4_SDMMC_TMOUT_OFFSET         0x0014
#define ESP32P4_SDMMC_CTYPE_OFFSET         0x0018
#define ESP32P4_SDMMC_BLKSIZ_OFFSET        0x001c
#define ESP32P4_SDMMC_BYTCNT_OFFSET        0x0020
#define ESP32P4_SDMMC_INTMASK_OFFSET       0x0024
#define ESP32P4_SDMMC_CMDARG_OFFSET        0x0028
#define ESP32P4_SDMMC_CMD_OFFSET           0x002c
#define ESP32P4_SDMMC_RESP0_OFFSET         0x0030
#define ESP32P4_SDMMC_RESP1_OFFSET         0x0034
#define ESP32P4_SDMMC_RESP2_OFFSET         0x0038
#define ESP32P4_SDMMC_RESP3_OFFSET         0x003c
#define ESP32P4_SDMMC_MINTSTS_OFFSET       0x0040
#define ESP32P4_SDMMC_RINTSTS_OFFSET       0x0044
#define ESP32P4_SDMMC_STATUS_OFFSET        0x0048
#define ESP32P4_SDMMC_FIFOTH_OFFSET        0x004c
#define ESP32P4_SDMMC_CDETECT_OFFSET       0x0050
#define ESP32P4_SDMMC_WRTPRT_OFFSET        0x0054
#define ESP32P4_SDMMC_TCBCNT_OFFSET        0x005c
#define ESP32P4_SDMMC_TBBCNT_OFFSET        0x0060
#define ESP32P4_SDMMC_DEBNCE_OFFSET        0x0064
#define ESP32P4_SDMMC_RSTN_OFFSET          0x0078
#define ESP32P4_SDMMC_BMOD_OFFSET          0x0080
#define ESP32P4_SDMMC_PLDMND_OFFSET        0x0084
#define ESP32P4_SDMMC_DBADDR_OFFSET        0x0088
#define ESP32P4_SDMMC_IDSTS_OFFSET         0x008c
#define ESP32P4_SDMMC_IDINTEN_OFFSET       0x0090
#define ESP32P4_SDMMC_DSCADDR_OFFSET       0x0094
#define ESP32P4_SDMMC_BUFADDR_OFFSET       0x0098
#define ESP32P4_SDMMC_DATA_OFFSET          0x0200
#define ESP32P4_SDMMC_CLOCK_OFFSET         0x0800

/* Register virtual addresses */

#define ESP32P4_SDMMC_CTRL                 (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CTRL_OFFSET)
#define ESP32P4_SDMMC_PWREN                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_PWREN_OFFSET)
#define ESP32P4_SDMMC_CLKDIV               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CLKDIV_OFFSET)
#define ESP32P4_SDMMC_CLKSRC               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CLKSRC_OFFSET)
#define ESP32P4_SDMMC_CLKENA               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CLKENA_OFFSET)
#define ESP32P4_SDMMC_TMOUT                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_TMOUT_OFFSET)
#define ESP32P4_SDMMC_CTYPE                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CTYPE_OFFSET)
#define ESP32P4_SDMMC_BLKSIZ               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_BLKSIZ_OFFSET)
#define ESP32P4_SDMMC_BYTCNT               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_BYTCNT_OFFSET)
#define ESP32P4_SDMMC_INTMASK              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_INTMASK_OFFSET)
#define ESP32P4_SDMMC_CMDARG               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CMDARG_OFFSET)
#define ESP32P4_SDMMC_CMD                  (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CMD_OFFSET)
#define ESP32P4_SDMMC_RESP0                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RESP0_OFFSET)
#define ESP32P4_SDMMC_RESP1                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RESP1_OFFSET)
#define ESP32P4_SDMMC_RESP2                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RESP2_OFFSET)
#define ESP32P4_SDMMC_RESP3                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RESP3_OFFSET)
#define ESP32P4_SDMMC_MINTSTS              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_MINTSTS_OFFSET)
#define ESP32P4_SDMMC_RINTSTS              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RINTSTS_OFFSET)
#define ESP32P4_SDMMC_STATUS               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_STATUS_OFFSET)
#define ESP32P4_SDMMC_FIFOTH               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_FIFOTH_OFFSET)
#define ESP32P4_SDMMC_CDETECT              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CDETECT_OFFSET)
#define ESP32P4_SDMMC_WRTPRT               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_WRTPRT_OFFSET)
#define ESP32P4_SDMMC_TCBCNT               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_TCBCNT_OFFSET)
#define ESP32P4_SDMMC_TBBCNT               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_TBBCNT_OFFSET)
#define ESP32P4_SDMMC_DEBNCE               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_DEBNCE_OFFSET)
#define ESP32P4_SDMMC_RSTN                 (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_RSTN_OFFSET)
#define ESP32P4_SDMMC_BMOD                 (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_BMOD_OFFSET)
#define ESP32P4_SDMMC_PLDMND               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_PLDMND_OFFSET)
#define ESP32P4_SDMMC_DBADDR               (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_DBADDR_OFFSET)
#define ESP32P4_SDMMC_IDSTS                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_IDSTS_OFFSET)
#define ESP32P4_SDMMC_IDINTEN              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_IDINTEN_OFFSET)
#define ESP32P4_SDMMC_DSCADDR              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_DSCADDR_OFFSET)
#define ESP32P4_SDMMC_BUFADDR              (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_BUFADDR_OFFSET)
#define ESP32P4_SDMMC_DATA                 (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_DATA_OFFSET)
#define ESP32P4_SDMMC_CLOCK                (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CLOCK_OFFSET)

/* Bit Definitions */

#define SDMMC_CTRL_CNTLRRESET              (1 << 0)
#define SDMMC_CTRL_FIFORESET               (1 << 1)
#define SDMMC_CTRL_DMARESET                (1 << 2)
#define SDMMC_CTRL_INTENABLE               (1 << 4)
#define SDMMC_CTRL_DMAENABLE               (1 << 5)
#define SDMMC_CTRL_INTDMA                  (1 << 25)

#define SDMMC_CLKDIV_SHIFT(n)              ((n) * 8)
#define SDMMC_CLKDIV_MASK(n)               (255 << SDMMC_CLKDIV_SHIFT(n))
#define SDMMC_CLKDIV(n, div)               ((div) << SDMMC_CLKDIV_SHIFT(n))

#define SDMMC_CLKSRC_SHIFT(slot)           ((slot) * 2)
#define SDMMC_CLKSRC_MASK(slot)            (3 << SDMMC_CLKSRC_SHIFT(slot))
#define SDMMC_CLKSRC_CLKDIV(slot, div)     ((div) << SDMMC_CLKSRC_SHIFT(slot))

#define SDMMC_CLKENA_ENABLE(slot)          (1 << (slot))
#define SDMMC_CLKENA_LOWPOWER(slot)        (1 << ((slot) + 16))

#define SDMMC_TMOUT_RESPONSE_MASK          0x000000ff
#define SDMMC_TMOUT_DATA_MASK              0xffffff00
#define SDMMC_TMOUT_DATA_SHIFT             8

#define SDMMC_CTYPE_WIDTH4_MASK(slot)      (1 << (slot))
#define SDMMC_CTYPE_WIDTH8_MASK(slot)      (1 << ((slot) + 16))

#define SDMMC_INT_CDET                     (1 << 0)
#define SDMMC_INT_RE                       (1 << 1)
#define SDMMC_INT_CDONE                    (1 << 2)
#define SDMMC_INT_DTO                      (1 << 3)
#define SDMMC_INT_TXDR                     (1 << 4)
#define SDMMC_INT_RXDR                     (1 << 5)
#define SDMMC_INT_RCRC                     (1 << 6)
#define SDMMC_INT_DCRC                     (1 << 7)
#define SDMMC_INT_RTO                      (1 << 8)
#define SDMMC_INT_DRTO                     (1 << 9)
#define SDMMC_INT_HTO                      (1 << 10)
#define SDMMC_INT_FRUN                     (1 << 11)
#define SDMMC_INT_HLE                      (1 << 12)
#define SDMMC_INT_SBE                      (1 << 13)
#define SDMMC_INT_ACD                      (1 << 14)
#define SDMMC_INT_EBE                      (1 << 15)
#define SDMMC_INT_ALL(slot)                (0xffff | (1 << ((slot) + 16)))

#define SDMMC_CMD_STARTCMD                 (1UL << 31)
#define SDMMC_CMD_USE_HOLE                 (1 << 29)
#define SDMMC_CMD_UPDCLOCK                 (1 << 21)
#define SDMMC_CMD_CARD_NUMBER(n)           ((n) << 16)
#define SDMMC_CMD_SENDINIT                 (1 << 15)
#define SDMMC_CMD_STOPABORT                (1 << 14)
#define SDMMC_CMD_WAITPREV                 (1 << 13)
#define SDMMC_CMD_AUTOSTOP                 (1 << 12)
#define SDMMC_CMD_WRITE                    (1 << 10)
#define SDMMC_CMD_DATAXFREXPTD             (1 << 9)
#define SDMMC_CMD_RESPCRC                  (1 << 8)
#define SDMMC_CMD_NORESPONSE               (0 << 6)
#define SDMMC_CMD_SHORTRESPONSE            (1 << 6)
#define SDMMC_CMD_LONGRESPONSE             (3 << 6)

#define SDMMC_STATUS_FIFOEMPTY             (1 << 2)
#define SDMMC_STATUS_FIFOFULL              (1 << 3)

#define SDMMC_FIFOTH_TXWMARK(n)            ((uint32_t)(n))
#define SDMMC_FIFOTH_RXWMARK(n)            (((uint32_t)(n)) << 16)

#define SDMMC_CDETECT_NOTPRESENT(slot)     (1 << (slot))
#define SDMMC_WRTPRT_PROTECTED(slot)       (1 << (slot))

#define SDMMC_BMOD_SWR                     (1 << 0)
#define SDMMC_BMOD_FB                      (1 << 1)
#define SDMMC_BMOD_DE                      (1 << 7)

#define SDMMC_IDSTS_FBE                    (1 << 2)
#define SDMMC_IDSTS_DU                     (1 << 4)
#define SDMMC_IDSTS_AIS                    (1 << 9)
#define SDMMC_IDINTEN_FBE                  (1 << 2)
#define SDMMC_IDINTEN_DU                   (1 << 4)
#define SDMMC_IDINTEN_AIS                  (1 << 9)

#define SDMMC_CLOCK_PHASE_DOUT_SHIFT       0
#define SDMMC_CLOCK_PHASE_MASK             0x000001ff
#define SDMMC_CLOCK_DIV_FACTOR_MASK        0x007fe000
#define SDMMC_CLOCK_DIV_FACTOR_H_SHIFT     9
#define SDMMC_CLOCK_DIV_FACTOR_H_MASK      (0xf << 9)
#define SDMMC_CLOCK_DIV_FACTOR_L_SHIFT     13
#define SDMMC_CLOCK_DIV_FACTOR_L_MASK      (0xf << 13)
#define SDMMC_CLOCK_DIV_FACTOR_N_SHIFT     17
#define SDMMC_CLOCK_DIV_FACTOR_N_MASK      (0xf << 17)
#define SDMMC_CLOCK_CLK_SEL_PLL160M        (1 << 23)

#define ESP32P4_SDMMC_CARDTHRCTL_OFFSET    0x0100
#define ESP32P4_SDMMC_CARDTHRCTL           (ESP32P4_SDMMC_BASE + ESP32P4_SDMMC_CARDTHRCTL_OFFSET)
#define SDMMC_CARDTHRCTL_RDTHREN           (1 << 0)
#define SDMMC_CARDTHRCTL_THRESHOLD_SHIFT   16


#define SDMMC_STATUS_FIFOCNT_S             17
#define SDMMC_STATUS_FIFOCNT_M             0x00001FFFU   

#define SDMMC_CLOCK_PHASE_SAM_SHIFT        3
#define SDMMC_CLOCK_PHASE_SAM_MASK         (0x7 << 3)

#endif /* __ARCH_RISCV_SRC_ESP32P4_HARDWARE_ESP32P4_SDMMC_H */