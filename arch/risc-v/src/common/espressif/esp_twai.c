/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_twai.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/can/can.h>
#include <nuttx/signal.h>

#include "riscv_internal.h"

#include "esp_gpio.h"
#include "esp_twai.h"
#include "esp_irq.h"
#include "esp_clk.h"

#include "periph_ctrl.h"
#include "hal/twai_hal.h"
#include "hal/twai_ll.h"
#include "hal/twai_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/reg_base.h"
#ifdef CONFIG_PM
#  include "include/esp_pm.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CAN_LOOPBACK) && defined(CONFIG_ESPRESSIF_TWAI_TEST_MODE)
#  define TX_PIN_ATTR (OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1)
#  define RX_PIN_ATTR (OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1)
#else
#  define TX_PIN_ATTR OUTPUT_FUNCTION_1
#  define RX_PIN_ATTR INPUT_FUNCTION_1
#endif


#ifdef CONFIG_ESPRESSIF_TWAI0
#  if defined(CONFIG_TWAI0_TIMING_100KBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_100KBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_100KBITS()
#    define TWAI0_BITRATE       100000
#  elif defined(CONFIG_TWAI0_TIMING_125KBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_125KBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI0_BITRATE       125000
#  elif defined(CONFIG_TWAI0_TIMING_250KBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_250KBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_250KBITS()
#    define TWAI0_BITRATE       250000
#  elif defined(CONFIG_TWAI0_TIMING_500KBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_500KBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_500KBITS()
#    define TWAI0_BITRATE       500000
#  elif defined(CONFIG_TWAI0_TIMING_800KBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_800KBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_800KBITS()
#    define TWAI0_BITRATE       800000
#  elif defined(CONFIG_TWAI0_TIMING_1MBITS) || defined(CONFIG_ESPRESSIF_TWAI0_TIMING_1MBITS)
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_1MBITS()
#    define TWAI0_BITRATE       1000000
#  else
#    define TWAI0_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI0_BITRATE       125000
#  endif
#endif

#ifdef CONFIG_ESPRESSIF_TWAI1
#  if defined(CONFIG_TWAI1_TIMING_100KBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_100KBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_100KBITS()
#    define TWAI1_BITRATE       100000
#  elif defined(CONFIG_TWAI1_TIMING_125KBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_125KBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI1_BITRATE       125000
#  elif defined(CONFIG_TWAI1_TIMING_250KBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_250KBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_250KBITS()
#    define TWAI1_BITRATE       250000
#  elif defined(CONFIG_TWAI1_TIMING_500KBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_500KBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_500KBITS()
#    define TWAI1_BITRATE       500000
#  elif defined(CONFIG_TWAI1_TIMING_800KBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_800KBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_800KBITS()
#    define TWAI1_BITRATE       800000
#  elif defined(CONFIG_TWAI1_TIMING_1MBITS) || defined(CONFIG_ESPRESSIF_TWAI1_TIMING_1MBITS)
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_1MBITS()
#    define TWAI1_BITRATE       1000000
#  else
#    define TWAI1_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI1_BITRATE       125000
#  endif
#endif


#ifdef CONFIG_ESPRESSIF_TWAI2
#  if defined(CONFIG_TWAI2_TIMING_100KBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_100KBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_100KBITS()
#    define TWAI2_BITRATE       100000
#  elif defined(CONFIG_TWAI2_TIMING_125KBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_125KBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI2_BITRATE       125000
#  elif defined(CONFIG_TWAI2_TIMING_250KBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_250KBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_250KBITS()
#    define TWAI2_BITRATE       250000
#  elif defined(CONFIG_TWAI2_TIMING_500KBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_500KBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_500KBITS()
#    define TWAI2_BITRATE       500000
#  elif defined(CONFIG_TWAI2_TIMING_800KBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_800KBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_800KBITS()
#    define TWAI2_BITRATE       800000
#  elif defined(CONFIG_TWAI2_TIMING_1MBITS) || defined(CONFIG_ESPRESSIF_TWAI2_TIMING_1MBITS)
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_1MBITS()
#    define TWAI2_BITRATE       1000000
#  else
#    define TWAI2_TIMING_CONFIG TWAI_TIMING_CONFIG_125KBITS()
#    define TWAI2_BITRATE       125000
#  endif
#endif



#if defined(CONFIG_ARCH_CHIP_ESP32C3)
#  define INT_ENA_REG(hw)       hw->interrupt_enable_reg.val
#elif defined(CONFIG_ARCH_CHIP_ESP32P4)
#  define INT_ENA_REG(hw)       hw->interrupt_ena.val
#else
#  define INT_ENA_REG(hw)       hw->interrupt_enable.val
#endif /* CONFIG_ARCH_CHIP_ESP32C3 */

#if !SOC_RCC_IS_INDEPENDENT
#  define TWAI_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define TWAI_RCC_ATOMIC()
#endif

#if SOC_PERIPH_CLK_CTRL_SHARED
#  define TWAI_PERI_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#  define TWAI_PERI_ATOMIC()
#endif

/* Configuration ************************************************************/

#ifndef CONFIG_CAN_EXTID
#  define EXTID 0
#else
#  define EXTID 1
#endif

#ifndef CONFIG_CAN_LOOPBACK
#  define LOOPBACK 0
#else
#  define LOOPBACK 1
#endif

/* Default values written to various registers on initialization */
#define DRIVER_DEFAULT_INTERRUPTS  (TWAI_LL_INTR_RI | TWAI_LL_INTR_TI | TWAI_LL_INTR_EI)

struct esp_twai_dev_s
{
  /* Device configuration */

  uint8_t port;                   /* TWAI port number */
  int8_t cpuint;                  /* CPU interrupt assigned to this TWAI */
  uint32_t bitrate;
  twai_hal_context_t ctx;         /* Context struct of common layer */
  twai_timing_config_t t_config;  /* Timing struct of common layer */
#ifdef CONFIG_PM
  esp_pm_lock_handle_t pm_lock;   /* Power management lock */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TWAI methods */

static void esp_twai_reset(struct can_dev_s *dev);
static int  esp_twai_setup(struct can_dev_s *dev);
static void esp_twai_shutdown(struct can_dev_s *dev);
static void esp_twai_rxint(struct can_dev_s *dev, bool enable);
static void esp_twai_txint(struct can_dev_s *dev, bool enable);
static int  esp_twai_ioctl(struct can_dev_s *dev, int cmd,
                           unsigned long arg);
static int  esp_twai_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  esp_twai_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool esp_twai_txready(struct can_dev_s *dev);
static bool esp_twai_txempty(struct can_dev_s *dev);

/* TWAI interrupts */

static int esp_twai_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_twaiops =
{
  .co_reset         = esp_twai_reset,
  .co_setup         = esp_twai_setup,
  .co_shutdown      = esp_twai_shutdown,
  .co_rxint         = esp_twai_rxint,
  .co_txint         = esp_twai_txint,
  .co_ioctl         = esp_twai_ioctl,
  .co_remoterequest = esp_twai_remoterequest,
  .co_send          = esp_twai_send,
  .co_txready       = esp_twai_txready,
  .co_txempty       = esp_twai_txempty,
};

#ifdef CONFIG_ESPRESSIF_TWAI0
static struct esp_twai_dev_s g_twai0priv =
{
  .port             = 0,
  .cpuint           = -ENOMEM,
  .bitrate          = TWAI0_BITRATE,
  .t_config         = TWAI0_TIMING_CONFIG,
#ifdef CONFIG_PM
  .pm_lock          = NULL,
#endif
};

static struct can_dev_s g_twai0dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai0priv,
};
#endif /* CONFIG_ESPRESSIF_TWAI0 */

#ifdef CONFIG_ESPRESSIF_TWAI1
static struct esp_twai_dev_s g_twai1priv =
{
  .port             = 1,
  .cpuint           = -ENOMEM,
  .bitrate          = TWAI1_BITRATE,
  .t_config         = TWAI1_TIMING_CONFIG,
#ifdef CONFIG_PM
  .pm_lock          = NULL,
#endif
};

static struct can_dev_s g_twai1dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai1priv,
};
#endif /* CONFIG_ESPRESSIF_TWAI1 */

#ifdef CONFIG_ESPRESSIF_TWAI2
static struct esp_twai_dev_s g_twai2priv =
{
  .port             = 2,
  .cpuint           = -ENOMEM,
  .bitrate          = TWAI2_BITRATE,
  .t_config         = TWAI2_TIMING_CONFIG,
#ifdef CONFIG_PM
  .pm_lock          = NULL,
#endif
};

static struct can_dev_s g_twai2dev =
{
  .cd_ops           = &g_twaiops,
  .cd_priv          = &g_twai2priv,
};
#endif /* CONFIG_ESPRESSIF_TWAI2 */

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


static int esp_twai_set_baudrate(struct esp_twai_dev_s *priv, uint32_t rate)
{
  /* On ESP32-P4, TWAI_CLK_SRC_DEFAULT is XTAL = 40MHz */
  uint32_t clock = 40000000;
  int best_error = 1000000000;
  int error;
  int best_tseg = 0;
  int best_brp = 0;
  int brp;
  int tseg;
  int tseg1;
  int tseg2;
  int sampl_pt = 80; /* 80% sample point */
  int sjw = 3;

  for (tseg = (1 + 1 + 2) * 2; tseg <= (8 + 16 + 2) * 2 + 1; tseg++)
    {
      brp = clock / ((1 + tseg / 2) * rate) + (tseg % 2);
      if (brp < 2 || brp > 128 || (brp % 2 != 0))
        {
          continue;
        }

      error = rate - clock / (brp * (1 + tseg / 2));
      if (error < 0)
        {
          error = -error;
        }

      if (error <= best_error)
        {
          best_error = error;
          best_tseg = tseg / 2;
          best_brp = brp;
        }
    }

  if (best_brp == 0)
    {
      canerr("ERROR: Cannot calculate valid CAN timing for rate %" PRIu32 "\n", rate);
      return -EINVAL;
    }

  tseg2 = best_tseg - (sampl_pt * (best_tseg + 1)) / 100 + 1;
  if (tseg2 < 1) tseg2 = 1;
  if (tseg2 > 8) tseg2 = 8;

  tseg1 = best_tseg - tseg2;
  if (tseg1 > 16)
    {
      tseg1 = 16;
      tseg2 = best_tseg - tseg1;
    }

  caninfo("Configuring TWAI Timing: Clock=%" PRIu32 ", Rate=%" PRIu32
        " -> BRP=%d, TS1=%d, TS2=%d, SJW=%d\n",
        clock, rate, best_brp, tseg1, tseg2, sjw);

  /* Update dev structure timing */
  priv->t_config.brp = best_brp;
  priv->t_config.tseg_1 = tseg1;
  priv->t_config.tseg_2 = tseg2;
  priv->t_config.sjw = sjw;
  priv->t_config.triple_sampling = false;

  /* Apply directly via TWAI Low-Level function */
  twai_ll_set_bus_timing(priv->ctx.dev, best_brp, sjw, tseg1, tseg2, false);

  return OK;
}



/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_twai_reset
 *
 * Description:
 *   Reset the TWAI device. Called early to initialize the hardware. This
 *   function is called, before esp_twai_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/
 static void esp_twai_reset(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  twai_hal_config_t hal_config =
    {
       // .clock_source_hz = TWAI_CLK_SRC_DEFAULT,
      .clock_source_hz = 40000000,   /* ESP32-P4 XTAL is 40 MHz */
      .controller_id = priv->port,
      .intr_mask = DRIVER_DEFAULT_INTERRUPTS,
#ifdef CONFIG_CAN_LOOPBACK
      .enable_self_test = true,
#else
      .enable_self_test = false,
#endif
      .enable_listen_only = false,
    };

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  TWAI_RCC_ATOMIC()
    {
      twai_ll_enable_bus_clock(priv->port, true);
      twai_ll_reset_register(priv->port);
    }

  TWAI_PERI_ATOMIC()
    {
      twai_ll_set_clock_source(priv->port, TWAI_CLK_SRC_DEFAULT);
      twai_ll_enable_clock(priv->port, true);
    }

  flags = enter_critical_section();

  ret = twai_hal_init(&priv->ctx, &hal_config);
  // DEBUGASSERT(ret);
  ASSERT(ret);

  /* Configure Acceptance Filter */
  twai_ll_set_acc_filter(priv->ctx.dev, f_config.acceptance_code,
                         f_config.acceptance_mask, f_config.single_filter);

  ret = esp_twai_set_baudrate(priv, priv->bitrate);
  if (ret != OK)
    {
      canerr("ERROR: bad bitrate %" PRIu32 "\n", priv->bitrate);
    }

  /* Start TWAI Hardware */
  twai_hal_start(&priv->ctx);
  twai_ll_set_tec(priv->ctx.dev, 0);
  twai_ll_set_rec(priv->ctx.dev, 0);

  /* Abort ongoing TX, release RX buffers */
  twai_ll_set_cmd_release_rx_buffer(priv->ctx.dev);
  twai_ll_set_cmd_abort_tx(priv->ctx.dev);
  twai_ll_set_cmd_clear_data_overrun(priv->ctx.dev);

  leave_critical_section(flags);
}


/****************************************************************************
 * Name: esp_twai_setup
 *
 * Description:
 *   Configure the TWAI. This method is called the first time that the TWAI
 *   the device is opened and it configures and attaches the TWAI interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int esp_twai_setup(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret = OK;
  int irq;

  caninfo("TWAI%" PRIu8 "\n", priv->port);

  flags = enter_critical_section();

  twai_ll_set_enabled_intrs(priv->ctx.dev, DRIVER_DEFAULT_INTERRUPTS);

  twai_ll_get_and_clear_intrs(priv->ctx.dev); /* clear latched interrupts */

  irq = twai_periph_signals[priv->port].irq_id;

  if (priv->cpuint != -ENOMEM)
    {
      /* Disable the provided CPU Interrupt to configure it. */

      up_disable_irq(ESP_SOURCE2IRQ(irq));
    }

  priv->cpuint = esp_setup_irq(irq,
                               ESP_IRQ_PRIORITY_DEFAULT,
                               ESP_IRQ_TRIGGER_LEVEL,
                               esp_twai_interrupt,
                               dev);
  if (priv->cpuint < 0)
    {
      /* Failed to allocate a CPU interrupt of this type. */

      ret = priv->cpuint;
      leave_critical_section(flags);

      return ret;
    }

  caninfo("irq=%d cpuint=%d\n", irq, priv->cpuint);

  /* Enable the CPU interrupt that is linked to the TWAI device. */

  up_enable_irq(ESP_SOURCE2IRQ(irq));

  leave_critical_section(flags);

  return ret;
}



/****************************************************************************
 * Name: esp_twai_shutdown
 *
 * Description:
 *   Disable the TWAI. This method is called when the TWAI device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void esp_twai_shutdown(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;

#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("shutdown TWAI%" PRIu8 "\n", priv->port);
#endif

  if (priv->cpuint != -ENOMEM)
    {
      int irq;

      irq = twai_periph_signals[priv->port].irq_id;

      /* Disable cpu interrupt */

      up_disable_irq(ESP_SOURCE2IRQ(irq));

      /* Free cpu interrupt that is attached to this peripheral */

      esp_teardown_irq(irq, priv->cpuint);
      priv->cpuint = -ENOMEM;
    }
}



/****************************************************************************
 * Name: esp_twai_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev    - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable receive interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void esp_twai_rxint(struct can_dev_s *dev, bool enable)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  flags = enter_critical_section();

  regval = INT_ENA_REG(priv->ctx.dev);
  if (enable)
    {
      regval |= TWAI_LL_INTR_RI;
    }
  else
    {
      regval &= ~TWAI_LL_INTR_RI;
    }

  twai_ll_set_enabled_intrs(priv->ctx.dev, regval);
  leave_critical_section(flags);
}



/****************************************************************************
 * Name: esp_twai_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable transmit interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void esp_twai_txint(struct can_dev_s *dev, bool enable)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  irqstate_t flags;

  caninfo("TWAI%" PRIu8 " enable: %d\n", priv->port, enable);

  /* Only disabling of the TX interrupt is supported here.  The TX interrupt
   * is automatically enabled just before a message is sent in order to avoid
   * lost TX interrupts.
   */

  if (enable == false)
    {
      /* TX interrupts are also disabled from the interrupt handler, so we
       * have to protect this code section.
       */

      flags = enter_critical_section();

      /* Disable all TX interrupts */

      regval = INT_ENA_REG(priv->ctx.dev);
      regval &= ~TWAI_LL_INTR_TI;
      twai_ll_set_enabled_intrs(priv->ctx.dev, regval);
      leave_critical_section(flags);
    }
}



/****************************************************************************
 * Name: esp_twai_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   cmd - A ioctl command.
 *   arg - A ioctl argument.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int esp_twai_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  int ret = -ENOTTY;

  caninfo("TWAI%" PRIu8 " cmd=%04x arg=%lu\n", priv->port, cmd, arg);

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */
      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;
          uint32_t brp;
          uint32_t clk_hz = 40000000; /* XTAL clock source */

          DEBUGASSERT(bt != NULL);

          /* Read hardware timing values directly */
          brp = (priv->ctx.dev->bus_timing_0.baud_presc + 1) * 2;
          bt->bt_sjw   = priv->ctx.dev->bus_timing_0.sync_jump_width + 1;
          bt->bt_tseg1 = priv->ctx.dev->bus_timing_1.time_segment1 + 1;
          bt->bt_tseg2 = priv->ctx.dev->bus_timing_1.time_segment2 + 1;

          bt->bt_baud = clk_hz / (brp * (bt->bt_tseg1 + bt->bt_tseg2 + 1));

          ret = OK;
        }
        break;

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  return ret;
}



/****************************************************************************
 * Name: esp_twai_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *   id  - Requested 11-bit data frame identifier
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int esp_twai_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canwarn("Remote request not implemented\n");
  return -ENOSYS;
}


/****************************************************************************
 * Name: esp_twai_send
 *
 * Description:
 *    Send one TWAI message.
 *
 *    One TWAI-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit TWAI identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit TWAI identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: TWAI data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   msg - A message to send.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
static int esp_twai_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct esp_twai_dev_s *priv = (struct esp_twai_dev_s *)dev->cd_priv;
  uint32_t regval;
  uint32_t len;
  uint32_t id;
  uint32_t frame_info;
  irqstate_t flags;
  uint32_t i;
  int ret = OK;

  caninfo("TWAI%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  len = (uint32_t)msg->cm_hdr.ch_dlc;
  if (len > CAN_MAXDATALEN)
    {
      len = CAN_MAXDATALEN;
    }

  frame_info = len;
  if (msg->cm_hdr.ch_rtr)
    {
      frame_info |= (1 << 6);
    }

  flags = enter_critical_section();

  /* Make sure TX interrupt is enabled */

  regval = INT_ENA_REG(priv->ctx.dev);
  regval |= TWAI_LL_INTR_TI;
  twai_ll_set_enabled_intrs(priv->ctx.dev, regval);

  /* Set up frame format */

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      /* 29-bit Extended Frame */
      frame_info |= (1 << 7);
      priv->ctx.dev->tx_rx_buffer[0].val = frame_info;

      id = (uint32_t)msg->cm_hdr.ch_id << 3;
      priv->ctx.dev->tx_rx_buffer[4].val = id & 0xff;
      id >>= 8;
      priv->ctx.dev->tx_rx_buffer[3].val = id & 0xff;
      id >>= 8;
      priv->ctx.dev->tx_rx_buffer[2].val = id & 0xff;
      id >>= 8;
      priv->ctx.dev->tx_rx_buffer[1].val = id & 0xff;

      for (i = 0; !msg->cm_hdr.ch_rtr && i < len; i++)
        {
          priv->ctx.dev->tx_rx_buffer[5 + i].val = msg->cm_data[i];
        }
    }
  else
#endif
    {
      /* 11-bit Standard Frame */
      priv->ctx.dev->tx_rx_buffer[0].val = frame_info;

      id = (uint32_t)msg->cm_hdr.ch_id << 5;
      priv->ctx.dev->tx_rx_buffer[1].val = (id >> 8) & 0xff;
      priv->ctx.dev->tx_rx_buffer[2].val = id & 0xff;

      for (i = 0; !msg->cm_hdr.ch_rtr && i < len; i++)
        {
          priv->ctx.dev->tx_rx_buffer[3 + i].val = msg->cm_data[i];
        }
    }

  /* Trigger Transmission */

#ifdef CONFIG_CAN_LOOPBACK
  twai_ll_set_cmd_self_rx_request(priv->ctx.dev);
#else
  twai_ll_set_cmd_tx(priv->ctx.dev);
#endif

  leave_critical_section(flags);

  return ret;
}


/****************************************************************************
 * Name: esp_twai_txready
 *
 * Description:
 *   Return true if the TWAI hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if the TWAI hardware is ready to accept another TX message.
 *
 ****************************************************************************/
static bool esp_twai_txready(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_ll_get_status(priv->ctx.dev);
  caninfo("TWAI%" PRIu8 " txready: %d\n", priv->port,
         ((regval & TWAI_LL_STATUS_TBS) != 0));
  return ((regval & TWAI_LL_STATUS_TBS) != 0);
}



/****************************************************************************
 * Name: esp_twai_txempty
 *
 * Description:
 *   Return true if all message have been sent. If for example, the TWAI
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the TWAI hardware.
 *
 ****************************************************************************/
static bool esp_twai_txempty(struct can_dev_s *dev)
{
  struct esp_twai_dev_s *priv = dev->cd_priv;
  uint32_t regval = twai_ll_get_status(priv->ctx.dev);

  caninfo("TWAI%" PRIu8 " txempty: %d\n", priv->port,
         ((regval & TWAI_LL_STATUS_TBS) != 0));
  return ((regval & TWAI_LL_STATUS_TBS) != 0);
}



/****************************************************************************
 * Name: esp_twai_interrupt
 *
 * Description:
 *   TWAI RX/TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - The pointer to driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/
// static int esp_twai_interrupt(int irq, void *context, void *arg)
// {
//   struct can_dev_s *dev = (struct can_dev_s *)arg;
//   struct esp_twai_dev_s *priv = dev->cd_priv;
//   struct can_hdr_s hdr;
//   uint8_t data[8];
//   uint32_t regval;
//   twai_hal_frame_t rx_frame;

//   /* Read and clear interrupt register */

//   regval = twai_ll_get_and_clear_intrs(priv->ctx.dev);

//   /* Check for a receive interrupt */

//   if ((regval & TWAI_LL_INTR_RI) != 0)
//     {
//       twai_frame_header_t header;

//       memset(&hdr, 0, sizeof(hdr));
//       memset(data, 0, sizeof(data));

//       /* Copy hardware buffer into rx_frame */

//       twai_ll_get_rx_buffer(priv->ctx.dev, &rx_frame);

//       /* Release the receive buffer immediately */

//       twai_ll_set_cmd_release_rx_buffer(priv->ctx.dev);

//       /* Parse the frame */

//       twai_hal_parse_frame(&priv->ctx, &rx_frame, &header, data,
//                            TWAI_FRAME_MAX_LEN);

//       /* Fill NuttX CAN header correctly */

//       hdr.ch_id     = header.id;
//       hdr.ch_dlc    = header.dlc > CAN_MAXDATALEN ? CAN_MAXDATALEN : header.dlc;
//       hdr.ch_rtr    = header.rtr ? 1 : 0;
// #ifdef CONFIG_CAN_EXTID
//       hdr.ch_extid  = header.ide;
// #endif
// #ifdef CONFIG_CAN_ERRORS
//       hdr.ch_error  = 0;  /* <-- MUST BE EXPLICITLY ZERO, otherwise it treats normal frames as error frames! */
// #endif

//       /* Deliver valid payload to NuttX CAN subsystem */

//       can_receive(dev, &hdr, data);
//     }

//   /* Check for TX buffer complete */

//   if ((regval & TWAI_LL_INTR_TI) != 0)
//     {
//       /* Disable all further TX buffer interrupts */

//       esp_twai_txint(dev, false);

//       /* Indicate that the TX is done and a new TX buffer is available */

//       can_txdone(dev);
//     }

//   return OK;
// }



static int esp_twai_interrupt(int irq, void *context, void *arg)
{
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct esp_twai_dev_s *priv = dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[8];
  uint32_t regval;
  uint32_t status;
  uint32_t budget;
  twai_hal_frame_t rx_frame;

  /* Read and clear the interrupt register */

  regval = twai_ll_get_and_clear_intrs(priv->ctx.dev);

  /* Receive: drain the whole FIFO, not just one frame */

  if ((regval & TWAI_LL_INTR_RI) != 0)
    {
      budget = 64;   /* Bound the loop so a flood can't starve the CPU */

      while (budget-- > 0 && twai_hal_get_rx_msg_count(&priv->ctx) > 0)
        {
          twai_frame_header_t header;

          /* Reads the frame and releases the buffer.  Returns false for
           * an overrun frame, which is dropped.
           */

          if (!twai_hal_read_rx_fifo(&priv->ctx, &rx_frame))
            {
              continue;
            }

          memset(&hdr, 0, sizeof(hdr));
          memset(data, 0, sizeof(data));

          twai_hal_parse_frame(&priv->ctx, &rx_frame, &header, data,
                               TWAI_FRAME_MAX_LEN);

          hdr.ch_id  = header.id;
          hdr.ch_dlc = header.dlc > CAN_MAXDATALEN ?
                       CAN_MAXDATALEN : header.dlc;
          hdr.ch_rtr = header.rtr ? 1 : 0;
#ifdef CONFIG_CAN_EXTID
          hdr.ch_extid = header.ide;
#endif
#ifdef CONFIG_CAN_ERRORS
          hdr.ch_error = 0;
#endif

          can_receive(dev, &hdr, data);
        }
    }

  /* Transmit complete */

  if ((regval & TWAI_LL_INTR_TI) != 0)
    {
      esp_twai_txint(dev, false);
      can_txdone(dev);
    }

  /* Error warning / bus status change */

  if ((regval & TWAI_LL_INTR_EI) != 0)
    {
      status = twai_ll_get_status(priv->ctx.dev);

      /* BS and ES both set means the controller just went bus-off (same
       * test the Espressif HAL uses).  BS set with ES clear means a
       * recovery is already in progress, and nothing is needed.
       */

      if ((status & TWAI_LL_STATUS_BS) != 0 &&
          (status & TWAI_LL_STATUS_ES) != 0)
        {
          /* The controller is already in reset mode.  Leaving it starts
           * the standard recovery (128 x 11 recessive bits).  EI fires
           * again when BS clears.
           */

          twai_hal_start_bus_recovery(&priv->ctx);
        }
    }

  return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_twaiinitialize
 *
 * Description:
 *   Initialize TWAI peripheral
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Valid TWAI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/
struct can_dev_s *esp_twaiinitialize(int port)
{
  struct can_dev_s *dev;
  irqstate_t flags;
#ifdef CONFIG_PM
  int ret;
  struct esp_twai_dev_s *priv;
#endif

  caninfo("TWAI%" PRIu8 "\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_ESPRESSIF_TWAI0
  if (port == 0)
    {
      int tx_sig = twai_periph_signals[0].tx_sig;
      int rx_sig = twai_periph_signals[0].rx_sig;

      /* Configure CAN GPIO pins */
      esp_configgpio(CONFIG_ESPRESSIF_TWAI0_TXPIN, OUTPUT_FUNCTION_2);
      esp_gpio_matrix_out(CONFIG_ESPRESSIF_TWAI0_TXPIN, tx_sig, 0, 0);

      esp_configgpio(CONFIG_ESPRESSIF_TWAI0_RXPIN, INPUT_FUNCTION_2);
      esp_gpio_matrix_in(CONFIG_ESPRESSIF_TWAI0_RXPIN, rx_sig, 0);


      dev = &g_twai0dev;
    }
  else
#endif


#ifdef CONFIG_ESPRESSIF_TWAI1
  if (port == 1)
    {
      int tx_sig = twai_periph_signals[1].tx_sig;
      int rx_sig = twai_periph_signals[1].rx_sig;

      /* Configure CAN GPIO pins for TWAI1 */

      esp_configgpio(CONFIG_ESPRESSIF_TWAI1_TXPIN, OUTPUT_FUNCTION_2);
      esp_gpio_matrix_out(CONFIG_ESPRESSIF_TWAI1_TXPIN, tx_sig, 0, 0);

      esp_configgpio(CONFIG_ESPRESSIF_TWAI1_RXPIN, INPUT_FUNCTION_2);
      esp_gpio_matrix_in(CONFIG_ESPRESSIF_TWAI1_RXPIN, rx_sig, 0);

      caninfo("tx_sig=%d rx_sig=%d\n", tx_sig, rx_sig);
      dev = &g_twai1dev;
    }
  else
#endif

#ifdef CONFIG_ESPRESSIF_TWAI2
  if (port == 2)
    {
      int tx_sig = twai_periph_signals[2].tx_sig;
      int rx_sig = twai_periph_signals[2].rx_sig;

      /* Configure CAN GPIO pins for TWAI2 */

      esp_configgpio(CONFIG_ESPRESSIF_TWAI2_TXPIN, OUTPUT_FUNCTION_2);
      esp_gpio_matrix_out(CONFIG_ESPRESSIF_TWAI2_TXPIN, tx_sig, 0, 0);

      esp_configgpio(CONFIG_ESPRESSIF_TWAI2_RXPIN, INPUT_FUNCTION_2);
      esp_gpio_matrix_in(CONFIG_ESPRESSIF_TWAI2_RXPIN, rx_sig, 0);

      caninfo("tx_sig=%d rx_sig=%d\n", tx_sig, rx_sig);
      dev = &g_twai2dev;
    }
  else
#endif

    {
      canerr("ERROR: Unsupported port: %d\n", port);
      leave_critical_section(flags);
      return NULL;
    }

#ifdef CONFIG_PM
  priv = (struct esp_twai_dev_s *)dev->cd_priv;

  if (priv->pm_lock == NULL)
    {
#  if TWAI_LL_SUPPORT(APB_CLK)
      if (TWAI_CLK_SRC_DEFAULT == TWAI_CLK_SRC_APB)
        {
          ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX,
                                   0,
                                   twai_periph_signals[port].module_name,
                                   &priv->pm_lock);
        }
#  else
      ret = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP,
                               0,
                               twai_periph_signals[port].module_name,
                               &priv->pm_lock);
#  endif
      if (ret != OK)
        {
          canerr("Failed to create TWAI%" PRIu8 "PM lock\n", port);
          leave_critical_section(flags);
          return NULL;
        }
    }
#endif

  /* Then just perform a TWAI reset operation */

  esp_twai_reset(dev);

  leave_critical_section(flags);

  return dev;
}
