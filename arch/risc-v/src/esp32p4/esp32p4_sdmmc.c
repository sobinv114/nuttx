/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_sdmmc.c
 *
 * ESP32-P4 SDMMC host driver for NuttX (DMA version).
 *
 *   - Slot 0 only, fixed IO_MUX pins:
 *       CLK=43 CMD=44 D0=39 D1=40 D2=41 D3=42
 *   - Data moves through the DesignWare internal DMA controller (IDMAC),
 *     exactly like ESP-IDF.  The CPU cannot pop the FIFO on this chip
 *     (verified: repeated reads return the first word), so PIO is NOT used.
 *   - All transfers go through a 64-byte aligned internal-RAM bounce buffer
 *     (FAT/mmcsd buffers are not cache-line aligned).
 *   - Cache maintenance uses esp_cache_msync() like ESP-IDF does.
 *
 * Required menuconfig:
 *   CONFIG_ESP32P4_SDMMC=y
 *   CONFIG_SDIO_BLOCKSETUP=y         (MANDATORY)
 *   CONFIG_SDIO_DMA=y                (or CONFIG_ESP32P4_SDMMC_DMA=y)
 *   CONFIG_SDIO_WIDTH_D1_ONLY=y      (recommended for first bring-up)
 *   CONFIG_MMCSD_MULTIBLOCK_LIMIT=1  (recommended for first bring-up)
 *
 * Board code must call  sdio_initialize(0)  after powering the card
 * (override esp32p4_sdmmc_power_on() for the on-chip LDO).
 *
 * Licensed under the Apache License, Version 2.0.
 ****************************************************************************/

#include <nuttx/config.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/debug.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include "soc/hp_sys_clkrst_reg.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/io_mux_reg.h"
#include "riscv_internal.h"
#include "esp_gpio.h"
#include "espressif/esp_irq.h"
#include "hardware/esp32p4_sdmmc.h"
#include "hardware/esp32p4_gpio_sigmap.h"
#include "esp32p4_sdmmc.h"


#ifdef CONFIG_ESPRESSIF_LDO
#include "esp_ldo_regulator.h"
#endif


#if defined(__has_include)
#  if __has_include("esp_cache.h")
#    include "esp_cache.h"
#    define SDMMC_HAVE_ESP_CACHE 1
#  endif
#endif

#ifdef CONFIG_ESP32P4_SDMMC

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "ESP32-P4 SDMMC needs CONFIG_SDIO_BLOCKSETUP=y (BYTCNT/BLKSIZ must be set before the command)"
#endif

/****************************************************************************
 * Cache maintenance (data buffers are in cached internal RAM on the P4)
 ****************************************************************************/

#if defined(ESP32P4_SDMMC_NO_CACHE_OPS)
#  define SD_CACHE_CLEAN(a, n)  do { } while (0)
#  define SD_CACHE_INVAL(a, n)  do { } while (0)
#elif defined(SDMMC_HAVE_ESP_CACHE)
#  define SD_CACHE_CLEAN(a, n)  \
     ((void)esp_cache_msync((void *)(a), (n), ESP_CACHE_MSYNC_FLAG_DIR_C2M))
#  define SD_CACHE_INVAL(a, n)  \
     ((void)esp_cache_msync((void *)(a), (n), ESP_CACHE_MSYNC_FLAG_DIR_M2C))
#elif defined(CONFIG_ARCH_DCACHE)
#  define SD_CACHE_CLEAN(a, n)  \
     up_clean_dcache((uintptr_t)(a), (uintptr_t)(a) + (n))
#  define SD_CACHE_INVAL(a, n)  \
     up_invalidate_dcache((uintptr_t)(a), (uintptr_t)(a) + (n))
#else
#  error "No cache maintenance available (esp_cache.h / CONFIG_ARCH_DCACHE). Define ESP32P4_SDMMC_NO_CACHE_OPS to build without (unsafe unless buffers are uncached)."
#endif

/****************************************************************************
 * Tunables
 ****************************************************************************/

/* Card clock for identification and for data transfer.  Start with 400 kHz,
 * then raise XFER to 10000 and later 20000.
 */

 #define SD_STATUS_DATA_BUSY        (1U << 9)  /* Bit 9 in STATUS register: Card busy on DAT0 */

#ifndef ESP32P4_SDMMC_ID_KHZ
#  define ESP32P4_SDMMC_ID_KHZ     400
#endif



#ifndef ESP32P4_SDMMC_DS_KHZ
#  define ESP32P4_SDMMC_DS_KHZ   50000   /* default speed (spec max 25 MHz) */
#endif
#ifndef ESP32P4_SDMMC_HS_KHZ
#  define ESP32P4_SDMMC_HS_KHZ   50000   /* maps to 160/3 = 53.33 MHz */
#endif


#ifndef ESP32P4_SDMMC_TRACE_DATA
#  define ESP32P4_SDMMC_TRACE_DATA 1     /* print first bytes of 8 reads */
#endif

#define ESP32P4_SDMMC_SLOT         0      /* only slot 0 (IO_MUX) */

/* Bounce buffer = largest single transfer (blocks * 512).  One descriptor
 * can carry at most 4096 bytes.
 */
#define ESP32P4_SDMMC_BOUNCE_SZ    (32 * 1024)      /* 32 KB buffer */
#define ESP32P4_SDMMC_NUM_DESC     (ESP32P4_SDMMC_BOUNCE_SZ / 4096) /* 8 descriptors */
#define SD_DMA_ALIGN               64


#if ESP32P4_SDMMC_BOUNCE_SZ > (ESP32P4_SDMMC_NUM_DESC * 4096)
#  error "Bounce buffer exceeds maximum capacity of the descriptor chain"
#endif

/****************************************************************************
 * Constants
 ****************************************************************************/

#define SDMMC_SRC_CLK_HZ           (160 * 1000 * 1000)

#define ESP32P4_SDMMC_VERID        (ESP32P4_SDMMC_BASE + 0x6c)
#define ESP32P4_SDMMC_HCON         (ESP32P4_SDMMC_BASE + 0x70)

/* LP clock/reset block: hp_sdmmc_emac_rst_ctrl, rst_en_sdmmc = bit 28 */

#define LP_CLKRST_BASE                    0x50111000
#define LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL  (LP_CLKRST_BASE + 0x4c)
#define LP_RST_EN_SDMMC                   (1U << 28)

#define ESP32P4_HP_CLKRST  ((hp_sys_clkrst_dev_t *)DR_REG_HP_SYS_CLKRST_BASE)

#define SDCARD_CMDTIMEOUT          MSEC2TICK(500)   /* Increased from 100 to 500 ms */
#define SDCARD_LONGTIMEOUT         MSEC2TICK(2000)

/* Interrupt groups (RINTSTS / INTMASK) */

#define SDCARD_INT_RESPERR   (SDMMC_INT_RE | SDMMC_INT_RCRC | SDMMC_INT_RTO)
#define SD_DATA_ERR_INT      (SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                              SDMMC_INT_EBE  | SDMMC_INT_SBE  | \
                              SDMMC_INT_FRUN)
#define SD_DMA_XFR_MASK      (SDMMC_INT_DTO | SD_DATA_ERR_INT)
#define SD_DATA_ALL_RINT     (SDMMC_INT_DTO | SDMMC_INT_RXDR | \
                              SDMMC_INT_TXDR | SDMMC_INT_HTO | \
                              SD_DATA_ERR_INT)

#define SDCARD_CMDDONE_MASK  (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_MASK (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  define SDCARD_INT_CDET    SDMMC_INT_CDET
#else
#  define SDCARD_INT_CDET    0
#endif

/* IDMAC status / interrupt-enable bits (IDSTS / IDINTEN) */

#define SD_ID_TI             (1U << 0)
#define SD_ID_RI             (1U << 1)
#define SD_ID_FBE            (1U << 2)
#define SD_ID_DU             (1U << 4)
#define SD_ID_CES            (1U << 5)
#define SD_ID_NI             (1U << 8)
#define SD_ID_AI             (1U << 9)
#define SD_ID_IRQS           (SD_ID_TI | SD_ID_RI | SD_ID_NI | \
                              SD_ID_FBE | SD_ID_DU | SD_ID_AI)

/* IDMAC descriptor DES0 bits (DesignWare, chain mode) */

#define SD_DES0_DIC          (1U << 1)    /* disable interrupt on completion */
#define SD_DES0_LD           (1U << 2)    /* last descriptor  */
#define SD_DES0_FS           (1U << 3)    /* first descriptor */
#define SD_DES0_CH           (1U << 4)    /* second address chained */
#define SD_DES0_OWN          (1U << 31)   /* owned by IDMAC */

#define SDRD(r)       getreg32(r)
#define SDWR(v, r)    putreg32((v), (r))

/****************************************************************************
 * Private types
 ****************************************************************************/

/* One IDMAC descriptor, padded to a full 64-byte cache line (as IDF) */

struct esp32p4_idmac_desc_s
{
  volatile uint32_t des0;     /* control / status */
  volatile uint32_t des1;     /* buffer sizes (BS1 = bits 12:0) */
  volatile uint32_t des2;     /* buffer 1 address */
  volatile uint32_t des3;     /* buffer 2 address / next descriptor */
  uint8_t           pad[48];
};

struct esp32p4_dev_s
{
  struct sdio_dev_s  dev;              /* Must be first */

  bool               is_bounce;        /* True if transfer uses g_bounce */

  /* --- Add details and statistics tracking below --- */
  uint32_t           cur_freq_khz;     /* Current operating frequency in kHz */
  uint8_t            cur_bus_width;    /* 1 or 4 bits */
  uint64_t           total_rx_bytes;   /* Total bytes read via DMA */
  uint64_t           total_tx_bytes;   /* Total bytes written via DMA */
  uint32_t           total_cmds;       /* Total SD commands sent */
  uint32_t           total_errs;       /* Total errors encountered */

  sem_t              waitsem;
  sdio_eventset_t    waitevents;
  uint32_t           waitmask;
  volatile sdio_eventset_t wkupevent;
  volatile sdio_eventset_t xfr_result;   /* latched transfer result */
  struct wdog_s      waitwdog;

  sdio_statset_t     cdstatus;
#ifdef CONFIG_SDIO_HAVE_CALLBACK
  sdio_eventset_t    cbevents;
  worker_t           callback;
  void              *cbarg;
  struct work_s      cbwork;
#endif

  uint8_t           *usrbuf;           /* caller's buffer */
  size_t             xfrlen;           /* transfer length in bytes */
  uint32_t           xfrmask;          /* data interrupt mask */
  volatile bool      data_over;        /* DTO seen */
  volatile bool      dma_done;         /* IDMAC TI/RI seen */
  volatile bool      dma_err;          /* IDMAC FBE/DU seen */
  bool               wrdir;            /* true = write to card */
  bool               attached;
  int                slot;
};

/****************************************************************************
 * Private function prototypes
 ****************************************************************************/

static void esp32p4_eventtimeout(wdparm_t arg);
static void esp32p4_endwait(struct esp32p4_dev_s *priv,
                            sdio_eventset_t wkupevent);
static void esp32p4_endtransfer(struct esp32p4_dev_s *priv,
                                sdio_eventset_t wkupevent);
static int  esp32p4_interrupt(int irq, void *context, void *arg);

static void esp32p4_reset(struct sdio_dev_s *dev);
static sdio_capset_t esp32p4_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t esp32p4_status(struct sdio_dev_s *dev);
static void esp32p4_widebus(struct sdio_dev_s *dev, bool enable);
static void esp32p4_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int  esp32p4_attach(struct sdio_dev_s *dev);
static int  esp32p4_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t arg);
static void esp32p4_blocksetup(struct sdio_dev_s *dev,
                               unsigned int blocklen, unsigned int nblocks);
static int  esp32p4_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                              size_t nbytes);
static int  esp32p4_sendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t nbytes);
static int  esp32p4_cancel(struct sdio_dev_s *dev);
static int  esp32p4_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  esp32p4_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                 uint32_t *rshort);
static int  esp32p4_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t rlong[4]);
static int  esp32p4_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort);
static void esp32p4_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t esp32p4_eventwait(struct sdio_dev_s *dev);

#ifdef CONFIG_SDIO_HAVE_CALLBACK
static void esp32p4_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset);
// static int  esp32p4_registercallback(struct sdio_dev_s *dev,
//                                      worker_t callback, void *arg);
static void esp32p4_callback(void *arg);

static int esp32p4_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg)
{
  /* ESP32-P4 slot has no card-detect pin: callbacks not supported */
  return -ENOSYS;
}

#endif

#ifdef CONFIG_SDIO_DMA
static bool esp32p4_dmasupported(struct sdio_dev_s *dev);
static int  esp32p4_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                 size_t buflen);
static int  esp32p4_dmasendsetup(struct sdio_dev_s *dev,
                                 const uint8_t *buffer, size_t buflen);
#endif

static int esp32p4_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg)
{
  /* ESP32-P4 slot has no card-detect pin: callbacks not supported */
  return -ENOSYS;
}

/****************************************************************************
 * Private data
 ****************************************************************************/

/* DMA descriptor chain and bounce buffer: internal RAM, cache-line aligned */
static struct esp32p4_idmac_desc_s g_desc[ESP32P4_SDMMC_NUM_DESC]
  __attribute__((aligned(SD_DMA_ALIGN)));
static uint8_t g_bounce[ESP32P4_SDMMC_BOUNCE_SZ]
  __attribute__((aligned(SD_DMA_ALIGN)));


static struct esp32p4_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = esp32p4_reset,
    .capabilities     = esp32p4_capabilities,
    .status           = esp32p4_status,
    .widebus          = esp32p4_widebus,
    .clock            = esp32p4_clock,
    .attach           = esp32p4_attach,
    .sendcmd          = esp32p4_sendcmd,
    .blocksetup       = esp32p4_blocksetup,
    .recvsetup        = esp32p4_recvsetup,
    .sendsetup        = esp32p4_sendsetup,
    .cancel           = esp32p4_cancel,
    .waitresponse     = esp32p4_waitresponse,
    .recv_r1          = esp32p4_recvshortcrc,
    .recv_r2          = esp32p4_recvlong,
    .recv_r3          = esp32p4_recvshort,
    .recv_r4          = esp32p4_recvshort,
    .recv_r5          = esp32p4_recvshortcrc,
    .recv_r6          = esp32p4_recvshortcrc,
    .recv_r7          = esp32p4_recvshort,
    .waitenable       = esp32p4_waitenable,
    .eventwait        = esp32p4_eventwait,
    .registercallback = esp32p4_registercallback,
#ifdef CONFIG_SDIO_HAVE_CALLBACK
    .callbackenable   = esp32p4_callbackenable,
    .registercallback = esp32p4_registercallback,
#endif
#ifdef CONFIG_SDIO_DMA
    // .dmasupported     = esp32p4_dmasupported,
    .dmarecvsetup     = esp32p4_dmarecvsetup,
    .dmasendsetup     = esp32p4_dmasendsetup,
#endif
  },
  .waitsem = SEM_INITIALIZER(0),
};

/* Slot 0 IO_MUX pins (function 0).  Second field: enable internal pull-up */

struct esp32p4_pin_s
{
  uint32_t reg;
  bool     pullup;
};

static const struct esp32p4_pin_s g_sdmmc_pins[] =
{
  { IO_MUX_GPIO43_REG, false },   /* CLK */
  { IO_MUX_GPIO44_REG, true  },   /* CMD */
  { IO_MUX_GPIO39_REG, true  },   /* D0  */
  { IO_MUX_GPIO40_REG, true  },   /* D1  */
  { IO_MUX_GPIO41_REG, true  },   /* D2  */
  { IO_MUX_GPIO42_REG, true  },   /* D3  */
};

/****************************************************************************
 * Board hook: card power
 *
 * On the ESP32-P4 Function-EV-Board the SD socket is powered by the on-chip
 * LDO (channel 4, 3.3 V).  Override this function in board code.
 ****************************************************************************/

// void __attribute__((weak)) esp32p4_sdmmc_power_on(void)
// {
// }





void __attribute__((weak)) esp32p4_sdmmc_power_on(void)
{
#ifdef CONFIG_ESPRESSIF_LDO
  esp_ldo_channel_handle_t ldo_hdl = NULL;
  esp_ldo_channel_config_t ldo_config = {
    .chan_id = 4,           /* Channel 4 for SD card slot */
    .voltage_mv = 3300,     /* 3.3V output */
  };

  int ret = esp_ldo_acquire_channel(&ldo_config, &ldo_hdl);
  if (ret != 0)
    {
      syslog(LOG_ERR, "SDMMC: Failed to acquire LDO channel 4: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "SDMMC: SD Card LDO 4 successfully acquired and powered on!\n");
      up_mdelay(100);
    }
#endif
}


/****************************************************************************
 * Private functions: debug
 ****************************************************************************/

static void esp32p4_dump(const char *tag)
{
  syslog(LOG_ERR,
         "SDMMC[%s] CTRL=%08" PRIx32 " STATUS=%08" PRIx32
         " RINTSTS=%08" PRIx32 " INTMASK=%08" PRIx32
         " IDSTS=%08" PRIx32 " CLKENA=%08" PRIx32
         " CLKDIV=%08" PRIx32 " CMD=%08" PRIx32 "\n",
         tag,
         SDRD(ESP32P4_SDMMC_CTRL),
         SDRD(ESP32P4_SDMMC_STATUS),
         SDRD(ESP32P4_SDMMC_RINTSTS),
         SDRD(ESP32P4_SDMMC_INTMASK),
         SDRD(ESP32P4_SDMMC_IDSTS),
         SDRD(ESP32P4_SDMMC_CLKENA),
         SDRD(ESP32P4_SDMMC_CLKDIV),
         SDRD(ESP32P4_SDMMC_CMD));
}

/****************************************************************************
 * Private functions: reset helpers
 ****************************************************************************/

static int esp32p4_ctrl_reset(uint32_t bits)
{
  int i;

  SDWR(SDRD(ESP32P4_SDMMC_CTRL) | bits, ESP32P4_SDMMC_CTRL);

  for (i = 0; i < 10000; i++)
    {
      if ((SDRD(ESP32P4_SDMMC_CTRL) & bits) == 0)
        {
          return OK;
        }

      up_udelay(1);
    }

  syslog(LOG_ERR, "SDMMC: reset 0x%02" PRIx32 " did not clear\n", bits);
  return -ETIMEDOUT;
}

static void esp32p4_fifo_reset(void)
{
  esp32p4_ctrl_reset(SDMMC_CTRL_FIFORESET);
}

/****************************************************************************
 * Private functions: IDMAC control (same sequence as sdmmc_ll.h)
 ****************************************************************************/

/* sdmmc_ll_init_dma(): dma_enable=1, bmod=0, bmod.sw_reset, idinten */

static void esp32p4_dma_init(void)
{
  int i;

  SDWR(SDRD(ESP32P4_SDMMC_CTRL) | SDMMC_CTRL_DMAENABLE,
       ESP32P4_SDMMC_CTRL);
  SDWR(0, ESP32P4_SDMMC_BMOD);
  SDWR(SDMMC_BMOD_SWR, ESP32P4_SDMMC_BMOD);

  for (i = 0; i < 1000; i++)
    {
      if ((SDRD(ESP32P4_SDMMC_BMOD) & SDMMC_BMOD_SWR) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);
  SDWR(SD_ID_IRQS, ESP32P4_SDMMC_IDINTEN);
}

/* sdmmc_ll_enable_dma(true) */

static void esp32p4_dma_enable(void)
{
  SDWR(SDRD(ESP32P4_SDMMC_CTRL) | SDMMC_CTRL_DMAENABLE | SDMMC_CTRL_INTDMA,
       ESP32P4_SDMMC_CTRL);
  SDWR(SDRD(ESP32P4_SDMMC_BMOD) | SDMMC_BMOD_DE | SDMMC_BMOD_FB,
       ESP32P4_SDMMC_BMOD);
}

/* sdmmc_ll_stop_dma() */

static void esp32p4_dma_stop(void)
{
  SDWR(SDRD(ESP32P4_SDMMC_CTRL) & ~SDMMC_CTRL_INTDMA, ESP32P4_SDMMC_CTRL);
  esp32p4_ctrl_reset(SDMMC_CTRL_DMARESET);
  SDWR(SDRD(ESP32P4_SDMMC_BMOD) & ~(SDMMC_BMOD_DE | SDMMC_BMOD_FB),
       ESP32P4_SDMMC_BMOD);
}


/* Returns true if the buffer can be directly DMA'd without a bounce buffer */

static inline bool esp32p4_can_dma(const void *buf, size_t len)
{

  return false;          /* TEST: always use the bounce buffer */

  uintptr_t addr = (uintptr_t)buf;

  /* Must be 64-byte cache-line aligned in address and length */

  if ((addr & (SD_DMA_ALIGN - 1)) != 0 || (len & (SD_DMA_ALIGN - 1)) != 0)
    {
      return false;
    }

  /* Must be in Internal SRAM (0x4ff00000 - 0x4ffc0000).
   * External PSRAM (0x48000000) CANNOT be directly accessed by IDMAC.
   */

  if (addr >= 0x4ff00000 && (addr + len) <= 0x4ffc0000)
    {
      return true;
    }

  return false;
}

/* Prepare one DMA transfer of 'nbytes' through the bounce buffer. */
static int esp32p4_dma_start(struct esp32p4_dev_s *priv, uint8_t *dma_buf,
                             size_t nbytes)
{
  size_t sync = (nbytes + SD_DMA_ALIGN - 1) & ~((size_t)SD_DMA_ALIGN - 1);
  size_t rem;
  uint32_t cur_buf;
  int i;
  int num_desc;

  if (nbytes == 0 || nbytes > ESP32P4_SDMMC_NUM_DESC * 4096 || (priv->is_bounce && nbytes > ESP32P4_SDMMC_BOUNCE_SZ))
     {
      syslog(LOG_ERR, "SDMMC: DMA length %zu not supported\n", nbytes);
      return -EINVAL;
    }
  /* Drop stale status */

  SDWR(SD_DATA_ALL_RINT, ESP32P4_SDMMC_RINTSTS);
  SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);
  priv->data_over  = false;
  priv->dma_done   = false;
  priv->dma_err    = false;
  priv->xfr_result = 0;

  if (!priv->wrdir)
    {
      SD_CACHE_INVAL(dma_buf, sync);
    }

  /* Populate chained descriptors */

  rem      = nbytes;
  cur_buf  = (uint32_t)(uintptr_t)dma_buf;
  num_desc = (nbytes + 4095) / 4096;

  for (i = 0; i < num_desc; i++)
    {
      size_t chunk = (rem > 4096) ? 4096 : rem;

      g_desc[i].des0 = 0;
      g_desc[i].des1 = (uint32_t)((chunk + 3) & ~3u);
      g_desc[i].des2 = cur_buf;
      g_desc[i].des3 = (uint32_t)(uintptr_t)&g_desc[i + 1];

      uint32_t flags = SD_DES0_OWN | SD_DES0_CH;

      if (i == 0)
        {
          flags |= SD_DES0_FS;
        }

      if (i == (num_desc - 1))
        {
          flags |= SD_DES0_LD;
          g_desc[i].des3 = 0;
        }

      g_desc[i].des0 = flags;

      cur_buf += chunk;
      rem     -= chunk;
    }

  SD_CACHE_CLEAN(g_desc, sizeof(g_desc[0]) * num_desc);
  __asm__ volatile ("fence" ::: "memory");

  SDWR((uint32_t)(uintptr_t)&g_desc[0], ESP32P4_SDMMC_DBADDR);

  esp32p4_dma_enable();
  SDWR(1, ESP32P4_SDMMC_PLDMND);

  return OK;
}

/****************************************************************************
 * Private functions: host clock (HP_SYS_CLKRST), same as sdmmc_ll.h
 ****************************************************************************/

static void esp32p4_set_host_div(unsigned int div)
{
  hp_sys_clkrst_dev_t *clk = ESP32P4_HP_CLKRST;
  irqstate_t flags;

  if (div < 2)
    {
      div = 2;
    }

  if (div > 16)
    {
      div = 16;
    }

  flags = enter_critical_section();

  /* sdmmc_ll_set_clock_div() */

  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_h = div / 2 - 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_n = div - 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_l = div - 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_cfg_update = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_cfg_update = 0;

  /* sdmmc_ll_select_clk_source(PLL160M) */

  clk->peri_clk_ctrl01.reg_sdio_ls_clk_src_sel = 0;
  clk->peri_clk_ctrl01.reg_sdio_ls_clk_en = 1;

  /* sdmmc_ll_init_phase_delay() */

  clk->peri_clk_ctrl02.reg_sdio_ls_drv_clk_en = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_sam_clk_en = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_slf_clk_en = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_drv_clk_edge_sel = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_sam_clk_edge_sel = 0;
  clk->peri_clk_ctrl02.reg_sdio_ls_slf_clk_edge_sel = 0;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_cfg_update = 1;
  clk->peri_clk_ctrl02.reg_sdio_ls_clk_edge_cfg_update = 0;

  leave_critical_section(flags);

  up_udelay(10);
}

/* Same divider table as ESP-IDF sd_host_slot_get_clk_dividers()
 * card clock = 160 MHz / host_div / (card_div ? card_div * 2 : 1)
 */
static void esp32p4_get_dividers(uint32_t freq_khz, int *host_div,
                                 int *card_div)
{
  if (freq_khz >= 50000)
    {
      *host_div = 3;     /* 160 MHz / 3 = 53.33 MHz (Real 50 MHz mode) */
      *card_div = 0;
    }
  else if (freq_khz >= 40000)
    {
      *host_div = 4;     /* 160 MHz / 4 = 40.00 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 20000)
    {
      *host_div = 8;     /* 160 MHz / 8 = 20.00 MHz */
      *card_div = 0;
    }
  else if (freq_khz == 400)
    {
      *host_div = 10;
      *card_div = 20;    /* 160 MHz / 10 / (20 * 2) = 400 kHz */
    }
  else
    {
      *host_div = SDMMC_SRC_CLK_HZ / (freq_khz * 1000);
      *card_div = 0;

      if (*host_div > 15)
        {
          *host_div = 2;
          *card_div = (SDMMC_SRC_CLK_HZ / 2) / (2 * freq_khz * 1000);
          if (((SDMMC_SRC_CLK_HZ / 2) % (2 * freq_khz * 1000)) > 0)
            {
              (*card_div)++;
            }
        }
      else if ((SDMMC_SRC_CLK_HZ % (freq_khz * 1000)) > 0)
        {
          (*host_div)++;
        }
    }
}

/****************************************************************************
 * Private functions: CIU command interface
 ****************************************************************************/

/* Hand a command to the CIU.  Like ESP-IDF, use_hold_reg is set on EVERY
 * command (including the clock-update command).
 */

static int esp32p4_ciu_sendcmd(uint32_t cmd, uint32_t arg)
{
  clock_t t0 = clock_systime_ticks();

  while ((SDRD(ESP32P4_SDMMC_CMD) & SDMMC_CMD_STARTCMD) != 0)
    {
      if (clock_systime_ticks() - t0 > SDCARD_CMDTIMEOUT)
        {
          syslog(LOG_ERR, "SDMMC: CIU busy before CMD=%08" PRIx32 "\n", cmd);
          esp32p4_dump("ciu-busy");
          return -ETIMEDOUT;
        }

      up_udelay(10);
    }

  SDWR(arg, ESP32P4_SDMMC_CMDARG);
  SDWR(cmd | SDMMC_CMD_STARTCMD | SDMMC_CMD_USE_HOLE, ESP32P4_SDMMC_CMD);

  t0 = clock_systime_ticks();
  while ((SDRD(ESP32P4_SDMMC_CMD) & SDMMC_CMD_STARTCMD) != 0)
    {
      if (clock_systime_ticks() - t0 > SDCARD_CMDTIMEOUT)
        {
          syslog(LOG_ERR, "SDMMC: CMD not taken CMD=%08" PRIx32 "\n", cmd);
          esp32p4_dump("cmd-stuck");
          return -ETIMEDOUT;
        }

      up_udelay(10);
    }

  return OK;
}

/* Clock update command (not a real command, makes the CIU latch the clock
 * registers).  Retry on Hardware-Locked-Error.
 */

static int esp32p4_update_clock(int slot)
{
  uint32_t cmd = SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV |
                 SDMMC_CMD_CARD_NUMBER(slot);
  int retry;
  int ret = -EBUSY;

  for (retry = 0; retry < 10; retry++)
    {
      SDWR(SDMMC_INT_HLE, ESP32P4_SDMMC_RINTSTS);

      ret = esp32p4_ciu_sendcmd(cmd, 0);
      if (ret < 0)
        {
          return ret;
        }

      if ((SDRD(ESP32P4_SDMMC_RINTSTS) & SDMMC_INT_HLE) == 0)
        {
          return OK;
        }

      up_udelay(100);
    }

  syslog(LOG_ERR, "SDMMC: clock update keeps hitting HLE\n");
  return ret;
}

/****************************************************************************
 * Private functions: interrupt mask handling
 ****************************************************************************/

static void esp32p4_enable_ints(struct esp32p4_dev_s *priv)
{
  SDWR(priv->xfrmask | priv->waitmask | SDCARD_INT_CDET,
       ESP32P4_SDMMC_INTMASK);
}

static void esp32p4_disable_allints(struct esp32p4_dev_s *priv)
{
  SDWR(SDCARD_INT_CDET, ESP32P4_SDMMC_INTMASK);
  priv->waitmask = 0;
  priv->xfrmask  = 0;
}

static void esp32p4_config_waitints(struct esp32p4_dev_s *priv,
                                    uint32_t waitmask,
                                    sdio_eventset_t waitevents,
                                    sdio_eventset_t wkupevent)
{
  irqstate_t flags = enter_critical_section();

  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
  esp32p4_enable_ints(priv);

  leave_critical_section(flags);
}

static void esp32p4_config_xfrints(struct esp32p4_dev_s *priv,
                                   uint32_t xfrmask)
{
  irqstate_t flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  esp32p4_enable_ints(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Private functions: wait / end helpers
 ****************************************************************************/

static void esp32p4_eventtimeout(wdparm_t arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)arg;

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      esp32p4_endwait(priv, SDIOWAIT_TIMEOUT);
    }
}

static void esp32p4_endwait(struct esp32p4_dev_s *priv,
                            sdio_eventset_t wkupevent)
{
  wd_cancel(&priv->waitwdog);
  esp32p4_config_waitints(priv, 0, 0, wkupevent);
  nxsem_post(&priv->waitsem);
}

static void esp32p4_endtransfer(struct esp32p4_dev_s *priv,
                                sdio_eventset_t wkupevent)
{
  irqstate_t flags = enter_critical_section();

  priv->xfr_result = wkupevent;          /* latch: survives early finish */

  esp32p4_config_xfrints(priv, 0);
  SDWR(SD_DATA_ALL_RINT, ESP32P4_SDMMC_RINTSTS);

  /* sdmmc_ll_stop_dma() as ESP-IDF does when the transfer is over */

  esp32p4_dma_stop();

  if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) != 0)
    {
      esp32p4_fifo_reset();
    }

  if ((priv->waitevents & wkupevent) != 0)
    {
      esp32p4_endwait(priv, wkupevent);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Interrupt handler
 ****************************************************************************/

static int esp32p4_interrupt(int irq, void *context, void *arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)arg;
  uint32_t pending;
  uint32_t idsts;

  DEBUGASSERT(priv != NULL);

  pending = SDRD(ESP32P4_SDMMC_RINTSTS) & SDRD(ESP32P4_SDMMC_INTMASK);
  idsts   = SDRD(ESP32P4_SDMMC_IDSTS)   & SDRD(ESP32P4_SDMMC_IDINTEN);

  if (pending == 0 && idsts == 0)
    {
      return OK;
    }

  /* IDMAC events */

  if (idsts != 0)
    {
      SDWR(idsts, ESP32P4_SDMMC_IDSTS);     /* write-1-to-clear */

      if ((idsts & (SD_ID_TI | SD_ID_RI)) != 0)
        {
          priv->dma_done = true;
        }

      if ((idsts & (SD_ID_FBE | SD_ID_DU)) != 0)
        {
          priv->dma_err = true;
          syslog(LOG_ERR, "SDMMC: IDMAC error IDSTS=%08" PRIx32 "\n", idsts);
        }
    }

  /* Command / response events (only enabled if someone waits for them) */

  if ((pending & (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)) != 0)
    {
      sdio_eventset_t wkupevent = 0;

      SDWR(pending & (SDMMC_INT_CDONE | SDCARD_INT_RESPERR),
           ESP32P4_SDMMC_RINTSTS);

      if ((pending & SDCARD_INT_RESPERR) != 0)
        {
          wkupevent = SDIOWAIT_RESPONSEDONE | SDIOWAIT_ERROR;
        }
      else if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
        {
          wkupevent = SDIOWAIT_RESPONSEDONE;
        }
      else if ((priv->waitevents & SDIOWAIT_CMDDONE) != 0)
        {
          wkupevent = SDIOWAIT_CMDDONE;
        }

      if (wkupevent != 0 && (priv->waitevents & wkupevent) != 0)
        {
          esp32p4_endwait(priv, wkupevent);
        }
    }

  /* Data transfer events: complete when DTO and IDMAC-done have both
   * arrived (in either order), or on the first error.
   */

  if (priv->xfrmask != 0)
    {
      uint32_t errbits = pending & SD_DATA_ERR_INT;

      if ((pending & SDMMC_INT_DTO) != 0)
        {
          priv->data_over = true;
        }

      SDWR(pending & (SDMMC_INT_DTO | SD_DATA_ERR_INT),
           ESP32P4_SDMMC_RINTSTS);

      if (errbits != 0 || priv->dma_err)
        {
          sdio_eventset_t wk = SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR;

          if ((errbits & SDMMC_INT_DRTO) != 0)
            {
              wk |= SDIOWAIT_TIMEOUT;
            }

          syslog(LOG_ERR, "SDMMC: data error RINTSTS=%08" PRIx32
                 " IDSTS=%08" PRIx32 "\n", pending, idsts);
          esp32p4_endtransfer(priv, wk);
        }
      else if (priv->data_over && priv->dma_done)
        {
          esp32p4_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }
    }

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if ((pending & SDMMC_INT_CDET) != 0)
    {
      SDWR(SDMMC_INT_CDET, ESP32P4_SDMMC_RINTSTS);
    }
#endif

  return OK;
}

/****************************************************************************
 * SDIO methods
 ****************************************************************************/

static void esp32p4_reset(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  irqstate_t flags = enter_critical_section();

  esp32p4_ctrl_reset(SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
                     SDMMC_CTRL_DMARESET);

  /* Global interrupt enable, then IDMAC init (as sdmmc_ll_init_dma) */

  SDWR(SDMMC_CTRL_INTENABLE, ESP32P4_SDMMC_CTRL);
  esp32p4_dma_init();

  SDWR(0xffffffff, ESP32P4_SDMMC_RINTSTS);
  SDWR(SDCARD_INT_CDET, ESP32P4_SDMMC_INTMASK);

  /* Power the card slot (harmless if not connected) */

  SDWR(1U << priv->slot, ESP32P4_SDMMC_PWREN);

  priv->waitevents = 0;
  priv->waitmask   = 0;
  priv->wkupevent  = 0;
  priv->xfr_result = 0;
  priv->xfrmask    = 0;
  priv->usrbuf     = NULL;
  priv->xfrlen     = 0;
  priv->data_over  = false;
  priv->dma_done   = false;
  priv->dma_err    = false;
  priv->wrdir      = false;
  priv->cdstatus   = 0;

  priv->total_rx_bytes = 0;
  priv->total_tx_bytes = 0;
  priv->total_cmds     = 0;
  priv->total_errs     = 0;

  wd_cancel(&priv->waitwdog);

  leave_critical_section(flags);
}


static sdio_capset_t esp32p4_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = SDIO_CAPS_DMASUPPORTED | SDIO_CAPS_DMABEFOREWRITE |
                   SDIO_CAPS_MMC_HS_MODE; //  SDIO_CAPS_SD_HS_MODE;

#ifdef CONFIG_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#else
  caps |= SDIO_CAPS_4BIT;   /* <--- ADD THIS LINE! */
#endif

  return caps;
}


static sdio_statset_t esp32p4_status(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if ((SDRD(ESP32P4_SDMMC_CDETECT) &
       SDMMC_CDETECT_NOTPRESENT(priv->slot)) == 0)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
#else
  priv->cdstatus |= SDIO_STATUS_PRESENT;
#endif

  return priv->cdstatus;
}


static void esp32p4_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t regval = SDRD(ESP32P4_SDMMC_CTYPE);

  regval &= ~SDMMC_CTYPE_WIDTH4_MASK(priv->slot);
  regval &= ~SDMMC_CTYPE_WIDTH8_MASK(priv->slot);

  if (wide)
    {
      regval |= SDMMC_CTYPE_WIDTH4_MASK(priv->slot);
      priv->cur_bus_width = 4;
    }
  else
    {
      priv->cur_bus_width = 1;
    }

  SDWR(regval, ESP32P4_SDMMC_CTYPE);
}

/* Same sequence as ESP-IDF sd_host_slot_set_card_clk():
 *   1. stop card clock + update
 *   2. program card divider, host divider + update
 *   3. enable card clock (+ low-power) + update
 *   4. data timeout = 100 ms, response timeout = 255
 */

static void esp32p4_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t freq_khz;
  bool     clk_en = true;
  int      host_div = 0;
  int      card_div = 0;
  uint32_t regval;
  uint32_t dto;

  switch (rate)
    {
      default:
      case CLOCK_SDIO_DISABLED:
        freq_khz = ESP32P4_SDMMC_ID_KHZ;
        clk_en   = false;
        break;

      case CLOCK_IDMODE:
        freq_khz = ESP32P4_SDMMC_ID_KHZ;
        break;

      case CLOCK_MMC_TRANSFER:
      case CLOCK_SD_TRANSFER_4BIT:
      case CLOCK_SD_TRANSFER_1BIT:
        freq_khz = ESP32P4_SDMMC_DS_KHZ;
        break;

      // case CLOCK_SD_TRANSFER_4BIT_HS:
      //   freq_khz = ESP32P4_SDMMC_HS_KHZ;
      //   break;
       #ifdef CLOCK_SD_TRANSFER_4BIT_HS
        case CLOCK_SD_TRANSFER_4BIT_HS:
          freq_khz = ESP32P4_SDMMC_HS_KHZ;
          break;
      #endif

    }

  /* 1. Stop the card clock */
  regval  = SDRD(ESP32P4_SDMMC_CLKENA);
  regval &= ~(SDMMC_CLKENA_ENABLE(priv->slot) |
              SDMMC_CLKENA_LOWPOWER(priv->slot));
  SDWR(regval, ESP32P4_SDMMC_CLKENA);
  esp32p4_update_clock(priv->slot);

  /* 2. Dividers */

  esp32p4_get_dividers(freq_khz, &host_div, &card_div);

  /* Compute actual hardware clock in kHz (e.g. 160000 / 3 = 53333 kHz) */
  priv->cur_freq_khz = (SDMMC_SRC_CLK_HZ / 1000) / host_div /
                       (card_div ? (card_div * 2) : 1);

  regval  = SDRD(ESP32P4_SDMMC_CLKSRC);
  regval &= ~SDMMC_CLKSRC_MASK(priv->slot);
  regval |= SDMMC_CLKSRC_CLKDIV(priv->slot, priv->slot);
  SDWR(regval, ESP32P4_SDMMC_CLKSRC);

  regval  = SDRD(ESP32P4_SDMMC_CLKDIV);
  regval &= ~SDMMC_CLKDIV_MASK(priv->slot);
  regval |= SDMMC_CLKDIV(priv->slot, card_div);
  SDWR(regval, ESP32P4_SDMMC_CLKDIV);

  esp32p4_set_host_div(host_div);
  esp32p4_update_clock(priv->slot);

  /* 3. Enable the card clock again */

  if (clk_en)
    {
      regval  = SDRD(ESP32P4_SDMMC_CLKENA);
      regval |= SDMMC_CLKENA_ENABLE(priv->slot) |
                SDMMC_CLKENA_LOWPOWER(priv->slot);
      SDWR(regval, ESP32P4_SDMMC_CLKENA);
      esp32p4_update_clock(priv->slot);
    }

  /* 4. Timeouts: data = 100 ms worth of card clocks, response = 255 */

  dto = freq_khz * 100;
  if (dto > 0xffffff)
    {
      dto = 0xffffff;
    }

  SDWR((dto << SDMMC_TMOUT_DATA_SHIFT) | SDMMC_TMOUT_RESPONSE_MASK,
       ESP32P4_SDMMC_TMOUT);

  syslog(LOG_INFO, "SDMMC: clock %s %" PRIu32 " kHz (host_div=%d "
         "card_div=%d)\n", clk_en ? "on" : "off", freq_khz,
         host_div, card_div);
}

static int esp32p4_attach(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  int cpuint;

  if (priv->attached)
    {
      return OK;
    }

  cpuint = esp_setup_irq(SDIO_HOST_INTR_SOURCE, ESP_IRQ_PRIORITY_DEFAULT,
                         ESP_IRQ_TRIGGER_LEVEL, esp32p4_interrupt, dev);
  if (cpuint < 0)
    {
      return cpuint;
    }

  SDWR(0, ESP32P4_SDMMC_INTMASK);
  SDWR(0xffffffff, ESP32P4_SDMMC_RINTSTS);
  SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);
  SDWR(SDRD(ESP32P4_SDMMC_CTRL) | SDMMC_CTRL_INTENABLE,
       ESP32P4_SDMMC_CTRL);
  SDWR(SDCARD_INT_CDET, ESP32P4_SDMMC_INTMASK);

  up_enable_irq(ESP_IRQ_SDIO_HOST);
  priv->attached = true;

  return OK;
}

static int esp32p4_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t regval = 0;

  priv->total_cmds++; /* <--- ADDED: Count this command */

  /* Drop stale command status from the previous command */

  SDWR(SDMMC_INT_CDONE | SDCARD_INT_RESPERR | SDMMC_INT_HLE,
       ESP32P4_SDMMC_RINTSTS);

  if (cmd == MMCSD_CMD12)
    {
      regval |= SDMMC_CMD_STOPABORT;
    }
  else if (cmd == MMCSD_CMD0)
    {
      regval |= SDMMC_CMD_SENDINIT;
    }
  else
    {
      regval |= SDMMC_CMD_WAITPREV;
    }

  if ((cmd & MMCSD_WRDATAXFR) == MMCSD_WRDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WRITE;
    }
  else if ((cmd & MMCSD_RDDATAXFR) == MMCSD_RDDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD;
    }

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
        regval |= SDMMC_CMD_NORESPONSE;
        break;

      case MMCSD_R1B_RESPONSE:
        regval |= SDMMC_CMD_RESPCRC | SDMMC_CMD_WAITPREV |
                  SDMMC_CMD_SHORTRESPONSE;
        break;

      case MMCSD_R3_RESPONSE:
      case MMCSD_R4_RESPONSE:
        regval |= SDMMC_CMD_SHORTRESPONSE;
        break;

      case MMCSD_R1_RESPONSE:
      case MMCSD_R5_RESPONSE:
      case MMCSD_R6_RESPONSE:
      case MMCSD_R7_RESPONSE:
        regval |= SDMMC_CMD_RESPCRC | SDMMC_CMD_SHORTRESPONSE;
        break;

      case MMCSD_R2_RESPONSE:
        regval |= SDMMC_CMD_LONGRESPONSE | SDMMC_CMD_RESPCRC;
        break;
    }

  regval |= (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= SDMMC_CMD_CARD_NUMBER(priv->slot);

  return esp32p4_ciu_sendcmd(regval, arg);
}



static void esp32p4_abort_stale(struct esp32p4_dev_s *priv)
{
  if (priv->xfrmask != 0 ||
      (SDRD(ESP32P4_SDMMC_BMOD) & SDMMC_BMOD_DE) != 0)
    {
      syslog(LOG_WARNING, "SDMMC: aborting stale transfer\n");

      esp32p4_disable_allints(priv);
      wd_cancel(&priv->waitwdog);
      esp32p4_dma_stop();
      esp32p4_fifo_reset();

      SDWR(SD_DATA_ALL_RINT, ESP32P4_SDMMC_RINTSTS);
      SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);

      priv->xfrmask    = 0;
      priv->waitevents = 0;
      priv->data_over  = false;
      priv->dma_done   = false;
    }
}

/* BLKSIZ and BYTCNT must be programmed BEFORE the data command is issued. */
static void esp32p4_blocksetup(struct sdio_dev_s *dev,
                               unsigned int blocklen, unsigned int nblocks)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  esp32p4_abort_stale(priv);
  esp32p4_fifo_reset();

  SDWR(blocklen, ESP32P4_SDMMC_BLKSIZ);
  SDWR(blocklen * nblocks, ESP32P4_SDMMC_BYTCNT);

  /* FIFOTH is left at its hardware default (as ESP-IDF) */
}

static int esp32p4_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);

  priv->usrbuf = buffer;
  priv->xfrlen = nbytes;
  priv->wrdir  = false;

  /* Check for Zero-Copy eligibility */

  if (esp32p4_can_dma(buffer, nbytes))
    {
      priv->is_bounce = false;
      ret = esp32p4_dma_start(priv, buffer, nbytes);
    }
  else
    {
      priv->is_bounce = true;
      ret = esp32p4_dma_start(priv, g_bounce, nbytes);
    }

  if (ret < 0)
    {
      return ret;
    }

  esp32p4_config_xfrints(priv, SD_DMA_XFR_MASK);
  return OK;
}


static int esp32p4_sendsetup(struct sdio_dev_s *dev, const uint8_t *buffer,
                             size_t nbytes)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  size_t sync = (nbytes + SD_DMA_ALIGN - 1) & ~((size_t)SD_DMA_ALIGN - 1);
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);

  priv->usrbuf = (uint8_t *)buffer;
  priv->xfrlen = nbytes;
  priv->wrdir  = true;

  /* Check for Zero-Copy eligibility */

  if (esp32p4_can_dma(buffer, nbytes))
    {
      priv->is_bounce = false;
      SD_CACHE_CLEAN(buffer, sync);
      ret = esp32p4_dma_start(priv, (uint8_t *)buffer, nbytes);
    }
  else
    {
      priv->is_bounce = true;

      if (nbytes > ESP32P4_SDMMC_BOUNCE_SZ)
        {
          syslog(LOG_ERR, "SDMMC: write length %zu exceeds bounce buffer\n", nbytes);
          return -EINVAL;
        }

      memcpy(g_bounce, buffer, nbytes);
      if (sync > nbytes)
        {
          memset(g_bounce + nbytes, 0, sync - nbytes);
        }

      SD_CACHE_CLEAN(g_bounce, sync);
      ret = esp32p4_dma_start(priv, g_bounce, nbytes);
    }

  if (ret < 0)
    {
      return ret;
    }

  esp32p4_config_xfrints(priv, SD_DMA_XFR_MASK);
  return OK;
}



static int esp32p4_cancel(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  esp32p4_disable_allints(priv);
  SDWR(0xffffffff, ESP32P4_SDMMC_RINTSTS);
  SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);
  wd_cancel(&priv->waitwdog);
  esp32p4_dma_stop();
  esp32p4_fifo_reset();
  priv->xfrlen = 0;
  return OK;
}

static int esp32p4_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  clock_t  t0 = clock_systime_ticks();
  clock_t  timeout;
  uint32_t rint;
  int      spin = 0;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
      case MMCSD_NO_RESPONSE:
      case MMCSD_R3_RESPONSE:
      case MMCSD_R7_RESPONSE:
        timeout = SDCARD_CMDTIMEOUT;
        break;

      default:
        timeout = SDCARD_LONGTIMEOUT;
        break;
    }

  for (; ; )
    {
      rint = SDRD(ESP32P4_SDMMC_RINTSTS);
      if ((rint & SDMMC_INT_CDONE) != 0)
        {
          break;
        }

      if (clock_systime_ticks() - t0 > timeout)
        {
          syslog(LOG_ERR, "SDMMC: waitresponse timeout cmd=0x%04" PRIx32
                 "\n", cmd);
          esp32p4_dump("waitresp");
          return -ETIMEDOUT;
        }

      if (spin++ < 1000)
        {
          up_udelay(10);
        }
      else
        {
          nxsig_usleep(1000);
        }
    }

  /* Only CDONE is cleared here; the error bits are consumed by recv_rX() */

  SDWR(SDMMC_INT_CDONE, ESP32P4_SDMMC_RINTSTS);

  if ((rint & SDMMC_INT_RTO) != 0)
    {
      return -ETIMEDOUT;
    }

  if ((rint & (SDMMC_INT_RCRC | SDMMC_INT_RE | SDMMC_INT_HLE)) != 0)
    {
      return -EIO;
    }

  return OK;
}

static int esp32p4_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                                uint32_t *rshort)
{
  uint32_t rint = SDRD(ESP32P4_SDMMC_RINTSTS);
  int ret = OK;

  if ((rint & SDMMC_INT_RTO) != 0)
    {
      ret = -ETIMEDOUT;
    }
  else if ((rint & (SDMMC_INT_RCRC | SDMMC_INT_RE)) != 0)
    {
      ret = -EIO;
    }

  SDWR(SDMMC_INT_CDONE | SDCARD_INT_RESPERR, ESP32P4_SDMMC_RINTSTS);

  if (rshort != NULL)
    {
      *rshort = SDRD(ESP32P4_SDMMC_RESP0);
    }

  return ret;
}

static int esp32p4_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t rlong[4])
{
  uint32_t rint = SDRD(ESP32P4_SDMMC_RINTSTS);
  int ret = OK;

  if ((rint & SDMMC_INT_RTO) != 0)
    {
      ret = -ETIMEDOUT;
    }
  else if ((rint & (SDMMC_INT_RCRC | SDMMC_INT_RE)) != 0)
    {
      ret = -EIO;
    }

  SDWR(SDMMC_INT_CDONE | SDCARD_INT_RESPERR, ESP32P4_SDMMC_RINTSTS);

  if (rlong != NULL)
    {
      rlong[0] = SDRD(ESP32P4_SDMMC_RESP3);
      rlong[1] = SDRD(ESP32P4_SDMMC_RESP2);
      rlong[2] = SDRD(ESP32P4_SDMMC_RESP1);
      rlong[3] = SDRD(ESP32P4_SDMMC_RESP0);
    }

  return ret;
}

static int esp32p4_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rshort)
{
  uint32_t rint = SDRD(ESP32P4_SDMMC_RINTSTS);
  int ret = OK;

  if ((rint & SDMMC_INT_RTO) != 0)
    {
      ret = -ETIMEDOUT;
    }

  SDWR(SDMMC_INT_CDONE | SDCARD_INT_RESPERR, ESP32P4_SDMMC_RINTSTS);

  if (rshort != NULL)
    {
      *rshort = SDRD(ESP32P4_SDMMC_RESP0);
    }

  return ret;
}

static void esp32p4_waitenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset, uint32_t timeout)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t waitmask = 0;

  /* Drop stale wake-ups from an earlier (timed out) wait */

  while (nxsem_trywait(&priv->waitsem) == OK)
    {
    }

  esp32p4_config_waitints(priv, 0, 0, 0);

  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDCARD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDCARD_RESPDONE_MASK;
    }

  /* SDIOWAIT_TRANSFERDONE is signalled from esp32p4_endtransfer() */

  esp32p4_config_waitints(priv, waitmask, eventset, 0);

  if ((eventset & SDIOWAIT_TIMEOUT) != 0)
    {
      if (timeout == 0)
        {
          esp32p4_endwait(priv, SDIOWAIT_TIMEOUT);
          return;
        }

      wd_start(&priv->waitwdog, MSEC2TICK(timeout), esp32p4_eventtimeout,
               (wdparm_t)priv);
    }

  /* NEW: the transfer may already have finished before we got here.
   * If so, wake the waiter now with the latched result.
   */

  {
    irqstate_t f = enter_critical_section();

    if (priv->xfr_result != 0 && (priv->xfr_result & eventset) != 0)
      {
        esp32p4_endwait(priv, priv->xfr_result);
      }

    leave_critical_section(f);
  }
}


/* Put the controller and the card back into a known idle state */

static void esp32p4_card_abort(struct sdio_dev_s *dev)
{
  esp32p4_dma_stop();
  esp32p4_fifo_reset();

  SDWR(0xffffffff, ESP32P4_SDMMC_RINTSTS);
  SDWR(0xffffffff, ESP32P4_SDMMC_IDSTS);

  if (esp32p4_sendcmd(dev, MMCSD_CMD12, 0) == OK)
    {
      if (esp32p4_waitresponse(dev, MMCSD_CMD12) == OK)
        {
          esp32p4_recvshortcrc(dev, MMCSD_CMD12, NULL);
        }
    }
}


// static sdio_eventset_t esp32p4_eventwait(struct sdio_dev_s *dev)
// {
//   struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
//   sdio_eventset_t wkupevent;
//   int ret;

//   ret = nxsem_tickwait_uninterruptible(&priv->waitsem, SDCARD_LONGTIMEOUT);
//   if (ret < 0)
//     {
//       syslog(LOG_ERR, "SDMMC: eventwait timeout\n");
//       esp32p4_dump("eventwait");
//       wd_cancel(&priv->waitwdog);
//       esp32p4_dma_stop();
//       esp32p4_fifo_reset();
//       priv->total_errs++;
//       wkupevent = SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT;
//     }
//   else
//     {
//       wkupevent = priv->wkupevent;

//       /* Card Busy check on DAT0 */

//       if (priv->wrdir && (wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) == 0)
//         {
//           clock_t  t0 = clock_systime_ticks();
//           unsigned spin = 0;

//           while ((SDRD(ESP32P4_SDMMC_STATUS) & SD_STATUS_DATA_BUSY) != 0)
//             {
//               if (clock_systime_ticks() - t0 > MSEC2TICK(1000))
//                 {
//                   wkupevent |= SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT;
//                   break;
//                 }

//               if (spin++ < 4000)
//                 {
//                   up_udelay(5);
//                 }
//               else
//                 {
//                   nxsig_usleep(1000);
//                 }
//             }
//         }

//       /* ================================================================ */
//       /* EMERGENCY CMD12 ABORT: If an error or timeout occurred, stop the */
//       /* card immediately so it doesn't stay wedged on the bus.           */
//       /* ================================================================ */
//       if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) != 0)
//         {
//           priv->total_errs++;

//           // esp32p4_sendcmd(dev, MMCSD_CMD12, 0);
//           // esp32p4_waitresponse(dev, MMCSD_CMD12);
//         }

//       /* Statistics */

//       if (priv->wrdir && (wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) == 0)
//         {
//           priv->total_tx_bytes += priv->xfrlen;
//         }

//       /* Handle Read completion (Zero-Copy vs Bounce) */

//       if (!priv->wrdir && priv->usrbuf != NULL && priv->xfrlen > 0 &&
//           (wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) == 0)
//         {
//           if (priv->is_bounce)
//             {
//               size_t sync = (priv->xfrlen + SD_DMA_ALIGN - 1) &
//                             ~((size_t)SD_DMA_ALIGN - 1);

//               SD_CACHE_INVAL(g_bounce, sync);
//               memcpy(priv->usrbuf, g_bounce, priv->xfrlen);
//             }
//           else
//             {
//               /* Zero-copy: cache already invalidated directly over caller's buffer */
//               SD_CACHE_INVAL(priv->usrbuf, priv->xfrlen);
//             }

//           priv->total_rx_bytes += priv->xfrlen;
//         }
//     }

//   esp32p4_disable_allints(priv);

//   return wkupevent;
// }



static sdio_eventset_t esp32p4_eventwait(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  sdio_eventset_t wkupevent;
  int ret;

  ret = nxsem_tickwait_uninterruptible(&priv->waitsem, SDCARD_LONGTIMEOUT);
  if (ret < 0)
    {
      /* Semaphore wait itself timed out (2 s) */

      wd_cancel(&priv->waitwdog);
      priv->total_errs++;
      wkupevent = SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT;
    }
  else
    {
      wkupevent = priv->wkupevent;

      /* Card busy check on DAT0 after a write */

      if (priv->wrdir &&
          (wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) == 0)
        {
          clock_t  t0 = clock_systime_ticks();
          unsigned spin = 0;

          while ((SDRD(ESP32P4_SDMMC_STATUS) & SD_STATUS_DATA_BUSY) != 0)
            {
              if (clock_systime_ticks() - t0 > MSEC2TICK(1000))
                {
                  wkupevent |= SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT;
                  break;
                }

              if (spin++ < 4000)
                {
                  up_udelay(5);
                }
              else
                {
                  nxsig_usleep(1000);
                }
            }
        }

      if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) == 0)
        {
          if (priv->wrdir)
            {
              priv->total_tx_bytes += priv->xfrlen;
            }
          else if (priv->usrbuf != NULL && priv->xfrlen > 0)
            {
              if (priv->is_bounce)
                {
                  size_t sync = (priv->xfrlen + SD_DMA_ALIGN - 1) &
                                ~((size_t)SD_DMA_ALIGN - 1);

                  SD_CACHE_INVAL(g_bounce, sync);
                  memcpy(priv->usrbuf, g_bounce, priv->xfrlen);
                }
              else
                {
                  SD_CACHE_INVAL(priv->usrbuf, priv->xfrlen);
                }

              priv->total_rx_bytes += priv->xfrlen;
            }
        }
    }

  esp32p4_disable_allints(priv);

  /* Recover after ANY failure: both the 2 s timeout path and the
   * watchdog/error path end up here.
   */

  if ((wkupevent & (SDIOWAIT_ERROR | SDIOWAIT_TIMEOUT)) != 0)
    {
      if (ret >= 0)
        {
          priv->total_errs++;
        }

      esp32p4_card_abort(dev);
    }

  priv->xfr_result = 0;
  return wkupevent;
}

#ifdef CONFIG_SDIO_HAVE_CALLBACK
static void esp32p4_callback(void *arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)arg;

  if (priv->callback != NULL)
    {
      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              return;
            }
        }
      else
        {
          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              return;
            }
        }

      priv->cbevents = 0;

      if (up_interrupt_context())
        {
          work_queue(HPWORK, &priv->cbwork, priv->callback, priv->cbarg, 0);
        }
      else
        {
          priv->callback(priv->cbarg);
        }
    }
}

static void esp32p4_callbackenable(struct sdio_dev_s *dev,
                                   sdio_eventset_t eventset)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  priv->cbevents = eventset;
  esp32p4_callback(priv);
}

static int esp32p4_registercallback(struct sdio_dev_s *dev,
                                    worker_t callback, void *arg)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}
#endif /* CONFIG_SDIO_HAVE_CALLBACK */

#ifdef CONFIG_SDIO_DMA
static bool esp32p4_dmasupported(struct sdio_dev_s *dev)
{
  return true;
}

static int esp32p4_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                                size_t buflen)
{
  return esp32p4_recvsetup(dev, buffer, buflen);
}

static int esp32p4_dmasendsetup(struct sdio_dev_s *dev,
                                const uint8_t *buffer, size_t buflen)
{
  return esp32p4_sendsetup(dev, buffer, buflen);
}
#endif

/****************************************************************************
 * Private functions: pins
 ****************************************************************************/

/* IO_MUX direct routing, as ESP-IDF configure_pin_iomux():
 * pull-down off, input enable, function 0 (SDMMC), drive strength 3.
 */

static void esp32p4_iomux_pin(const struct esp32p4_pin_s *pin)
{
  uint32_t v = SDRD(pin->reg);

  v &= ~(MCU_SEL_M | FUN_DRV_M | FUN_PD | FUN_PU);
  v |= (0u << MCU_SEL_S);          /* IO_MUX function 0 = SDMMC */
  v |= (3u << FUN_DRV_S);          /* drive strength 3          */
  v |= FUN_IE;                     /* input enable              */

  if (pin->pullup)
    {
      v |= FUN_PU;
    }

  SDWR(v, pin->reg);
}

/****************************************************************************
 * Public functions
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct esp32p4_dev_s *priv = &g_sdiodev;
  int i;

  if (slotno != ESP32P4_SDMMC_SLOT)
    {
      syslog(LOG_ERR, "SDMMC: only slot 0 is supported (got %d)\n", slotno);
      return NULL;
    }

  priv->slot = slotno;

  /* 0. Card power (board hook) */

  esp32p4_sdmmc_power_on();
  up_mdelay(10);

  /* 1. Bus clock on, pulse the peripheral reset (as sd_host_sdmmc.c) */

  ESP32P4_HP_CLKRST->soc_clk_ctrl1.reg_sdmmc_sys_clk_en = 1;
  up_udelay(20);

  modifyreg32(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL, 0, LP_RST_EN_SDMMC);
  up_udelay(10);
  modifyreg32(LP_CLKRST_HP_SDMMC_EMAC_RST_CTRL, LP_RST_EN_SDMMC, 0);
  up_udelay(20);

  /* 2. 160 MHz reference clock, host divider, phase setup.
   *    ESP-IDF initialises the host divider to 2 at controller creation.
   */

  modifyreg32(HP_SYS_CLKRST_REF_CLK_CTRL2_REG, 0,
              HP_SYS_CLKRST_REG_REF_160M_CLK_EN);
  up_udelay(20);

  esp32p4_set_host_div(2);

  syslog(LOG_INFO, "SDMMC: VERID=0x%08" PRIx32 " HCON=0x%08" PRIx32
         " FIFOTH=0x%08" PRIx32 "\n",
         SDRD(ESP32P4_SDMMC_VERID), SDRD(ESP32P4_SDMMC_HCON),
         SDRD(ESP32P4_SDMMC_FIFOTH));
  syslog(LOG_INFO, "SDMMC: DMA bounce=%p desc=%p (align %d)\n",
         g_bounce, g_desc, SD_DMA_ALIGN);

  /* 3. Pins: slot 0 uses the fixed IO_MUX pads */

  for (i = 0; i < (int)(sizeof(g_sdmmc_pins) / sizeof(g_sdmmc_pins[0])); i++)
    {
      esp32p4_iomux_pin(&g_sdmmc_pins[i]);
    }

#if defined(GPIO_MATRIX_CONST_ZERO_INPUT) && \
    defined(SD_CARD_DETECT_N_1_PAD_IN_IDX)
  /* No CD / WP pins: card present, not write protected, no SDIO irq */

  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ZERO_INPUT,
                     SD_CARD_DETECT_N_1_PAD_IN_IDX, false);
#  if defined(GPIO_MATRIX_CONST_ONE_INPUT)
  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                     SD_CARD_WRITE_PRT_1_PAD_IN_IDX, false);
  esp_gpio_matrix_in(GPIO_MATRIX_CONST_ONE_INPUT,
                     SD_CARD_INT_N_1_PAD_IN_IDX, false);
#  endif
#endif

  /* 4. Controller reset and default state (includes IDMAC init) */

  priv->attached = false;
  priv->dev.reset(&priv->dev);

  esp32p4_dump("init");

  return &priv->dev;
}


/****************************************************************************
 * Public Function: esp32p4_sdmmc_dump_details
 *
 * Description:
 *   Prints full hardware and controller information, live clock speeds,
 *   bus width, and transfer statistics.
 ****************************************************************************/

void esp32p4_sdmmc_dump_details(struct sdio_dev_s *dev)
{
  struct esp32p4_dev_s *priv = (struct esp32p4_dev_s *)dev;
  uint32_t verid  = SDRD(ESP32P4_SDMMC_VERID);
  uint32_t hcon   = SDRD(ESP32P4_SDMMC_HCON);
  uint32_t ctype  = SDRD(ESP32P4_SDMMC_CTYPE);
  uint32_t status = SDRD(ESP32P4_SDMMC_STATUS);
  uint32_t fifoth = SDRD(ESP32P4_SDMMC_FIFOTH);

  /* FIFO depth = RX watermark bits [27:16] + 1 */
  uint32_t fifo_depth = ((fifoth >> 16) & 0xfff) + 1;

  syslog(LOG_INFO, "========================================\n");
  syslog(LOG_INFO, "       ESP32-P4 SDMMC HOST DETAILS      \n");
  syslog(LOG_INFO, "========================================\n");

  /* 1. IP Hardware Info */
  syslog(LOG_INFO, "Hardware IP Core  : Synopsys DesignWare SDMMC\n");
  syslog(LOG_INFO, "Core Version      : %c%c v%x.%02x (0x%08" PRIx32 ")\n",
         (char)((verid >> 24) & 0xff), (char)((verid >> 16) & 0xff),
         (unsigned int)((verid >> 12) & 0x0f), (unsigned int)(verid & 0xff),
         verid);
  syslog(LOG_INFO, "Hardware Config   : HCON=0x%08" PRIx32 "\n", hcon);
  syslog(LOG_INFO, "FIFO Depth        : %" PRIu32 " words (%" PRIu32 " bytes)\n",
         fifo_depth, fifo_depth * 4);
  syslog(LOG_INFO, "DMA Mode          : Internal IDMAC (Chained Descriptors)\n");

  /* 2. Clock & Bus Configuration */
  syslog(LOG_INFO, "----------------------------------------\n");
  syslog(LOG_INFO, "Current Bus Clock : %" PRIu32 " kHz (%" PRIu32 ".%02" PRIu32 " MHz)\n",
         priv->cur_freq_khz,
         priv->cur_freq_khz / 1000,
         (priv->cur_freq_khz % 1000) / 10);
  syslog(LOG_INFO, "Current Bus Width : %d-bit (CTYPE=0x%08" PRIx32 ")\n",
         (ctype & 1) ? 4 : 1, ctype);
  syslog(LOG_INFO, "Data Line Busy    : %s\n",
         (status & (1 << 9)) ? "YES" : "NO");

  /* 3. Live Runtime Statistics */
  syslog(LOG_INFO, "----------------------------------------\n");
  syslog(LOG_INFO, "Total Commands    : %" PRIu32 "\n", priv->total_cmds);
  syslog(LOG_INFO, "Total RX Bytes    : %llu bytes (%" PRIu64 " KB)\n",
         priv->total_rx_bytes, priv->total_rx_bytes / 1024);
  syslog(LOG_INFO, "Total TX Bytes    : %llu bytes (%" PRIu64 " KB)\n",
         priv->total_tx_bytes, priv->total_tx_bytes / 1024);
  syslog(LOG_INFO, "Total Errors      : %" PRIu32 "\n", priv->total_errs);
  syslog(LOG_INFO, "========================================\n");
}


#endif /* CONFIG_ESP32P4_SDMMC */
