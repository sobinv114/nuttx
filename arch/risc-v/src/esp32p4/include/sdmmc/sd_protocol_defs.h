#pragma once

#include <stdint.h>
#include <limits.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MMC commands */                              /* response type */
#define MMC_GO_IDLE_STATE               0       /* R0 */
#define MMC_SEND_OP_COND                1       /* R3 */
#define MMC_ALL_SEND_CID                2       /* R2 */
#define MMC_SET_RELATIVE_ADDR           3       /* R1 */
#define MMC_SWITCH                      6       /* R1B */
#define MMC_SELECT_CARD                 7       /* R1 */
#define MMC_SEND_EXT_CSD                8       /* R1 */
#define MMC_SEND_CSD                    9       /* R2 */
#define MMC_SEND_CID                    10      /* R1 */
#define MMC_READ_DAT_UNTIL_STOP         11      /* R1 */
#define MMC_STOP_TRANSMISSION           12      /* R1B */
#define MMC_SEND_STATUS                 13      /* R1 */
#define MMC_SET_BLOCKLEN                16      /* R1 */
#define MMC_READ_BLOCK_SINGLE           17      /* R1 */
#define MMC_READ_BLOCK_MULTIPLE         18      /* R1 */
#define MMC_SEND_TUNING_BLOCK           19      /* R1 */
#define MMC_WRITE_DAT_UNTIL_STOP        20      /* R1 */
#define MMC_SET_BLOCK_COUNT             23      /* R1 */
#define MMC_WRITE_BLOCK_SINGLE          24      /* R1 */
#define MMC_WRITE_BLOCK_MULTIPLE        25      /* R1 */
#define MMC_ERASE_GROUP_START           35      /* R1 */
#define MMC_ERASE_GROUP_END             36      /* R1 */
#define MMC_ERASE                       38      /* R1B */
#define MMC_APP_CMD                     55      /* R1 */

/* SD commands */                               /* response type */
#define SD_SEND_RELATIVE_ADDR           3       /* R6 */
#define SD_SEND_SWITCH_FUNC             6       /* R1 */
#define SD_SEND_IF_COND                 8       /* R7 */
#define SD_SWITCH_VOLTAGE               11      /* R1 */
#define SD_ERASE_GROUP_START            32      /* R1 */
#define SD_ERASE_GROUP_END              33      /* R1 */
#define SD_READ_OCR                     58      /* R3 */
#define SD_CRC_ON_OFF                   59      /* R1 */

/* SD application commands */                   /* response type */
#define SD_APP_SET_BUS_WIDTH            6       /* R1 */
#define SD_APP_SD_STATUS                13      /* R2 */
#define SD_APP_SEND_NUM_WR_BLOCKS       22      /* R1 */
#define SD_APP_OP_COND                  41      /* R3 */
#define SD_APP_SEND_SCR                 51      /* R1 */

/* SD IO commands */
#define SD_IO_SEND_OP_COND              5       /* R4 */
#define SD_IO_RW_DIRECT                 52      /* R5 */
#define SD_IO_RW_EXTENDED               53      /* R5 */

/* OCR bits */
#define MMC_OCR_MEM_READY               (1<<31)
#define MMC_OCR_ACCESS_MODE_MASK        0x60000000
#define MMC_OCR_SECTOR_MODE             (1<<30)
#define MMC_OCR_BYTE_MODE               (1<<29)
#define MMC_OCR_3_5V_3_6V               (1<<23)
#define MMC_OCR_3_4V_3_5V               (1<<22)
#define MMC_OCR_3_3V_3_4V               (1<<21)
#define MMC_OCR_3_2V_3_3V               (1<<20)
#define MMC_OCR_3_1V_3_2V               (1<<19)
#define MMC_OCR_3_0V_3_1V               (1<<18)
#define MMC_OCR_2_9V_3_0V               (1<<17)
#define MMC_OCR_2_8V_2_9V               (1<<16)
#define MMC_OCR_2_7V_2_8V               (1<<15)
#define MMC_OCR_2_6V_2_7V               (1<<14)
#define MMC_OCR_2_5V_2_6V               (1<<13)
#define MMC_OCR_2_4V_2_5V               (1<<12)
#define MMC_OCR_2_3V_2_4V               (1<<11)
#define MMC_OCR_2_2V_2_3V               (1<<10)
#define MMC_OCR_2_1V_2_2V               (1<<9)
#define MMC_OCR_2_0V_2_1V               (1<<8)
#define MMC_OCR_1_65V_1_95V             (1<<7)

#define SD_OCR_CARD_READY               MMC_OCR_MEM_READY
#define SD_OCR_SDHC_CAP                 (1<<30)
#define SD_OCR_XPC                      (1<<28)
#define SD_OCR_S18_RA                   (1<<24)
#define SD_OCR_VOL_MASK                 0xFF8000
#define SD_OCR_3_5V_3_6V                MMC_OCR_3_5V_3_6V
#define SD_OCR_3_4V_3_5V                MMC_OCR_3_4V_3_5V
#define SD_OCR_3_3V_3_4V                MMC_OCR_3_3V_3_4V
#define SD_OCR_3_2V_3_3V                MMC_OCR_3_2V_3_3V
#define SD_OCR_3_1V_3_2V                MMC_OCR_3_1V_3_2V
#define SD_OCR_3_0V_3_1V                MMC_OCR_3_0V_3_1V
#define SD_OCR_2_9V_3_0V                MMC_OCR_2_9V_3_0V
#define SD_OCR_2_8V_2_9V                MMC_OCR_2_8V_2_9V
#define SD_OCR_2_7V_2_8V                MMC_OCR_2_7V_2_8V

#define MMC_R1_READY_FOR_DATA           (1<<8)
#define MMC_R1_APP_CMD                  (1<<5)
#define MMC_R1_SWITCH_ERROR             (1<<7)
#define MMC_R1_CURRENT_STATE_POS        (9)
#define MMC_R1_CURRENT_STATE_MASK       (0x1E00)
#define MMC_R1_CURRENT_STATE_TRAN       (4)
#define MMC_R1_CURRENT_STATE_STATUS(status) (((status) & MMC_R1_CURRENT_STATE_MASK) >> MMC_R1_CURRENT_STATE_POS)

#define MMC_R1(resp)                    ((resp)[0])
#define MMC_R3(resp)                    ((resp)[0])
#define MMC_R4(resp)                    ((resp)[0])
#define MMC_R5(resp)                    ((resp)[0])
#define SD_R6(resp)                     ((resp)[0])
#define MMC_R1_CURRENT_STATE(resp)      (((resp)[0] >> 9) & 0xf)

#define MMC_ARG_RCA(rca)                ((rca) << 16)
#define SD_R6_RCA(resp)                 (SD_R6((resp)) >> 16)

#define SD_ARG_BUS_WIDTH_1              0
#define SD_ARG_BUS_WIDTH_4              2

#define SDMMC_TIMING_LEGACY             0
#define SDMMC_TIMING_HIGHSPEED          1
#define SDMMC_TIMING_MMC_DDR52          2

#ifdef __cplusplus
}
#endif