#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sd_protocol_defs.h"

#ifndef BIT
#  define BIT(nr) (1UL << (nr))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define SDMMC_FREQ_DEFAULT      20000
#define SDMMC_FREQ_HIGHSPEED    40000
#define SDMMC_FREQ_PROBING      400
#define SDMMC_FREQ_52M          52000
#define SDMMC_FREQ_26M          26000

typedef uint32_t sdmmc_response_t[4];

typedef struct {
    uint32_t opcode;
    uint32_t arg;
    sdmmc_response_t response;
    void* data;
    size_t datalen;
    size_t buflen;
    size_t blklen;
    int flags;
#define SCF_ITSDONE      0x0001
#define SCF_CMD(flags)   ((flags) & 0x00f0)
#define SCF_CMD_AC       0x0000
#define SCF_CMD_ADTC     0x0010
#define SCF_CMD_BC       0x0020
#define SCF_CMD_BCR      0x0030
#define SCF_CMD_READ     0x0040
#define SCF_RSP_BSY      0x0100
#define SCF_RSP_136      0x0200
#define SCF_RSP_CRC      0x0400
#define SCF_RSP_IDX      0x0800
#define SCF_RSP_PRESENT  0x1000
#define SCF_RSP_R0       0
#define SCF_RSP_R1       (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R1B      (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX|SCF_RSP_BSY)
#define SCF_RSP_R2       (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_136)
#define SCF_RSP_R3       (SCF_RSP_PRESENT)
#define SCF_RSP_R4       (SCF_RSP_PRESENT)
#define SCF_RSP_R5       (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R5B      (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX|SCF_RSP_BSY)
#define SCF_RSP_R6       (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_RSP_R7       (SCF_RSP_PRESENT|SCF_RSP_CRC|SCF_RSP_IDX)
#define SCF_WAIT_BUSY    0x2000
    esp_err_t error;
    uint32_t timeout_ms;
} sdmmc_command_t;

typedef enum {
    SDMMC_DELAY_PHASE_0,
    SDMMC_DELAY_PHASE_1,
    SDMMC_DELAY_PHASE_2,
    SDMMC_DELAY_PHASE_3,
    SDMMC_DELAY_PHASE_AUTO,
} sdmmc_delay_phase_t;

typedef uint32_t sdmmc_delay_line_t;

#ifdef __cplusplus
}
#endif