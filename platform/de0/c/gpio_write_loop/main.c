#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "generated_header.h"
#include "../common/common_lib.h"
#include "../common/spi_bitbang.h"

/* The existing build scripts compile only main.c and common_lib.c.
 * Include the SPI implementation here to keep those scripts unchanged. */
#include "../common/spi_bitbang.c"

#define UART_TX_LEVEL_ADDR (UART_TX_ADDR + 0x8u)
#define TIME_FREQ_ADDR 0x0030000Cu
#define TIME_LOW_ADDR  0x00300010u
#define TIME_HIGH_ADDR 0x00300014u
#define CC1101_VERSION_ADDR 0x31u
#define SCOPE_GPIO_MASK 0x5Fu
#define SCOPE_PHASE_HZ 6400u

static uint32_t read_reg(uint32_t address)
{
    return *(volatile uint32_t *)(uintptr_t)address;
}

static void wait_uart_tx(void)
{
    while (read_reg(UART_TX_LEVEL_ADDR) != 0u) {
    }
}

static uint64_t read_elapsed_time(void)
{
    /* Reading frequency captures a coherent 64-bit counter in the RTL. */
    (void)read_reg(TIME_FREQ_ADDR);
    uint32_t low = read_reg(TIME_LOW_ADDR);
    uint32_t high = read_reg(TIME_HIGH_ADDR);
    return ((uint64_t)high << 32) | low;
}

static void show_build_information(void)
{
    uint32_t date = read_reg(AXI_SLAVE_REG_REG0);
    uint32_t time = read_reg(AXI_SLAVE_REG_REG1);

    int day = (int)(date & 0xFFu);
    int month = (int)((date >> 8) & 0xFFu);
    int year = (int)((date >> 16) & 0xFFFFu);
    int hour = (int)(time & 0xFFu);
    int minute = (int)((time >> 8) & 0xFFu);
    int second = (int)((time >> 16) & 0xFFu);

    dprint("FPGA build date: %d/%d/%d\r\n", day, month, year);
    wait_uart_tx();
    dprint("FPGA build time: %d:%d:%d\r\n", hour, minute, second);
    wait_uart_tx();
    dprint("ELF build date: %d/%d/%d\r\n",
           generated_day, generated_month, generated_year);
    wait_uart_tx();
    dprint("ELF build time: %d:%d:%d\r\n",
           generated_hour, generated_minute, generated_second);
    wait_uart_tx();
}

static void show_cc1101_version(uint8_t module, const char *name)
{
    uint8_t version;
    int result = cc1101_read_reg_ex(module, CC1101_VERSION_ADDR, &version);

    if (result == SPI_BB_OK) {
        dprint("CC1101 %s VERSION: 0x%x\r\n", name, (unsigned int)version);
    } else if (result == SPI_BB_ETIME) {
        dprint("CC1101 %s: SPI ready timeout\r\n", name);
    } else {
        dprint("CC1101 %s: SPI read error (%d)\r\n", name, result);
    }
    wait_uart_tx();
}

static void show_cc1101_versions(void)
{
    show_cc1101_version(SPI_MODULE_TX, "TX");
    show_cc1101_version(SPI_MODULE_RX, "RX");
}

static bool quit_requested(void)
{
    /* Limit each pass to the current FIFO contents so timing still advances. */
    uint32_t pending = read_reg(UART_RX_LEVEL_ADDR);
    while (pending != 0u) {
        uint8_t key = (uint8_t)read_reg(UART_RX_DATA_ADDR);
        if (key == 'q' || key == 'Q') {
            return true;
        }
        --pending;
    }
    return false;
}

static void read_versions_periodically(void)
{
    uint32_t frequency = read_reg(TIME_FREQ_ADDR);
    if (frequency == 0u) {
        dprint("Error: elapsed time frequency is zero.\r\n");
        wait_uart_tx();
        return;
    }

    uint64_t interval = frequency / 2u;
    if (interval == 0u) {
        interval = 1u;
    }

    dprint("Reading every 500 ms. Press Q to return to the menu.\r\n");
    wait_uart_tx();

    uint64_t last_read = read_elapsed_time();
    show_cc1101_versions();
    uint64_t after_read = read_elapsed_time();
    if (after_read - last_read >= interval) {
        last_read = after_read;
    }

    while (!quit_requested()) {
        uint64_t now = read_elapsed_time();
        if (now - last_read >= interval) {
            last_read = now;
            show_cc1101_versions();
            after_read = read_elapsed_time();
            if (after_read - last_read >= interval) {
                /* Do not issue catch-up reads after a slow transaction. */
                last_read = after_read;
            }
        }
    }
}

static void gpio_scope_test(void)
{
    uint32_t frequency = read_reg(TIME_FREQ_ADDR);
    if (frequency < SCOPE_PHASE_HZ) {
        dprint("Error: elapsed time frequency must be at least 6400 Hz.\r\n");
        wait_uart_tx();
        return;
    }

    dprint("Signal   GPIO bit   FPGA port    Frequency\r\n");
    wait_uart_tx();
    dprint("TX CSN   0          GPIO0_D_24    100 Hz\r\n");
    wait_uart_tx();
    dprint("TX SCK   4          GPIO0_D_25    200 Hz\r\n");
    wait_uart_tx();
    dprint("TX MOSI  1          GPIO0_D_26    400 Hz\r\n");
    wait_uart_tx();
    dprint("RX CSN   2          GPIO0_D_28    800 Hz\r\n");
    wait_uart_tx();
    dprint("RX SCK   6          GPIO0_D_29    1600 Hz\r\n");
    wait_uart_tx();
    dprint("RX MOSI  3          GPIO0_D_30    3200 Hz\r\n");
    wait_uart_tx();
    dprint("\r\nPress Q to return to the menu.\r\n");
    wait_uart_tx();

    /* Diagnostic patterns may be interpreted as SPI commands by attached
     * radios. Their configuration is not preserved by this test.
     * Frequencies are nominal: timer rounding and MMIO latency add jitter. */
    uint32_t preserved = GPIO_OUT_REG & ~SCOPE_GPIO_MASK;
    uint32_t phase_ticks = frequency / SCOPE_PHASE_HZ;
    GPIO_OUT_REG = preserved;
    uint32_t previous_pattern = 0u;
    uint64_t start = read_elapsed_time();

    while (!quit_requested()) {
        /* Derive phase from elapsed time to skip missed edges without
         * issuing a burst of catch-up writes. Only six phase bits matter. */
        uint32_t phase = (uint32_t)((read_elapsed_time() - start) / phase_ticks);
        uint32_t pattern = (((phase >> 5) & 1u) << 0)  /* TX CSN  */
                         | (((phase >> 4) & 1u) << 4)  /* TX SCK  */
                         | (((phase >> 3) & 1u) << 1)  /* TX MOSI */
                         | (((phase >> 2) & 1u) << 2)  /* RX CSN  */
                         | (((phase >> 1) & 1u) << 6)  /* RX SCK  */
                         | ((phase & 1u) << 3);        /* RX MOSI */
        if (pattern != previous_pattern) {
            GPIO_OUT_REG = preserved | pattern;
            previous_pattern = pattern;
        }
    }

    /* Restore idle levels and resynchronize the driver's output shadow. */
    spi_init();
}

/* CC1101 register profile, 26 MHz crystal. See TI SWRS061I.
 * FREQ=0x10B071: 433.9198 MHz; DRATE_E=7, M=0x83: 4797.94 bit/s.
 * CHANBW_E=3, M=0: 101.5625 kHz; DEVIATN=0x15: 5.1575 kHz.
 * Every configuration register 0x00..0x2E is explicitly initialized. */
static const uint8_t radio_profile_4800[] = {
    0x2E, 0x2E, 0x2E, /* IOCFG2/1/0: high impedance, no GDO required */
    0x07,             /* FIFOTHR */
    0xD3, 0x91,       /* SYNC1/0: repeated to form D391D391 */
    0x08,             /* PKTLEN: eight payload bytes */
    0x04,             /* PKTCTRL1: append status, no CRC autoflush/address */
    0x44,             /* PKTCTRL0: whitening + CRC, fixed length FIFO mode */
    0x00, 0x00,       /* ADDR, CHANNR */
    0x06, 0x00,       /* FSCTRL1/0: IF 152.34 kHz, no frequency offset */
    0x10, 0xB0, 0x71, /* FREQ2/1/0 */
    0xC7, 0x83,       /* MDMCFG4/3: RX bandwidth and data rate */
    0x13,             /* MDMCFG2: GFSK, 30/32 sync, Manchester off */
    0x72, 0xF8,       /* MDMCFG1/0: 24-byte preamble, FEC off */
    0x15,             /* DEVIATN */
    0x07,             /* MCSM2: no RX timeout */
    0x00,             /* MCSM1: CCA off, RX/TX return to IDLE */
    0x18,             /* MCSM0: calibrate on IDLE -> RX/TX */
    0x16, 0x6C,       /* FOCCFG, BSCFG */
    0x43, 0x40, 0x91, /* AGCCTRL2/1/0 */
    0x87, 0x6B, 0xFB, /* WOREVT1/0, WORCTRL (WOR unused) */
    0x56, 0x10,       /* FREND1/0: use PATABLE entry zero */
    0xE9, 0x2A, 0x00, 0x1F, /* FSCAL3/2/1/0 */
    0x41, 0x00,       /* RCCTRL1/0 */
    0x59, 0x7F, 0x3F, /* FSTEST, PTEST, AGCTEST */
    0x81, 0x35, 0x09  /* TEST2/1/0: narrow-band RX settings */
};

/* 26 MHz: 249938.96 bit/s, +/-126953.125 Hz, RX BW 541666.67 Hz. */
static const uint8_t radio_profile_250000[] = {
    0x2E, 0x2E, 0x2E, /* IOCFG2/1/0 */
    0x07, 0xD3, 0x91, /* FIFOTHR, SYNC1/0 */
    0x08, 0x04, 0x44, /* PKTLEN, PKTCTRL1/0: status, CRC, whitening */
    0x00, 0x00,       /* ADDR, CHANNR */
    0x0C, 0x00,       /* FSCTRL1/0: IF 304.69 kHz */
    0x10, 0xB0, 0x71, /* FREQ2/1/0: 433.9198 MHz */
    0x2D, 0x3B,       /* MDMCFG4/3: bandwidth and data rate */
    0x13, 0x72, 0xF8, /* MDMCFG2/1/0: GFSK, 30/32, 24-byte preamble */
    0x62,             /* DEVIATN */
    0x07, 0x00, 0x18, /* MCSM2/1/0 */
    0x1D, 0x1C,       /* FOCCFG, BSCFG */
    0xC7, 0x00, 0xB0, /* AGCCTRL2/1/0 */
    0x87, 0x6B, 0xFB, /* WOREVT1/0, WORCTRL */
    0xB6, 0x10,       /* FREND1/0 */
    0xE9, 0x2A, 0x00, 0x1F, /* FSCAL3/2/1/0 */
    0x41, 0x00,       /* RCCTRL1/0 */
    0x59, 0x7F, 0x3F, /* FSTEST, PTEST, AGCTEST */
    0x88, 0x31, 0x09  /* TEST2/1/0: wide-band RX settings */
};

struct radio_test_profile {
    const uint8_t *registers;
    unsigned int register_count;
    int nominal_bps;
    const char *deviation_khz;
    const char *bandwidth_khz;
};

static const struct radio_test_profile radio_test_4800 = {
    radio_profile_4800, sizeof(radio_profile_4800), 4800, "5.16", "101.56"
};
static const struct radio_test_profile radio_test_250000 = {
    radio_profile_250000, sizeof(radio_profile_250000), 250000, "126.95", "541.67"
};

#define RADIO_PARTNUM  0x30u
#define RADIO_FREQEST  0x32u
#define RADIO_STATE    0x35u
#define RADIO_TXBYTES  0x3Au
#define RADIO_RXBYTES  0x3Bu
#define RADIO_PATABLE  0x3Eu
#define RADIO_FIFO     0x3Fu
#define RADIO_SRES     0x30u
#define RADIO_SRX      0x34u
#define RADIO_STX      0x35u
#define RADIO_SIDLE    0x36u
#define RADIO_SFRX     0x3Au
#define RADIO_SFTX     0x3Bu
#define RADIO_IDLE     0x01u
#define RADIO_RX       0x0Du

static const uint8_t radio_payload[8] = {0x00, 0xFF, 0xAA, 0x55,
                                         0x12, 0x34, 0x56, 0x78};
enum radio_result {
    RADIO_OK, RADIO_QUIT, RADIO_TIMEOUT, RADIO_ERROR,
    RADIO_CRC_ERROR, RADIO_MISMATCH
};

static uint64_t radio_timeout;
static bool radio_stop;

static bool radio_quit(void)
{
    if (quit_requested()) radio_stop = true;
    return radio_stop;
}

static bool radio_delay(uint64_t ticks)
{
    uint64_t start = read_elapsed_time();
    while (read_elapsed_time() - start < ticks) {
        if (radio_quit()) return false;
    }
    return !radio_quit();
}

/* Status counters can change while shifting out a byte. Require two
 * consecutive equal reads; bound retries even if the input is noisy. */
static bool radio_read_stable(uint8_t module, uint8_t address, uint8_t *value)
{
    uint8_t previous, current;
    if (cc1101_read_reg_ex(module, address, &previous) != SPI_BB_OK)
        return false;
    for (unsigned int i = 0; i < 16u; ++i) {
        if (cc1101_read_reg_ex(module, address, &current) != SPI_BB_OK)
            return false;
        if (current == previous) {
            *value = current;
            return true;
        }
        previous = current;
    }
    return false;
}

static enum radio_result radio_wait_state(uint8_t module, uint8_t expected,
                                          bool allow_quit)
{
    uint64_t start = read_elapsed_time();
    do {
        uint8_t state;
        if (allow_quit && radio_quit()) return RADIO_QUIT;
        if (!radio_read_stable(module, RADIO_STATE, &state)) return RADIO_ERROR;
        state &= 0x1Fu;
        if (state == expected) return RADIO_OK;
        /* SIDLE is also the recovery path out of FIFO error states. */
        if (expected != RADIO_IDLE && (state == 0x11u || state == 0x16u))
            return RADIO_ERROR;
    } while (read_elapsed_time() - start < radio_timeout);
    return RADIO_TIMEOUT;
}

static enum radio_result radio_flush(uint8_t module, bool allow_quit)
{
    if (cc1101_strobe_ex(module, RADIO_SIDLE, NULL) != SPI_BB_OK)
        return RADIO_ERROR;
    enum radio_result result = radio_wait_state(module, RADIO_IDLE, allow_quit);
    if (result != RADIO_OK) return result;
    if (cc1101_strobe_ex(module, RADIO_SFRX, NULL) != SPI_BB_OK ||
        cc1101_strobe_ex(module, RADIO_SFTX, NULL) != SPI_BB_OK)
        return RADIO_ERROR;
    return RADIO_OK;
}

static enum radio_result radio_configure(uint8_t module, uint32_t frequency,
                                         const struct radio_test_profile *profile)
{
    /* Fixed 1 ms guards replace SO-ready polling at the user's request.
     * Reset sequence: SCK high, SI low, CSN low/high, then SRES.
     * Leave CSN low through the post-SRES guard. */
    uint64_t guard = ((uint64_t)frequency + 999u) / 1000u;
    gpio_set(sck_pin[module], 1);
    gpio_set(mosi_pin[module], 0);
    gpio_set(cs_pin[module], 0);
    if (!radio_delay(guard)) goto cancelled;
    gpio_set(cs_pin[module], 1);
    if (!radio_delay(guard)) goto cancelled;
    cc1101_select(module);
    if (!radio_delay(guard)) goto cancelled;
    (void)spi_transfer(module, RADIO_SRES);
    if (!radio_delay(guard)) goto cancelled;
    cc1101_deselect(module);

    uint8_t part, version;
    if (cc1101_read_reg_ex(module, RADIO_PARTNUM, &part) != SPI_BB_OK ||
        cc1101_read_reg_ex(module, CC1101_VERSION_ADDR, &version) != SPI_BB_OK)
        return RADIO_ERROR;
    dprint("%s PARTNUM=0x%x VERSION=0x%x\r\n",
           module == SPI_MODULE_TX ? "TX" : "RX",
           (unsigned int)part, (unsigned int)version);
    wait_uart_tx();
    /* CC1101 PARTNUM is zero; allow silicon revisions other than 0x14. */
    if (part != 0x00u || version == 0x00u || version == 0xFFu)
        return RADIO_ERROR;

    for (unsigned int address = 0; address < profile->register_count; ++address) {
        if (radio_quit()) return RADIO_QUIT;
        if (cc1101_write_reg(module, (uint8_t)address,
                             profile->registers[address]) != SPI_BB_OK)
            return RADIO_ERROR;
    }
    const uint8_t power = 0x12u; /* TI 433 MHz table: nominal -30 dBm */
    if (cc1101_write_burst(module, RADIO_PATABLE, &power, 1u) != SPI_BB_OK)
        return RADIO_ERROR;
    for (unsigned int address = 0; address < profile->register_count; ++address) {
        uint8_t value;
        if (radio_quit()) return RADIO_QUIT;
        if (cc1101_read_reg_ex(module, (uint8_t)address, &value) != SPI_BB_OK)
            return RADIO_ERROR;
        if (value != profile->registers[address]) {
            dprint("%s configuration mismatch at 0x%x: expected 0x%x, got 0x%x\r\n",
                   module == SPI_MODULE_TX ? "TX" : "RX", address,
                   (unsigned int)profile->registers[address], (unsigned int)value);
            wait_uart_tx();
            return RADIO_ERROR;
        }
    }
    uint8_t value;
    if (cc1101_read_burst(module, RADIO_PATABLE, &value, 1u) != SPI_BB_OK ||
        value != power) {
        dprint("PATABLE verification failed.\r\n");
        wait_uart_tx();
        return RADIO_ERROR;
    }
    return radio_flush(module, true);

cancelled:
    cc1101_deselect(module);
    return RADIO_QUIT;
}

static void radio_print_bytes(const uint8_t *bytes, unsigned int count)
{
    static const char digits[] = "0123456789ABCDEF";
    for (unsigned int i = 0; i < count; ++i) {
        uart_send_byte((uint8_t)digits[bytes[i] >> 4]);
        uart_send_byte((uint8_t)digits[bytes[i] & 15u]);
        if (i + 1u < count) uart_send_byte(' ');
    }
}

struct radio_observation {
    bool complete;
    bool offset_valid;
    uint8_t offset_raw;
    uint8_t packet[10];
};

static void radio_print_measurements(const struct radio_observation *observation)
{
    if (!observation->complete) {
        dprint("RSSI: unavailable, LQI: unavailable, CRC: unavailable\r\n");
        wait_uart_tx();
        dprint("Frequency offset: unavailable\r\n");
        wait_uart_tx();
        return;
    }
    const uint8_t *packet = observation->packet;
    int raw = packet[8];
    if (raw >= 128) raw -= 256;
    int half_dbm = raw - 148; /* RSSI_signed/2 - 74 dBm */
    unsigned int magnitude = (unsigned int)(half_dbm < 0 ? -half_dbm : half_dbm);
    dprint("RSSI: %s%d.%d dBm, LQI: %d/127, CRC: %s\r\n",
           half_dbm < 0 ? "-" : "", (int)(magnitude / 2u),
           (magnitude & 1u) ? 5 : 0, (int)(packet[9] & 0x7Fu),
           (packet[9] & 0x80u) ? "OK" : "ERROR");
    wait_uart_tx();
    if (observation->offset_valid) {
        int offset = observation->offset_raw;
        if (offset >= 128) offset -= 256;
        /* FREQEST is signed, with resolution FXTAL/2^14. Round the
         * magnitude to nearest Hz, then restore the sign (no float). */
        int64_t scaled = (int64_t)offset * INT64_C(26000000);
        int hz = scaled < 0
               ? -(int)((-scaled + 8192) / 16384)
               : (int)((scaled + 8192) / 16384);
        dprint("Frequency offset: %d Hz (raw: %d)\r\n", hz, offset);
    } else {
        dprint("Frequency offset: unavailable\r\n");
    }
    wait_uart_tx();
}

static void radio_print_packet(const struct radio_observation *observation)
{
    dprint("TX: ");
    radio_print_bytes(radio_payload, sizeof(radio_payload));
    dprint("\r\n");
    wait_uart_tx();
    dprint("RX: ");
    radio_print_bytes(observation->packet, sizeof(radio_payload));
    dprint("\r\n");
    wait_uart_tx();
    radio_print_measurements(observation);
}

/* Acquisition is silent: callers choose per-packet or periodic reporting. */
static enum radio_result radio_exchange(struct radio_observation *observation)
{
    observation->complete = false;
    observation->offset_valid = false;
    enum radio_result result = radio_flush(SPI_MODULE_TX, true);
    if (result != RADIO_OK) return result;
    result = radio_flush(SPI_MODULE_RX, true);
    if (result != RADIO_OK) return result;
    if (cc1101_strobe_ex(SPI_MODULE_RX, RADIO_SRX, NULL) != SPI_BB_OK)
        return RADIO_ERROR;
    result = radio_wait_state(SPI_MODULE_RX, RADIO_RX, true);
    if (result != RADIO_OK) return result;
    if (radio_quit()) return RADIO_QUIT;
    if (cc1101_write_burst(SPI_MODULE_TX, RADIO_FIFO, radio_payload,
                           sizeof(radio_payload)) != SPI_BB_OK ||
        cc1101_strobe_ex(SPI_MODULE_TX, RADIO_STX, NULL) != SPI_BB_OK)
        return RADIO_ERROR;

    /* Do not mistake the initial calibration/IDLE interval for TX done:
     * require both an empty TX FIFO and the final IDLE state. */
    uint64_t start = read_elapsed_time();
    while (1) {
        uint8_t bytes, state;
        if (radio_quit()) return RADIO_QUIT;
        if (!radio_read_stable(SPI_MODULE_TX, RADIO_TXBYTES, &bytes) ||
            !radio_read_stable(SPI_MODULE_TX, RADIO_STATE, &state))
            return RADIO_ERROR;
        if (bytes & 0x80u) return RADIO_ERROR;
        if ((bytes & 0x7Fu) == 0u && (state & 0x1Fu) == RADIO_IDLE) break;
        if (read_elapsed_time() - start >= radio_timeout) {
            return RADIO_TIMEOUT;
        }
    }

    start = read_elapsed_time();
    while (1) {
        uint8_t bytes;
        if (radio_quit()) return RADIO_QUIT;
        if (!radio_read_stable(SPI_MODULE_RX, RADIO_RXBYTES, &bytes))
            return RADIO_ERROR;
        if (bytes & 0x80u) return RADIO_ERROR;
        if ((bytes & 0x7Fu) >= 10u) {
            if ((bytes & 0x7Fu) != sizeof(observation->packet)) {
                return RADIO_ERROR;
            }
            result = radio_wait_state(SPI_MODULE_RX, RADIO_IDLE, true);
            if (result != RADIO_OK) return result;
            if (cc1101_read_burst(SPI_MODULE_RX, RADIO_FIFO, observation->packet,
                                  sizeof(observation->packet)) != SPI_BB_OK)
                return RADIO_ERROR;
            observation->complete = true;
            /* RX is now IDLE: capture this packet's estimate before any
             * restart/flush. A missing estimate does not reject payload. */
            observation->offset_valid = cc1101_read_reg_ex(SPI_MODULE_RX,
                RADIO_FREQEST, &observation->offset_raw) == SPI_BB_OK;
            if (!(observation->packet[9] & 0x80u)) return RADIO_CRC_ERROR;
            for (unsigned int i = 0; i < sizeof(radio_payload); ++i) {
                if (observation->packet[i] != radio_payload[i]) return RADIO_MISMATCH;
            }
            return RADIO_OK;
        }
        if (read_elapsed_time() - start >= radio_timeout) {
            return RADIO_TIMEOUT;
        }
    }
}

static void radio_print_counts(const char *name, unsigned int attempts,
                                const unsigned int *counts)
{
    dprint("%s: attempts=%d PASS=%d CRC=%d\r\n", name, (int)attempts,
           (int)counts[RADIO_OK], (int)counts[RADIO_CRC_ERROR]);
    wait_uart_tx();
    dprint("MISMATCH=%d TIMEOUT=%d RADIO=%d\r\n",
           (int)counts[RADIO_MISMATCH], (int)counts[RADIO_TIMEOUT],
           (int)counts[RADIO_ERROR]);
    wait_uart_tx();
}

static void radio_print_period(unsigned int attempts, const unsigned int *counts,
                                unsigned int total, const unsigned int *totals,
                                const struct radio_observation *last)
{
    bool error = counts[RADIO_CRC_ERROR] || counts[RADIO_MISMATCH] ||
                 counts[RADIO_TIMEOUT] || counts[RADIO_ERROR];
    dprint("Period status: %s\r\n", attempts == 0u ? "NO COMPLETED PACKET" :
           (error ? "ERROR" : "OK"));
    wait_uart_tx();
    radio_print_counts("Period", attempts, counts);
    radio_print_counts("Totals", total, totals);
    dprint("Last complete RX packet in this period:\r\n");
    wait_uart_tx();
    radio_print_measurements(last);
}

static void radio_packet_test(const struct radio_test_profile *profile,
                              bool continuous)
{
    uint32_t frequency = read_reg(TIME_FREQ_ADDR);
    if (frequency == 0u) {
        dprint("Error: elapsed time frequency is zero.\r\n");
        wait_uart_tx();
        return;
    }
    radio_stop = false;
    radio_timeout = ((uint64_t)frequency + 3u) / 4u;
    uint64_t interval = ((uint64_t)frequency + 1u) / 2u;
    unsigned int counts[6] = {0};
    unsigned int attempts = 0;
    unsigned int period_counts[6] = {0};
    unsigned int period_attempts = 0;
    struct radio_observation last = {0};
    uint64_t report_start = 0;
    static const char *const labels[] = {
        "PASS", "CANCELLED", "TIMEOUT", "RADIO ERROR", "CRC ERROR", "PAYLOAD MISMATCH"
    };

    dprint("GFSK TX -> RX: 433.92 MHz, %d bit/s, -30 dBm, crystal 26 MHz\r\n",
           profile->nominal_bps);
    wait_uart_tx();
    dprint("Deviation: +/-%s kHz; RX bandwidth: %s kHz\r\n",
           profile->deviation_khz, profile->bandwidth_khz);
    wait_uart_tx();
    dprint("Preamble: 24 bytes; sync: D391D391 (30/32); CRC16: enabled\r\n");
    wait_uart_tx();
    dprint("Whitening: enabled; payload: 00 FF AA 55 12 34 56 78\r\n");
    wait_uart_tx();
    dprint("Press Q to return to the menu.\r\n");
    wait_uart_tx();
    if (continuous) {
        dprint("Packets run back-to-back; reports every second.\r\n");
        wait_uart_tx();
    }

    for (uint8_t module = SPI_MODULE_TX; module <= SPI_MODULE_RX; ++module) {
        enum radio_result result = radio_configure(module, frequency, profile);
        if (result != RADIO_OK) {
            if (result != RADIO_QUIT) {
                dprint("%s initialization failed: %s. Check MISO and crystal.\r\n",
                       module == SPI_MODULE_TX ? "TX" : "RX", labels[result]);
                wait_uart_tx();
            }
            goto cleanup;
        }
    }
    report_start = read_elapsed_time();
    while (!radio_quit()) {
        uint64_t start = read_elapsed_time();
        struct radio_observation observation = {0};
        enum radio_result result = radio_exchange(&observation);
        if (result == RADIO_QUIT) break;
        ++attempts;
        ++counts[result];
        ++period_attempts;
        ++period_counts[result];
        if (observation.complete) last = observation;
        if (!continuous) {
            if (observation.complete) radio_print_packet(&observation);
            dprint("Packet %d: %s\r\n", (int)attempts, labels[result]);
            wait_uart_tx();
            dprint("Totals: PASS=%d CRC=%d MISMATCH=%d TIMEOUT=%d RADIO=%d\r\n",
                   (int)counts[RADIO_OK], (int)counts[RADIO_CRC_ERROR],
                   (int)counts[RADIO_MISMATCH], (int)counts[RADIO_TIMEOUT],
                   (int)counts[RADIO_ERROR]);
            wait_uart_tx();
        }
        if (result != RADIO_OK) {
            enum radio_result tx = radio_flush(SPI_MODULE_TX, true);
            enum radio_result rx = radio_flush(SPI_MODULE_RX, true);
            if (tx != RADIO_OK || rx != RADIO_OK) {
                if (!radio_stop) {
                    dprint("Radio recovery failed; stopping test.\r\n");
                    wait_uart_tx();
                }
                break;
            }
        }
        if (continuous) {
            if (read_elapsed_time() - report_start >= frequency) {
                radio_print_period(period_attempts, period_counts, attempts, counts, &last);
                for (unsigned int i = 0; i < 6u; ++i) period_counts[i] = 0u;
                period_attempts = 0u;
                last.complete = false;
                last.offset_valid = false;
                /* Start the next period after UART output, no catch-up. */
                report_start = read_elapsed_time();
            }
        } else {
            /* Preserve start-to-start spacing in modes 5 and 6. */
            while (read_elapsed_time() - start < interval) {
                if (radio_quit()) goto cleanup;
            }
        }
    }

cleanup:
    for (uint8_t module = SPI_MODULE_TX; module <= SPI_MODULE_RX; ++module) {
        if (radio_flush(module, false) != RADIO_OK) {
            dprint("Warning: %s could not be returned to IDLE.\r\n",
                   module == SPI_MODULE_TX ? "TX" : "RX");
            wait_uart_tx();
        }
        cc1101_deselect(module);
    }
    if (continuous && period_attempts != 0u) {
        dprint("Final partial period:\r\n");
        wait_uart_tx();
        radio_print_period(period_attempts, period_counts, attempts, counts, &last);
    }
    dprint("Summary: attempts=%d PASS=%d CRC=%d MISMATCH=%d TIMEOUT=%d RADIO=%d\r\n",
           (int)attempts, (int)counts[RADIO_OK], (int)counts[RADIO_CRC_ERROR],
           (int)counts[RADIO_MISMATCH], (int)counts[RADIO_TIMEOUT],
           (int)counts[RADIO_ERROR]);
    wait_uart_tx();
}

static void show_menu(void)
{
    dprint("\r\n1. Show FPGA and ELF build information\r\n");
    wait_uart_tx();
    dprint("2. Read CC1101 versions (TX and RX)\r\n");
    wait_uart_tx();
    dprint("3. Read CC1101 versions every 500 ms\r\n");
    wait_uart_tx();
    dprint("4. GPIO scope test (100 to 3200 Hz)\r\n");
    wait_uart_tx();
    dprint("5. GFSK TX/RX packet test (4800 bit/s)\r\n");
    wait_uart_tx();
    dprint("6. GFSK TX/RX packet test (250000 bit/s)\r\n");
    wait_uart_tx();
    dprint("7. Continuous 4800 bit/s packet test (1-second report)\r\n");
    wait_uart_tx();
    dprint("Select an option: ");
    wait_uart_tx();
}

int main(void)
{
    spi_init();

    while (1) {
        show_menu();
        uint8_t choice;
        do {
            choice = dget();
        } while (choice == '\r' || choice == '\n');

        dprint("\r\n");
        wait_uart_tx();
        switch (choice) {
        case '1':
            show_build_information();
            break;
        case '2':
            show_cc1101_versions();
            break;
        case '3':
            read_versions_periodically();
            break;
        case '4':
            gpio_scope_test();
            break;
        case '5':
            radio_packet_test(&radio_test_4800, false);
            break;
        case '6':
            radio_packet_test(&radio_test_250000, false);
            break;
        case '7':
            radio_packet_test(&radio_test_4800, true);
            break;
        default:
            dprint("Invalid option. Please select 1, 2, 3, 4, 5, 6 or 7.\r\n");
            wait_uart_tx();
            break;
        }
    }
}
