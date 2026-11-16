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
        default:
            dprint("Invalid option. Please select 1, 2, 3 or 4.\r\n");
            wait_uart_tx();
            break;
        }
    }
}
