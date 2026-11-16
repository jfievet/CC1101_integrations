#ifndef SPI_BITBANG_H
#define SPI_BITBANG_H

#include <stdint.h>
#include <stddef.h>

/* Adapt these three definitions to your SoC GPIO register map. */
#ifndef SPI_GPIO_BASE
#define SPI_GPIO_BASE       0x00200000u /* Assumption: verify */
#endif
#ifndef SPI_GPIO_OUT_OFFSET
#define SPI_GPIO_OUT_OFFSET 0x00u       /* Assumption: verify */
#endif
#ifndef SPI_GPIO_IN_OFFSET
#define SPI_GPIO_IN_OFFSET  0x04u       /* Assumption: verify */
#endif

/* top.vhd mapping: module 0 = TX, module 1 = RX. */
#define SPI_MODULE_TX 0u
#define SPI_MODULE_RX 1u

/* CC1101 command header bits */
#define CC1101_READ   0x80u
#define CC1101_BURST  0x40u

/* Error codes */
#define SPI_BB_OK       0
#define SPI_BB_EINVAL -1
#define SPI_BB_ETIME  -2

/* GPIO writes are read-modify-write from a software shadow. This driver
 * must be the sole writer of the GPIO output register, or all writers must
 * coordinate updates to avoid overwriting each other's GPIO bits. */
void spi_init(void);
void spi_set_half_period_cycles(uint32_t cycles);
uint8_t spi_transfer(uint8_t module, uint8_t data);
void cc1101_select(uint8_t module);
void cc1101_deselect(uint8_t module);
int cc1101_wait_ready(uint8_t module, uint32_t timeout_polls);

int cc1101_read_reg_ex(uint8_t module, uint8_t addr, uint8_t *value);
int cc1101_write_reg(uint8_t module, uint8_t addr, uint8_t value);
int cc1101_strobe_ex(uint8_t module, uint8_t cmd, uint8_t *status);
int cc1101_read_burst(uint8_t module, uint8_t addr, uint8_t *data, size_t len);
int cc1101_write_burst(uint8_t module, uint8_t addr, const uint8_t *data, size_t len);

/* Convenience wrappers. Prefer _ex for error reporting. */
uint8_t cc1101_read_reg(uint8_t module, uint8_t addr);
void cc1101_strobe(uint8_t module, uint8_t cmd);

#endif