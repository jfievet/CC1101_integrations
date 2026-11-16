#include "spi_bitbang.h"

#define GPIO_OUT_REG (*(volatile uint32_t *)(uintptr_t)(SPI_GPIO_BASE + SPI_GPIO_OUT_OFFSET))
#define GPIO_IN_REG  (*(volatile uint32_t *)(uintptr_t)(SPI_GPIO_BASE + SPI_GPIO_IN_OFFSET))
#define BIT(n) (UINT32_C(1) << (n))

/* Output GPIO indexes from top.vhd */
static const uint8_t cs_pin[2]   = {0u, 2u};
static const uint8_t mosi_pin[2] = {1u, 3u};
static const uint8_t sck_pin[2]  = {4u, 6u};
static const uint8_t miso_pin[2] = {5u, 7u};

static uint32_t gpio_shadow;
static uint32_t half_period_cycles = 32u;
static int initialized;

static int valid(uint8_t module) { return module <= SPI_MODULE_RX; }

static void gpio_set(uint8_t pin, int high)
{
    if (high) gpio_shadow |= BIT(pin);
    else      gpio_shadow &= ~BIT(pin);
    GPIO_OUT_REG = gpio_shadow;
}

static int gpio_get(uint8_t pin)
{
    return (GPIO_IN_REG & BIT(pin)) != 0u;
}

static void delay_half_period(void)
{
    /* Approximate delay, not calibrated; inspect disassembly and scope.
     * A compiler barrier prevents the loop from being optimized away. */
    for (uint32_t i = 0; i < half_period_cycles; ++i)
        __asm__ volatile ("" ::: "memory");
}

void spi_set_half_period_cycles(uint32_t cycles)
{
    half_period_cycles = cycles;
}

void spi_init(void)
{
    /* Assumes OUT register is readable and retains its output latch. */
    gpio_shadow = GPIO_OUT_REG;
    gpio_shadow |= BIT(0) | BIT(2);  /* CSN inactive */
    gpio_shadow &= ~(BIT(1) | BIT(3) | BIT(4) | BIT(6));
    GPIO_OUT_REG = gpio_shadow;
    initialized = 1;
}

uint8_t spi_transfer(uint8_t module, uint8_t data)
{
    uint8_t received = 0u;
    if (!initialized || !valid(module)) return 0u;

    for (int bit = 7; bit >= 0; --bit) {
        gpio_set(mosi_pin[module], (data >> bit) & 1u);
        delay_half_period();
        gpio_set(sck_pin[module], 1);
        delay_half_period();
        received = (uint8_t)((received << 1) | (uint8_t)gpio_get(miso_pin[module]));
        gpio_set(sck_pin[module], 0);
    }
    return received;
}

void cc1101_select(uint8_t module)
{
    if (!initialized || !valid(module)) return;
    gpio_set(sck_pin[module], 0);
    gpio_set(cs_pin[module], 0);
    delay_half_period();
}

void cc1101_deselect(uint8_t module)
{
    if (!initialized || !valid(module)) return;
    gpio_set(sck_pin[module], 0);
    gpio_set(cs_pin[module], 1);
    delay_half_period();
}

int cc1101_wait_ready(uint8_t module, uint32_t timeout_polls)
{
    if (!initialized || !valid(module)) return SPI_BB_EINVAL;
    while (timeout_polls--) {
        if (!gpio_get(miso_pin[module])) return SPI_BB_OK;
        delay_half_period();
    }
    return SPI_BB_ETIME;
}

static int start_transaction(uint8_t module)
{
    if (!initialized || !valid(module)) return SPI_BB_EINVAL;
    cc1101_select(module);
    /* Assume the radio is ready: do not gate clock generation on MISO.
     * Keep the CSN setup delay provided by cc1101_select(). */
    return SPI_BB_OK;
}

int cc1101_read_reg_ex(uint8_t module, uint8_t addr, uint8_t *value)
{
    if (!value || addr > 0x3fu) return SPI_BB_EINVAL;
    int rc = start_transaction(module);
    if (rc != SPI_BB_OK) return rc;
    /* CC1101 status registers 0x30..0x3D require burst bit even for 1 byte. */
    uint8_t header = (uint8_t)(addr | CC1101_READ);
    if (addr >= 0x30u && addr <= 0x3du) header |= CC1101_BURST;
    spi_transfer(module, header);
    *value = spi_transfer(module, 0x00u);
    cc1101_deselect(module);
    return SPI_BB_OK;
}

uint8_t cc1101_read_reg(uint8_t module, uint8_t addr)
{
    uint8_t value = 0xffu;
    (void)cc1101_read_reg_ex(module, addr, &value);
    return value;
}

int cc1101_write_reg(uint8_t module, uint8_t addr, uint8_t value)
{
    /* Single-register writes are for configuration registers 0x00..0x2E. */
    if (addr > 0x2eu) return SPI_BB_EINVAL;
    int rc = start_transaction(module);
    if (rc != SPI_BB_OK) return rc;
    spi_transfer(module, addr);
    spi_transfer(module, value);
    cc1101_deselect(module);
    return SPI_BB_OK;
}

int cc1101_strobe_ex(uint8_t module, uint8_t cmd, uint8_t *status)
{
    if (cmd < 0x30u || cmd > 0x3du) return SPI_BB_EINVAL;
    int rc = start_transaction(module);
    if (rc != SPI_BB_OK) return rc;
    uint8_t result = spi_transfer(module, cmd);
    cc1101_deselect(module);
    if (status) *status = result;
    return SPI_BB_OK;
}

void cc1101_strobe(uint8_t module, uint8_t cmd)
{
    (void)cc1101_strobe_ex(module, cmd, NULL);
}

int cc1101_read_burst(uint8_t module, uint8_t addr, uint8_t *data, size_t len)
{
    if (!data || !len || addr > 0x3fu || (addr > 0x2eu && addr != 0x3fu))
        return SPI_BB_EINVAL;
    int rc = start_transaction(module);
    if (rc != SPI_BB_OK) return rc;
    spi_transfer(module, (uint8_t)(addr | CC1101_READ | CC1101_BURST));
    for (size_t i = 0; i < len; ++i) data[i] = spi_transfer(module, 0x00u);
    cc1101_deselect(module);
    return SPI_BB_OK;
}

int cc1101_write_burst(uint8_t module, uint8_t addr, const uint8_t *data, size_t len)
{
    if (!data || !len || (addr > 0x2eu && addr != 0x3fu))
        return SPI_BB_EINVAL;
    int rc = start_transaction(module);
    if (rc != SPI_BB_OK) return rc;
    spi_transfer(module, (uint8_t)(addr | CC1101_BURST));
    for (size_t i = 0; i < len; ++i) spi_transfer(module, data[i]);
    cc1101_deselect(module);
    return SPI_BB_OK;
}
