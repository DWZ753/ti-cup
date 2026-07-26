#include "spi.h"
#include "ti_msp_dl_config.h"

#define SPI_MAX_INSTANCES 2

/* ========== 句柄注册表 ========== */
static SPI_Handle  g_handle_pool[SPI_MAX_INSTANCES];
static SPI_Handle *g_spi_instances[SPI_MAX_INSTANCES];
static uint8_t     g_spi_count;

static uint8_t get_spi_index(SPI_Regs *spi)
{
    if (spi == SPI0) return 0;
    if (spi == SPI1) return 1;
    return SPI_MAX_INSTANCES;
}

/* ========== 通用 API ========== */

SPI_Handle* SPI_Init(const SPI_Config *config)
{
    uint8_t idx;

    if (config == NULL || config->spi == NULL) return NULL;
    idx = get_spi_index(config->spi);
    if (idx >= SPI_MAX_INSTANCES) return NULL;
    if (g_spi_instances[idx] != NULL) return NULL;
    if (g_spi_count >= SPI_MAX_INSTANCES) return NULL;

    SPI_Handle *h = &g_handle_pool[g_spi_count];
    h->spi = config->spi;

    g_spi_instances[idx] = h;
    g_spi_count++;

    return h;
}

uint8_t SPI_Transfer(SPI_Handle *h, uint8_t tx_data)
{
    DL_SPI_transmitData8(h->spi, tx_data);
    while (DL_SPI_isRXFIFOEmpty(h->spi));
    uint8_t rx = DL_SPI_receiveData8(h->spi);
    while (DL_SPI_isBusy(h->spi));
    return rx;
}
