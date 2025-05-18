/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/bus_spi.h"
//#include "dma.h"
#include "drivers/io.h"
//#include "io_impl.h"
//#include "drivers/nvic.h"
//#include "rcc.h"

static spiDevice_t spiHardwareMap[] = {
};

#ifndef SPI1
#define SPI1 NULL
#endif
#ifndef SPI2
#define SPI2 NULL
#endif
#ifndef SPI3
#define SPI3 NULL
#endif

#ifndef SPI4
#define SPI4 NULL
#endif
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;

    if (instance == SPI4)
        return SPIDEV_4;

    return SPIINVALID;
}

void spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return;
    }

    spiHardwareMap[device].errorCount++;
}

bool spiInitDevice(SPIDevice device, bool leadingEdge)
{
    spiDevice_t *spi = &(spiHardwareMap[device]);

    if (!spi->dev) {
        return false;
    }

    if (spi->initDone) {
        return true;
    }

    spi->initDone = true;
    return true;
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
    uint8_t value = 0xFF;
    if (!spiTransfer(instance, &value, &txByte, 1)) {
        return 0xFF;
    }
    return value;
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{

}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *rxData, const uint8_t *txData, int len)
{

    return true;
}

void spiSetSpeed(SPI_TypeDef *instance, SPIClockSpeed_e speed)
{
    SPIDevice device = spiDeviceByInstance(instance);

}

SPI_TypeDef * spiInstanceByDevice(SPIDevice device)
{
    return spiHardwareMap[device].dev;
}
