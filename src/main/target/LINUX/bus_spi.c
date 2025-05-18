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

#ifdef USE_SPI

#include "drivers/bus_spi.h"
//#include "drivers/exti.h"
//#include "drivers/io.h"
//#include "drivers/io_impl.h"
//#include "drivers/rcc.h"



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
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance)
{
    if (instance == SPI1)
        return SPIDEV_1;

    if (instance == SPI2)
        return SPIDEV_2;

    if (instance == SPI3)
        return SPIDEV_3;

    return SPIINVALID;
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

uint32_t spiTimeoutUserCallback(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return -1;
    }
    spiHardwareMap[device].errorCount++;
    return spiHardwareMap[device].errorCount;
}

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data)
{
    uint16_t spiTimeout = 1000;



    return ((uint8_t)SPI_I2S_ReceiveData(instance));
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance)
{
    return false;
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len)
{
    uint16_t spiTimeout = 1000;



    return true;
}

void spiSetSpeed(SPI_TypeDef *instance, SPIClockSpeed_e speed)
{
#define BR_CLEAR_MASK 0xFFC7
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return;
    }

    SPI_Cmd(instance, DISABLE);



    SPI_Cmd(instance, ENABLE);
}

uint16_t spiGetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device == SPIINVALID) {
        return 0;
    }
    return spiHardwareMap[device].errorCount;
}

void spiResetErrorCounter(SPI_TypeDef *instance)
{
    SPIDevice device = spiDeviceByInstance(instance);
    if (device != SPIINVALID) {
        spiHardwareMap[device].errorCount = 0;
    }
}

SPI_TypeDef * spiInstanceByDevice(SPIDevice device)
{
    return spiHardwareMap[device].dev;
}
#endif