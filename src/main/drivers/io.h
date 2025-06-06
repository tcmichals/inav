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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "resource.h"

#include "drivers/io_types.h"

// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred

// expand pinid to to ioTag_t
#define IO_TAG(pinid) DEFIO_TAG(pinid)

#if defined(STM32F7) || defined(STM32H7)

//speed is packed inside modebits 5 and 2,
#define IO_CONFIG(mode, speed, pupd) ((mode) | ((speed) << 2) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT_OD, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_FAST     IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF_PP,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_AF_OD_UP       IO_CONFIG(GPIO_MODE_AF_OD,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,      GPIO_SPEED_FREQ_LOW,  GPIO_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP)

#elif defined(STM32F4)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_PP, GPIO_PuPd_NOPULL)  // TODO
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_25MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_Mode_OUT, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_FAST     IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_PP, GPIO_PuPd_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#define IOCFG_AF_OD_UP       IO_CONFIG(GPIO_Mode_AF,  0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_IPD            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_Mode_IN,  0, 0,             GPIO_PuPd_NOPULL)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_Mode_IN,  GPIO_Speed_25MHz, 0, GPIO_PuPd_UP)

#elif defined(AT32F43x)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)  // TODO
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_OPEN_DRAIN, GPIO_PULL_NONE)
#define IOCFG_AF_PP_FAST     IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_DOWN)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_DOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN, GPIO_PULL_NONE)
#define IOCFG_AF_OD_UP       IO_CONFIG(GPIO_MODE_MUX,  GPIO_DRIVE_STRENGTH_STRONGER, GPIO_OUTPUT_OPEN_DRAIN, GPIO_PULL_UP)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_DOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_NONE)
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,  GPIO_DRIVE_STRENGTH_MODERATE, GPIO_OUTPUT_PUSH_PULL, GPIO_PULL_UP)

#elif defined(UNIT_TEST) || defined(SITL_BUILD) || defined(LINUX_BUILD)

# define IOCFG_OUT_PP         0
# define IOCFG_OUT_OD         0
# define IOCFG_AF_PP          0
# define IOCFG_AF_OD          0
# define IOCFG_AF_OD_UP       0
# define IOCFG_IPD            0
# define IOCFG_IPU            0
# define IOCFG_IN_FLOATING    0

#else
# warning "Unknown TARGET"
#endif

// declare available IO pins. Available pins are specified per target
#include "io_def.h"

bool IORead(IO_t io);
void IOWrite(IO_t io, bool value);
void IOHi(IO_t io);
void IOLo(IO_t io);
void IOToggle(IO_t io);

void IOInit(IO_t io, resourceOwner_e owner, resourceType_e resource, uint8_t index);
void IORelease(IO_t io);  // unimplemented
resourceOwner_e IOGetOwner(IO_t io);
resourceType_e IOGetResources(IO_t io);
IO_t IOGetByTag(ioTag_t tag);

void IOConfigGPIO(IO_t io, ioConfig_t cfg);
void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af);

void IOInitGlobal(void);
