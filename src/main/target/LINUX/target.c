/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"
#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"

//BUSDEV_REGISTER_SPI_TAG(busdev_bmi270,   DEVHW_BMI270,  BMI270_SPI_BUS,   BMI270_CS_PIN,   NONE,   0,  DEVFLAGS_NONE,  IMU_BMI270_ALIGN);
//BUSDEV_REGISTER_SPI_TAG(busdev_icm42688, DEVHW_ICM42605, ICM42688_SPI_BUS, ICM42688_CS_PIN, NONE, 0, DEVFLAGS_NONE, IMU_ICM42605_ALIGN);

timerHardware_t timerHardware[] = {};


const int timerHardwareCount = 0;
