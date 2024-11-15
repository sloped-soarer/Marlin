/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Xvico X1 board (STM32F103RCT6) pin assignments.
 */

#include "env_validate.h"

#if HOTENDS > 1 || E_STEPPERS > 1
  #error "XVICO X1/X3 supports up to 1 hotends / E steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "XVICO X-MICRO"
#endif
#define BOARD_WEBSITE_URL "www.xvico3d.com"

#define BOARD_NO_NATIVE_USB

//#define DISABLE_DEBUG
#define DISABLE_JTAG

//
// Servos
//
//#define SERVO0_PIN                          PA3

//
// Limit Switches
//
#define X_STOP_PIN    PC7
#define Y_STOP_PIN    PA8
#define Z_MIN_PIN     PA11
//#  LEVEL_PIN        PB13
//
// Steppers
//
#define X_STEP_PIN    PB2
#define X_DIR_PIN     PB12
#define X_ENABLE_PIN  PC6

#define Y_STEP_PIN    PC5 
#define Y_DIR_PIN     PB0
#define Y_ENABLE_PIN  PB1

#define Z_STEP_PIN    PC0
#define Z_DIR_PIN     PC1
#define Z_ENABLE_PIN  PC4

#define E0_STEP_PIN   PC15
#define E0_DIR_PIN    PC14
#define E0_ENABLE_PIN PC13

//
// Heaters 0,1 / Fans / Bed
//
#define HEATER_0_PIN           PA1
#define FAN_PIN                PA2
#define HEATER_BED_PIN         PA0
//#define CONTROLLER_FAN_PIN     PA3
//#define EXTRUDER_0_AUTO_FAN_PIN PA3 in Configuration_adv.h

//
// Temperature Sensors
//
#define TEMP_BED_PIN    PC3  // TB
#define TEMP_0_PIN      PC2  // TH1

#define FIL_RUNOUT_PIN  PA12 // MT_DET

//
// LCD Pins
//
/*
 * TFT is 2.4" 320x240 ILI9341 with touchscreen using XPT2046 controller.
 * See https://tinkerfiddle.blogspot.com/2019/02/xvico-pioneer-x3-board-reverse.html
 * for an almost complete reverse engineer of the Xvico X-MICRO board. Many thanks to Jim Wagner. 
 *
 * Pins PB10 (I2C2_SCL/USART3_TX) and PB11 (I2C2_SDA/USART3_RX) appear to be unused.
 * Pins PB14 (SPI2_MISO/TIM1_CH2N/USART3_RTS) and PB15 (SPI2_MOSI/I2S2_SD/TIM1_CH3N) appear to be unused.
 *
 * This is a fudge ! ...
 * PB15 is used to satisfy the compiler, as it is not used and because the TFT_CS on the Binmaker
 * Display Carrier board is permanently connected to ground (active) !.
 * 
 */
 
#define TFT_CS_PIN      PB15 // Chip select permanently active on binmaker TFT carrier board.
 
#define TFT_RESET_PIN   PB7
#define TFT_A0_PIN      PB6
#define TFT_MOSI_PIN    PB5
#define TFT_MISO_PIN    PB4
#define TFT_SCK_PIN     PB3

#define TOUCH_MISO_PIN  PA6
#define TOUCH_MOSI_PIN  PA7
#define TOUCH_SCK_PIN   PA5
#define TOUCH_CS_PIN    PB8


//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SDIO_SUPPORT
  #define SD_DETECT_PIN                     PA15
  #define ONBOARD_SD_CS_PIN                 PC11
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif


//
// Persistent Storage
// If no option is selected below the SD Card will be used
//
// The Xvico X-MICRO board has a Winbond W25Q64JVSSIQ fitted.

#if NO_EEPROM_SELECTED
//#define SPI_EEPROM
#define SPI_FLASH                             // need MARLIN_DEV_MODE for M993/M994 EEPROM backup tests
//#define FLASH_EEPROM_EMULATION
#endif

#if ENABLED(SPI_EEPROM)
//  #define SPI_CHAN_EEPROM1                     1
//  #define SPI_EEPROM1_CS_PIN     PA4
//  #define EEPROM_SCK_PIN         PA5
//  #define EEPROM_MISO_PIN        PA6
//  #define EEPROM_MOSI_PIN        PA7
//  #define EEPROM_PAGE_SIZE               0x1000U  // 4K (from datasheet)
//  #define MARLIN_EEPROM_SIZE 16UL * (EEPROM_PAGE_SIZE)   // Limit to 64K for now...
#elif ENABLED(SPI_FLASH)
  // Winbond W25Q64 (8MB/64Mbits)
  #define SPI_FLASH_SIZE                0x800000  // 8MB
  #define SPI_FLASH_CS_PIN              PA4
  #define SPI_FLASH_MOSI_PIN            PA7
  #define SPI_FLASH_MISO_PIN            PA6
  #define SPI_FLASH_SCK_PIN             PA5

//  #define SPI_FLASH_SIZE                0x40000U  // limit to 256K (M993 will reboot with 512)
//  #define SPI_FLASH_CS_PIN                  PA4
//  #define SPI_FLASH_MOSI_PIN                PA7
//  #define SPI_FLASH_MISO_PIN                PA6
//  #define SPI_FLASH_SCK_PIN                 PA5
#elif ENABLED(FLASH_EEPROM_EMULATION)
  // SoC Flash (framework-arduinoststm32-maple/STM32F1/libraries/EEPROM/EEPROM.h)
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE (EEPROM_PAGE_SIZE)
#else
  #define MARLIN_EEPROM_SIZE              0x800U  // On SD, Limit to 2K, require this amount of RAM
#endif

