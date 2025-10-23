/* Copyright 2025 muzkr https://github.com/muzkr
 * Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <string.h>

#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "board.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_gpio.h"
#include "bsp/dp32g030/saradc.h"
#include "bsp/dp32g030/syscon.h"
#include "driver/adc.h"
#include "driver/backlight.h"
#ifdef ENABLE_FMRADIO
    #include "driver/bk1080.h"
#endif

#include "driver/crc.h"
#include "driver/eeprom.h"
#include "driver/flash.h"
#include "driver/gpio.h"
#include "driver/system.h"
#include "driver/st7565.h"
#include "frequencies.h"
#include "helper/battery.h"
#include "misc.h"
#include "settings.h"
#if defined(ENABLE_OVERLAY)
    #include "sram-overlay.h"
#endif

#if defined(ENABLE_OVERLAY)
    void BOARD_FLASH_Init(void)
    {
        FLASH_Init(FLASH_READ_MODE_1_CYCLE);
        FLASH_ConfigureTrimValues();
        SYSTEM_ConfigureClocks();

        overlay_FLASH_MainClock       = 48000000;
        overlay_FLASH_ClockMultiplier = 48;

        FLASH_Init(FLASH_READ_MODE_2_CYCLE);
    }
#endif

void BOARD_GPIO_Init(void)
{
    LL_IOP_GRP1_EnableClock(
        LL_IOP_GRP1_PERIPH_GPIOA   //
        | LL_IOP_GRP1_PERIPH_GPIOB //
        | LL_IOP_GRP1_PERIPH_GPIOC //
        | LL_IOP_GRP1_PERIPH_GPIOF //
    );

    LL_GPIO_InitTypeDef InitStruct;
    LL_GPIO_StructInit(&InitStruct);
    InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    InitStruct.Pull = LL_GPIO_PULL_UP;
    InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    // ---------------------
    // Input pins

    InitStruct.Mode = LL_GPIO_MODE_INPUT;

    // Keypad rows: PB15:12
    InitStruct.Pin = LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12;
    LL_GPIO_Init(GPIOB, &InitStruct);

    // PTT: PB10
    InitStruct.Pin = LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOB, &InitStruct);

    // -----------------------
    //  Output pins

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6); // LCD A0
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_2); // LCD CS

    InitStruct.Mode = LL_GPIO_MODE_OUTPUT;

    // Keypad cols: PB6:3
    InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_5 | LL_GPIO_PIN_4 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOB, &InitStruct);

    // Audio PA: PA8
    // LCD A0: PA6
    // SPI flash CS: PA3
    InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_6 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOA, &InitStruct);

    // BK4819 SCK: B8
    // BK4819 SDA: B9
    // LCD CS: PB2
    InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_8 | LL_GPIO_PIN_2;
    LL_GPIO_Init(GPIOB, &InitStruct);

    // Flashlight: PC13
    InitStruct.Pin = LL_GPIO_PIN_13;
    LL_GPIO_Init(GPIOC, &InitStruct);

    // BK1080 SCK: PF5
    // BK1080 SDA: PF6
    // Backlight: PF8
    // BK4819 CS: PF9
    InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_8 | LL_GPIO_PIN_6 | LL_GPIO_PIN_5;
    LL_GPIO_Init(GPIOF, &InitStruct);

#ifndef ENABLE_SWD
    // A14:13
    InitStruct.Pin = LL_GPIO_PIN_14 | LL_GPIO_PIN_13;
    LL_GPIO_Init(GPIOA, &InitStruct);
#endif // ENABLE_SWD
}

void BOARD_ADC_Init(void)
{
    
}

void BOARD_ADC_GetBatteryInfo(uint16_t *pVoltage, uint16_t *pCurrent)
{
    
}

void BOARD_Init(void)
{
    BOARD_GPIO_Init();
    BACKLIGHT_InitHardware();
    BOARD_ADC_Init();
    ST7565_Init();
#ifdef ENABLE_FMRADIO
    BK1080_Init0();
#endif

#if defined(ENABLE_UART) || defined(ENABLED_AIRCOPY)
    CRC_Init();
#endif

}
