/* Copyright 2025 muzkr https://github.com/muzkr
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

#include "driver/dac.h"
#include "driver/systick.h"
#include "py32f071_ll_bus.h"
#include "py32f071_ll_gpio.h"
#include "py32f071_ll_dac.h"
#include "py32f071_ll_tim.h"

#define TIMx TIM6
#define DAC_CHANNEL LL_DAC_CHANNEL_1

static inline void TIM_Init()
{
    LL_TIM_SetPrescaler(TIMx, 3);
    LL_TIM_SetAutoReload(TIMx, 1499);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_UPDATE);
}

void DAC_Init()
{
    // Channel 1: PA4
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
    LL_DAC_SetTriggerSource(DAC1, DAC_CHANNEL, LL_DAC_TRIG_EXT_TIM6_TRGO);
    LL_DAC_SetOutputBuffer(DAC1, DAC_CHANNEL, LL_DAC_OUTPUT_BUFFER_ENABLE);
    LL_DAC_EnableDMAReq(DAC1, DAC_CHANNEL);
    LL_DAC_Enable(DAC1, DAC_CHANNEL);

    SYSTICK_DelayUs(15);

    TIM_Init();

    LL_DAC_EnableTrigger(DAC1, DAC_CHANNEL);
}
