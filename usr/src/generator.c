#include <main.h>
#include "generator.h"

/* H7
 * TIM1 Configuration
 * CLK  - 400 mHz
 * AHB2 - 200 mHz
 * PRE           100 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */


void GEN_setParams() {

    LL_TIM_SetPrescaler(TIM1, 99);
    LL_TIM_SetAutoReload(TIM1, 99);
    LL_TIM_OC_SetCompareCH1(TIM1, 40);

    // Start tim1 ch1
    // Enable output channel 1
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    // Enable the TIM main Output
    LL_TIM_EnableAllOutputs(TIM1);
    // Enable counter
    LL_TIM_EnableCounter(TIM1);

    // Start tim2 ch1
    // Enable output channel 1
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    // Enable counter
    LL_TIM_EnableCounter(TIM2);
}
