#include <stdio.h>
#include <_main.h>
#include "generator.h"

/* F7
 * TIM1 Configuration
 * CLK  216 mHz
 * PRE           108 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */
/* H7
 * TIM1 Configuration
 * CLK  - 400 mHz
 * AHB2 - 200 mHz
 * PRE           100 - 1 => 2 MHz
 * COUNT PERIOD  100 - 1 => 20 KHz
 */

struct GEN_param {
    uint32_t TIM_Prescaler;
    uint32_t TIM_Period;
    uint32_t Frequency;    // Hz
};
typedef struct GEN_param GEN_PARAM;

#define GEN_Parameters_Size 26
const GEN_PARAM GEN_Parameters[GEN_Parameters_Size] = {
        {0, 19, 10000000},
        {0, 24, 8000000},
        {0, 32, 6060606},
        {0, 49, 4000000},
        {0, 99, 2000000},
        {9, 19, 1000000},
        {9, 24, 800000},
        {8, 36, 600600},
        {9, 49, 400000},
        {9, 99, 200000},
        {99, 19, 100000},
        {99, 24, 80000},
        {32, 100, 60006},
        {99, 49, 40000},
        {99, 99, 20000},
        {999, 19, 10000},
        {999, 24, 8000},
        {329, 100, 6000},
        {999, 49, 4000},
        {999, 99, 2000},
        {9999, 19, 1000},
        {9999, 24, 800},
        {3299, 100, 600},
        {9999, 49, 400},
        {9999, 99, 200},
        {9999, 199, 100}
};

uint32_t currentGenParam = 4;

uint32_t tim1Prescaler = 99;
uint32_t tim1Period = 99;
uint32_t tim1Pulse = 40;
uint32_t tim1Freq = 0;

void GEN_step(int16_t step) {
    char msg[200];

    if (step == 0) return;

    if (step > 0) {
        if (currentGenParam > 0) currentGenParam--;
    } else {
        if (currentGenParam < GEN_Parameters_Size-1) currentGenParam++;
    }

    GEN_setParams();

    sprintf(msg, "After step. param: %u, presc: %u, period: %u freq: %u\n",
            currentGenParam, tim1Prescaler, tim1Period, tim1Freq);
    DBG_Trace(msg);
}


void GEN_setParams() {
    tim1Prescaler = GEN_Parameters[currentGenParam].TIM_Prescaler;
    tim1Period = GEN_Parameters[currentGenParam].TIM_Period;
    tim1Freq = GEN_Parameters[currentGenParam].Frequency;
    tim1Pulse = tim1Period * 40 / 100;

    LL_TIM_SetPrescaler(TIM1, tim1Period);
    LL_TIM_SetAutoReload(TIM1, tim1Period);
    LL_TIM_OC_SetCompareCH1(TIM1, tim1Pulse);

    // Start tim4 ch1
    // Enable output channel 1
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    // Enable counter
    LL_TIM_EnableCounter(TIM1);
    /* Force update generation */
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    /*
    TIM_OC_InitTypeDef sConfigOC;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = tim1Prescaler;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = tim1Period;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = tim1Pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);*/
}

void GEN_setParams2() {
  /* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz */
  // LL_TIM_SetPrescaler(TIM1, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));

  /* Enable TIM1_ARR register preload. Writing to or reading from the         */
  /* auto-reload register accesses the preload register. The content of the   */
  /* preload register are transferred into the shadow register at each update */
  /* event (UEV).                                                             */  
  // LL_TIM_EnableARRPreload(TIM1);

  /* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  // uint32_t TimOutClock = SystemCoreClock/1;
  // LL_TIM_SetAutoReload(TIM1, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM1), 100));

  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output mode */
  /* Reset value is LL_TIM_OCMODE_FROZEN */
  // LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);

  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  //LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);

  /* Set compare value to half of the counter period (50% duty cycle ) */
  // LL_TIM_OC_SetCompareCH1(TIM1, ( (LL_TIM_GetAutoReload(TIM1) + 1 ) / 2));

  /* Enable TIM1_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM1_CCR1 preload value is loaded in the active        */
  /* at each update event.                                                    */
  // LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  /**************************/
  /* TIM1 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1*/

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

  /* Enable counter */
  LL_TIM_EnableCounter(TIM1);

  /* Force update generation */
  // LL_TIM_GenerateEvent_UPDATE(TIM1);
}
