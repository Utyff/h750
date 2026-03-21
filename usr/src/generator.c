#include <stdio.h>
#include <main.h>
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


void GEN_setParams() {

    LL_TIM_SetPrescaler(TIM1, 99);
    LL_TIM_SetAutoReload(TIM1, 99);
    LL_TIM_OC_SetCompareCH1(TIM1, 40);

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
