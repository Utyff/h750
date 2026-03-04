#include <_main.h>
#include <dwt.h>
#include <graph.h>
#include <DataBuffer.h>
#include "adc.h"

// max ADC clock = 100mHz; Recommended ADC clock = 40mHz
struct ADC_param {
    uint32_t ADC_Prescaler;
    uint32_t ADC_SampleTime;
    float SampleTime;    // microseconds
    float ScreenTime;    // microseconds
};
typedef struct ADC_param ADC_PARAM;

#define ADC_Parameters_Size  31
const ADC_PARAM ADC_Parameters[ADC_Parameters_Size] = {
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_64CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV12, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV12, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_64CYCLES_5, 0.f, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f, 0.f}
}; //*/

uint32_t ADC_Prescaler = LL_ADC_CLOCK_ASYNC_DIV1;
uint32_t ADC_SampleTime = LL_ADC_SAMPLINGTIME_1CYCLE_5;

uint16_t ScreenTime = 0;      // index in ScreenTimes
uint16_t ScreenTime_adj = 0;  // 0-9 shift in ScreenTime
const float ScreenTimes[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000};  // sweep screen, microseconds

uint32_t ADCStartTick;         // time when start ADC buffer fill
uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
uint32_t ADCElapsedTick;       // the last time buffer fill

/**
 * Copy of MX_ADC1_Init()
 */
void ADC_start() {

    if (LL_ADC_REG_IsConversionOngoing(ADC1) != 0UL) {
        Error_Handler();
    }

    LL_ADC_Enable(ADC1);
    LL_mDelay(2);

    // Set DMA transfer addresses of source and destination
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1,
                           (uint32_t) &(ADC1->DR),
                           (uint32_t)&samplesBuffer,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // Set DMA transfer size
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, BUF_SIZE);
    // Enable DMA transfer interruption: transfer error
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

    LL_ADC_REG_StartConversion(ADC1);

    ADCStartTick = DWT_Get_Current_Tick();
}
