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

static void ADC1_Init(void);

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

    ADC1_Init();

    // Set DMA transfer addresses of source and destination
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_1,
                           ADC1->DR,
                           (uint32_t)&samplesBuffer,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // Set DMA transfer size
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, BUF_SIZE);
    // Enable DMA transfer interruption: transfer error
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);

    // LL_ADC_EnableInternalRegulator(ADC1);
    LL_ADC_Enable(ADC1);
    LL_ADC_REG_StartConversion(ADC1);

    ADCStartTick = DWT_Get_Current_Tick();
}

uint32_t halfCount = 0;
uint32_t cpltCount = 0;

void ADC_step_up() {
    if (ScreenTime_adj < 9)
        ScreenTime_adj++;
    else if (ScreenTime < sizeof(ScreenTimes) / sizeof(ScreenTimes[0]) - 2) // last value forbidden to assign
        ScreenTime_adj = 0, ScreenTime++;
}


void ADC_step_down() {
    if (ScreenTime_adj > 0)
        ScreenTime_adj--;
    else if (ScreenTime > 0)
        ScreenTime_adj = 9, ScreenTime--;
}


float ADC_getTime() {
    float time = ScreenTimes[ScreenTime];
    // next time always exist because last forbidden to assign
    float adj = (ScreenTimes[ScreenTime + 1] - time) * ScreenTime_adj / 10;
    time += adj;
    return time;
}

s16 sStep;
float time;
int ii = 0;

void ADC_step(int16_t step) {
    if (step == 0) return;
    if (step > 0) {
        if (++ii >= ADC_Parameters_Size) ii = ADC_Parameters_Size-1;
    } else {
        if (--ii < 0) ii = 0;
    }
    ADC_Prescaler = ADC_Parameters[ii].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[ii].ADC_SampleTime;
    return;

    if (step > 0) ADC_step_up();
    else ADC_step_down();
    sStep = step;

    time = ADC_getTime(); // get screen sweep time

    // looking last parameters set with ScreenTime less than required time
    int i = 1;
    while (ADC_Parameters[i].ScreenTime < time) {
        i++;
        if (i >= ADC_Parameters_Size) break;
    }

    i--;
    ii = i;
    ADC_Prescaler = ADC_Parameters[i].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[i].ADC_SampleTime;

    // set X scale
    scaleX = ADC_Parameters[i].ScreenTime / time;
//*/
//    ADC_start();
}


/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void ADC1_Init(void) {

}
