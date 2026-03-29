#include <_main.h>
#include <dwt.h>
#include <DataBuffer.h>
#include "adc.h"

// ADC clock freq (Hz)
#define ADC_CLOCK 50000000.f
// RM0433 page 952
// 8 bit. TSAR timings depending on resolution
#define CONV_TICS 4.5f

// max ADC clock = 100mHz; Recommended ADC clock = 40mHz
struct ADC_param {
    uint32_t ADC_Prescaler;
    uint32_t ADC_SampleTime;
    float SampleTime;    // microseconds
};
typedef struct ADC_param ADC_PARAM;

#define ADC_Parameters_Size  31
const ADC_PARAM ADC_Parameters[ADC_Parameters_Size] = {
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV1,  LL_ADC_SAMPLINGTIME_64CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV12, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV12, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_SAMPLINGTIME_1CYCLE_5,   0.f},
        {LL_ADC_CLOCK_ASYNC_DIV8,  LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV16, LL_ADC_SAMPLINGTIME_2CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV6,  LL_ADC_SAMPLINGTIME_16CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV10, LL_ADC_SAMPLINGTIME_8CYCLES_5,  0.f},
        {LL_ADC_CLOCK_ASYNC_DIV2,  LL_ADC_SAMPLINGTIME_64CYCLES_5, 0.f},
        {LL_ADC_CLOCK_ASYNC_DIV4,  LL_ADC_SAMPLINGTIME_32CYCLES_5, 0.f}
};

uint32_t ADC_Prescaler = LL_ADC_CLOCK_ASYNC_DIV2;
uint32_t ADC_SampleTime = LL_ADC_SAMPLINGTIME_2CYCLES_5;
float    ADC_MeasureTime = 0;
uint8_t  ADC_param = 4;

uint16_t ScreenTime = 0;      // index in ScreenTimes
uint16_t ScreenTime_adj = 0;  // 0-9 shift in ScreenTime
const float ScreenTimes[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000};  // sweep screen, microseconds

uint8_t  ADCworks;
uint32_t ADCStartTick;         // time when start ADC buffer fill
uint32_t ADCHalfElapsedTick;   // the last time half buffer fill
uint32_t ADCElapsedTick;       // the last time buffer fill

static void ADC1_Init(void);
static void ADC2_Init(void);
float ADC_calcSampleTime();


void ADC_start() {

    if (ADCworks != 0) {
        return;
    }
    ADCworks = 1;

    ADC1_Init();
    ADC2_Init();

    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    LL_mDelay(2);

    // Set DMA transfer addresses of source and destination
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0,
                           (uint32_t) &(ADC12_COMMON->CDR),
                           (uint32_t)&samplesBuffer,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    // Set DMA transfer size
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, BUF_SIZE/2);
    // Enable DMA transfer interruption: transfer error
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

    LL_ADC_REG_StartConversion(ADC1);

    ADCStartTick = DWT_Get_Current_Tick();
}


static void ADC1_Init(void) {

  LL_ADC_REG_StopConversion(ADC1);
  LL_ADC_Disable(ADC1);
  LL_ADC_Disable(ADC2);

  // ADC1 DMA Init
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_ADC1);
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_0, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  MODIFY_REG(ADC1->CFGR, ADC_CFGR_RES, LL_ADC_RESOLUTION_8B|ADC_CFGR_RES_1 | ADC_CFGR_RES_0);
  // Common config
  MODIFY_REG(ADC12_COMMON->CCR,
             ADC_CCR_CKMODE | ADC_CCR_PRESC | ADC_CCR_DUAL | ADC_CCR_DAMDF | ADC_CCR_DELAY,
             ADC_Prescaler | LL_ADC_MULTI_DUAL_REG_INTERL | LL_ADC_MULTI_REG_DMA_RES_8B
  );

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  LL_mDelay(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

  // Configure Regular Channel
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, ADC_SampleTime);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelPreselection(ADC1, LL_ADC_CHANNEL_3);

}

static void ADC2_Init(void) {

  /** Common config */
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  MODIFY_REG(ADC2->CFGR, ADC_CFGR_RES, LL_ADC_RESOLUTION_8B|ADC_CFGR_RES_1 | ADC_CFGR_RES_0);
  LL_ADC_REG_SetDataTransferMode(ADC2, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  LL_mDelay(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

  // Configure Regular Channel
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3, ADC_SampleTime);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelPreselection(ADC2, LL_ADC_CHANNEL_3);

}


void ADC_step(int16_t step) {
    if (step == 0) return;
    if (step > 0) {
        if (ADC_param < ADC_Parameters_Size-1) ADC_param++;
    } else {
        if (ADC_param > 0) ADC_param--;
    }
    ADC_Prescaler = ADC_Parameters[ADC_param].ADC_Prescaler;
    ADC_SampleTime = ADC_Parameters[ADC_param].ADC_SampleTime;
    ADC_MeasureTime = ADC_calcSampleTime();
}

// return time for 1 measuring. (ns)
float ADC_calcSampleTime() {
    float presc = 0;
    float sampling = 0;

    switch (ADC_Prescaler) {
        case LL_ADC_CLOCK_ASYNC_DIV1:
            presc = 1;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV2:
            presc = 2;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV4:
            presc = 4;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV6:
            presc = 6;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV8:
            presc = 8;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV10:
            presc = 10;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV12:
            presc = 12;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV16:
            presc = 16;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV32:
            presc = 32;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV64:
            presc = 64;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV128:
            presc = 128;
            break;
        case LL_ADC_CLOCK_ASYNC_DIV256:
            presc = 256;
            break;
        default:
            Error_Handler();
    }

    switch (ADC_SampleTime) {
        case LL_ADC_SAMPLINGTIME_1CYCLE_5:
            sampling = 1.5f;
            break;
        case LL_ADC_SAMPLINGTIME_2CYCLES_5:
            sampling = 2.5f;
            break;
        case LL_ADC_SAMPLINGTIME_8CYCLES_5:
            sampling = 8.5f;
            break;
        case LL_ADC_SAMPLINGTIME_16CYCLES_5:
            sampling = 16.5f;
            break;
        case LL_ADC_SAMPLINGTIME_32CYCLES_5:
            sampling = 32.5f;
            break;
        case LL_ADC_SAMPLINGTIME_64CYCLES_5:
            sampling = 64.5f;
            break;
        case LL_ADC_SAMPLINGTIME_387CYCLES_5:
            sampling = 387.5f;
            break;
        case LL_ADC_SAMPLINGTIME_810CYCLES_5:
            sampling = 810.5f;
            break;
        default:
            Error_Handler();
    }

    return (CONV_TICS + sampling) * 1/(ADC_CLOCK/presc /1000000.f) * 1000.f /2.f ;
}
