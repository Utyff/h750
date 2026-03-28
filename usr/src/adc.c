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
float ADC_calcSampleTime();


void ADC_start() {

    if (ADCworks != 0) {
        return;
    }
    ADCworks = 1;

    ADC1_Init();

    // LL_ADC_EnableIT_ADRDY(ADC1);
    LL_ADC_EnableIT_EOC(ADC1);
    LL_ADC_EnableIT_EOS(ADC1);
    LL_ADC_EnableIT_OVR(ADC1);

    if ((ADC1->CR & (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP | ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)) != 0UL) {
        Error_Handler();
    }
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {}

    LL_ADC_REG_StartConversion(ADC1);

    ADCStartTick = DWT_Get_Current_Tick();
}


static void ADC1_Init(void) {

  LL_ADC_REG_StopConversion(ADC1);
  while (LL_ADC_REG_IsStopConversionOngoing(ADC1)) {}
  LL_ADC_Disable(ADC1);
  while (LL_ADC_IsDisableOngoing(ADC1)) {}

  MODIFY_REG(ADC1->CFGR, ADC_CFGR_RES, LL_ADC_RESOLUTION_8B);
  // Common config
  MODIFY_REG(ADC12_COMMON->CCR,
             ADC_CCR_CKMODE | ADC_CCR_PRESC | ADC_CCR_DUAL | ADC_CCR_DAMDF | ADC_CCR_DELAY,
             ADC_Prescaler | LL_ADC_MULTI_INDEPENDENT
  );

  LL_ADC_SetBoostMode(ADC1, LL_ADC_BOOST_MODE_50MHZ);
  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  LL_mDelay(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

  LL_ADC_StartCalibration(ADC1, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}

  // Configure Regular Channel
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, ADC_SampleTime);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelPreselection(ADC1, LL_ADC_CHANNEL_3);
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

    return (CONV_TICS + sampling) * 1/(ADC_CLOCK/presc /1000000.f) * 1000.f;
}
