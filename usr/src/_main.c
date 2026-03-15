#include <stdlib.h>
#include <stdio.h>
#include <_main.h>
#include <delay.h>
#include <draw.h>
#include <keys.h>
#include <DataBuffer.h>
#include <generator.h>
#include <adc.h>
#include <dac.h>
#include <stm32h7xx_ll_fmc.h>
#include "stm32h7xx_hal.h"


void CORECheck();

void FPUCheck();

extern int ii;
extern float time;

const char buildDate[] = __DATE__;
const char buildTime[] = __TIME__;

//touch_point_t touchPoint1;
//touch_point_t touchPoint2;

void PLL2_PeriphCommonClock_Config(void)
{
    LL_RCC_PLL2P_Enable();
    LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_8_16);
    LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_MEDIUM);
    LL_RCC_PLL2_SetM(2);
    LL_RCC_PLL2_SetN(20);
    LL_RCC_PLL2_SetP(2);
    LL_RCC_PLL2_SetQ(2);
    LL_RCC_PLL2_SetR(1);
    LL_RCC_PLL2_Enable();

    // Wait till PLL is ready
    while(LL_RCC_PLL2_IsReady() != 1){}

    LL_RCC_SetClockSource(LL_RCC_FMC_CLKSOURCE_PLL2R);
}


void FMC_Init(void) {
  PLL2_PeriphCommonClock_Config();

  FMC_NORSRAM_TimingTypeDef Timing = {0};
  FMC_NORSRAM_InitTypeDef   fmcInit = {0};

  fmcInit.NSBank = FMC_NORSRAM_BANK1;
  fmcInit.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  fmcInit.MemoryType = FMC_MEMORY_TYPE_SRAM;
  fmcInit.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  fmcInit.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  fmcInit.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  fmcInit.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  fmcInit.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  fmcInit.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  fmcInit.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  fmcInit.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  fmcInit.WriteBurst = FMC_WRITE_BURST_DISABLE;
  fmcInit.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  fmcInit.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  fmcInit.PageSize = FMC_PAGE_SIZE_NONE;

  FMC_NORSRAM_Init(FMC_Bank1_R, &fmcInit);

  // Timing
  Timing.AddressSetupTime = 3;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 2;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;

  FMC_NORSRAM_Timing_Init(FMC_Bank1_R, &Timing, fmcInit.NSBank);

  __IO uint32_t tmpreg;
  SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  UNUSED(tmpreg);

  // Enable the NORSRAM device
  __FMC_NORSRAM_ENABLE(FMC_Bank1_R, fmcInit.NSBank);

  // Enable FMC Peripheral
  __FMC_ENABLE();

  // HAL_SetFMCMemorySwappingConfig(FMC_SWAPBMAP_SDRAM_SRAM);
  MODIFY_REG(FMC_Bank1_R->BTCR[0], FMC_BCR1_BMAP, FMC_SWAPBMAP_SDRAM_SRAM);
}


void mainInitialize() {
    char buf[120];
    sprintf(buf, "\n\nBuild: %s %s\n", buildDate, buildTime);
    DBG_Trace(buf);

    CORECheck();
    FPUCheck();

    DWT_Init();
    FMC_Init();
    LCD_Init();
    // LCD_Clear(BLACK);
    KEYS_init();

    ADCworks = 0;
    ADC_start();

    // GEN_setParams();
    DAC_startSin();
}

u32 ticks =0;

void mainCycle() {
    if ((random() & 7) < 2) GPIOB->ODR ^= LED1_Pin;
//    getPoint(0, &touchPoint1);
//    getPoint(1, &touchPoint2);

    // drawScreen();
    KEYS_scan();

    // POINT_COLOR = WHITE;
    // BACK_COLOR = CLR_BACKGROUND;
    // LCD_ShowxNum(0, LINE1_Y, TIM8->CNT, 5, 12, 0x0);
    // LCD_ShowxNum(30, LINE1_Y, (u32) button1Count, 5, 12, 0x0);
//    LCD_ShowxNum(60, 214, (u32) ii, 5, 12, 0x01);
//    LCD_ShowxNum(90, 214, (u32) time / 10, 5, 12, 0x01);
//    LCD_ShowxNum(120, 214, (u32) firstHalf, 5, 12, 0x01);

    // POINT_COLOR = MAGENTA;
    // LCD_ShowxNum(0,  LINE2_Y, ADCElapsedTick, 10, 12, 0x0);

    // if (adc1cplt != 0) {
    //     adc1cplt = 0;
    //     ADC_start();
    // }

    delay_ms(30);
}

void UART_Transmit(const char *msg) {
  static char __ALIGNED(__SCB_DCACHE_LINE_SIZE) SECTION_RAM_D2 txBuffer[250];
  const uint32_t txBufferSize = strlen(msg);

  while (DMA1_0_busy){}
  DMA1_0_busy = 1;

  stpcpy(txBuffer, msg);
  // SCB_CleanDCache_by_Addr((uint32_t*)txBuffer, (int32_t)txBufferSize);

  // Stream 0 = TX
  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_STREAM_0,
                         (uint32_t) txBuffer,
                         (uint32_t) &(USART1->TDR),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, txBufferSize);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
}

#ifdef DEBUG_TRACE_SWO

void SWO_Trace(uint8_t *msg) {
    for (int i = 0; msg[i] != 0; i++) {
        ITM_SendChar(msg[i]);
    }
}

#endif

void FPUCheck(void) {
    char buf[120];
    uint32_t mvfr0;

    sprintf(buf, "%08X %08X %08X\n%08X %08X %08X\n",
            *(volatile uint32_t *) 0xE000EF34,   // FPCCR  0xC0000000
            *(volatile uint32_t *) 0xE000EF38,   // FPCAR
            *(volatile uint32_t *) 0xE000EF3C,   // FPDSCR
            *(volatile uint32_t *) 0xE000EF40,   // MVFR0  0x10110021 vs 0x10110221
            *(volatile uint32_t *) 0xE000EF44,   // MVFR1  0x11000011 vs 0x12000011
            *(volatile uint32_t *) 0xE000EF48);  // MVFR2  0x00000040
    DBG_Trace(buf);

    mvfr0 = *(volatile uint32_t *) 0xE000EF40;

    switch (mvfr0) {
        case 0x00000000 :
            sprintf(buf, "No FPU\n");
            break;
        case 0x10110021 :
            sprintf(buf, "FPU-S Single-precision only\n");
            break;
        case 0x10110221 :
            sprintf(buf, "FPU-D Single-precision and Double-precision\n");
            break;
        default :
            sprintf(buf, "Unknown FPU\n");
    }
    DBG_Trace(buf);
}

void CORECheck(void) {
    char buf[120];
    uint32_t cpuid = SCB->CPUID;
    uint32_t var, pat;

    sprintf(buf, "\nCPUID %08X DEVID %03X REVID %04X\n", cpuid, DBGMCU->IDCODE & 0xFFF, DBGMCU->IDCODE >> 16);
    DBG_Trace(buf);

    pat = (cpuid & 0x0000000F);
    var = (cpuid & 0x00F00000) >> 20;

    if ((cpuid & 0xFF000000) == 0x41000000) // ARM
    {
        switch ((cpuid & 0x0000FFF0) >> 4) {
            case 0xC20 :
                sprintf(buf, "Cortex M0 r%dp%d\n", var, pat);
                break;
            case 0xC60 :
                sprintf(buf, "Cortex M0+ r%dp%d\n", var, pat);
                break;
            case 0xC21 :
                sprintf(buf, "Cortex M1 r%dp%d\n", var, pat);
                break;
            case 0xC23 :
                sprintf(buf, "Cortex M3 r%dp%d\n", var, pat);
                break;
            case 0xC24 :
                sprintf(buf, "Cortex M4 r%dp%d\n", var, pat);
                break;
            case 0xC27 :
                sprintf(buf, "Cortex M7 r%dp%d\n", var, pat);
                break;
            case 0xD21 :
                sprintf(buf, "Cortex M33 r%dp%d\n", var, pat);
                break;

            default :
                sprintf(buf, "Unknown CORE\n");
        }
    } else
        sprintf(buf, "Unknown CORE IMPLEMENTER\n");
    DBG_Trace(buf);
}
