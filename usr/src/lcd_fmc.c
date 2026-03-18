#include <_main.h>
#include <stm32h7xx_hal.h>
#include <stm32h7xx_ll_fmc.h>


void PLL2_PeriphCommonClock_Config(void) {
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
    while(LL_RCC_PLL2_IsReady() != 1) {}

    LL_RCC_SetClockSource(LL_RCC_FMC_CLKSOURCE_PLL2R);

    // Peripheral clock enable
    __IO uint32_t tmpreg;
    SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
    // Delay after an RCC peripheral clock enabling
    tmpreg = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
    UNUSED(tmpreg);
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

  // Enable the NORSRAM device
  __FMC_NORSRAM_ENABLE(FMC_Bank1_R, fmcInit.NSBank);

  // Enable FMC Peripheral
  __FMC_ENABLE();

  // HAL_SetFMCMemorySwappingConfig(FMC_SWAPBMAP_SDRAM_SRAM);
  MODIFY_REG(FMC_Bank1_R->BTCR[0], FMC_BCR1_BMAP, FMC_SWAPBMAP_SDRAM_SRAM);
}
