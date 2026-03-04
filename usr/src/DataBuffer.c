#include "DataBuffer.h"

u8 __ALIGNED(__SCB_DCACHE_LINE_SIZE) SECTION_RAM_D2 samplesBuffer[BUF_SIZE];

u8 firstHalf = 0;
u8 adc1cplt = 0;
