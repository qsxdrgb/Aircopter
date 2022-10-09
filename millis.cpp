#include "stm32f4xx_hal.h"
#include "millis.h"

//volatile unsigned long  _millis;

void millis_begin(void) {
    HAL_InitTick(0);
}

//extern "C" void SysTick_Handle(void) {
//    _millis++;
//}

unsigned long millis(void) {
    return HAL_GetTick();
}
