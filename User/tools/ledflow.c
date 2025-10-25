#include "ledflow.h"

void LED_Flow()
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    osDelay(200);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    osDelay(200);
}