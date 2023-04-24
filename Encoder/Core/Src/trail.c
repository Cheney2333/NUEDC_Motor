#include "trail.h"

void testModule()
{
    if(HAL_GPIO_ReadPin(L2_Port, L2_Pin) == GPIO_PIN_SET) 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    else 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

