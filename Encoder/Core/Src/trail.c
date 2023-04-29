#include "trail.h"

// 获取循迹模块的状态
CarDirection GetCarDirection(void)
{
    if (L2 == GPIO_PIN_SET) {
        return LEFT;
    } else if (L1 == GPIO_PIN_SET) {
        return microLEFT;
    } else if (R1 == GPIO_PIN_SET) {
        return microRIGHT;
    } else if (R2 == GPIO_PIN_SET) {
        return RIGHT;
    } else {
        return FORWARD;
    }
}

