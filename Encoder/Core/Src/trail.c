#include "trail.h"

// 获取循迹模块的状态
float getTrailStatus() {
    int thisTrailStatus = 0;
    if(L2==0 && L1==0 && center==1 && R1==0 && R2==0) thisTrailStatus = 0.0;  // 直行
    else if(L2==1 && L1==1 && center==0 && R1==0 && R2==0) thisTrailStatus = -4.0;  // 大幅度左转
    else if(L2==1 && L1==0 && center==0 && R1==0 && R2==0) thisTrailStatus = -3.0;  // 中幅度左转
    else if(L2==0 && L1==1 && center==0 && R1==0 && R2==0) thisTrailStatus = -2.0;  // 左转
    else if(L2==0 && L1==0 && center==0 && R1==1 && R2==0) thisTrailStatus = 2.0;   // 右转
    else if(L2==0 && L1==0 && center==0 && R1==0 && R2==1) thisTrailStatus = 3.0;   // 中幅度右转
    else if(L2==0 && L1==0 && center==0 && R1==1 && R2==1) thisTrailStatus = 4.0;   // 大幅度右转
    // HW_PID_PWM = Trail_PID(0, thisTrailStatus, &p);
    return thisTrailStatus;
}
