/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       bsp_dwt.c
  * @brief      DWT CYCCNT 微秒级时基实现
  *
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-26      Phase0          1. done
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */

#include "bsp_dwt.h"
#include "main.h"

void DWT_Init(void)
{
    // 已在运行则直接返回：清零会让运行统计与时间戳同时跳变
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u)
    {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetCyccnt(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_GetUs(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000u);
}
/*------------------------------ End of File ------------------------------*/
