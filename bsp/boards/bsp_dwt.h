/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       bsp_dwt.h
  * @brief      DWT CYCCNT 微秒级时基（压测/延迟测量的公共地基）
  *
  * @note       Cortex-M4 内核自带的周期计数器，168 MHz 下约 25.6 s 回绕一次。
  *             求时间差一律用无符号减法，只要测量窗口小于回绕周期即天然正确。
  *
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2026-10-26      Phase0          1. done
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "struct_typedef.h"

/**
 * @brief  使能 DWT CYCCNT 周期计数器
 * @note   幂等：已在运行时直接返回，不清零，避免统计时基跳变
 */
void DWT_Init(void);

/**
 * @brief  读取周期计数器原始值（168 MHz 下约 25.6 s 回绕一次）
 */
uint32_t DWT_GetCyccnt(void);

/**
 * @brief  读取微秒时间戳（回绕周期同上）
 * @note   求差值请用无符号减法：uint32_t diff = t2 - t1;
 */
uint32_t DWT_GetUs(void);

#endif  // BSP_DWT_H
/*------------------------------ End of File ------------------------------*/
