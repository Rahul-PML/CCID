#include "main.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

void NMI_Handler(void)              { while(1) {} }
void HardFault_Handler(void)        { while(1) {} }
void MemManage_Handler(void)        { while(1) {} }
void BusFault_Handler(void)         { while(1) {} }
void UsageFault_Handler(void)       { while(1) {} }
void SVC_Handler(void)              { }
void DebugMon_Handler(void)         { }
void PendSV_Handler(void)           { }
void SysTick_Handler(void)          { HAL_IncTick(); }

void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}
