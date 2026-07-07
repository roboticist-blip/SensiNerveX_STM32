/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Utils.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void HardFault_DumpAndHalt(uint32_t *stack_frame);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Cortex-M4 fault diagnostic: prints the stacked exception frame
 *         (R0-R3, R12, LR, PC, PSR) and the fault status registers
 *         (CFSR/HFSR/MMFAR/BFAR) over the debug UART before halting.
 *
 * Added because the default HardFault_Handler (bare `while(1){}`, see
 * below) produces zero output on a real fault — indistinguishable from
 * the firmware simply hanging. This turns "it's stuck" into concrete
 * evidence: PC tells you which instruction faulted (cross-reference
 * against the .map file, or addr2line on the built ELF for that PC),
 * and CFSR's bit pattern tells you the fault class (precise bus fault,
 * MPU violation, divide-by-zero, undefined instruction, etc).
 *
 * Uses Utils_LogWrite() directly (blocking HAL_UART_Transmit) rather
 * than the LOG_ERR macro — this runs in fault context, before we know
 * anything about the system's state, so keeping it to the one primitive
 * we're confident still works (UART peripheral + clocks are unaffected
 * by most fault causes) is deliberate.
 */
void HardFault_DumpAndHalt(uint32_t *stack_frame)
{
    uint32_t r0  = stack_frame[0];
    uint32_t r1  = stack_frame[1];
    uint32_t r2  = stack_frame[2];
    uint32_t r3  = stack_frame[3];
    uint32_t r12 = stack_frame[4];
    uint32_t lr  = stack_frame[5];
    uint32_t pc  = stack_frame[6];
    uint32_t psr = stack_frame[7];

    uint32_t cfsr  = SCB->CFSR;
    uint32_t hfsr  = SCB->HFSR;
    uint32_t mmfar = SCB->MMFAR;
    uint32_t bfar  = SCB->BFAR;

    char buf[196];
    int n;

    n = snprintf(buf, sizeof(buf),
        "\r\n[FAULT] PC=0x%08lX LR=0x%08lX PSR=0x%08lX\r\n",
        (unsigned long)pc, (unsigned long)lr, (unsigned long)psr);
    Utils_LogWrite(buf, (uint16_t)n);

    n = snprintf(buf, sizeof(buf),
        "[FAULT] R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX R12=0x%08lX\r\n",
        (unsigned long)r0, (unsigned long)r1, (unsigned long)r2,
        (unsigned long)r3, (unsigned long)r12);
    Utils_LogWrite(buf, (uint16_t)n);

    n = snprintf(buf, sizeof(buf),
        "[FAULT] CFSR=0x%08lX HFSR=0x%08lX MMFAR=0x%08lX BFAR=0x%08lX\r\n",
        (unsigned long)cfsr, (unsigned long)hfsr,
        (unsigned long)mmfar, (unsigned long)bfar);
    Utils_LogWrite(buf, (uint16_t)n);

    /* Decode the CFSR bits that come up in practice. Full bit layout is
     * in the Cortex-M4 Technical Reference Manual (SCB_CFSR); this
     * covers the common cases rather than every bit. */
    if (cfsr & (1UL << 0))  { n = snprintf(buf, sizeof(buf), "[FAULT] -> IACCVIOL (MPU: instruction fetch)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 1))  { n = snprintf(buf, sizeof(buf), "[FAULT] -> DACCVIOL (MPU: data access)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 8))  { n = snprintf(buf, sizeof(buf), "[FAULT] -> IBUSERR (bus fault on instruction fetch)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 9))  { n = snprintf(buf, sizeof(buf), "[FAULT] -> PRECISERR (precise data bus fault, BFAR valid)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 10)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> IMPRECISERR (imprecise data bus fault)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 16)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> UNDEFINSTR (undefined instruction)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 17)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> INVSTATE (invalid EPSR/Thumb state)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 25)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> DIVBYZERO\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (cfsr & (1UL << 24)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> UNALIGNED (unaligned access trap)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }
    if (hfsr & (1UL << 30)) { n = snprintf(buf, sizeof(buf), "[FAULT] -> FORCED (escalated from a lower-priority fault)\r\n"); Utils_LogWrite(buf, (uint16_t)n); }

    n = snprintf(buf, sizeof(buf), "[FAULT] Halting. Cross-reference PC against build/*.map "
                                    "or addr2line -e build/*.elf 0x%08lX\r\n", (unsigned long)pc);
    Utils_LogWrite(buf, (uint16_t)n);

    while (1) { /* halt — diagnostic already sent */ }
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* Naked: no compiler-generated prologue, so the assembly below sees
   * the exact CPU state the NVIC left on fault entry — required to
   * correctly identify which stack (MSP vs PSP) holds the exception
   * frame. See HardFault_DumpAndHalt() above for what this leads to. */
  __asm volatile
  (
    " tst lr, #4                \n"
    " ite eq                    \n"
    " mrseq r0, msp             \n"
    " mrsne r0, psp             \n"
    " b HardFault_DumpAndHalt   \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  __asm volatile
  (
    " tst lr, #4                \n"
    " ite eq                    \n"
    " mrseq r0, msp             \n"
    " mrsne r0, psp             \n"
    " b HardFault_DumpAndHalt   \n"
  );
  /* USER CODE END MemoryManagement_IRQn 0 */
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  __asm volatile
  (
    " tst lr, #4                \n"
    " ite eq                    \n"
    " mrseq r0, msp             \n"
    " mrsne r0, psp             \n"
    " b HardFault_DumpAndHalt   \n"
  );
  /* USER CODE END BusFault_IRQn 0 */
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __asm volatile
  (
    " tst lr, #4                \n"
    " ite eq                    \n"
    " mrseq r0, msp             \n"
    " mrsne r0, psp             \n"
    " b HardFault_DumpAndHalt   \n"
  );
  /* USER CODE END UsageFault_IRQn 0 */
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */

 // void TIM1_UP_TIM10_IRQHandler(void)
//{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
//}

  /**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */