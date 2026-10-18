/**
 ******************************************************************************
 * @file           : bootloader_jump.h
 * @brief          : Software entry mechanism to ST Factory ROM Bootloader.
 *                   Uses persistent Backup Register magic key (0xDEADBEEF)
 *                   and memory remapping for single-click USB-C firmware
 * updates.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 RUDRAKINUXIS Project.
 * All rights reserved.
 *
 ******************************************************************************
 */

#ifndef __BOOTLOADER_JUMP_H
#define __BOOTLOADER_JUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/** @brief Magic signature written to Backup Register to trigger bootloader
 * entry */
#define BOOTLOADER_MAGIC_KEY (0xDEADBEEFUL)

/** @brief STM32G4 Factory System Memory (ROM Bootloader) base address */
#define STM32G4_SYSTEM_MEMORY_ADDR (0x1FFF0000UL)

/*============================================================================*/
/*                      FUNCTION PROTOTYPES                                   */
/*============================================================================*/

/**
 * @brief  Checks for the magic key on early system boot prior to peripheral
 * init. If present, remaps system memory and executes a jump to the ST ROM
 * bootloader. This function MUST be called at the very top of main().
 * @retval None
 */
void Bootloader_CheckAndJump(void);

/**
 * @brief  Writes the magic key (0xDEADBEEF) into the RTC/TAMP backup register
 *         and issues a soft system reset (NVIC_SystemReset).
 * @retval None
 */
void Bootloader_RequestJump(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_JUMP_H */
