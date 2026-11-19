/**
 ******************************************************************************
 * @file           : bootloader_jump.c
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

#include "bootloader_jump.h"

typedef void (*pFunction)(void);

/**
 * @brief  Checks for the magic key on early system boot prior to peripheral
 * init. If present, remaps system memory and executes a jump to the ST ROM
 * bootloader. This function MUST be called at the very top of main().
 */
void Bootloader_CheckAndJump(void) {
  /* Enable Power Controller clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Enable access to Backup domain */
  LL_PWR_EnableBkUpAccess();

  /* Read magic key from persistent TAMP Backup Register 0 */
  uint32_t magic = TAMP->BKP0R;

  if (magic == BOOTLOADER_MAGIC_KEY) {
    /* Clear magic key so next reset boots normal user firmware */
    TAMP->BKP0R = 0U;

    /* Disable access to Backup domain */
    LL_PWR_DisableBkUpAccess();

    /* Disable all interrupts */
    __disable_irq();

    /* Disable and reset SysTick timer */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    /* Clear all pending NVIC interrupt requests */
    for (uint8_t i = 0U; i < 8U; i++) {
      NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    /* Enable SYSCFG clock and remap System Flash (ROM) to 0x00000000 */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_SetRemapMemory(LL_SYSCFG_REMAP_SYSTEMFLASH);

    /* Read Bootloader Reset Handler address and Main Stack Pointer */
    uint32_t bootloader_msp = *(__IO uint32_t *)(STM32G4_SYSTEM_MEMORY_ADDR);
    uint32_t bootloader_entry =
        *(__IO uint32_t *)(STM32G4_SYSTEM_MEMORY_ADDR + 4U);

    pFunction JumpToBootloader = (pFunction)(uintptr_t)bootloader_entry;

    /* Set Main Stack Pointer (MSP) to Bootloader stack */
    __set_MSP(bootloader_msp);

    /* Jump directly into ST Factory ROM Bootloader */
    JumpToBootloader();

    /* Execution never reaches here */
    while (1) {
    }
  }

  /* Normal boot: disable backup write access */
  LL_PWR_DisableBkUpAccess();
}

/**
 * @brief  Writes the magic key (0xDEADBEEF) into the RTC/TAMP backup register
 *         and issues a soft system reset (NVIC_SystemReset).
 */
void Bootloader_RequestJump(void) {
  /* Ensure power controller clock is active */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Enable write access to Backup Domain */
  LL_PWR_EnableBkUpAccess();

  /* Write 0xDEADBEEF signature to Backup Register 0 */
  TAMP->BKP0R = BOOTLOADER_MAGIC_KEY;

  /* Issue soft system reset to trigger early check on next boot */
  NVIC_SystemReset();
}
