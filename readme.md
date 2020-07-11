# Dual Boot Bank Swap Boot Loader Example for STM32G4

This boot loader/firmware updater mainly blinks an LED. When you press a button connected to PC13 (B1 on the stm32g474re nucleo board), the program clears some pages at the start of bank 2, then copies itself there, toggles the BFB2 (boot-from-bank-2) bit, then resets and executes the new firmware at the start of bank 2. That firmware is the copy from bank 1. The firmware blinks the LED faster when on the second bank.

# Challenge 1

There is a problem with the STM32 System Bootloader and the STM32CubeMX generated linker file. Discussion is ongoing on the stm32 forum community, with STM32 employees participating. Updates have been promised, but as of July 11th 2020, the generated linker file still must be edited manually. See line 39 of STM32G474RETX\_FLASH.ld, it must be 96K for the RAM size. The actual RAM is 96K SRAM + 32K CCM-RAM = 128K RAM as it says on the package. But with 128K RAM, the first bytes in the binary (pointer to first stack frame) are somehow rejected by the ST System Boot Loader.

# Challenge 2

When erasing/writing to the 'other' bank (the bank mapped at 0x08040000), care must be taken to erase/write to the correct address, taking the address into account. Erase to the actual physical bank, write to the remapped address (see PC13 callback in main.c).

# Implementation

All relevant implementation is in main.c (except for the small but crucial change in the linker file).
