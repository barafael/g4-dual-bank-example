# Dual Boot Bank Swap Boot Loader Example for STM32G4

This boot loader/firmware updater mainly blinks an LED. When you press a button connected to PC13 (B1 on the stm32g474re nucleo board), the program clears some pages at the start of bank 2, then copies itself there, toggles the BFB2 (boot-from-bank-2) bit, then resets and executes the new firmware at the start of bank 2. That firmware is the copy from bank 1. The firmware blinks the LED faster when on the second bank.

Note: this may be affected by erratum 2.2.2 in "STM32G471xx/473xx/474xx/483xx/484xx Errata sheet":

2.2.2 Data cache might be corrupted during Flash memory read-while-write operation

If you find out anything about this, please let me know :)

# Challenge 1

There is a problem with the STM32 System Bootloader and the STM32CubeMX generated linker file. Discussion is ongoing on the stm32 forum community, with STM32 employees participating. Updates have been promised, but as of July 11th 2020, the generated linker file still must be edited manually. See line 39 of STM32G474RETX\_FLASH.ld, it must be 96K for the RAM size. The actual RAM is 96K SRAM + 32K CCM-RAM = 128K RAM as it says on the package. But with 128K RAM, the first bytes in the binary (pointer to first stack frame) are somehow rejected by the ST System Boot Loader.

# Challenge 2

When erasing/writing to the 'other' bank (the bank mapped at 0x08040000), care must be taken to erase/write to the correct address, taking the address into account. Erase to the actual physical bank, write to the remapped address (see PC13 callback in main.c).

# Challenge 3

When using many peripherals, many interrupts, the core sometimes goes into a state which I cannot explain when erasing bank 1 from bank 2. Doing a full bank erase works without problems in this case.

# Challenge 4

The chip errata state something about caches being corrupted during RWW(read-while-write). I'm pretty sure this is relevant here.

# Implementation

All relevant implementation is in main.c (except for the small but crucial change in the linker file).
