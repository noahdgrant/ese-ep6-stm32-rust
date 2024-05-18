/* Linker script for STM32F303RE Nucleo board */
MEMORY {
    FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 512K
    CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 16K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}
