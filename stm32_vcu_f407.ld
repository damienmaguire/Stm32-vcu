/*
 * Linker script for the ZombieVerter VCU on STM32F407VET6 or VGT6,
 * LQFP100. This is not the F107 script.
 *
 * Flash, matching include/hwdefs.h. Saving a store erases its whole
 * sector, so each store has a sector of its own. Sector 0 is reserved
 * for a bootloader that is not in this tree. The image will not run
 * from reset until that bootloader (or a debugger) starts it.
 *
 *   0x08000000  16 KiB  sector 0  bootloader, not linked here
 *   0x08004000  16 KiB  sector 1  parameters
 *   0x08008000  16 KiB  sector 2  CAN map
 *   0x0800C000  16 KiB  sector 3  pin definitions
 *   0x08010000  448 KiB sectors 4-7  application
 *
 * Sectors 4-7 exist on both the 512 KiB VE and the 1 MiB VG. The VG-only
 * sectors 8-11 (from 0x08080000) are left unused so one image is safe
 * on either part.
 *
 * RAM is the 128 KiB SRAM at 0x20000000. The 64 KiB CCM at 0x10000000
 * is not in this map: the DMA controller cannot read or write it.
 */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08010000, LENGTH = 448K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE cortex-m-generic.ld
