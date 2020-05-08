/*
 * libmaple linker script for "Flash" builds.
 *
 * A Flash build puts .text (and .rodata) in Flash, and .data/.bss/heap in SRAM
 * but offsets the sections by enough space to store the CDC bootloader
 * which lives in the 3 lower Flash pages (6kB)
 */

MEMORY
{
  ccm (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 40K
  rom (rx)  : ORIGIN = 0x08001800, LENGTH = 250K
}


/* Provide memory region aliases for common.inc */
REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom);
REGION_ALIAS("REGION_CCM", ccm);

/* Let common.inc handle the real work. */
INCLUDE common.inc
