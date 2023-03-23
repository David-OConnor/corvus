/* STM32H743ZI2 */

/* https://github.com/stm32-rs/stm32h7xx-hal/blob/master/memory.x */
/* Note: Do you need names like SRAM1 etc to make this work with rust? */
/* See also: ITCM vice ITCMRAM */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M
  RAM (xrw)  : ORIGIN = 0x24000000, LENGTH = 512K
}


/* For H743. Nested comments appear not to work properly. */
/*
DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
*/

/* STM32H723 */

/*
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1M
  RAM (xrw)  : ORIGIN = 0x24000000, LENGTH = 320K

  ITCMRAM (xrw)    : ORIGIN = 0x00000000,   LENGTH = 64K
  DTCMRAM (xrw)    : ORIGIN = 0x20000000,   LENGTH = 128K
  RAM_D1  (xrw)    : ORIGIN = 0x24000000,   LENGTH = 320K
  RAM_D2  (xrw)    : ORIGIN = 0x30000000,   LENGTH = 32K
  RAM_D3  (xrw)    : ORIGIN = 0x38000000,   LENGTH = 16K

}
*/
