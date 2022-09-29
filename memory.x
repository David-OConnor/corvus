/* STM32G473 */

/* todo: 112 or 96k of flash on 473? Hard to tell from user man.  */
/* todo Probably 128k. Check Cube */

MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM    : ORIGIN = 0x20000000, LENGTH = 128K
}


/* STM32H743ZI2 */

/*
MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 2M
  RAM    : ORIGIN = 0x24000000, LENGTH = 512K
}
*/

/* STM32H723 */

/*
MEMORY
{
  FLASH  : ORIGIN = 0x08000000, LENGTH = 1M
  RAM    : ORIGIN = 0x24000000, LENGTH = 384K
}
*/
