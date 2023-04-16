/* STM32G473 */


MEMORY
{
  RAM (xrw)         : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH             : ORIGIN = 0x08000000, LENGTH = 512K
  /* CCM (xrw)         : ORIGIN = 0x2001F000, LENGTH = 4K*/
}
