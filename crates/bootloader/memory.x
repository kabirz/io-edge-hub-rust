/* io-edge-hub bootloader: sectors 0-4 (0x08000000-0x0801FFFF); the active
 * application partition starts at 0x08020000 (sector 5, a 128 KiB swap page
 * boundary — required by the embassy-boot partition asserts). */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 0x20000
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
