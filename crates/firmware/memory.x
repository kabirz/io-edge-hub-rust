/* io-edge-hub Rust app: links at the embassy-boot ACTIVE slot start.
 * Bootloader occupies 0x08000000..0x08020000; ACTIVE = 0x08020000..0x08080000
 * (three 128K sectors — embassy-boot page size = max internal erase sector). */
MEMORY
{
  FLASH : ORIGIN = 0x08020000, LENGTH = 0x60000
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
