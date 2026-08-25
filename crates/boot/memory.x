/* embassy-boot bootloader for io-edge-hub.
 * BOOT region = first 128K of internal flash (the binary is ~24K; the slack
 * absorbs debug builds and growth). ACTIVE lives at 0x08020000, see
 * crates/proto/src/fw_upg.rs `partitions`. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
