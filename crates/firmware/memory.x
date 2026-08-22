/* io-edge-hub Rust app: links at slot0 + 0x200 image header (MCUboot --pad-header --header-size 512) */
MEMORY
{
  FLASH : ORIGIN = 0x08010200, LENGTH = 0x6FE00
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
