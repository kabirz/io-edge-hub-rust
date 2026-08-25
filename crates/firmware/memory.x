/* io-edge-hub Rust app: the embassy-boot active partition — three 128 KiB
 * swap pages starting at sector 5 (see proto::fw_upg for the full map). No
 * image header: the app vector table sits at the partition start. */
MEMORY
{
  FLASH : ORIGIN = 0x08020000, LENGTH = 0x60000
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  CCRAM : ORIGIN = 0x10000000, LENGTH = 64K
}
