/* Optional .ccm.bss payload section in CCRAM (64K, otherwise unused).
 *
 * Anchored AFTER .got (the last output section): cortex-m-rt pushes __ebss
 * past anything inserted after .bss (its RAM-contiguous zeroing contract),
 * which would make the startup zero-loop walk off the end of main RAM into
 * the 0x10000000 CCRAM hole. Buffers here are fully initialized by their
 * owners (StaticCell::init) before use, so no startup zeroing is required.
 */
SECTIONS
{
    .ccm.bss (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        __sccm = .;
        *(.ccm.bss .ccm.bss.*)
        . = ALIGN(4);
        __eccm = .;
    } > CCRAM
}
INSERT AFTER .got;
