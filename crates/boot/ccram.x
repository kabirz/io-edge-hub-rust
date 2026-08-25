/* Same shape as the firmware crate's ccram.x: an empty NOLOAD payload
 * section so the global `-Tccram.x` rustflag links for this crate too. */
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
