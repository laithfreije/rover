MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* Reserve the last 8K of the 2MB flash for persisted BLE bond storage
     * (see STORAGE_* in bluetooth.rs); shrink the code region so the linker
     * never places code there. */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 8K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
}
