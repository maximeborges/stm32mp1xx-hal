/* Linker script for the STM32MP157 */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* We don't really have FLASH and RAM, just DDR
     but we keep the two segments to compatibility with cortex-m-rt */
  ISR          (RX)  : ORIGIN = 0x00000000, LENGTH = 0x00000398
  FLASH        (RWX) : ORIGIN = 0x10000000, LENGTH = 64K
  RAM          (RW)  : ORIGIN = 0x10020000, LENGTH = 64K - 16K
  TRACE_DATA   (RW)  : ORIGIN = 0x1002C000, LENGTH = 16K
  IPC_DATA     (RW)  : ORIGIN = 0x10040000, LENGTH = 4K + 4K + 16K
}

SECTIONS
{
  PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

  /* ## Sections in FLASH */
  /* ### Vector table */
  .vector_table ORIGIN(ISR) :
  {
    /* Initial Stack Pointer (SP) value */
    LONG(_stack_start);

    /* Reset vector */
    KEEP(*(.vector_table.reset_vector)); /* this is the `__RESET_VECTOR` symbol */
    __reset_vector = .;

    /* Exceptions */
    KEEP(*(.vector_table.exceptions)); /* this is the `__EXCEPTIONS` symbol */
    __eexceptions = .;

    /* Device specific interrupts */
    KEEP(*(.vector_table.interrupts)); /* this is the `__INTERRUPTS` symbol */
  } > ISR

  PROVIDE(_stext = ORIGIN(FLASH));

  /* ### .text */
  .text ORIGIN(FLASH) :
  {
    /* place these 2 close to each other or the `b` instruction will fail to link */
    *(.PreResetTrampoline);
    *(.Reset);

    *(.text .text.*);
    *(.HardFaultTrampoline);
    *(.HardFault.*);
    . = ALIGN(4);
    __etext = .;
  } > FLASH

  /* ### .rodata */
  .rodata __etext : ALIGN(4)
  {
    *(.rodata .rodata.*);

    /* 4-byte align the end (VMA) of this section.
       This is required by LLD to ensure the LMA of the following .data
       section will have the correct alignment. */
    . = ALIGN(4);
    __erodata = .;
  } > FLASH

  /* ## Sections in RAM */
  /* ### .data */
  .data : AT(__erodata) ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
    __edata = .;
  } > RAM

  /* LMA of .data */
  __sidata = LOADADDR(.data);

  /* ### .bss */
  .bss : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    . = ALIGN(4); /* 4-byte align the end (VMA) of this section */
    __ebss = .;
  } > RAM

  /* ### .uninit */
  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    *(.uninit .uninit.*);
    . = ALIGN(4);
  } > RAM

  /* Place the heap right after `.uninit` */
  . = ALIGN(4);
  __sheap = .;

  /* ## .got */
  /* Dynamic relocations are unsupported. This section is only used to detect relocatable code in
     the input files and raise an error if relocatable code is found */
  .got (NOLOAD) :
  {
    KEEP(*(.got .got.*));
  }

  /* ## Discarded sections */
  /DISCARD/ :
  {
    /* Unused exception related info that only wastes space */
    *(.ARM.exidx);
    *(.ARM.exidx.*);
    *(.ARM.extab.*);
  }
}