/*
 * STM32H743VIT6 linker script for the ANC helmet.
 *   FLASH  bank 1 only (1 MB) - bank 2 holds calibration + exposure log (store.c)
 *   DTCM   128 KB  .data/.bss/heap/stack: all DSP state, zero wait state, never cached
 *   ITCM   64 KB   .itcm: ISR + DSP hot path, copied from flash at boot (main.c)
 *   AXI    512 KB  .axisram (NOLOAD): DMA buffers (DMA1 cannot reach DTCM)
 */
ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x1000;
_Min_Stack_Size = 0x2000;

MEMORY
{
  ITCM   (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  DTCM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  AXI    (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  /* hot code, executed from ITCM */
  _sitcm = LOADADDR(.itcm);
  .itcm :
  {
    . = ALIGN(4);
    _itcm_start = .;
    *(.itcm) *(.itcm*)
    . = ALIGN(4);
    _itcm_end = .;
  } >ITCM AT> FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*) *(.RamFunc) *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >DTCM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCM

  .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit) *(.noinit*) . = ALIGN(4); } >DTCM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCM

  .axisram (NOLOAD) :
  {
    . = ALIGN(32);
    _saxi = .;
    *(.axisram) *(.axisram*)
    . = ALIGN(32);
    _eaxi = .;
  } >AXI

  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
