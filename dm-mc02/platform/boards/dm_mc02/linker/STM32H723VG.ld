ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  ITCM (rx) : ORIGIN = 0x00000000, LENGTH = 64K
  DTCM (rw) : ORIGIN = 0x20000000, LENGTH = 120K
  MSP (rw) : ORIGIN = 0x2001E000, LENGTH = 8K
  AXI (rw) : ORIGIN = 0x24000000, LENGTH = 320K
  D2 (rw) : ORIGIN = 0x30000000, LENGTH = 32K
  D3 (rw) : ORIGIN = 0x38000000, LENGTH = 16K
}
_estack = ORIGIN(MSP) + LENGTH(MSP);
_main_stack_size = 8K;
SECTIONS {
  .isr_vector : { . = ALIGN(512); KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
  .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > AXI AT> FLASH
  _sidata = LOADADDR(.data);
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > AXI
  .dtcm_bss (NOLOAD) : { . = ALIGN(8); __dtcm_start = .; *(.dtcm_bss*) . = ALIGN(8); __dtcm_end = .; } > DTCM
  .dma_buffer (NOLOAD) : { . = ALIGN(32); __dma_start = .; *(.dma_buffer*) . = ALIGN(32); __dma_end = .; } > D2
  .bdma_buffer (NOLOAD) : { . = ALIGN(32); __bdma_start = .; *(.bdma_buffer*) . = ALIGN(32); __bdma_end = .; } > D3
  .noinit_fault (NOLOAD) : { . = ALIGN(32); KEEP(*(.noinit_fault*)) . = ALIGN(32); } > D3
  .main_stack (NOLOAD) : { . += _main_stack_size; } > MSP
  /DISCARD/ : { *(.note.GNU-stack) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
ASSERT(__dtcm_end <= _estack - _main_stack_size, "DTCM overlaps MSP stack")
ASSERT(SIZEOF(.dma_buffer) <= 8K, "DMA budget exceeded")
ASSERT(SIZEOF(.bdma_buffer) + SIZEOF(.noinit_fault) <= 4K, "D3 budget exceeded")
ASSERT(_ebss <= ORIGIN(AXI) + 192K, "AXI budget exceeded")
