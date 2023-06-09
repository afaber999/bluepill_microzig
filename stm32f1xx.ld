/*
 * link loader file for stm32fxx 
 *
 * Target CPU:  ARM Cortex-M3
 * Target Chip: STM32F103
 */

ENTRY(_start);

MEMORY
{
  flash0 (rx!w) : ORIGIN = 0x08000000, LENGTH = 0x00010000
  ram0   (rw!x) : ORIGIN = 0x20000000, LENGTH = 0x00005000
  usbpma (rw!x) : ORIGIN = 0x40006000, LENGTH = 0x000003FF
}

SECTIONS
{
  .text :
  {
     KEEP(*(microzig_flash_start))
     *(.text*)
     *(.rodata*)
     . = ALIGN(4);
  } > flash0

  .ARM.exidx : {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      . = ALIGN(4);
  } >flash0

  .data :
  {
     microzig_data_start = .;
     *(.data*)
     . = ALIGN(4);
     microzig_data_end = .;
  } > ram0 AT> flash0

  .bss (NOLOAD) :
  {
      microzig_bss_start = .;
      *(.bss)
      *(.bss*)
     . = ALIGN(4);
      microzig_bss_end = .;
  } > ram0

	.UsbBuffer (NOLOAD) : {
    stm32_usb_buf_start = .;
		*(.usbbuf)
		*(.usbbuf*)
	} > usbpma

  microzig_data_load_start = LOADADDR(.data);
}
