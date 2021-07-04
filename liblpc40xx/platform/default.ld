__flash = 0x00000000;
__flash_size = 512K;
__ram = 0x10000000;
__ram_size = 64K;
__stack_size = 1K;

INCLUDE picolibcpp.ld

PROVIDE(__heap_start = 0x20000000);
PROVIDE(__heap_size = 32K);
PROVIDE(__heap_end = __heap_start + __heap_size);
