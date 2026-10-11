set pagination off
set confirm off
target remote :3333

monitor reset halt
thbreak *ExitRun0Mode
continue
printf "EARLY_PC=%#x\n", $pc
x/5wx 0x58004800
set $pc = ((unsigned int)&Default_Handler & ~1U)
detach
