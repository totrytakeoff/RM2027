set pagination off
set confirm off
set print pretty on
set remotetimeout 15
target extended-remote localhost:3333
monitor halt
info registers pc lr sp
p/x *(unsigned int *)0xe000ed28
p/x *(unsigned int *)0xe000ed2c
p/x *(unsigned int *)0xe000ed14
p rm_fault_record
p 'board.c'::board_info
p 'bringup.c'::imu
p 'bringup.c'::ins_snapshot
p 'runtime.c'::stats
p 'bringup.c'::remote_snapshot
p 'serial.c'::endpoints[0]
p 'can.c'::buses[0].stats
p 'output_gate.c'::gate
p/x *(unsigned int *)0x58020814
p/x *(unsigned int *)0x58020414
monitor reset run
detach
quit
