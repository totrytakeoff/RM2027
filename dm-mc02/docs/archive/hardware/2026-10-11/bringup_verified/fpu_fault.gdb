set pagination off
set confirm off
target remote :3333

monitor reset run
python import time; time.sleep(2)
monitor halt
set $primask = 1
set {unsigned int}0xe000ed94 = 0
set {unsigned int}0xe000ed14 = ({unsigned int}0xe000ed14 & ~0x10000U)
restore docs/hardware_runs/2026-10-11/final/fpu_fault.bin binary 0x2404f000
thbreak *RmFault_FromException
set $pc = 0x2404f000
continue
printf "EXC_RETURN=%#x\n", $r1
printf "FRAME_SP=%#x\n", $r0
x/26wx $r0
thbreak *0x800190c
continue
python
import json
v=gdb.parse_and_eval('rm_fault_record')
fault={f.name:int(v[f.name]) for f in v.type.fields() if f.name!='frame'}
fault['frame']=[int(v['frame'][i]) for i in range(8)]
def word(a): return int(gdb.parse_and_eval('*(unsigned int*)%d'%a))
print('RM_CAPTURE='+json.dumps({'fault':fault,'gpiob_odr':word(0x58020414),
 'gpioc_odr':word(0x58020814),'can_cccr':[word(a+0x18) for a in (0x4000a000,0x4000a400,0x4000d400)]}))
end
detach
