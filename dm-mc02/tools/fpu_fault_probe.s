.syntax unified
.cpu cortex-m7
.fpu fpv5-d16
.thumb
.section .text,"ax",%progbits
.global RmTest_FpuFault
.thumb_func
RmTest_FpuFault:
    vmov.f32 s0, #1.0
    vadd.f32 s0, s0, s0
.global RmTest_FpuFaultSite
RmTest_FpuFaultSite:
    udf #0
    b .
