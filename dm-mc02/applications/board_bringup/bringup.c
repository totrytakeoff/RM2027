#include "bringup_app.h"
#include "bringup.h"
#include "bmi088.h"
#include "output_gate.h"
#include "logging.h"
#include "vision_protocol.h"
#include "safety_manager.h"
#include "binding.h"
#include <string.h>
#if !defined(RM_CHASSIS_DEMO)
_Static_assert(RM_ENABLE_MOTION==0U,"Bringup is receive-only; use a dedicated robot profile for motion");
#endif
_Static_assert(RM_ENABLE_HEATER==0U,"Heater control requires a validated temperature profile");
static RmBmi088 imu;
static RmBmiState reported_imu_state;
static RmImuSample imu_snapshot;
static RmIns ins;
static RmInsSnapshot ins_snapshot;
static RmInsState reported_ins_state;
static RmRemote remote;
/* I/O owns the parser. Readers and the edge consumer only touch this mailbox. */
static RemoteControlState remote_snapshot;
static uint32_t remote_snapshot_generation;
static RmDjiRegistry motor_registry;
static RmDjiFeedbackState motor_feedback[RM_DJI_MAX_MOTORS]; /* I/O owner */
static RmDjiFeedbackState dji_candidates[RM_CAN_COUNT][11];
static RmDjiCommand motor_commands[RM_DJI_MAX_MOTORS]; /* Assembly mailbox */
static RmDjiGroup motor_groups[RM_DJI_MAX_GROUPS];
static uint32_t motor_submit_failures;
static RmVisionRxStream vision_stream;
static RmVisionTarget vision_target;
static RmSafetyManager safety;
static uint32_t serial_generation[RM_SERIAL_COUNT];
static uint32_t observed_can[RM_CAN_COUNT], observed_serial[RM_SERIAL_COUNT];
static bool bootstrap_done;
static void ins_tick(uint64_t now_us)
{
    RmBmi088_Tick(&imu,now_us);
    uint64_t sample_now=RmTime_NowUs();
    if (imu.state==RM_BMI_FAILED) RmIns_Fail(&ins,RM_INS_ERROR_SENSOR);
    else RmIns_Push(&ins,&imu.sample,sample_now);
    RmIns_Poll(&ins,sample_now);
    RmInsSnapshot fused;
    (void)RmIns_Read(&ins,sample_now,&fused);
    RmCriticalState key=RmCritical_Enter();
    imu_snapshot=imu.sample;
    ins_snapshot=fused;
    if (fused.state==RM_INS_FAILED || (fused.state==RM_INS_RUNNING && fused.valid)) bootstrap_done=true;
    RmCritical_Exit(key);
    if (imu.state!=reported_imu_state && (imu.state==RM_BMI_READY || imu.state==RM_BMI_FAILED)) {
        RmLog_Publish(imu.state==RM_BMI_READY?RM_LOG_IMU_READY:RM_LOG_IMU_FAILED,
            ((uint32_t)imu.accel_id<<8)|imu.gyro_id,imu.phase);
        reported_imu_state=imu.state;
    }
    if (fused.state!=reported_ins_state) {
        RmLog_Publish(RM_LOG_INS_STATE,(uint32_t)fused.state,(uint32_t)fused.error);
        reported_ins_state=fused.state;
    }
}
static void io_tick(uint64_t now_us)
{
    uint32_t now_ms=(uint32_t)(now_us/1000U);
    RmSerial_Service(now_ms); RmCan_Service(now_ms);
    for (unsigned bus=0;bus<RM_CAN_COUNT;++bus) {
        RmCanFrame frame;
        for (unsigned n=0;n<RM_IO_FRAMES_PER_BUS && RmCan_Read((RmCanId)bus,&frame);++n) {
            ++observed_can[bus];
            if (!frame.extended && frame.identifier>=0x201U && frame.identifier<=0x20bU) {
                unsigned index=frame.identifier-0x201U;
                RmDjiFeedbackState candidate=dji_candidates[bus][index];
                if (RmDji_Decode(&candidate,&frame,20U)) {
                    RmCriticalState key=RmCritical_Enter(); dji_candidates[bus][index]=candidate; RmCritical_Exit(key);
                }
            }
            for (unsigned i=0;i<motor_registry.count;++i) {
                const RmDjiMotor *m=&motor_registry.motors[i];
                if (m->config.bus==(RmCanId)bus && m->feedback_id==frame.identifier && !frame.extended) {
                    RmDjiFeedbackState next=motor_feedback[i];
                    if (RmDji_Decode(&next,&frame,m->config.feedback_timeout_ms)) {
                        RmCriticalState key=RmCritical_Enter(); motor_feedback[i]=next; RmCritical_Exit(key);
                    }
                }
            }
        }
    }
    for (unsigned id=0;id<RM_SERIAL_COUNT;++id) {
        RmSerialChunk chunk;
        for (unsigned n=0;n<RM_IO_CHUNKS_PER_SERIAL && RmSerial_Read((RmSerialId)id,&chunk);++n) {
            observed_serial[id]+=chunk.length;
            if (id==RM_SERIAL_VISION) {
                if (serial_generation[id]!=chunk.generation) RmVisionRx_Reset(&vision_stream);
                (void)RmVisionRx_Push(&vision_stream,chunk.data,chunk.length,&vision_target);
            } else if (id==RM_SERIAL_REMOTE) {
                RmRemote_Push(&remote,chunk.data,chunk.length,(uint32_t)(chunk.timestamp_us/1000U),chunk.generation);
            }
            serial_generation[id]=chunk.generation;
        }
    }
    /* React to a transport discontinuity even before a new chunk is available. */
    RmSerialStats remote_transport;
    if (RmSerial_GetStats(RM_SERIAL_REMOTE,&remote_transport) && remote.generation!=remote_transport.generation)
        RmRemote_Discontinuity(&remote,remote_transport.generation);
    RemoteControlState next;
    (void)RmRemote_Take(&remote,RmTime_NowMs(),&next);
    RmCriticalState key=RmCritical_Enter();
    uint32_t publish_ms=RmTime_NowMs();
    if (!RmSerial_GetStats(RM_SERIAL_REMOTE,&remote_transport) || !remote_transport.online ||
        remote.generation!=remote_transport.generation) memset(&next,0,sizeof(next));
    if (next.data_valid && remote_snapshot.data_valid &&
        remote_snapshot_generation==remote.generation &&
        publish_ms-remote_snapshot.sample_time_ms<rm_bringup_binding.remote.timeout_ms) {
        next.keys_pressed |= remote_snapshot.keys_pressed;
        next.keys_released |= remote_snapshot.keys_released;
        next.buttons_pressed |= remote_snapshot.buttons_pressed;
        next.buttons_released |= remote_snapshot.buttons_released;
        next.switch_changed_mask |= remote_snapshot.switch_changed_mask;
    }
    remote_snapshot=next; remote_snapshot_generation=remote.generation;
    RmCritical_Exit(key);
}
static void motor_tick(uint64_t now_us)
{
    static uint64_t last;
    float dt=last?(float)(now_us-last)*1e-6f:0.005f; last=now_us;
    RmDjiCommand commands[RM_DJI_MAX_MOTORS];
    RmCriticalState key=RmCritical_Enter();
    for (unsigned i=0;i<motor_registry.count;++i) motor_registry.motors[i].feedback=motor_feedback[i];
    memcpy(commands,motor_commands,sizeof(commands));
    RmCritical_Exit(key);
    for (unsigned i=0;i<motor_registry.count;++i) {
        RmCanStats transport; RmDjiMotor *m=&motor_registry.motors[i];
        if (!RmCan_GetStats(m->config.bus,&transport) || !transport.online ||
            m->feedback.value.generation!=transport.generation) {
            m->feedback.value.valid=false; m->feedback.value.position_valid=false;
        }
    }
    RmOutputGateSnapshot gate; RmOutputGate_Read(&gate);
    bool permit=RM_ENABLE_MOTION && RmOutputGate_Allows(gate.epoch,(uint32_t)(now_us/1000U));
    uint32_t frame_deadline=gate.deadline_ms;
    uint32_t now_ms=(uint32_t)(now_us/1000U);
    if ((int32_t)(frame_deadline-now_ms)>10) frame_deadline=now_ms+10U;
    unsigned count=RmDji_Step(&motor_registry,commands,motor_registry.count,now_us,dt,permit,
        gate.epoch,frame_deadline,motor_groups);
    /* Zero frames are emitted only for explicitly bound DJI command groups. */
    for (unsigned i=0;i<count;++i) {
        RmStatus result=permit?RmCan_Submit(&motor_groups[i].frame,gate.epoch,motor_groups[i].deadline_ms):
            RmCan_SubmitZeroFrame(&motor_groups[i].frame,(uint32_t)(now_us/1000U)+20U);
        if (result!=RM_OK) ++motor_submit_failures;
    }
}
static void health_tick(uint64_t now_us)
{
    RmBoard_HeaterOff();
    RmInsSnapshot fused; RemoteControlState input;
    (void)RmBringup_ReadIns(&fused); (void)RmBringup_ReadRemote(&input);
    if (!fused.valid || !input.data_valid) RmOutputGate_Stop(1U<<12);
    RmOutputGateSnapshot gate; RmOutputGate_Read(&gate);
    if (gate.permitted && !RmOutputGate_Allows(gate.epoch,(uint32_t)(now_us/1000U))) RmOutputGate_Stop(1U<<8);
}
static void control_tick(uint64_t now_us)
{
    (void)now_us;
    RemoteControlState input;
    (void)RmBringup_TakeRemote(&input);
    const RmSafetyInputs inputs={.initialization_complete=true,.input_online=false,
        .task_health_ok=true,.device_health_ok=false};
    (void)RmSafety_Update(&safety,&inputs);
    RmOutputGate_Stop(safety.reasons);
}
static void diagnostics_tick(uint64_t now_us)
{
    static uint32_t next_report;
    uint32_t now_ms=(uint32_t)(now_us/1000U);
    if ((int32_t)(now_ms-next_report)>=0) {
        next_report=now_ms+1000U;
        RmImuSample s;
        RmCriticalState key=RmCritical_Enter(); s=imu_snapshot; RmCritical_Exit(key);
        RmLog_Publish(RM_LOG_HEARTBEAT,now_ms,RmLog_Dropped());
        RmLog_Publish(RM_LOG_IMU_SAMPLE,s.sequence,s.valid?1U:0U);
        RmInsSnapshot fused; RemoteControlState input;
        (void)RmBringup_ReadIns(&fused); (void)RmBringup_ReadRemote(&input);
        RmLog_Publish(RM_LOG_INS_SAMPLE,fused.sequence,fused.valid?1U:0U);
        RmLog_Publish(RM_LOG_REMOTE_SAMPLE,input.sample_sequence,(input.data_valid?1U:0U)|(input.failsafe?2U:0U));
        RmLog_Publish(RM_LOG_MOTOR_IO,motor_registry.count,motor_submit_failures);
        for (unsigned bus=0;bus<RM_CAN_COUNT;++bus) {
            uint32_t seen=0,valid=0;
            for (unsigned id=0x201U;id<=0x20bU;++id) {
                RmDjiFeedback feedback;
                bool online=RmBringup_ReadDjiCandidate((RmCanId)bus,id,&feedback);
                if (feedback.sequence) seen|=1U<<(id-0x201U);
                if (online) valid|=1U<<(id-0x201U);
            }
            RmLog_Publish(RM_LOG_DJI_CANDIDATES,(bus<<28)|seen,valid);
        }
        for (unsigned i=0;i<RM_CAN_COUNT;++i) {
            RmCanStats st;
            if (RmCan_GetStats((RmCanId)i,&st)) RmLog_Publish(RM_LOG_CAN_STATS,(i<<28)|(st.rx_frames&0x0FFFFFFFU),st.rx_dropped);
        }
        for (unsigned i=0;i<RM_SERIAL_COUNT;++i) {
            RmSerialStats st;
            if (RmSerial_GetStats((RmSerialId)i,&st)) {
                RmLog_Publish(RM_LOG_SERIAL_STATS,(i<<28)|(st.rx_bytes&0x0FFFFFFFU),st.rx_dropped);
                RmLog_Publish(RM_LOG_SERIAL_RECOVERY,(i<<28)|(st.online?1U:0U)|(st.quarantined?2U:0U),
                    ((st.abort_timeouts&0xffffU)<<16)|(st.recovery_exhausted&0xffffU));
            }
        }
        for (unsigned i=0;i<RM_TASK_COUNT;++i) {
            RmTaskStats st;
            if (RmRuntime_ReadTask((RmTaskId)i,&st)) {
                RmLog_Publish(RM_LOG_TASK_TIMING,(i<<28)|(st.deadline_misses&0x0FFFFFFFU),st.max_runtime_us);
                RmLog_Publish(RM_LOG_TASK_STACK,(i<<28)|(st.run_count&0x0FFFFFFFU),st.stack_free_words);
            }
        }
    }
    RmLog_Drain(2U);
}
static bool complete(void)
{
    RmCriticalState key=RmCritical_Enter(); bool done=bootstrap_done; RmCritical_Exit(key); return done;
}
static void fault(uint32_t reasons)
{
    RmOutputGate_Stop(reasons); RmBoard_HeaterOff();
    static uint32_t last_log;
    uint32_t now=RmTime_NowMs();
    if (now-last_log>=1000U) { last_log=now; RmLog_Publish(RM_LOG_TASK_FAULT,reasons,0U); }
}
static const RmRuntimeConfig runtime={
    .tick={ins_tick,io_tick,motor_tick,health_tick,control_tick,diagnostics_tick},
    .bootstrap_complete=complete,.fault=fault
};
const RmRuntimeConfig *RmBringup_Init(void)
{
    RmOutputGate_Init(); RmSafety_Init(&safety); RmVision_Init(); RmVisionRx_Reset(&vision_stream);
    RmCan_SetSafetyHooks(RmOutputGate_Allows,RmOutputGate_Stop);
    RmOutputGate_SetStopHook(RmCan_CancelMotion);
    if (!RmIns_Init(&ins,&rm_bringup_binding.ins,RmTime_NowUs()) ||
        !RmRemote_Init(&remote,&rm_bringup_binding.remote) ||
        !RmDji_Init(&motor_registry,rm_bringup_binding.motors,rm_bringup_binding.motor_count)) RmFault_Panic(0x4002U);
    if (!RmSerial_Start() || !RmCan_Start()) RmFault_Panic(0x4001U);
    RmBmi088_Init(&imu,RmBoard_ImuBus(),RM_IMU_BOOT_TIMEOUT_US);
    const RmBoardInfo *b=RmBoard_GetInfo();
    RmLog_Publish(RM_LOG_BOOT,b->cpuid,b->hal_version);
    RmLog_Publish(RM_LOG_CLOCKS,b->cpu_hz,b->fdcan_hz);
    RmLog_Publish(RM_LOG_RESET_FLAGS,b->reset_flags,b->dcache_enabled?1U:0U);
    RmLog_Publish(RM_LOG_BINDING,(uint32_t)rm_bringup_binding.remote.protocol,rm_bringup_binding.motor_count);
    RmFaultInfo previous;
    if (RmFault_ReadPrevious(&previous))
        RmLog_Publish(RM_LOG_PREVIOUS_FAULT,previous.reason,previous.frame[6]);
    return &runtime;
}
bool RmBringup_ReadIns(RmInsSnapshot *out)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter(); *out=ins_snapshot; RmCritical_Exit(key);
    uint64_t now=RmTime_NowUs();
    out->valid=out->valid && now>=out->timestamp_us && now-out->timestamp_us<=rm_bringup_binding.ins.freshness_us;
    return out->valid;
}
static bool read_remote(RemoteControlState *out,bool consume)
{
    if (!out) return false;
    RmCriticalState key=RmCritical_Enter();
    *out=remote_snapshot;
    RmSerialStats transport;
    bool good=out->data_valid && !out->failsafe &&
        RmTime_NowMs()-out->sample_time_ms<rm_bringup_binding.remote.timeout_ms &&
        RmSerial_GetStats(RM_SERIAL_REMOTE,&transport) && transport.online &&
        transport.generation==remote_snapshot_generation;
    if (!good) {
        RemoteControlType type=out->type; uint32_t caps=out->capabilities; uint8_t failsafe=out->failsafe;
        memset(out,0,sizeof(*out)); out->type=type; out->capabilities=caps; out->failsafe=failsafe;
    }
    if (consume) {
        remote_snapshot.keys_pressed=remote_snapshot.keys_released=0U;
        remote_snapshot.buttons_pressed=remote_snapshot.buttons_released=0U;
        remote_snapshot.switch_changed_mask=0U;
    }
    RmCritical_Exit(key); return good;
}
bool RmBringup_ReadRemote(RemoteControlState *out) { return read_remote(out,false); }
bool RmBringup_TakeRemote(RemoteControlState *out) { return read_remote(out,true); }
bool RmBringup_ReadDjiCandidate(RmCanId bus,uint32_t id,RmDjiFeedback *out)
{
    if (!out || (unsigned)bus>=RM_CAN_COUNT || id<0x201U || id>0x20bU) return false;
    RmCriticalState key=RmCritical_Enter();
    *out=dji_candidates[bus][id-0x201U].value;
    RmCanStats transport;
    bool connected=RmCan_GetStats(bus,&transport) && transport.online && out->generation==transport.generation;
    RmCritical_Exit(key);
    uint64_t now=RmTime_NowUs();
    out->valid=out->valid && connected && now>=out->timestamp_us && now-out->timestamp_us<20000U;
    if (!out->valid) out->position_valid=false;
    return out->valid;
}

bool RmBringup_ReadMotor(unsigned motor,RmDjiFeedback *out)
{
    if (!out || motor>=motor_registry.count) return false;
    const RmDjiConfig *config=&motor_registry.motors[motor].config;
    RmCriticalState key=RmCritical_Enter();
    *out=motor_feedback[motor].value;
    RmCanStats transport;
    bool connected=RmCan_GetStats(config->bus,&transport) && transport.online &&
        out->generation==transport.generation;
    uint64_t now=RmTime_NowUs();
    out->valid=out->valid && connected && now>=out->timestamp_us &&
        now-out->timestamp_us<(uint64_t)config->feedback_timeout_ms*1000U;
    if (!out->valid) out->position_valid=false;
    RmCritical_Exit(key);
    return out->valid;
}

bool RmBringup_SubmitMotorCommands(const RmDjiCommand *commands,unsigned count,uint32_t epoch)
{
    if (!commands || count!=motor_registry.count || !count) return false;
    RmCriticalState key=RmCritical_Enter();
    uint32_t now=RmTime_NowMs();
    bool valid=RmOutputGate_Allows(epoch,now);
    for (unsigned i=0;i<count && valid;++i) {
        const RmDjiCommand *c=&commands[i];
        valid=c->epoch==epoch && (int32_t)(c->deadline_ms-now)>0 &&
            c->deadline_ms-now<=motor_registry.motors[i].config.command_timeout_ms &&
            isfinite(c->reference) && isfinite(c->speed_ff) && isfinite(c->current_ff) &&
            isfinite(c->external_angle) && isfinite(c->external_speed);
    }
    if (valid) memcpy(motor_commands,commands,count*sizeof(*commands));
    RmCritical_Exit(key);
    return valid;
}
