#include "logging.h"
#include "rm_platform.h"
typedef struct { uint32_t time_ms, event, a, b; } Record;
#define LOG_DEPTH 64U
static Record ring[LOG_DEPTH];
static uint32_t head, tail, dropped;
static RmSerialId output_serial=RM_SERIAL_CONSOLE;
bool RmLog_SetOutput(RmSerialId serial)
{
    if (serial!=RM_SERIAL_CONSOLE && serial!=RM_SERIAL_VISION && serial!=RM_SERIAL_REFEREE) return false;
    output_serial=serial; return true;
}
static size_t append_text(char *line,size_t n,const char *text)
{
    while (*text) line[n++]=*text++;
    return n;
}
static size_t append_number(char *line,size_t n,uint32_t value)
{
    char digits[10]; unsigned count=0;
    do { digits[count++]=(char)('0'+value%10U); value/=10U; } while (value);
    while (count) line[n++]=digits[--count];
    return n;
}
void RmLog_Publish(RmLogEvent event,uint32_t a,uint32_t b)
{
    RmCriticalState key=RmCritical_Enter();
    if (head-tail>=LOG_DEPTH) ++dropped;
    else { ring[head%LOG_DEPTH]=(Record){RmTime_NowMs(),(uint32_t)event,a,b}; ++head; }
    RmCritical_Exit(key);
}
uint32_t RmLog_Dropped(void)
{
    RmCriticalState key=RmCritical_Enter(); uint32_t n=dropped; RmCritical_Exit(key); return n;
}
void RmLog_Drain(unsigned budget)
{
    /* Sole consumer; formatting and DMA submission only happen in diagnostics task. */
    while (budget--) {
        RmCriticalState key=RmCritical_Enter();
        if (head==tail) { RmCritical_Exit(key); return; }
        Record r=ring[tail%LOG_DEPTH];
        RmCritical_Exit(key);
        /* Four uint32 values need at most 40 digits plus 20 literal bytes.
           Integer-only formatting avoids libc printf's hidden heap paths. */
        char line[64]; size_t n=0;
        n=append_text(line,n,"RM "); n=append_number(line,n,r.time_ms);
        n=append_text(line,n," event="); n=append_number(line,n,r.event);
        n=append_text(line,n," a="); n=append_number(line,n,r.a);
        n=append_text(line,n," b="); n=append_number(line,n,r.b);
        n=append_text(line,n,"\r\n");
        if (RmSerial_TrySendCopy(output_serial,line,n)!=RM_OK) return;
        key=RmCritical_Enter(); ++tail; RmCritical_Exit(key);
    }
}
