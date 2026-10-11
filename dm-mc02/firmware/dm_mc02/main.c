#include "rm_platform.h"
#include "bringup_app.h"
int main(void)
{
    RmBoard_Init();
    RmRuntime_Start(RmBringup_Init());
}
