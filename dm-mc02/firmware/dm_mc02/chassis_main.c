#include "rm_platform.h"
#include "chassis_app.h"
int main(void)
{
    RmBoard_Init();
    RmRuntime_Start(RmChassisApp_Init());
}
