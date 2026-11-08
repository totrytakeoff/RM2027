#include "rm_platform.h"
#include "bringup_app.h"

const RmRuntimeConfig *RmMotorTest_Init(void);

int main(void)
{
    RmBoard_Init();
    RmRuntime_Start(RmMotorTest_Init());
}
