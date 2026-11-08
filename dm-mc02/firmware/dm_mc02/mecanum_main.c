#include "rm_platform.h"
#include "mecanum_app.h"
int main(void)
{
    RmBoard_Init();
    RmRuntime_Start(RmMecanumApp_Init());
}
