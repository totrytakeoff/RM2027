#include "robot_firmware.h"

#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "robot_app.h"
#include "robot_config.h"
#include "robot_tasks.h"
#include "rm_watchdog.h"

bool RobotFirmware_Init(void)
{
    DWT_Init(ROBOT_CPU_FREQUENCY_MHZ);
    DaemonServiceInit();

    if (!CANConfigureDispatch(CAN_DISPATCH_DEFERRED) ||
        !USARTConfigureDispatch(USART_DISPATCH_DEFERRED)) {
        return false;
    }

#if MINIMAL_DEBUG_ENABLE && ((MINIMAL_DEBUG_MODE & MINIMAL_DEBUG_MODE_TEXT) != 0)
    BSPLogInit();
#endif

    if (!RobotApp_Init()) {
        RobotApp_ForceSafeStop();
        return false;
    }
    if (!RobotTasks_Create()) {
        RobotApp_ForceSafeStop();
        return false;
    }
    if (!RmWatchdog_Start(ROBOT_HARDWARE_WATCHDOG_TIMEOUT_MS)) {
        RobotApp_ForceSafeStop();
        return false;
    }

    return true;
}
