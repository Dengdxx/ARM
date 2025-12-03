#include "common_inc.h"

/**
 * @file main.cpp
 * @brief 机器人固件的主入口点和控制循环定义。
 *
 * 此文件包含主要应用程序逻辑，包括硬件外围设备（OLED、IMU、定时器、PWM）的设置，
 * 用于控制循环和显示更新的 FreeRTOS 线程定义，以及主入口点 `Main()`。
 */

// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000);
// Robot instance
DummyRobot dummy(&hcan1);


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;

/**
 * @brief 固定频率控制循环更新的线程。
 *
 * 此线程等待定时器中断的通知，然后根据机器人当前的命令模式执行控制操作。
 * 它处理关节运动、轨迹跟踪和电机调整。
 *
 * @param argument 未使用的线程参数。
 */
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dummy.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);
                    dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    break;
            }
        } else
        {
            // Just update Joint states
            dummy.UpdateJointAngles();
            dummy.UpdateJointPose6D();
        }
    }
}


osThreadId_t ControlLoopUpdateHandle;

/**
 * @brief 异步控制循环更新的线程。
 *
 * 此线程不断处理来自命令处理程序队列的命令。
 *
 * @param argument 未使用的线程参数。
 */
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}


osThreadId_t oledTaskHandle;

/**
 * @brief 用于更新 OLED 显示屏的线程。
 *
 * 此线程定期使用 IMU 数据、关节角度、末端执行器姿态和机器人状态信息更新 OLED 屏幕。
 *
 * @param argument 未使用的线程参数。
 */
void ThreadOledUpdate(void* argument)
{
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    for (;;)
    {
        mpu6050.Update(true);

        oled.clearBuffer();
        oled.setFont(u8g2_font_5x8_tr);
        oled.setCursor(0, 10);
        oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
        oled.setCursor(85, 10);
        oled.printf("| FPS:%lu", 1000000 / (micros() - t));
        t = micros();

        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
                    (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
                    (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));

        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
                    (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
                    (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        if (dummy.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
        }

        oled.sendBuffer();
    }
}


/* Timer Callbacks -------------------------------------------------------*/

/**
 * @brief 定时器 7 的回调函数。
 *
 * 当定时器到期时调用此函数。它通知 `ThreadControlLoopFixUpdate` 线程运行。
 */
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Default Entry -------------------------------------------------------*/

/**
 * @brief 主应用程序入口点。
 *
 * 初始化通信接口、机器人实例、IMU、OLED 和 PWM。
 * 创建并启动用户线程（控制循环、显示更新）。
 * 启动控制循环定时器和调度程序。
 */
void Main(void)
{
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init IMU.
    do
    {
        mpu6050.Init();
        osDelay(100);
    } while (!mpu6050.testConnection());
    mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}
