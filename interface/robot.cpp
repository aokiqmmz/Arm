/**
 ******************************************************************************
 * @file    robot.cpp/h
 * @brief   Main program. 主程序
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/robot.h"

#include "cmsis_os.h"
#include "hardware_config.h"

// #include "base/monitor/can_monitor.h"
// #include "app/client_ui.h"
#include "app/control/control.h"
#include "base/monitor/can_monitor.h"
#include "base/monitor/imu_monitor.h"
#include "base/monitor/motor_monitor.h"
// #include "app/serial_tool.h"
#include "base/remote/remote.h"
#include "base/servo/servo.h"
#include "common/communication/cap_comm/cap_comm.h"
#include "common/communication/cv_comm/cv_comm.h"
#include "common/communication/referee_comm/referee_comm.h"

#ifdef RC_UART
RC rc(RC_UART);
#else
RC rc;
#endif  // RC_UART
#ifdef CV_UART
CVComm cv_comm(CV_UART);
#else
CVComm cv_comm;
#endif  // CV_UART
#ifdef REFEREE_UART
RefereeComm referee(REFEREE_UART);
// UI ui(REFEREE_UART, &referee, ui_func, sizeof(ui_func) / sizeof(void*));
#else
RefereeComm referee;
#endif  // REFEREE_UART
#ifdef SERVO_UART
ServoZX361D gate_servo(SERVO_UART);
#else
ServoZX361D gate_servo;
#endif  // SERVO_UART
// #ifdef DEBUG_UART
// SerialStudio serial_tool(DEBUG_UART);
// #else
// SerialStudio serial_tool;
// #endif  // DEBUG_UART

// CapComm ultra_cap(&hcan2);

/* FreeRTOS tasks-----------------------------------------------------------*/
osThreadId controlTaskHandle;
void controlTask(void const* argument) {
  rc.init();
  controlInit();
  for (;;) {
    rc.handle();
    controlLoop();
    osDelay(5);
  }
}

osThreadId appTaskHandle;
void appTask(void const* argument) {
  robotCmdInit();
  for (;;) {
    robotCmdSend();
    ModuleControl();
    osDelay(1);
  }
}

osThreadId motorTaskHandle;
void motorTask(void const* argument) {
  allMotorsInit();
  for (;;) {
    allMotorsHandle();
    osDelay(1);
  }
}

osThreadId canTaskHandle;
void canTask(void const* argument) {
  canFilterInit();
  for (;;) {
    canTxMonitor();
  }
}

osThreadId imuTaskHandle;
void imuTask(void const* argument) {
  imu::initAll();
  for (;;) {
    imu::handleAll();
    osDelay(1);
  }
}

osThreadId minipcCommTaskHandle;
void minipcCommTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  cv_comm.init();
  for (;;) {
    cv_comm.txHandle(&tick);
    //    osDelay(1);
  }
}

osThreadId refereeCommTaskHandle;
void refereeCommTask(void const* argument) {
  referee.init();
  //  ui.init();
  for (;;) {
    referee.handle();
    //    ui.handle();
    osDelay(1);
  }
}

// osThreadId serialToolTaskHandle;
// void serialToolTask(void const* argument) {
//   uint32_t tick = osKernelSysTick();
//   for (;;) {
//     serial_tool.handle();
//     osDelayUntil(&tick, 20);
//   }
// }

// Create and config tasks
void rtosTaskInit(void) {
  osThreadDef(control_task, controlTask, osPriorityAboveNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

  osThreadDef(motor_task, motorTask, osPriorityHigh, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motor_task), NULL);

  osThreadDef(app_task, appTask, osPriorityHigh, 0, 128);
  appTaskHandle = osThreadCreate(osThread(app_task), NULL);

  osThreadDef(can_task, canTask, osPriorityHigh, 0, 128);
  canTaskHandle = osThreadCreate(osThread(can_task), NULL);

  osThreadDef(imu_task, imuTask, osPriorityRealtime, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(imu_task), NULL);

  osThreadDef(minipc_comm_task, minipcCommTask, osPriorityNormal, 0, 512);
  minipcCommTaskHandle = osThreadCreate(osThread(minipc_comm_task), NULL);

  osThreadDef(referee_comm_task, refereeCommTask, osPriorityNormal, 0, 512);
  refereeCommTaskHandle = osThreadCreate(osThread(referee_comm_task), NULL);

  //  osThreadDef(serial_tool_task, serialToolTask, osPriorityLow, 0, 1024);
  //  serialToolTaskHandle = osThreadCreate(osThread(serial_tool_task), NULL);
}
