#ifndef TASK_PUBLIC_H
#define TASK_PUBLIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"

void IMU_task(void * argument);
void StartReceiveTask(void *argument);
void StartGimbalTask(void *argument);
void StartScreenTask(void *argument);
void StartTTSTask(void *argument);

extern osThreadId_t GimbalTaskHandle;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "Gimbal.h"
#include "QGIMBAL_PID.h"

extern Gimbal gimbal;
extern QGIMBAL_PID vision_x_pid;
extern QGIMBAL_PID vision_y_pid;

#endif

#endif //TASK_PUBLIC_H
