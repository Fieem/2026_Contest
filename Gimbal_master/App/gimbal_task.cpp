#include "FreeRTOS.h"
#include "task.h"
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "QD4310.h"
#include "QGIMBAL_PID.h"
#include "Gimbal.h"
#include "simple_kalman.h"
#include "tts_task.h"
#include "tts_word.h"

constexpr static float yaw_center = 0.0f;   // 云台偏航中心位置,单位: rad
constexpr static float pitch_center = 0.0f; // 云台俯仰中心位置,单位: rad

extern float INS_angle[3];       // yaw,pitch,roll

QD4310 YawMotor(&hfdcan1, 0x00);   // 云台偏航电机
QD4310 PitchMotor(&hfdcan1, 0x01); // 云台俯仰电机

Gimbal gimbal(
    YawMotor, PitchMotor,
    yaw_center, pitch_center,
    QGIMBAL_PID{
        QGIMBAL_PID::PID_type::position_type,
        5.7f, 0.11f, 120.0f,
        1.8f, -1.8f,
        1, -1
    },
    QGIMBAL_PID{
        QGIMBAL_PID::PID_type::position_type,
        4.6f, 0.17f, 30.0f,
        1.8f, -1.8f,
        1, -1
    },
    0.001f);

QGIMBAL_PID vision_x_pid{
    QGIMBAL_PID::PID_type::position_type,
    0.0003f, 0.00005f, 0.000f,
    20, -20,
    200, -200
};
QGIMBAL_PID vision_y_pid{
    QGIMBAL_PID::PID_type::position_type,
    0.0003f, 0.00005f, 0.000f,
    20, -20,
    200, -200
};

extern SimpleKalman2D_t yaw_kf;
extern SimpleKalman2D_t pitch_kf;

void CAN_InterfaceInit();

void StartGimbalTask(void *argument) {
    CAN_InterfaceInit();
    YawMotor.enable();
    PitchMotor.enable();

    // 上电复位云台角度
    YawMotor.setAngle(yaw_center);
    PitchMotor.setAngle(pitch_center);
    // 等待陀螺仪初始化完成
    osDelay(pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(Laser_En_GPIO_Port,Laser_En_Pin, GPIO_PIN_SET);
    gimbal.enable();
    osDelay(50);
    gimbal.enable_stability();
    while (true) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        gimbal.Ctrl_ISR(INS_angle[0], INS_angle[1]);
    }
}

void CAN_InterfaceInit() {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan == &hfdcan1) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.Identifier >= 0x500 && rx_header.Identifier <= 0x508) {
            if (rx_header.Identifier == 0x500) {
                YawMotor.update(rx_data);
            } else if (rx_header.Identifier == 0x501) {
                PitchMotor.update(rx_data);
            }
        }
    }
}