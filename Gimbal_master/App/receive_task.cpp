#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include <cmath>
#include <sys/types.h>

#include "simple_kalman.h"
#include "task_public.h"
#include "usart.h"
#include "screen_protocol.h"
#include "tts_task.h"

#define MAX_FLOAT_NUM 8
#define FRAME_HEADER  0xA5
#define RAW_BUF_SIZE 256
#define yaw_control_kxmax 0.002f
#define yaw_control_kxmin 0.0005f
#define pitch_control_kxmax 0.001f
#define pitch_control_kxmin 0.0004f

extern QueueHandle_t screenQueue;
extern uint8_t screenRxBuffer[10];

#define mapping(x, in_min, in_max, out_min, out_max) \
((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

// 接收协议结构体
typedef struct {
    uint16_t cmd_id;
    uint16_t flags;
    uint8_t  float_num;
    float    data[MAX_FLOAT_NUM];
} ReceiveFrame_t;

// 串口状态机
typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_LEN,
    STATE_RECEIVE_PAYLOAD
} RxState_t;

typedef struct {
    uint8_t data[RAW_BUF_SIZE];
    uint8_t len;
} RawPacket_t;

QueueHandle_t uartQueue = NULL;
uint8_t aRxBuffer[1]; // 中断接收单字节缓冲区
const char *id_to_chinese[] = {
    "苹果",     // 1
    "香蕉",     // 2
    "正方形",   // 3
    "草莓",     // 4
    "梯形",     // 5
    "三角形",   // 6
    "西瓜",     // 7
    "圆"        // 8
};

const char *id_to_english[] = {
    "Apple",      // 1
    "Banana",     // 2
    "Square",     // 3
    "Strawberry", // 4
    "Trapezoid",  // 5
    "Triangle",   // 6
    "Watermelon", // 7
    "Circle"      // 8
};

SimpleKalman2D_t yaw_kf;
SimpleKalman2D_t pitch_kf;
float filtered_yaw_error;
float filtered_pitch_error;
float yaw_angle_change;
float pitch_angle_change;
uint8_t last_id = 0;
uint8_t label_id;
uint8_t stable_count = 0;
uint8_t finish_flag = 0;

void StartReceiveTask(void *argument)
{
    RawPacket_t raw_pkt;
    ReceiveFrame_t frame; // 最终解析出的结构体

    SimpleKalman2D_Init(&yaw_kf, 0.1f, 1.0f, 1.0f, 0.001f, 5.0f);
    SimpleKalman2D_Init(&pitch_kf, 0.1f, 1.0f, 1.0f, 0.001f, 5.0f);

    uartQueue = xQueueCreate(10, sizeof(RawPacket_t));

    HAL_UART_Receive_IT(&huart1, aRxBuffer, 1);

    for(;;)
    {
        if (xQueueReceive(uartQueue, &raw_pkt, portMAX_DELAY) == pdPASS)
        {
            frame.cmd_id = raw_pkt.data[0] | (raw_pkt.data[1] << 8);
            frame.flags  = raw_pkt.data[2] | (raw_pkt.data[3] << 8);

            frame.float_num = (raw_pkt.len - 4) / 4;
            if (frame.float_num > MAX_FLOAT_NUM) frame.float_num = MAX_FLOAT_NUM;

            memcpy(frame.data, &raw_pkt.data[4], frame.float_num * sizeof(float));

            label_id = static_cast<uint8_t>(frame.data[3]);
            if (label_id != 8)
            {
                if (label_id == 99 && finish_flag == 0)
                {
                    TTS_Speak("七个物体");
                    printsf(0, "7");
                    printsf(0, "");
                    finish_flag = 1;
                }
                if (label_id != last_id) {
                    // ID 变化 → 重置计数
                    last_id = label_id;
                    stable_count = 0;
                } else {
                    // ID 连续相同 → 计数+1
                    stable_count++;

                    if (stable_count == 20) {
                        TTS_Speak(id_to_chinese[label_id - 1]);  // 播报中文
                        printsf(0, id_to_english[label_id - 1]);
                        printsf(0, "");
                    }
                }
            }
            //执行业务逻辑
            LIMIT_MIN_MAX(frame.data[0], -100.0f, 100.0f);
            LIMIT_MIN_MAX(frame.data[1], -100.0f, 100.0f);
            LIMIT_MIN_MAX(frame.data[2], 500.0f, 1560.0f);

            filtered_yaw_error = SimpleKalman2D_Update(&yaw_kf, frame.data[0]);
            filtered_pitch_error = SimpleKalman2D_Update(&pitch_kf, frame.data[1]);

            yaw_angle_change = vision_x_pid.calc(filtered_yaw_error);
            pitch_angle_change = vision_y_pid.calc(filtered_pitch_error);
            yaw_angle_change = yaw_angle_change * mapping(frame.data[2], 500.0f, 1560.0f, yaw_control_kxmax, yaw_control_kxmin);
            pitch_angle_change = 0.001f * pitch_angle_change;

            // if (fabs(yaw_angle_change) < 0.02f) yaw_angle_change = 0.0f;
            // if (fabs(pitch_angle_change) < 0.02f) pitch_angle_change = 0.0f;

            gimbal.Ctrl(yaw_angle_change, pitch_angle_change);
        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        static RxState_t state = STATE_WAIT_HEADER;
        static RawPacket_t temp_pkt;
        static uint8_t rx_cnt = 0;
        uint8_t byte = aRxBuffer[0];
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        switch (state)
        {
        case STATE_WAIT_HEADER:
            if (byte == FRAME_HEADER) {
                state = STATE_WAIT_LEN;
            }
            break;

        case STATE_WAIT_LEN:
            if (byte == FRAME_HEADER) {
                state = STATE_WAIT_LEN;
            }else
            {
                temp_pkt.len = byte;
                rx_cnt = 0;
                // 基础长度校验：4字节(ID+Flags) + 至少0字节数据
                if (temp_pkt.len >= 4 && temp_pkt.len <= (RAW_BUF_SIZE)) {
                    state = STATE_RECEIVE_PAYLOAD;
                } else {
                    state = STATE_WAIT_HEADER;
                }
            }
            break;

        case STATE_RECEIVE_PAYLOAD:
            if (byte == FRAME_HEADER && rx_cnt < 2) {
                state = STATE_WAIT_LEN;
                rx_cnt = 0;
                break;
            }
            temp_pkt.data[rx_cnt++] = byte;

            if (rx_cnt >= temp_pkt.len)
            {
                // --- ISR 减负核心：直接投递原始包 ---
                if (uartQueue != NULL) {
                    xQueueSendFromISR(uartQueue, &temp_pkt, &xHigherPriorityTaskWoken);
                }
                state = STATE_WAIT_HEADER;
            }
            break;
        }

        HAL_UART_Receive_IT(huart, aRxBuffer, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (huart->Instance == USART10)
    {
        static char screen_cmd_buffer[SCREEN_CMD_BUF_SIZE];
        static uint8_t screen_cmd_idx = 0;
        uint8_t byte = screenRxBuffer[0];
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (byte == SCREEN_CMD_END_CHAR) {
            screen_cmd_buffer[screen_cmd_idx] = '\0';
            if (screen_cmd_idx > 0 && screenQueue != NULL) {
                ScreenCmd_t cmd;
                cmd.len = screen_cmd_idx;
                memcpy(cmd.cmd, screen_cmd_buffer, screen_cmd_idx);
                xQueueSendFromISR(screenQueue, &cmd, &xHigherPriorityTaskWoken);
            }
            screen_cmd_idx = 0;
        } else if (screen_cmd_idx < SCREEN_CMD_BUF_SIZE - 1) {
            screen_cmd_buffer[screen_cmd_idx++] = byte;
        }

        HAL_UART_Receive_IT(huart, screenRxBuffer, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
        HAL_UART_Receive_IT(huart, aRxBuffer, 1);
    }else if (huart->Instance == USART10) {
        HAL_UART_Receive_IT(huart, screenRxBuffer, 1);
    }
}

// #include "FreeRTOS.h"
// #include "queue.h"
// #include "task.h"
// #include <string.h>
// #include "task_public.h"
// #include "usart.h"
//
// #define MAX_FLOAT_NUM 16
// #define FRAME_HEADER  0xA5
// #define UART_RX_BUF_SIZE  256
//
// typedef struct {
//     uint16_t cmd_id;
//     uint16_t flags;
//     uint8_t  float_num;
//     float    data[MAX_FLOAT_NUM];
// } ReceiveFrame_t;
//
// QueueHandle_t uartQueue = NULL;
// uint8_t UART1_RxBuffer[UART_RX_BUF_SIZE] __attribute__((aligned(32)));
// void StartReceiveTask(void *argument)
// {
//     ReceiveFrame_t received_frame;
//     uartQueue = xQueueCreate(10, sizeof(ReceiveFrame_t));
//
//     if (uartQueue == NULL) for(;;);
//
//     // 启动 DMA 接收到空闲中断
//     // 只要串口线空闲或缓冲区满，就会触发 HAL_UARTEx_RxEventCallback
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUF_SIZE);
//
//     for(;;)
//     {
//         if (xQueueReceive(uartQueue, &received_frame, portMAX_DELAY) == pdPASS)
//         {
//             // 处理逻辑
//             if (received_frame.cmd_id == 0x0101) {
//                 // 激光追踪逻辑...
//             }
//         }
//     }
// }
//
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     if (huart->Instance == USART1)
//     {
//         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//         SCB_InvalidateDCache_by_Addr((uint32_t *)UART1_RxBuffer, UART_RX_BUF_SIZE);
//         uint16_t i = 0;
//
//         // 在收到的 Size 长度数据中循环搜索
//         while (i < Size)
//         {
//             // 1. 寻找帧头
//             if (UART1_RxBuffer[i] == FRAME_HEADER)
//             {
//                 // 检查剩余长度是否足够读出 Length 字节 (i+1)
//                 if (i + 1 >= Size) break;
//
//                 uint8_t payload_len = UART1_RxBuffer[i + 1];
//
//                 // 2. 检查整包数据是否已完整接收 (Header + Len + Payload)
//                 if (i + 2 + payload_len <= Size)
//                 {
//                     ReceiveFrame_t frame;
//                     uint8_t *payload_ptr = &UART1_RxBuffer[i + 2];
//
//                     // 3. 解析 CmdID 和 Flags (小端序)
//                     frame.cmd_id = payload_ptr[0] | (payload_ptr[1] << 8);
//                     frame.flags  = payload_ptr[2] | (payload_ptr[3] << 8);
//
//                     // 4. 解析浮点数
//                     frame.float_num = (payload_len - 4) / 4;
//                     if (frame.float_num > MAX_FLOAT_NUM)
//                         frame.float_num = MAX_FLOAT_NUM;
//
//                     // 使用 memcpy 解决对齐问题
//                     memcpy(frame.data, &payload_ptr[4], frame.float_num * sizeof(float));
//
//                     // 5. 推入队列
//                     xQueueSendFromISR(uartQueue, &frame, &xHigherPriorityTaskWoken);
//
//                     // 指针跳过已处理的整包：Header(1) + Len(1) + Payload(len)
//                     i += (2 + payload_len);
//                 }
//                 else
//                 {
//                     // 数据包不完整（半包），留在缓冲区等下次处理
//                     break;
//                 }
//             }
//             else
//             {
//                 i++; // 没找到帧头，继续找下一个字节
//             }
//         }
//
//         // 关键：重新开启 DMA 接收
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RxBuffer, UART_RX_BUF_SIZE);
//         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
// }
//
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1)
//     {
//         __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
//
//         HAL_UART_DMAStop(huart);
//         HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_RxBuffer, UART_RX_BUF_SIZE);
//     }
// }