// screen_task.cpp
#include <cstring>
#include <cstdlib>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include "task_public.h"
#include "screen_protocol.h"
#include "tts_task.h"

QueueHandle_t screenQueue = NULL;
uint8_t screenRxBuffer[10];
int current_mode = 0;

#define TX_BUF_SIZE 512
#define UART7_TX_BUF_SIZE 256
static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;
static uint8_t tx_byte = 0;
uint8_t send_data;

extern UART_HandleTypeDef huart10;

static void uart10_start_tx_from_isr_or_task(void)
{
    if (tx_busy != 0 || tx_head == tx_tail) {
        return;
    }

    tx_busy = 1;
    tx_byte = tx_buf[tx_tail];
    tx_tail = (tx_tail + 1U) % TX_BUF_SIZE;
    HAL_UART_Transmit_IT(&huart10, &tx_byte, 1);
}

static void uart10_putchar(uint8_t c)
{
    taskENTER_CRITICAL();

    uint16_t next = (tx_head + 1U) % TX_BUF_SIZE;
    if (next != tx_tail) {
        tx_buf[tx_head] = c;
        tx_head = next;
        uart10_start_tx_from_isr_or_task();
    }

    taskEXIT_CRITICAL();
}

extern "C" int __io_putchar(int ch)
{
    uart10_putchar((uint8_t)ch);
    return ch;
}

extern UART_HandleTypeDef huart7;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10) {
        if (tx_head != tx_tail) {
            tx_byte = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1U) % TX_BUF_SIZE;
            HAL_UART_Transmit_IT(&huart10, &tx_byte, 1);
        } else {
            tx_busy = 0;
        }
    } else if (huart == &huart7) {
        extern uint8_t uart7_tx_buf[];
        extern volatile uint16_t uart7_tx_head;
        extern volatile uint16_t uart7_tx_tail;
        extern volatile uint8_t uart7_tx_busy;

        if (uart7_tx_head != uart7_tx_tail) {
            uint8_t c = uart7_tx_buf[uart7_tx_tail];
            uart7_tx_tail = (uart7_tx_tail + 1) % UART7_TX_BUF_SIZE;
            HAL_UART_Transmit_IT(&huart7, &c, 1);
        } else {
            uart7_tx_busy = 0;
        }
    }
}

void StartScreenTask(void *argument)
{
    ScreenCmd_t cmd;
    screenQueue = xQueueCreate(5, sizeof(ScreenCmd_t));

    HAL_UART_Receive_IT(&huart10, screenRxBuffer, 1);

    for(;;)
    {
        if (xQueueReceive(screenQueue, &cmd, portMAX_DELAY) == pdPASS)
        {
            if (strncmp(cmd.cmd, "MODE=", 5) == 0) {
                char num[2] = {cmd.cmd[5], 0};
                current_mode = atoi(num);
                // TTS_Speak("你好\r\n");
                //printsf_clear(0);
                // printsf(0, "MODE=%d", current_mode);
                // printsf(0, "");
                switch(current_mode)
                {
                case 0:
                    send_data = 0x00;  // 模式0 发送 0x00
                    break;
                case 1:
                    send_data = 0x01;  // 模式1 发送 0x01
                    break;
                case 2:
                    send_data = 0x02;  // 模式2 发送 0x02（圆周运动）
                    break;
                case 3:
                    send_data = 0x03;  // 模式3 发送 0x03
                    break;
                case 4:
                    send_data = 0x04;
                    break;
                default:
                    send_data = 0x00;  // 错误默认值
                    break;
                }

                // 串口发送
                HAL_UART_Transmit(&huart1, &send_data, 1, 100);
            }
        }
    }
}

void prints(uint8_t index, const char *content)
{
    if (content == NULL)
    {
        content = "";
    }

    printf("t%u.txt=\"%s\"", (unsigned int)index, content);
    printf("%c%c%c", (char)0xFF, (char)0xFF, (char)0xFF);
}



static void printsf_store_line(const char *line)
{
    uint8_t pos;
    if (s_log_count < PRINTSF_MAX_LINES)
    {
        pos = (uint8_t)((s_log_head + s_log_count) % PRINTSF_MAX_LINES);
        s_log_count++;
    }
    else
    {
        pos = s_log_head;
        s_log_head = (uint8_t)((s_log_head + 1U) % PRINTSF_MAX_LINES);
    }

    strncpy(s_log_lines[pos], line, PRINTSF_MAX_LINE_LEN - 1U);
    s_log_lines[pos][PRINTSF_MAX_LINE_LEN - 1U] = '\0';
}

void printsf(uint8_t index, const char *fmt, ...)
{
    char line[PRINTSF_MAX_LINE_LEN];
    char merged[PRINTSF_TEXT_BUF_LEN];
    va_list args;
    size_t off = 0U;
    uint8_t i;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    for (i = 0U; line[i] != '\0'; i++)
    {
        if (line[i] == '"') line[i] = '\'';
    }

    printsf_store_line(line);

    merged[0] = '\0';
    for (i = 0U; i < s_log_count; i++)
    {
        uint8_t pos = (uint8_t)((s_log_head + i) % PRINTSF_MAX_LINES);
        int n = snprintf(merged + off, sizeof(merged) - off, "%s%s",
                         s_log_lines[pos], (i + 1U < s_log_count) ? "\r\n" : "");
        if (n < 0) break;
        if ((size_t)n >= (sizeof(merged) - off))
        {
            off = sizeof(merged) - 1U;
            merged[off] = '\0';
            break;
        }
        off += (size_t)n;
    }

    prints(index, merged);
}


void printsf_clear(uint8_t index)
{
    uint8_t i;

    s_log_head = 0U;
    s_log_count = 0U;

    for (i = 0U; i < PRINTSF_MAX_LINES; i++)
    {
        s_log_lines[i][0] = '\0';
    }

    prints(index, "");
}
