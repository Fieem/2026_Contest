#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usart.h"
#include <cstring>
#include "tts_task.h"


#define TTS_TEXT_MAX_LEN 64

typedef struct {
    char text[TTS_TEXT_MAX_LEN];
} TTS_Message_t;

static QueueHandle_t ttsQueue = nullptr;

#define UART7_TX_BUF_SIZE 256
uint8_t uart7_tx_buf[UART7_TX_BUF_SIZE];
volatile uint16_t uart7_tx_head = 0;
volatile uint16_t uart7_tx_tail = 0;
volatile uint8_t uart7_tx_busy = 0;

extern UART_HandleTypeDef huart7;

static void uart7_putchar(uint8_t c)
{
    uint16_t next = (uart7_tx_head + 1) % UART7_TX_BUF_SIZE;
    if (next == uart7_tx_tail) {
        return;
    }

    uart7_tx_buf[uart7_tx_head] = c;
    uart7_tx_head = next;

    if (uart7_tx_busy == 0) {
        uart7_tx_busy = 1;
        uint8_t data = uart7_tx_buf[uart7_tx_tail];
        uart7_tx_tail = (uart7_tx_tail + 1) % UART7_TX_BUF_SIZE;
        HAL_UART_Transmit_IT(&huart7, &data, 1);
    }
}

void TTS_Transmit(const char *text, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uart7_putchar(text[i]);
    }
}

void TTS_Speak(const char *text)
{
    if (text == nullptr || ttsQueue == nullptr) return;

    TTS_Message_t msg{};
    strncpy(msg.text, text, TTS_TEXT_MAX_LEN - 1);
    msg.text[TTS_TEXT_MAX_LEN - 1] = '\0';
    xQueueSend(ttsQueue, &msg, 0);
}

extern "C" void StartTTSTask(void *argument)
{
    TTS_Message_t msg{};

    ttsQueue = xQueueCreate(5, sizeof(TTS_Message_t));

    for (;;) {
        if (xQueueReceive(ttsQueue, &msg, portMAX_DELAY) == pdPASS) {
            TTS_Transmit(msg.text, strlen(msg.text));
        }
    }
}
