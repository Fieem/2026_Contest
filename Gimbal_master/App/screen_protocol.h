#ifndef SCREEN_PROTOCOL_H
#define SCREEN_PROTOCOL_H

#include <cstdint>

#define SCREEN_CMD_BUF_SIZE 64
#define SCREEN_CMD_END_CHAR '!'

typedef struct {
    char cmd[SCREEN_CMD_BUF_SIZE];
    uint8_t len;
} ScreenCmd_t;

#define TEST_VOFA_FRAME_MAX_LEN   (64U)
#define PRINTSF_MAX_LINES      (8U)    // 最多缓存8行
#define PRINTSF_MAX_LINE_LEN   (256U)   // 每行最大255字符
#define PRINTSF_TEXT_BUF_LEN   (PRINTSF_MAX_LINES * (PRINTSF_MAX_LINE_LEN + 2U) + 1U)

static char s_vofa_frame_buf[TEST_VOFA_FRAME_MAX_LEN];
static uint8_t s_vofa_frame_len = 0U;

static char s_log_lines[PRINTSF_MAX_LINES][PRINTSF_MAX_LINE_LEN];
static uint8_t s_log_head = 0U;   // 最旧行下标
static uint8_t s_log_count = 0U;  // 当前行数

// Nextion 文本控件打印:
// 例: prints(0, "helloworld") -> t0.txt="helloworld" + 0xFF 0xFF 0xFF
void prints(uint8_t index, const char *content);

// 带格式化参数的 Nextion 文本控件打印:
// 例: printsf(0, "speed=%.2f", speed);
void printsf(uint8_t index, const char *fmt, ...);

void printsf_clear(uint8_t index);

#endif