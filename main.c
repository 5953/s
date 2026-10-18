#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define SWIM_PIN 2u
#define NRST_PIN 4u
#define LINE_MAX 96

static volatile bool stop_requested = false;

#define P(fmt, ...) do { printf(fmt, ##__VA_ARGS__); stdio_flush(); } while (0)

/* 引脚基础操作：开漏模式 */
static void pin_low(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}
static void pin_release(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}
static void init_pin(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

/* 读取串口，支持随时输入 stop 终止 */
static bool check_stop(void) {
    static char cmd[16];
    static unsigned used = 0;
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r' || ch == '\n') {
            cmd[used] = '\0';
            if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "q") == 0) {
                stop_requested = true;
                used = 0;
                return true;
            }
            used = 0;
        } else if ((ch == 8 || ch == 127) && used > 0) {
            used--;
        } else if (ch >= 32 && ch <= 126 && used + 1 < sizeof(cmd)) {
            cmd[used++] = (char)ch;
        }
    }
    return stop_requested;
}

/* 
 * 纯正的 SWIM 进入序列波形（无 printf 打断） 
 * 时序：
 * 1. NRST 拉低
 * 2. 延时 10us
 * 3. SWIM 拉低 16us
 * 4. SWIM 释放 5us
 * 5. 4个 1kHz (500us低，500us高)
 * 6. 4个 2kHz (250us低，250us高)
 * 7. 释放所有引脚
 */
static void send_swim_waveform(void) {
    pin_release(SWIM_PIN);
    pin_low(NRST_PIN);
    busy_wait_us_32(10);

    pin_low(SWIM_PIN);
    busy_wait_us_32(16);
    pin_release(SWIM_PIN);
    busy_wait_us_32(5);

    // 4 x 1kHz
    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);
        busy_wait_us_32(500);
        pin_release(SWIM_PIN);
        busy_wait_us_32(500);
    }

    // 4 x 2kHz
    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);
        busy_wait_us_32(250);
        pin_release(SWIM_PIN);
        busy_wait_us_32(250);
    }

    pin_release(NRST_PIN);
    pin_release(SWIM_PIN);
}

/* 
 * 专为逻辑分析仪设计的循环命令
 * 连发 N 次，间隔 500ms（留时间给逻辑分析仪刷屏和你看图）
 */
static void run_loop(unsigned cycles) {
    if (cycles == 0 || cycles > 50) {
        P("usage: loop N   (N=1..50)\r\n");
        return;
    }

    P("\r\nStarting logic analyzer loop: %u waveforms\r\n", cycles);
    P("Trigger mode: CH1(NRST) Falling Edge. Timebase: 2ms/div.\r\n");
    P("Send 'stop' to abort.\r\n");

    stop_requested = false;

    for (unsigned i = 0; i < cycles && !stop_requested; i++) {
        send_swim_waveform();
        
        // 发送完毕后，引脚都是高阻态。
        // 这 500ms 内，逻辑分析仪会显示一条平稳的高电平（因为外部有上拉）
        if (i < cycles - 1) {
            absolute_time_t start = get_absolute_time();
            while (absolute_time_diff_us(start, get_absolute_time()) < 500000) {
                if (check_stop()) break;
                sleep_us(1000); // 分成小段延时，方便响应 stop
            }
        }
    }

    pin_release(SWIM_PIN);
    pin_release(NRST_PIN);
    P("\r\nLoop finished.\r\n");
}

/* 简单闪烁测试，用于确认逻辑分析仪接线无误 */
static void run_blink(void) {
    P("\r\nBlinking GP2 (SWIM). Measure with multimeter or LA.\r\n");
    for (int i = 0; i < 20; i++) {
        pin_low(SWIM_PIN); sleep_ms(500);
        pin_release(SWIM_PIN); sleep_ms(500);
        if (check_stop()) break;
    }
    pin_release(SWIM_PIN);
}

static void process_command(char *cmd) {
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    size_t len = strlen(cmd);
    while (len && isspace((unsigned char)cmd[len-1])) cmd[--len] = '\0';

    if (len == 0) return;

    if (strcmp(cmd, "help") == 0) {
        P("\r\n=== Commands ===\r\n");
        P("loop N    - Send SWIM entry waveform N times (delay 500ms)\r\n");
        P("blink     - Blink GP2 slowly to test LA wiring\r\n");
        P("stop      - Stop the current loop\r\n");
    } 
    else if (strncmp(cmd, "loop ", 5) == 0) {
        unsigned n = 0;
        if (sscanf(cmd + 5, "%u", &n) == 1) {
            run_loop(n);
        } else {
            P("Usage: loop N\r\n");
        }
    }
    else if (strcmp(cmd, "blink") == 0) {
        run_blink();
    }
    else if (strcmp(cmd, "stop") == 0) {
        stop_requested = true;
        pin_release(SWIM_PIN);
        pin_release(NRST_PIN);
        P("Stopped, pins released.\r\n");
    }
    else {
        P("Unknown command: %s\r\n", cmd);
    }
}

static void wait_for_usb(void) {
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && !time_reached(deadline)) {
        tight_loop_contents();
    }
    sleep_ms(100);
    stdio_flush();
}

int main(void) {
    stdio_init_all();
    wait_for_usb();

    init_pin(SWIM_PIN);
    init_pin(NRST_PIN);
    pin_release(SWIM_PIN);
    pin_release(NRST_PIN);

    P("\r\n=== RP2040 SWIM Logic Analyzer Tool ===\r\n");
    P("Type 'help' for commands.\r\n> ");

    char line[LINE_MAX];
    unsigned used = 0;

    while (true) {
        int ch = getchar_timeout_us(10000);
        if (ch == PICO_ERROR_TIMEOUT) continue;

        if (ch == '\r' || ch == '\n') {
            if (used > 0) {
                line[used] = '\0';
                process_command(line);
                used = 0;
                P("\r\n> ");
            }
        } else if ((ch == 8 || ch == 127) && used > 0) {
            used--;
        } else if (ch >= 32 && ch <= 126 && used + 1 < sizeof(line)) {
            line[used++] = (char)ch;
        }
    }
    return 0;
}
