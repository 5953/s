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

/* 每条输出后强制刷新 USB CDC，防止长列表被截断 */
#define P(fmt, ...) do { printf(fmt, ##__VA_ARGS__); stdio_flush(); } while (0)

/* ---------- 函数声明 ---------- */
static bool check_stop(void);
static bool wait_ms_check(uint32_t ms);
static void pin_low(uint pin);
static void pin_high(uint pin);
static void pin_release(uint pin);

/* ---------- 基础引脚操作 ---------- */
static void pin_low(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

/* 推挽输出高电平（用于测试，确保万用表能测到 3.3V） */
static void pin_high(uint pin) {
    gpio_put(pin, 1);
    gpio_set_dir(pin, GPIO_OUT);
}

/* 释放为高阻（依靠外部上拉） */
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

/* ==========================================
 *            NRST (GP4) 专项测试命令
 * ========================================== */

/* 1. 静态控制：高、低、释放 */
static void run_nrst_static(const char *state) {
    if (strcmp(state, "high") == 0) {
        pin_high(NRST_PIN);
        P("NRST (GP4) forced HIGH (Push-Pull). Measure ~3.3V.\r\n");
    } else if (strcmp(state, "low") == 0) {
        pin_low(NRST_PIN);
        P("NRST (GP4) forced LOW. Measure ~0V.\r\n");
    } else if (strcmp(state, "release") == 0) {
        pin_release(NRST_PIN);
        P("NRST (GP4) released to High-Z.\r\n");
        P("(If no external pull-up, multimeter may read 0V or float.)\r\n");
    } else {
        P("Usage: nrst high | low | release\r\n");
    }
}

/* 2. 慢速闪烁：500ms 低，500ms 推挽高 */
static void run_nrst_blink(unsigned cycles) {
    if (cycles == 0 || cycles > 50) {
        P("Usage: nrst-blink N (N=1..50)\r\n");
        return;
    }
    P("\r\nBlinking NRST %u times (500ms each state)...\r\n", cycles);
    
    for (unsigned i = 0; i < cycles && !stop_requested; i++) {
        pin_low(NRST_PIN);
        P("[NRST] %u/%u LOW\r\n", i + 1, cycles);
        sleep_ms(500);

        pin_high(NRST_PIN); // 强制推挽高
        P("[NRST] %u/%u HIGH\r\n", i + 1, cycles);
        sleep_ms(500);
    }
    pin_release(NRST_PIN);
    P("NRST blink finished.\r\n");
}

/* 3. 低频脉冲串：每次 100ms 低，900ms 高（万用表指针会明显跳动） */
static void run_nrst_pulse(unsigned cycles) {
    if (cycles == 0 || cycles > 50) {
        P("Usage: nrst-pulse N (N=1..50)\r\n");
        return;
    }
    P("\r\nPulsing NRST %u times (100ms LOW, 900ms HIGH)...\r\n", cycles);
    
    for (unsigned i = 0; i < cycles && !stop_requested; i++) {
        pin_low(NRST_PIN);
        sleep_ms(100);
        pin_high(NRST_PIN);
        sleep_ms(900);
    }
    pin_release(NRST_PIN);
    P("NRST pulse finished.\r\n");
}

/* 4. 方波：50Hz 方波，方便逻辑分析仪抓取 */
static void run_nrst_square(void) {
    P("\r\nGenerating 50Hz square wave on NRST for 10 seconds...\r\n");
    P("Logic analyzer: Trigger on CH1 (GP4) Falling Edge.\r\n");
    
    for (int i = 0; i < 500 && !stop_requested; i++) {
        pin_low(NRST_PIN);
        sleep_ms(10);
        pin_high(NRST_PIN);
        sleep_ms(10);
        if (check_stop()) break;
    }
    pin_release(NRST_PIN);
    P("NRST square wave finished.\r\n");
}


/* ==========================================
 *            SWIM (GP2) 测试命令
 * ========================================== */

/* 发送一次标准的进入序列 */
static void send_swim_waveform(void) {
    pin_release(SWIM_PIN);
    pin_low(NRST_PIN);
    busy_wait_us_32(10);

    pin_low(SWIM_PIN);
    busy_wait_us_32(16);
    pin_release(SWIM_PIN);
    busy_wait_us_32(5);

    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);
        busy_wait_us_32(500);
        pin_release(SWIM_PIN);
        busy_wait_us_32(500);
    }

    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);
        busy_wait_us_32(250);
        pin_release(SWIM_PIN);
        busy_wait_us_32(250);
    }

    pin_release(NRST_PIN);
    pin_release(SWIM_PIN);
}

/* 专为逻辑分析仪设计的循环发波形命令 */
static void run_swim_loop(unsigned cycles) {
    if (cycles == 0 || cycles > 50) {
        P("Usage: loop N (N=1..50)\r\n");
        return;
    }

    P("\r\nSending SWIM waveform %u times (500ms apart)...\r\n", cycles);
    P("LA Trigger: CH1(NRST) Falling Edge.\r\n");
    P("Send 'stop' to abort.\r\n");

    stop_requested = false;
    for (unsigned i = 0; i < cycles && !stop_requested; i++) {
        send_swim_waveform();
        if (i < cycles - 1) {
            if (!wait_ms_check(500)) break;
        }
    }
    pin_release(SWIM_PIN);
    pin_release(NRST_PIN);
    P("Loop finished.\r\n");
}

/* ---------- 底层等待与中断检测 ---------- */

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

/* 粗粒度等待，期间可以检测 stop */
static bool wait_ms_check(uint32_t ms) {
    while (ms > 0 && !check_stop()) {
        uint32_t step = (ms > 10) ? 10 : ms;
        sleep_ms(step);
        ms -= step;
    }
    return !stop_requested;
}

/* ==========================================
 *            命令行处理
 * ========================================== */

static void process_command(char *cmd) {
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    size_t len = strlen(cmd);
    while (len && isspace((unsigned char)cmd[len-1])) cmd[--len] = '\0';

    if (len == 0) return;

    if (strcmp(cmd, "help") == 0) {
        P("\r\n=== Commands ===\r\n");
        P("--- NRST (GP4) Tests ---\r\n");
        P("nrst high         - Force GP4 HIGH (~3.3V)\r\n");
        P("nrst low          - Force GP4 LOW (~0V)\r\n");
        P("nrst release      - GP4 High-Z\r\n");
        P("nrst-blink N      - Blink GP4 N times (500ms LOW/HIGH)\r\n");
        P("nrst-pulse N      - Pulse GP4 N times (100ms LOW, 900ms HIGH)\r\n");
        P("nrst-square       - 50Hz square wave on GP4 for 10s\r\n");
        P("\r\n--- SWIM (GP2) Tests ---\r\n");
        P("loop N            - Send SWIM waveform N times (500ms apart)\r\n");
        P("stop              - Stop current test\r\n");
    } 
    else if (strncmp(cmd, "nrst ", 5) == 0) {
        run_nrst_static(cmd + 5);
    }
    else if (strncmp(cmd, "nrst-blink ", 11) == 0) {
        unsigned n = 0;
        if (sscanf(cmd + 11, "%u", &n) == 1) run_nrst_blink(n);
        else P("Usage: nrst-blink N\r\n");
    }
    else if (strncmp(cmd, "nrst-pulse ", 11) == 0) {
        unsigned n = 0;
        if (sscanf(cmd + 11, "%u", &n) == 1) run_nrst_pulse(n);
        else P("Usage: nrst-pulse N\r\n");
    }
    else if (strcmp(cmd, "nrst-square") == 0) {
        run_nrst_square();
    }
    else if (strncmp(cmd, "loop ", 5) == 0) {
        unsigned n = 0;
        if (sscanf(cmd + 5, "%u", &n) == 1) run_swim_loop(n);
        else P("Usage: loop N\r\n");
    }
    else if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "q") == 0) {
        stop_requested = true;
        pin_release(SWIM_PIN);
        pin_release(NRST_PIN);
        P("Stopped, pins released.\r\n");
    }
    else {
        P("Unknown command: %s\r\n", cmd);
    }
}

/* 等待 USB 主机连接 */
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

    P("\r\n=== RP2040 SWIM & NRST Test Tool ===\r\n");
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
