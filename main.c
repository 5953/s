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

/* 同步脉冲宽度有效窗口
 * 内部上拉 ~50kΩ，上升沿约 1~3µs，测得宽度会略大于真实值，窗口放宽到 40µs */
#define SYNC_WIDTH_MIN_US 5
#define SYNC_WIDTH_MAX_US 40

static volatile bool stop_requested = false;

#define P(fmt, ...) do { printf(fmt, ##__VA_ARGS__); stdio_flush(); } while (0)

/* ---------- 基础引脚操作 ---------- */

/* 拉低：推挽输出 0 */
static void pin_low(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

/* 释放：输入 + 启用内部上拉
 * 外部无上拉时，靠 RP2040 内部 ~50kΩ 维持高电平 */
static void pin_release(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static void init_pin(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);   /* 初始化即上拉，避免浮空 */
}

/* ---------- 发送 SWIM 进入序列 ---------- */
static void send_swim_waveform(void) {
    pin_release(SWIM_PIN);
    pin_low(NRST_PIN);
    busy_wait_us_32(10);

    /* 首次低脉冲：规范 1~16µs，取 10µs 留余量 */
    pin_low(SWIM_PIN);
    busy_wait_us_32(10);
    pin_release(SWIM_PIN);
    busy_wait_us_32(10);   /* 高电平给内部上拉的上升沿留时间 */

    /* 4 个 500µs 脉冲 */
    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);     busy_wait_us_32(500);
        pin_release(SWIM_PIN); busy_wait_us_32(500);
    }
    /* 再 4 个 500µs 脉冲 */
    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN);     busy_wait_us_32(500);
        pin_release(SWIM_PIN); busy_wait_us_32(500);
    }

    pin_release(NRST_PIN);
    pin_release(SWIM_PIN);
}

/* 等待 STM8 在 SWIM 上返回同步帧（约 16µs 低脉冲） */
static bool wait_for_sync(uint32_t timeout_us, int64_t *low_width_us) {
    pin_release(SWIM_PIN);

    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    while (!time_reached(deadline)) {
        if (gpio_get(SWIM_PIN) == 0) {
            absolute_time_t pull_low_time = get_absolute_time();

            absolute_time_t low_deadline = make_timeout_time_us(100);
            while (!time_reached(low_deadline)) {
                if (gpio_get(SWIM_PIN) == 1) {
                    int64_t w = absolute_time_diff_us(pull_low_time, get_absolute_time());
                    *low_width_us = w;
                    /* 只有宽度在有效窗口内才算真正同步，过滤毛刺 */
                    return (w >= SYNC_WIDTH_MIN_US && w <= SYNC_WIDTH_MAX_US);
                }
            }
            /* 一直低电平没恢复：stuck */
            *low_width_us = 100;
            return false;
        }
    }
    return false;
}

/* 单次完整测试 */
static bool do_single_sync_test(int index, int total, bool print_result) {
    send_swim_waveform();

    int64_t width = 0;
    bool got_sync = wait_for_sync(5000, &width);

    if (print_result) {
        if (got_sync) {
            P("[OK] #%d/%d Sync received! LOW width = %lld us\r\n",
              index, total, (long long)width);
        } else {
            P("[FAIL] #%d/%d No sync. last LOW width = %lld us\r\n",
              index, total, (long long)width);
        }
    }
    return got_sync;
}

/* 循环测试命令 */
static void run_sync_auto(unsigned cycles) {
    if (cycles == 0 || cycles > 50) {
        P("Usage: sync-auto N (N=1..50)\r\n");
        return;
    }

    P("\r\nStarting auto sync test (%u cycles)...\r\n", cycles);
    P("Mode: NO external pull-up, using RP2040 internal pull-up (~50k).\r\n");
    P("Waiting up to 5ms for target response after each entry.\r\n");

    stop_requested = false;
    unsigned success = 0;

    for (unsigned i = 1; i <= cycles && !stop_requested; i++) {
        if (do_single_sync_test(i, cycles, true)) success++;
        if (i < cycles && !stop_requested) {
            sleep_ms(500);
        }
    }

    P("\r\n=== RESULT ===\r\n");
    P("Attempts: %u, Success: %u, Fail: %u\r\n", cycles, success, cycles - success);

    if (success > 0) {
        P("SUCCESS! Target responded. Entry sequence is CORRECT.\r\n");
    } else {
        P("FAILED: No sync detected.\r\n");
        P("Checklist:\r\n");
        P("1. STM8 powered at 3.3V? Common GND connected?\r\n");
        P("2. SWIM=GP2, NRST=GP4 wired to correct pins?\r\n");
        P("3. Internal pull-up enabled (this build). If still fails,\r\n");
        P("   add external 4.7k~10k pull-up on SWIM.\r\n");
        P("4. NRST driven LOW by RP2040 during entry? Check with meter.\r\n");
        P("5. Keep wires short; 470nF on VCAP; try lower SWIM speed.\r\n");
    }
}

static void process_command(char *cmd) {
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    size_t len = strlen(cmd);
    while (len && isspace((unsigned char)cmd[len-1])) cmd[--len] = '\0';

    if (len == 0) return;

    if (strcmp(cmd, "help") == 0) {
        P("\r\n=== Commands ===\r\n");
        P("sync-auto N   - Send entry N times and check for target sync\r\n");
        P("sync-test     - Test just one single sync\r\n");
        P("stop / q      - Stop current test\r\n");
    }
    else if (strcmp(cmd, "sync-test") == 0) {
        P("\r\nSingle sync test...\r\n");
        do_single_sync_test(1, 1, true);
    }
    else if (strncmp(cmd, "sync-auto ", 10) == 0) {
        unsigned n = 0;
        if (sscanf(cmd + 10, "%u", &n) == 1) run_sync_auto(n);
        else P("Usage: sync-auto N\r\n");
    }
    else if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "q") == 0) {
        stop_requested = true;
        pin_release(SWIM_PIN);
        pin_release(NRST_PIN);
        P("Stopped.\r\n");
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

    P("\r\n=== RP2040 SWIM Auto-Sync Tester (internal pull-up) ===\r\n");
    P("Target: STM8. SWIM=GP2, NRST=GP4. No external pull-up.\r\n");
    P("Type 'sync-auto 10' to start.\r\n> ");

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
