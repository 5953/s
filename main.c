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

static bool check_stop(void);
static void pin_low(uint pin);
static void pin_high(uint pin);
static void pin_release(uint pin);

/* ---------- 基础引脚操作 ---------- */
static void pin_low(uint pin) { gpio_put(pin, 0); gpio_set_dir(pin, GPIO_OUT); }
static void pin_high(uint pin) { gpio_put(pin, 1); gpio_set_dir(pin, GPIO_OUT); }
static void pin_release(uint pin) { gpio_set_dir(pin, GPIO_IN); gpio_disable_pulls(pin); }

static void init_pin(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

/* ---------- 核心：发送序列并等待同步帧 ---------- */
static void send_swim_waveform(void) {
    pin_release(SWIM_PIN);
    pin_low(NRST_PIN);
    busy_wait_us_32(10);

    pin_low(SWIM_PIN);
    busy_wait_us_32(16);
    pin_release(SWIM_PIN);
    busy_wait_us_32(5);

    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN); busy_wait_us_32(500);
        pin_release(SWIM_PIN); busy_wait_us_32(500);
    }
    for (unsigned i = 0; i < 4; i++) {
        pin_low(SWIM_PIN); busy_wait_us_32(250);
        pin_release(SWIM_PIN); busy_wait_us_32(250);
    }

    pin_release(NRST_PIN);
    pin_release(SWIM_PIN);
}

/* 返回 true 表示收到同步，false 表示超时 */
static bool wait_for_sync(uint32_t timeout_us, int64_t *low_width_us) {
    // 发完之后，引脚必须是高阻态，准备接收 STM8 的拉低
    pin_release(SWIM_PIN);
    
    absolute_time_t start = get_absolute_time();
    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    // 1. 等待 SWIM 变低 (STM8 主动拉低)
    while (!time_reached(deadline)) {
        if (gpio_get(SWIM_PIN) == 0) {
            absolute_time_t pull_low_time = get_absolute_time();
            int64_t wait_time = absolute_time_diff_us(start, pull_low_time);
            if (wait_time > 1000) {
                 // 如果超过 1ms 才拉低，可能是噪声或后续问题，但先记录下来
            }
            
            // 2. 测量拉低的持续时间 (期望约 16us)
            absolute_time_t low_deadline = make_timeout_time_us(100); // 最多等 100us 变高
            while (!time_reached(low_deadline)) {
                if (gpio_get(SWIM_PIN) == 1) {
                    *low_width_us = absolute_time_diff_us(pull_low_time, get_absolute_time());
                    return true;
                }
            }
            // 一直低电平没恢复
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
    // 等待 5ms (5000us)
    bool got_sync = wait_for_sync(5000, &width);
    
    if (print_result) {
        if (got_sync) {
            P("[OK] #%d/%d Sync received! LOW width = %lld us\r\n", 
              index, total, (long long)width);
        } else {
            P("[FAIL] #%d/%d No sync (timeout or stuck).\r\n", index, total);
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
    P("Waiting up to 5ms for target response after each entry.\r\n");

    stop_requested = false;
    unsigned success = 0;

    for (unsigned i = 1; i <= cycles && !stop_requested; i++) {
        if (do_single_sync_test(i, cycles, true)) success++;
        
        if (i < cycles && !stop_requested) {
            sleep_ms(500); // 间隔 500ms，给芯片喘口气
        }
    }

    P("\r\n=== RESULT ===\r\n");
    P("Attempts: %u, Success: %u, Fail: %u\r\n", cycles, success, cycles - success);
    
    if (success > 0) {
        P("SUCCESS! Target responded. Entry sequence is CORRECT.\r\n");
        P("Electrical and timing requirements are met.\r\n");
    } else {
        P("FAILED: No sync detected.\r\n");
        P("Checklist:\r\n");
        P("1. Is the STM8 powered (3.3V)? Common GND connected?\r\n");
        P("2. SWIM / NRST wires connected to correct pins?\r\n");
        P("3. Try replacing 750R pull-up with 4.7k or 10k.\r\n");
        P("4. Is NRST driven LOW by RP2040 during the entry? (Check with meter)\r\n");
    }
}

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

static void process_command(char *cmd) {
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    size_t len = strlen(cmd);
    while (len && isspace((unsigned char)cmd[len-1])) cmd[--len] = '\0';

    if (len == 0) return;

    if (strcmp(cmd, "help") == 0) {
        P("\r\n=== Commands ===\r\n");
        P("sync-auto N   - Send entry N times and check for target sync\r\n");
        P("sync-test     - Test just one single sync\r\n");
        P("stop          - Stop current test\r\n");
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

    P("\r\n=== RP2040 SWIM Auto-Sync Tester ===\r\n");
    P("Target: STM8. Connect GP2=SWIM, GP4=NRST.\r\n");
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
