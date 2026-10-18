#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define TEST_PIN 2u
#define NRST_PIN 4u
#define LINE_MAX 96

/*
 *   硬件前提：SWIM(GP2) 上接 750Ω 外部上拉到 3V3。
 * - GP2 输出低时静态灌电流约 3.3V/750Ω ≈ 4.4mA，RP2040 单脚 12mA 内。
 * - 释放时靠 750Ω 上拉回到 3V3，上升时间 τ≈750Ω×C_line，
 *   C_line≈50pF 时到 90% 约 86ns，接近 ST UM0470 的 <100ns 要求。
 * - 接真实 STM8 时，SWIM 只允许“输出低 / 高阻”，不要 push-pull 输出高。
 */

static volatile bool stop_requested = false;

/* 每条输出后 flush，防止 USB CDC 长列表被截断 */
#define P(fmt, ...) do { printf(fmt, ##__VA_ARGS__); stdio_flush(); } while (0)

/* ================= 引脚操作 ================= */

static void pin_low(uint pin)
{
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

/* push-pull 高：仅用于板级自测；接 STM8 时不要对 SWIM 用 */
static void pin_high(uint pin)
{
    gpio_put(pin, 1);
    gpio_set_dir(pin, GPIO_OUT);
}

/* 高阻：750Ω 外部上拉负责拉高 */
static void pin_release(uint pin)
{
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

static void release_all(void)
{
    pin_release(TEST_PIN);
    pin_release(NRST_PIN);
}

static void init_pin(uint pin)
{
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

/* ================= 状态 / 帮助 ================= */

static void show_status(void)
{
    P("\r\n");
    P("System clock : %lu Hz\r\n", (unsigned long)clock_get_hz(clk_sys));
    P("GP2 dir=%s level=%u  (SWIM, ext 750R pull-up)\r\n",
      gpio_get_dir(TEST_PIN) == GPIO_OUT ? "OUTPUT" : "INPUT/HIGH-Z",
      gpio_get(TEST_PIN) ? 1u : 0u);
    P("GP4 dir=%s level=%u  (NRST / spare)\r\n",
      gpio_get_dir(NRST_PIN) == GPIO_OUT ? "OUTPUT" : "INPUT/HIGH-Z",
      gpio_get(NRST_PIN) ? 1u : 0u);
    P("Flash read/erase/program: disabled (electrical test only)\r\n");
}

static void show_help(void)
{
    P("\r\n");
    P("===== RP2040-Zero GPIO / SWIM Diagnostic =====\r\n");
    P("---- basic ----\r\n");
    P("help                   Show this help\r\n");
    P("status                 Show pin status and sysclk\r\n");
    P("low                    Hold GP2 driven LOW (0V)\r\n");
    P("high                   Drive GP2 to 3.3V (push-pull)\r\n");
    P("release                Release GP2 and GP4 to high-Z\r\n");
    P("static low|high|rel    Aliases for low/high/release\r\n");
    P("---- slow patterns ----\r\n");
    P("slow                   LOW 1000ms / released 1000ms x10\r\n");
    P("blink                  LOW 500ms  / released 500ms  x10\r\n");
    P("pulse 100              LOW 100ms  every 1s, x10\r\n");
    P("pulse 10               LOW 10ms   every 1s, x10\r\n");
    P("pulse 1                LOW 1ms    every 1s, x10\r\n");
    P("---- square wave ----\r\n");
    P("sqr HZ                 Square wave HZ (1..1000), 30s\r\n");
    P("square 10/100/1000     10Hz/100Hz/1kHz, 10s (1kHz 5s)\r\n");
    P("---- multimeter aids ----\r\n");
    P("short-test             GP2 driven LOW, read back (short to 3V3?)\r\n");
    P("open-test              GP2 released, read back (pull-up OK?)\r\n");
    P("probe                  Monitor GP2 changes for 10s\r\n");
    P("---- logic analyzer aids ----\r\n");
    P("pulse-us N             Single LOW pulse, N us\r\n");
    P("pulse-train W C G      C x W us LOW pulses, G ms apart\r\n");
    P("width-sweep            LOW widths 1us..1ms, 10 each\r\n");
    P("toggle                 Fast C-loop toggle on GP2, 1s\r\n");
    P("pwm HZ DUTY            Hardware PWM on GP2 (HZ 8..1M, DUTY 0..100)\r\n");
    P("pattern HEX N B        Push-pull bit pattern, N bits, B us/bit\r\n");
    P("---- SWIM entry ----\r\n");
    P("entry-waveform         One SWIM-style entry sequence\r\n");
    P("entry-loop N           N entry sequences, 200ms apart\r\n");
    P("marker                 GP4 marker pulse, then entry sequence\r\n");
    P("swim-sync-wait         Entry, then wait 200ms for target sync\r\n");
    P("---- misc ----\r\n");
    P("nrst low|high|release  Control GP4 only\r\n");
    P("loopback               GP2<->GP4 jumper loopback test\r\n");
    P("stop / q               Stop a running pattern\r\n");
    P("\r\n");
    P("Hardware: 750R external pull-up from 3V3 to GP2 assumed.\r\n");
    P("Electrical: GP2/GP4 are 3.3V GPIOs. NEVER apply 5V.\r\n");
    P("Note: this firmware does NOT read/erase/program the STM8.\r\n");
}

/* ================= 运行中读 stop ================= */

static char cmd_buf[24];
static unsigned cmd_used = 0;

static void reset_check_stop(void)
{
    cmd_used = 0;
    cmd_buf[0] = '\0';
}

static bool check_stop(void)
{
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r' || ch == '\n') {
            cmd_buf[cmd_used] = '\0';
            if (strcmp(cmd_buf, "stop") == 0 || strcmp(cmd_buf, "q") == 0) {
                stop_requested = true;
                reset_check_stop();
                return true;
            }
            reset_check_stop();
        } else if ((ch == 8 || ch == 127) && cmd_used > 0) {
            cmd_used--;
        } else if (ch >= 32 && ch <= 126 && cmd_used + 1 < sizeof(cmd_buf)) {
            cmd_buf[cmd_used++] = (char)ch;
        }
    }
    return stop_requested;
}

static bool wait_ms_check(uint32_t milliseconds)
{
    while (milliseconds > 0) {
        if (check_stop()) return false;
        uint32_t part = milliseconds > 5 ? 5 : milliseconds;
        sleep_ms(part);
        milliseconds -= part;
    }
    return !check_stop();
}

/* ================= 慢速模式 ================= */

static void run_slow(unsigned cycles, uint32_t low_ms, uint32_t release_ms)
{
    stop_requested = false;
    reset_check_stop();

    P("\r\nSlow test: %u cycles, LOW=%lums, REL=%lums\r\n",
      cycles, (unsigned long)low_ms, (unsigned long)release_ms);
    P("Type 'stop' + Enter to abort.\r\n");

    for (unsigned i = 0; i < cycles && !stop_requested; i++) {
        pin_low(TEST_PIN);
        P("[LOW]     %u/%u\r\n", i + 1, cycles);
        if (!wait_ms_check(low_ms)) break;

        pin_release(TEST_PIN);
        P("[RELEASE] %u/%u\r\n", i + 1, cycles);
        if (!wait_ms_check(release_ms)) break;
    }

    pin_release(TEST_PIN);
    P("Slow test finished. GP2 released.\r\n");
}

/* ================= 方波（推挽）================= */

static void run_square(unsigned hz, uint32_t seconds)
{
    if (hz == 0 || hz > 1000) {
        P("Supported frequency: 1-1000 Hz\r\n");
        return;
    }

    uint32_t half_us = 500000u / hz;
    if (half_us == 0) half_us = 1;

    uint64_t total_cycles = (uint64_t)seconds * (uint64_t)hz;
    if (total_cycles == 0) total_cycles = 1;

    stop_requested = false;
    reset_check_stop();

    P("\r\nSquare wave: %u Hz for %lu s (approx)\r\n",
      hz, (unsigned long)seconds);

    for (uint64_t i = 0; i < total_cycles && !stop_requested; i++) {
        /* LOW 半周期 */
        pin_low(TEST_PIN);
        if (half_us >= 1000) {
            if (!wait_ms_check(half_us / 1000)) break;
            if (half_us % 1000) busy_wait_us_32(half_us % 1000);
        } else {
            busy_wait_us_32(half_us);
        }
        if (check_stop()) break;

        /* HIGH 半周期 */
        pin_high(TEST_PIN);
        if (half_us >= 1000) {
            if (!wait_ms_check(half_us / 1000)) break;
            if (half_us % 1000) busy_wait_us_32(half_us % 1000);
        } else {
            busy_wait_us_32(half_us);
        }
        if (check_stop()) break;
    }

    pin_release(TEST_PIN);
    P("Square wave finished. GP2 released.\r\n");
}

/* ================= 万用表辅助 ================= */

static void run_short_test(void)
{
    P("\r\nShort-to-3V3 test on GP2...\r\n");
    pin_low(TEST_PIN);
    busy_wait_us_32(100);
    int level = gpio_get(TEST_PIN);

    if (level == 1) {
        P("[FAIL] GP2 driven LOW but reads HIGH -> shorted to 3V3?\r\n");
        P("       Check wiring before continuing.\r\n");
    } else {
        P("[PASS] GP2 driven LOW and reads LOW.\r\n");
    }
    pin_release(TEST_PIN);
}

static void run_open_test(void)
{
    P("\r\nOpen/pull-up test on GP2...\r\n");
    pin_release(TEST_PIN);
    busy_wait_us_32(100);

    if (gpio_get(TEST_PIN) == 1) {
        P("[PASS] GP2 released and reads HIGH (750R pull-up OK).\r\n");
    } else {
        P("[FAIL] GP2 released but reads LOW -> open wire, no pull-up,\r\n");
        P("       or something is driving it low.\r\n");
    }
}

static void run_probe(void)
{
    P("\r\nProbing GP2 for 10 s (any external driver?)\r\n");
    P("GP2 left in high-Z; press stop to abort.\r\n");

    pin_release(TEST_PIN);
    stop_requested = false;
    reset_check_stop();

    int last = gpio_get(TEST_PIN);
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t t_end = make_timeout_time_ms(10000);

    while (!time_reached(t_end) && !stop_requested) {
        int now = gpio_get(TEST_PIN);
        if (now != last) {
            int64_t ms = absolute_time_diff_us(t0, get_absolute_time()) / 1000;
            P("[%lld ms] GP2 -> %d\r\n", (long long)ms, now);
            last = now;
        }
        if (check_stop()) break;
        tight_loop_contents();
    }
    pin_release(TEST_PIN);
    P("Probe done.\r\n");
}

/* ================= 逻辑分析仪辅助 ================= */

static void run_pulse_us(uint32_t us)
{
    if (us == 0 || us > 1000000) {
        P("usage: pulse-us N   (N = 1..1000000 us)\r\n");
        return;
    }
    P("\r\nSingle LOW pulse: %lu us\r\n", (unsigned long)us);
    pin_low(TEST_PIN);
    busy_wait_us_32(us);
    pin_release(TEST_PIN);
    P("Pulse done.\r\n");
}

static void run_pulse_train(uint32_t width_us, uint32_t count, uint32_t gap_ms)
{
    if (width_us == 0 || width_us > 1000000 || count == 0 || count > 1000) {
        P("usage: pulse-train WIDTH_US COUNT GAP_MS\r\n");
        P("  e.g. pulse-train 16 20 10\r\n");
        return;
    }
    P("\r\npulse-train: %lu x %lu us, gap %lu ms\r\n",
      (unsigned long)count, (unsigned long)width_us, (unsigned long)gap_ms);

    stop_requested = false;
    reset_check_stop();

    for (uint32_t i = 0; i < count; i++) {
        pin_low(TEST_PIN);
        busy_wait_us_32(width_us);
        pin_release(TEST_PIN);

        if (i + 1 < count) {
            if (!wait_ms_check(gap_ms)) break;
        }
    }
    pin_release(TEST_PIN);
    P("pulse-train done.\r\n");
}

static void run_width_sweep(void)
{
    static const uint32_t widths[] = {1, 2, 5, 10, 16, 20, 50, 100, 200, 500, 1000};
    P("\r\nWidth sweep on GP2 (LOW pulse + 10ms gap)\r\n");

    stop_requested = false;
    reset_check_stop();

    for (unsigned i = 0; i < sizeof(widths) / sizeof(widths[0]); i++) {
        for (int k = 0; k < 10 && !stop_requested; k++) {
            pin_low(TEST_PIN);
            busy_wait_us_32(widths[i]);
            pin_release(TEST_PIN);
            busy_wait_us_32(10000);
            if (check_stop()) break;
        }
        P("  width=%lu us done\r\n", (unsigned long)widths[i]);
    }
    pin_release(TEST_PIN);
    P("Width sweep finished.\r\n");
}

static void run_toggle(void)
{
    P("\r\nFast toggle on GP2 for 1000 ms...\r\n");
    stop_requested = false;
    reset_check_stop();

    uint32_t count = 0;
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t t_end = make_timeout_time_ms(1000);

    while (!time_reached(t_end) && !stop_requested) {
        gpio_put(TEST_PIN, 1);
        gpio_put(TEST_PIN, 0);
        count++;
        if ((count & 0x3FF) == 0 && check_stop()) break;
    }

    int64_t us = absolute_time_diff_us(t0, get_absolute_time());
    P("Toggles: %lu in %lld us -> ~%lu Hz (C loop, not exact)\r\n",
      (unsigned long)count, (long long)us,
      us ? (unsigned long)((uint64_t)count * 1000000ull / (uint64_t)us) : 0);
    pin_release(TEST_PIN);
}

static void run_pwm(uint32_t hz, uint32_t duty_pct)
{
    if (hz < 8 || hz > 1000000 || duty_pct > 100) {
        P("usage: pwm HZ DUTY   (HZ 8..1000000, DUTY 0..100)\r\n");
        P("  (RP2040 PWM min freq ~8 Hz at current sysclk)\r\n");
        return;
    }
    uint slice = pwm_gpio_to_slice_num(TEST_PIN);
    uint chan  = pwm_gpio_to_channel(TEST_PIN);

    uint32_t sys = clock_get_hz(clk_sys);
    uint32_t div = 1;
    uint32_t wrap = sys / hz;
    while (wrap > 65535 && div < 255) {
        div++;
        wrap = sys / (hz * div);
    }
    if (wrap > 65535) { P("HZ too low for PWM\r\n"); return; }

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, (uint8_t)div);
    pwm_config_set_wrap(&cfg, (uint16_t)(wrap - 1));
    pwm_init(slice, &cfg, false);
    pwm_set_chan_level(slice, chan, (uint16_t)((wrap * duty_pct) / 100));
    pwm_set_enabled(slice, true);
    gpio_set_function(TEST_PIN, GPIO_FUNC_PWM);

    P("\r\nPWM on GP2: %lu Hz, duty %lu%%\r\n",
      (unsigned long)hz, (unsigned long)duty_pct);
    P("Send 'stop' to stop.\r\n");

    stop_requested = false;
    reset_check_stop();
    while (!stop_requested) {
        if (check_stop()) break;
        sleep_ms(10);
    }

    pwm_set_enabled(slice, false);
    gpio_set_function(TEST_PIN, GPIO_FUNC_SIO);
    pin_release(TEST_PIN);
    P("PWM stopped.\r\n");
}

static void run_pattern(uint32_t bits, uint32_t nbits, uint32_t bit_us)
{
    if (nbits == 0 || nbits > 32 || bit_us == 0) {
        P("usage: pattern HEX NBITS BIT_US\r\n");
        P("  e.g. pattern 0xA5 8 10\r\n");
        return;
    }
    P("\r\nPattern 0x%lX, %lu bits, %lu us/bit on GP2\r\n",
      (unsigned long)bits, (unsigned long)nbits, (unsigned long)bit_us);
    P("NOTE: push-pull output. Do NOT use with real STM8 connected.\r\n");

    for (uint32_t i = 0; i < nbits; i++) {
        int b = (bits >> (nbits - 1 - i)) & 1;
        if (b) pin_high(TEST_PIN); else pin_low(TEST_PIN);
        busy_wait_us_32(bit_us);
    }
    pin_release(TEST_PIN);
    P("Pattern sent.\r\n");
}

/* ================= SWIM 进入序列 ================= */

/* 内部：完整的 SWIM 进入序列（GP4 先拉低，GP2 16us 低 + 4x1k + 4x2k） */
static void entry_sequence_body(void)
{
    pin_release(TEST_PIN);
    pin_low(NRST_PIN);
    busy_wait_us_32(10);

    pin_low(TEST_PIN);
    busy_wait_us_32(16);
    pin_release(TEST_PIN);
    busy_wait_us_32(5);

    for (int i = 0; i < 4; i++) {
        pin_low(TEST_PIN);     busy_wait_us_32(500);
        pin_release(TEST_PIN); busy_wait_us_32(500);
    }
    for (int i = 0; i < 4; i++) {
        pin_low(TEST_PIN);     busy_wait_us_32(250);
        pin_release(TEST_PIN); busy_wait_us_32(250);
    }
    release_all();
}

static void run_entry_waveform(void)
{
    P("\r\nEntry waveform test (no STM8 traffic).\r\n");
    P("Expect: NRST low, SWIM 16us low, 4x1kHz, 4x2kHz.\r\n");
    P("With 750R pull-up, rise time should be <100ns (scope check).\r\n");
    P("LA trigger: CH1(GP4) falling edge.\r\n");

    entry_sequence_body();
    P("Waveform done. Pins released.\r\n");
}

static void run_entry_loop(unsigned n)
{
    if (n == 0 || n > 50) {
        P("usage: entry-loop N   (N = 1..50)\r\n");
        return;
    }
    P("\r\nentry-loop: %u sequences, 200ms apart\r\n", n);
    P("Set LA trigger: CH1(GP4) falling edge, then capture.\r\n");

    stop_requested = false;
    reset_check_stop();

    for (unsigned i = 0; i < n && !stop_requested; i++) {
        P("  sequence %u/%u\r\n", i + 1, n);
        entry_sequence_body();
        if (!wait_ms_check(200)) break;
    }
    P("entry-loop done.\r\n");
}

static void run_marker_then_entry(void)
{
    P("\r\nMarker pulse on GP4, then entry sequence on GP2.\r\n");
    P("LA trigger: CH1(GP4) rising edge -> marker is first.\r\n");

    /* 标记：GP4 高 10us，低 10us */
    pin_high(NRST_PIN);
    busy_wait_us_32(10);
    pin_release(NRST_PIN);
    busy_wait_us_32(20);

    entry_sequence_body();
    P("Marker + entry done.\r\n");
}

static void run_swim_sync_wait(void)
{
    P("\r\nSWIM entry + wait for target sync pulse...\r\n");
    P("This is the step the original firmware was missing.\r\n");
    P("LA trigger: CH1(GP4) falling edge, capture ~250ms.\r\n");

    entry_sequence_body();

    /* entry_sequence_body 末尾已经 release_all，这里重新只保留 GP2 高阻 */
    pin_release(TEST_PIN);
    P("Entry sent. Waiting up to 200ms for target to pull SWIM LOW...\r\n");

    absolute_time_t t0 = get_absolute_time();
    absolute_time_t t_end = make_timeout_time_ms(200);
    bool seen = false;
    int64_t low_us = 0;

    while (!time_reached(t_end)) {
        if (gpio_get(TEST_PIN) == 0) {
            seen = true;
            low_us = absolute_time_diff_us(t0, get_absolute_time());
            break;
        }
        tight_loop_contents();
    }

    if (!seen) {
        P("[INFO] No sync detected in 200ms.\r\n");
        P("       Target absent, not powered, or entry not accepted.\r\n");
    } else {
        absolute_time_t t1 = get_absolute_time();
        absolute_time_t t2 = make_timeout_time_ms(20);
        int64_t rise_us = 0;
        while (!time_reached(t2)) {
            if (gpio_get(TEST_PIN) == 1) {
                rise_us = absolute_time_diff_us(t1, get_absolute_time());
                break;
            }
            tight_loop_contents();
        }
        P("[OK] Target pulled SWIM low at +%lld us after release.\r\n",
          (long long)low_us);
        if (rise_us)
            P("     LOW duration ~%lld us (expect ~16us @8MHz HSI).\r\n",
              (long long)rise_us);
        else
            P("     Still LOW; target may be holding it.\r\n");
    }

    release_all();
    P("SWIM sync wait finished.\r\n");
}

/* ================= 杂项 ================= */

static void run_nrst(const char *arg)
{
    if (strcmp(arg, "low") == 0) {
        pin_low(NRST_PIN);
        P("NRST (GP4) driven LOW.\r\n");
    } else if (strcmp(arg, "high") == 0) {
        pin_high(NRST_PIN);
        P("NRST (GP4) driven HIGH.\r\n");
    } else if (strcmp(arg, "release") == 0) {
        pin_release(NRST_PIN);
        P("NRST (GP4) released.\r\n");
    } else {
        P("usage: nrst low|high|release\r\n");
    }
}

static void run_loopback(void)
{
    P("\r\nLoopback test (requires GP2<->GP4 jumper)\r\n");
    P("Output on GP2, read on GP4, 10 transitions.\r\n");

    pin_release(NRST_PIN);
    gpio_set_dir(NRST_PIN, GPIO_IN);
    gpio_disable_pulls(NRST_PIN);

    int fails = 0;
    for (int i = 0; i < 10; i++) {
        pin_low(TEST_PIN);
        busy_wait_us_32(10);
        int a = gpio_get(NRST_PIN);

        pin_high(TEST_PIN);
        busy_wait_us_32(10);
        int b = gpio_get(NRST_PIN);

        if (a != 0 || b != 1) fails++;
        P("  cycle %d: low->%d, high->%d\r\n", i + 1, a, b);
    }
    pin_release(TEST_PIN);
    if (fails) P("[FAIL] %d/10 mismatches\r\n", fails);
    else       P("[PASS] loopback OK\r\n");
}

/* ================= 命令分发 ================= */

static void process_command(char *command)
{
    while (*command && isspace((unsigned char)*command)) command++;

    size_t length = strlen(command);
    while (length && isspace((unsigned char)command[length - 1]))
        command[--length] = '\0';

    if (length == 0) return;

    /* ---- basic ---- */
    if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        show_help();
    } else if (strcmp(command, "status") == 0) {
        show_status();
    } else if (strcmp(command, "low") == 0) {
        pin_low(TEST_PIN);
        P("GP2 driven LOW. Expect ~0V, ~4.4mA through 750R.\r\n");
        show_status();
    } else if (strcmp(command, "high") == 0) {
        pin_high(TEST_PIN);
        P("GP2 driven HIGH (push-pull). Expect ~3.3V.\r\n");
        P("Only use this if nothing else drives GP2.\r\n");
        show_status();
    } else if (strcmp(command, "release") == 0) {
        release_all();
        P("GP2/GP4 released to high-Z.\r\n");
        P("With 750R pull-up, GP2 should read ~3.3V immediately.\r\n");
        show_status();
    } else if (strcmp(command, "static low") == 0) {
        pin_low(TEST_PIN);
        P("GP2 LOW.\r\n");
    } else if (strcmp(command, "static high") == 0) {
        pin_high(TEST_PIN);
        P("GP2 HIGH.\r\n");
    } else if (strcmp(command, "static rel") == 0 ||
               strcmp(command, "static release") == 0) {
        pin_release(TEST_PIN);
        P("GP2 released.\r\n");

    /* ---- slow patterns ---- */
    } else if (strcmp(command, "slow") == 0) {
        run_slow(10, 1000, 1000);
    } else if (strcmp(command, "blink") == 0) {
        run_slow(10, 500, 500);
    } else if (strcmp(command, "pulse 100") == 0) {
        run_slow(10, 100, 900);
    } else if (strcmp(command, "pulse 10") == 0) {
        run_slow(10, 10, 990);
    } else if (strcmp(command, "pulse 1") == 0) {
        run_slow(10, 1, 999);

    /* ---- square ---- */
    } else if (strncmp(command, "sqr ", 4) == 0) {
        unsigned hz = 0;
        if (sscanf(command + 4, "%u", &hz) == 1) run_square(hz, 30);
        else P("usage: sqr HZ\r\n");
    } else if (strcmp(command, "square 10") == 0) {
        run_square(10, 10);
    } else if (strcmp(command, "square 100") == 0) {
        run_square(100, 10);
    } else if (strcmp(command, "square 1000") == 0) {
        run_square(1000, 5);

    /* ---- multimeter aids ---- */
    } else if (strcmp(command, "short-test") == 0) {
        run_short_test();
    } else if (strcmp(command, "open-test") == 0) {
        run_open_test();
    } else if (strcmp(command, "probe") == 0) {
        run_probe();

    /* ---- logic analyzer aids ---- */
    } else if (strncmp(command, "pulse-us ", 9) == 0) {
        long n = strtol(command + 9, NULL, 10);
        if (n > 0) run_pulse_us((uint32_t)n);
        else P("usage: pulse-us N\r\n");
    } else if (strncmp(command, "pulse-train ", 12) == 0) {
        unsigned w = 0, c = 0, g = 0;
        if (sscanf(command + 12, "%u %u %u", &w, &c, &g) == 3)
            run_pulse_train(w, c, g);
        else P("usage: pulse-train WIDTH_US COUNT GAP_MS\r\n");
    } else if (strcmp(command, "width-sweep") == 0) {
        run_width_sweep();
    } else if (strcmp(command, "toggle") == 0) {
        run_toggle();
    } else if (strncmp(command, "pwm ", 4) == 0) {
        unsigned hz = 0, duty = 0;
        if (sscanf(command + 4, "%u %u", &hz, &duty) == 2) run_pwm(hz, duty);
        else P("usage: pwm HZ DUTY\r\n");
    } else if (strncmp(command, "pattern ", 8) == 0) {
        unsigned bits = 0, nbits = 0, bit_us = 0;
        if (sscanf(command + 8, "%x %u %u", &bits, &nbits, &bit_us) == 3)
            run_pattern(bits, nbits, bit_us);
        else P("usage: pattern HEX NBITS BIT_US\r\n");

    /* ---- SWIM entry ---- */
    } else if (strcmp(command, "entry-waveform") == 0 ||
               strcmp(command, "waveform") == 0) {
        run_entry_waveform();
    } else if (strncmp(command, "entry-loop ", 11) == 0) {
        long n = strtol(command + 11, NULL, 10);
        if (n > 0) run_entry_loop((unsigned)n);
        else P("usage: entry-loop N\r\n");
    } else if (strcmp(command, "marker") == 0) {
        run_marker_then_entry();
    } else if (strcmp(command, "swim-sync-wait") == 0) {
        run_swim_sync_wait();

    /* ---- misc ---- */
    } else if (strncmp(command, "nrst ", 5) == 0) {
        run_nrst(command + 5);
    } else if (strcmp(command, "loopback") == 0) {
        run_loopback();
    } else if (strcmp(command, "stop") == 0 || strcmp(command, "q") == 0) {
        stop_requested = true;
        pin_release(TEST_PIN);
        P("GP2 released.\r\n");
    } else {
        P("Unknown command: %s\r\n", command);
        P("Type 'help'.\r\n");
    }
}

/* ================= 启动：等 USB 枚举 ================= */

static void wait_for_usb_host(void)
{
    absolute_time_t deadline = make_timeout_time_ms(3000);
    while (!stdio_usb_connected() && !time_reached(deadline)) {
        tight_loop_contents();
    }
    sleep_ms(100);
    stdio_flush();
}

int main(void)
{
    stdio_init_all();

    /* 关键：先等 USB 主机连上，否则前几条 printf 会被 USB CDC 丢弃 */
    wait_for_usb_host();

    init_pin(TEST_PIN);
    init_pin(NRST_PIN);
    release_all();

    P("\r\n");
    P("========================================\r\n");
    P(" RP2040-Zero GPIO / SWIM diagnostic\r\n");
    P(" 750R external pull-up assumed on GP2\r\n");
    P(" Non-destructive; no STM8 read/erase/prog\r\n");
    P(" Type 'help' and press Enter\r\n");
    P("========================================\r\n");
    show_status();

    char line[LINE_MAX];
    unsigned used = 0;

    P("\r\n> ");

    while (true) {
        int ch = getchar_timeout_us(10000);

        if (ch == PICO_ERROR_TIMEOUT) {
            tight_loop_contents();
            continue;
        }

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
