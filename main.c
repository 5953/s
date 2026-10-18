#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define SWIM_PIN 2
#define NRST_PIN 4

/*
 * Safe SWIM electrical diagnostic firmware.
 * It does NOT identify a chip, read memory, erase, or program Flash.
 *
 * Open-drain emulation:
 *   low     = output-low
 *   release = input/high-Z
 * External pull-up is required for a released line to read high.
 */

static void od_init(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);             // preload output latch low
    gpio_set_dir(pin, GPIO_IN);   // begin released/high-Z
    gpio_disable_pulls(pin);
}

static inline void od_low(uint pin) {
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

static inline void od_release(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

static void release_all(void) {
    od_release(SWIM_PIN);
    od_release(NRST_PIN);
}

static void run_entry_waveform(void) {
    printf("\r\n[TEST] SWIM entry waveform starting\r\n");
    printf("[TEST] GP2=SWIM, GP4=NRST; no chip read/erase/program\r\n");

    od_release(SWIM_PIN);
    od_low(NRST_PIN);
    busy_wait_us_32(10);

    od_low(SWIM_PIN);
    busy_wait_us_32(16);
    od_release(SWIM_PIN);
    busy_wait_us_32(5);

    for (unsigned i = 0; i < 4; ++i) {
        od_low(SWIM_PIN);
        busy_wait_us_32(500);
        od_release(SWIM_PIN);
        busy_wait_us_32(500);
    }

    for (unsigned i = 0; i < 4; ++i) {
        od_low(SWIM_PIN);
        busy_wait_us_32(250);
        od_release(SWIM_PIN);
        busy_wait_us_32(250);
    }

    release_all();
    printf("[TEST] waveform sent; pins released\r\n");
    printf("[TEST] expected: 16 us low, 4 x 1 kHz, 4 x 2 kHz\r\n");
}

static void slow_test(void) {
    printf("\r\n[TEST] slow test: GP2 low 500 ms, released 500 ms, 10 cycles\r\n");
    printf("[TEST] Use a multimeter: GP2 should alternate near 0 V and 3.3 V\r\n");
    printf("[TEST] No target chip required. Please wait about 10 seconds.\r\n");

    od_release(SWIM_PIN);
    for (unsigned i = 0; i < 10; ++i) {
        od_low(SWIM_PIN);
        printf("[SLOW] cycle %u/10: GP2 driven LOW\r\n", i + 1);
        sleep_ms(500);
        od_release(SWIM_PIN);
        printf("[SLOW] cycle %u/10: GP2 released (pull-up should make it HIGH)\r\n", i + 1);
        sleep_ms(500);
    }
    od_release(SWIM_PIN);
    printf("[TEST] slow test finished; GP2 is released\r\n");
}

static void print_help(void) {
    printf("\r\nCommands (type command and press Enter):\r\n");
    printf("  help             - show this help\r\n");
    printf("  status           - show pin state and GPIO input level\r\n");
    printf("  low              - hold GP2 LOW until 'release' is sent\r\n");
    printf("  release          - release GP2 and GP4 to high-Z\r\n");
    printf("  slow-test        - 10 cycles: GP2 LOW 500 ms / released 500 ms\r\n");
    printf("  entry-waveform   - send one short SWIM timing test\r\n");
    printf("  repeat N         - send entry waveform N times (1..20)\r\n");
    printf("\r\nSafety: chip read, erase, and Flash programming are not implemented.\r\n");
    printf("Electrical: GP2/GP4 are 3.3 V GPIOs; NEVER apply 5 V.\r\n");
}

static void print_pin(const char *name, uint pin) {
    printf("%s GP%u: direction=%s, sampled_level=%u\r\n",
           name, pin,
           gpio_get_dir(pin) == GPIO_OUT ? "OUTPUT" : "INPUT/high-Z",
           gpio_get(pin) ? 1u : 0u);
}

static void print_status(void) {
    printf("\r\nRP2040-Zero SWIM electrical test\r\n");
    printf("System clock = %lu Hz\r\n", (unsigned long)clock_get_hz(clk_sys));
    print_pin("SWIM", SWIM_PIN);
    print_pin("NRST", NRST_PIN);
    printf("Mode = electrical test only; Flash operations disabled\r\n");
    printf("For released GP2 to read HIGH, fit an external pull-up to 3V3.\r\n");
}

static void process_line(char *line) {
    while (*line && isspace((unsigned char)*line)) ++line;
    size_t len = strlen(line);
    while (len && isspace((unsigned char)line[len - 1])) line[--len] = '\0';
    if (len == 0) return;

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        print_help();
    } else if (strcmp(line, "status") == 0) {
        print_status();
    } else if (strcmp(line, "low") == 0) {
        od_low(SWIM_PIN);
        printf("[OK] GP2 is now driven LOW. Measure GP2-to-GND; expect near 0 V.\r\n");
        printf("[OK] Send 'release' to return GP2 to high-Z.\r\n");
        print_pin("SWIM", SWIM_PIN);
    } else if (strcmp(line, "release") == 0) {
        release_all();
        printf("[OK] GP2 and GP4 released to high-Z.\r\n");
        printf("[INFO] With a pull-up resistor to 3V3, GP2 should measure near 3.3 V.\r\n");
        print_pin("SWIM", SWIM_PIN);
    } else if (strcmp(line, "slow-test") == 0) {
        slow_test();
    } else if (strcmp(line, "entry-waveform") == 0 || strcmp(line, "waveform") == 0) {
        run_entry_waveform();
    } else if (strncmp(line, "repeat ", 7) == 0) {
        char *end = NULL;
        long n = strtol(line + 7, &end, 10);
        if (end == line + 7 || *end != '\0' || n < 1 || n > 20) {
            printf("[ERR] usage: repeat N (N=1..20)\r\n");
            return;
        }
        for (long i = 0; i < n; ++i) {
            run_entry_waveform();
            if (i + 1 < n) sleep_ms(1000);
        }
    } else if (strcmp(line, "quit") == 0) {
        printf("No shell to exit; firmware remains ready.\r\n");
    } else {
        printf("[ERR] unknown command: %s\r\n", line);
        printf("Type 'help'.\r\n");
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);

    od_init(SWIM_PIN);
    od_init(NRST_PIN);
    release_all();

    printf("\r\n========================================\r\n");
    printf(" RP2040-Zero SWIM Slow Diagnostic Test\r\n");
    printf(" Non-destructive; no STM8 operations\r\n");
    printf(" Type 'help' and press Enter\r\n");
    printf("========================================\r\n");
    print_status();

    char line[96];
    size_t used = 0;
    printf("\r\n> ");
    while (true) {
        int ch = getchar_timeout_us(10000);
        if (ch == PICO_ERROR_TIMEOUT) {
            tight_loop_contents();
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (used > 0) {
                line[used] = '\0';
                process_line(line);
                used = 0;
                printf("\r\n> ");
            }
        } else if ((ch == 8 || ch == 127) && used > 0) {
            --used;
        } else if (ch >= 32 && ch <= 126) {
            if (used + 1 < sizeof(line)) line[used++] = (char)ch;
        }
    }
    return 0;
}
