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
 * Open-drain emulation:
 *   drive low: output latch is 0, then set pin to output
 *   release:  set pin to input/high-Z
 *
 * Use external pull-ups. Do not use the RP2040 internal pull-up as the
 * only pull-up for a protocol timing test.
 */
static void od_init(uint pin) {
    gpio_init(pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
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

static void pulse_train(uint32_t half_period_us, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        od_low(SWIM_PIN);
        busy_wait_us_32(half_period_us);
        od_release(SWIM_PIN);
        busy_wait_us_32(half_period_us);
    }
}

/*
 * Safe, non-destructive entry-waveform test only.
 * It does not read synchronization from the target and never accesses Flash.
 * NRST is held low for the waveform, then released high-Z.
 */
static void run_entry_waveform(void) {
    printf("\r\n[TEST] SWIM entry waveform starting\r\n");
    printf("[TEST] GP2=SWIM, GP4=NRST; no chip read/erase/program\r\n");

    od_release(SWIM_PIN);
    od_low(NRST_PIN);
    busy_wait_us_32(10);

    /* Start sequence: SWIM low for 16 us. */
    od_low(SWIM_PIN);
    busy_wait_us_32(16);
    od_release(SWIM_PIN);
    /* Keep a short, measurable high gap so the 16 us start pulse does not
       merge into the first 500 us low half-cycle. */
    busy_wait_us_32(5);

    /* Four 1 kHz cycles: 500 us low + 500 us released/high. */
    pulse_train(500, 4);

    /* Four 2 kHz cycles: 250 us low + 250 us released/high. */
    pulse_train(250, 4);

    od_release(SWIM_PIN);
    od_release(NRST_PIN);

    printf("[TEST] waveform sent; pins released\r\n");
    printf("[TEST] expected: 16 us low, 4 x 1 kHz, 4 x 2 kHz\r\n");
}

static void print_help(void) {
    printf("\r\nCommands:\r\n");
    printf("  help             - show this help\r\n");
    printf("  status           - show pin assignment\r\n");
    printf("  entry-waveform   - send one non-destructive SWIM entry waveform\r\n");
    printf("  repeat N         - send waveform N times (1..20), 1 s apart\r\n");
    printf("  release          - release GP2 and GP4 to high-Z\r\n");
    printf("\r\nSafety: no target ID, memory read, erase, or Flash programming is implemented.\r\n");
}

static void print_status(void) {
    printf("\r\nRP2040-Zero SWIM waveform test\r\n");
    printf("SWIM = GP%d (open-drain emulation)\r\n", SWIM_PIN);
    printf("NRST = GP%d (open-drain emulation)\r\n", NRST_PIN);
    printf("System clock = %lu Hz\r\n", (unsigned long)clock_get_hz(clk_sys));
    printf("Mode = waveform test only; Flash operations disabled\r\n");
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
    } else if (strcmp(line, "entry-waveform") == 0 || strcmp(line, "waveform") == 0) {
        run_entry_waveform();
    } else if (strcmp(line, "release") == 0) {
        release_all();
        printf("[OK] GP2 and GP4 released to high-Z\r\n");
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
    sleep_ms(1500); /* give USB CDC time to enumerate */

    od_init(SWIM_PIN);
    od_init(NRST_PIN);
    release_all();

    printf("\r\n========================================\r\n");
    printf(" RP2040-Zero STM8 SWIM Waveform Test\r\n");
    printf(" Non-destructive test firmware\r\n");
    printf(" Type 'help' and press Enter\r\n");
    printf("========================================\r\n");
    print_status();

    char line[96];
    size_t used = 0;
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
            if (used + 1 < sizeof(line)) {
                line[used++] = (char)ch;
            }
        }
    }
    return 0;
}
