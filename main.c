
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#define TEST_PIN 2u
#define NRST_PIN 4u
#define LINE_MAX 96

static volatile bool stop_requested = false;

/* GP2 拉低 */
static void pin_low(uint pin)
{
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_OUT);
}

/* GP2 推挽输出高电平 */
static void pin_high(uint pin)
{
    gpio_put(pin, 1);
    gpio_set_dir(pin, GPIO_OUT);
}

/* 释放为高阻态 */
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

static void show_status(void)
{
    printf("\r\nGP2 direction: %s, level=%u\r\n",
        gpio_get_dir(TEST_PIN) ? "OUTPUT" : "INPUT/HIGH-Z",
        gpio_get(TEST_PIN) ? 1u : 0u);

    printf("GP4 direction: %s, level=%u\r\n",
        gpio_get_dir(NRST_PIN) ? "OUTPUT" : "INPUT/HIGH-Z",
        gpio_get(NRST_PIN) ? 1u : 0u);

    printf("STM8 read/erase/program functions are disabled.\r\n");
}

static void show_help(void)
{
    printf("\r\n===== RP2040-Zero GPIO Test =====\r\n");
    printf("help           Show help\r\n");
    printf("status         Show pin status\r\n");
    printf("low            Hold GP2 at 0V\r\n");
    printf("high           Drive GP2 to 3.3V\r\n");
    printf("release        Release GP2 and GP4\r\n");
    printf("slow           1 second LOW / 1 second released\r\n");
    printf("blink          500ms LOW / 500ms released\r\n");
    printf("pulse 100      100ms LOW every second\r\n");
    printf("pulse 10       10ms LOW every second\r\n");
    printf("pulse 1        1ms LOW every second\r\n");
    printf("square 10      10Hz square wave\r\n");
    printf("square 100     100Hz square wave\r\n");
    printf("square 1000    1kHz square wave\r\n");
    printf("entry-waveform Microsecond test waveform\r\n");
    printf("stop           Stop a running pattern\r\n");
    printf("\r\nUse a 4.7k-10k pull-up from 3V3 to GP2 for release tests.\r\n");
    printf("Never apply 5V to RP2040 GPIO.\r\n");
}

/* 运行波形时，检查是否收到 stop 命令 */
static bool check_stop(void)
{
    static char command[24];
    static unsigned used = 0;

    int ch;

    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (ch == '\r' || ch == '\n')
        {
            command[used] = '\0';

            if (strcmp(command, "stop") == 0 ||
                strcmp(command, "q") == 0)
            {
                stop_requested = true;
                used = 0;
                return true;
            }

            used = 0;
        }
        else if ((ch == 8 || ch == 127) && used > 0)
        {
            used--;
        }
        else if (ch >= 32 && ch <= 126 &&
                 used + 1 < sizeof(command))
        {
            command[used++] = (char)ch;
        }
    }

    return stop_requested;
}

static bool wait_ms_check(uint32_t milliseconds)
{
    while (milliseconds > 0)
    {
        if (check_stop())
            return false;

        uint32_t part =
            milliseconds > 5 ? 5 : milliseconds;

        sleep_ms(part);
        milliseconds -= part;
    }

    return !check_stop();
}

/* 开漏风格的慢速电平切换 */
static void run_slow(
    unsigned cycles,
    uint32_t low_ms,
    uint32_t release_ms)
{
    stop_requested = false;

    printf("\r\nStarting slow GPIO test.\r\n");
    printf("Send stop + Enter to stop.\r\n");

    for (unsigned i = 0;
         i < cycles && !stop_requested;
         i++)
    {
        pin_low(TEST_PIN);

        printf("[LOW] Cycle %u/%u\r\n",
               i + 1, cycles);

        if (!wait_ms_check(low_ms))
            break;

        pin_release(TEST_PIN);

        printf("[RELEASE] Cycle %u/%u\r\n",
               i + 1, cycles);

        if (!wait_ms_check(release_ms))
            break;
    }

    pin_release(TEST_PIN);

    printf("Test finished. GP2 released.\r\n");
}

/* 推挽输出方波 */
static void run_square(unsigned hz, uint32_t seconds)
{
    if (hz == 0 || hz > 1000)
    {
        printf("Supported frequency: 1-1000Hz\r\n");
        return;
    }

    uint32_t half_us = 500000u / hz;
    uint32_t elapsed_ms = 0;
    uint32_t total_ms = seconds * 1000u;

    if (half_us == 0)
        half_us = 1;

    stop_requested = false;

    printf("Square wave: %u Hz\r\n", hz);

    while (elapsed_ms < total_ms &&
           !stop_requested)
    {
        pin_low(TEST_PIN);

        if (half_us >= 1000)
        {
            if (!wait_ms_check(half_us / 1000))
                break;

            if (half_us % 1000)
                busy_wait_us_32(half_us % 1000);

            elapsed_ms += half_us / 1000;
        }
        else
        {
            busy_wait_us_32(half_us);
            if (check_stop())
                break;
        }

        pin_high(TEST_PIN);

        if (half_us >= 1000)
        {
            if (!wait_ms_check(half_us / 1000))
                break;

            if (half_us % 1000)
                busy_wait_us_32(half_us % 1000);

            elapsed_ms += half_us / 1000;
        }
        else
        {
            busy_wait_us_32(half_us);
            if (check_stop())
                break;
        }
    }

    pin_release(TEST_PIN);
    printf("Square wave finished.\r\n");
}

/* 微秒级测试波形，不进行芯片操作 */
static void run_entry_waveform(void)
{
    printf("Starting microsecond waveform test.\r\n");

    pin_release(TEST_PIN);
    pin_low(NRST_PIN);

    busy_wait_us_32(10);

    pin_low(TEST_PIN);
    busy_wait_us_32(16);

    pin_release(TEST_PIN);
    busy_wait_us_32(5);

    for (unsigned i = 0; i < 4; i++)
    {
        pin_low(TEST_PIN);
        busy_wait_us_32(500);

        pin_release(TEST_PIN);
        busy_wait_us_32(500);
    }

    for (unsigned i = 0; i < 4; i++)
    {
        pin_low(TEST_PIN);
        busy_wait_us_32(250);

        pin_release(TEST_PIN);
        busy_wait_us_32(250);
    }

    release_all();

    printf("Waveform complete. Pins released.\r\n");
}

static void process_command(char *command)
{
    while (*command &&
           isspace((unsigned char)*command))
        command++;

    size_t length = strlen(command);

    while (length &&
           isspace((unsigned char)command[length - 1]))
        command[--length] = '\0';

    if (length == 0)
        return;

    if (strcmp(command, "help") == 0)
    {
        show_help();
    }
    else if (strcmp(command, "status") == 0)
    {
        show_status();
    }
    else if (strcmp(command, "low") == 0)
    {
        pin_low(TEST_PIN);
        printf("GP2 LOW: measure near 0V.\r\n");
    }
    else if (strcmp(command, "high") == 0)
    {
        pin_high(TEST_PIN);
        printf("GP2 HIGH: measure near 3.3V.\r\n");
        printf("Only use when GP2 is not connected to another driver.\r\n");
    }
    else if (strcmp(command, "release") == 0)
    {
        release_all();
        printf("GP2/GP4 released to high-Z.\r\n");
    }
    else if (strcmp(command, "slow") == 0)
    {
        run_slow(10, 1000, 1000);
    }
    else if (strcmp(command, "blink") == 0)
    {
        run_slow(10, 500, 500);
    }
    else if (strcmp(command, "pulse 100") == 0)
    {
        run_slow(10, 100, 900);
    }
    else if (strcmp(command, "pulse 10") == 0)
    {
        run_slow(10, 10, 990);
    }
    else if (strcmp(command, "pulse 1") == 0)
    {
        run_slow(10, 1, 999);
    }
    else if (strcmp(command, "square 10") == 0)
    {
        run_square(10, 10);
    }
    else if (strcmp(command, "square 100") == 0)
    {
        run_square(100, 10);
    }
    else if (strcmp(command, "square 1000") == 0)
    {
        run_square(1000, 5);
    }
    else if (strcmp(command, "entry-waveform") == 0)
    {
        run_entry_waveform();
    }
    else if (strcmp(command, "stop") == 0 ||
             strcmp(command, "q") == 0)
    {
        stop_requested = true;
        pin_release(TEST_PIN);
        printf("GP2 released.\r\n");
    }
    else
    {
        printf("Unknown command: %s\r\n", command);
        printf("Type help.\r\n");
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    init_pin(TEST_PIN);
    init_pin(NRST_PIN);
    release_all();

    printf("\r\nRP2040-Zero GPIO Multi-mode Test\r\n");
    printf("GP2=Test output; GP4=released.\r\n");
    printf("No STM8 read, erase, or programming.\r\n");
    printf("Type help and press Enter.\r\n");

    show_status();

    char line[LINE_MAX];
    unsigned used = 0;

    printf("\r\n> ");

    while (true)
    {
        int ch = getchar_timeout_us(10000);

        if (ch == PICO_ERROR_TIMEOUT)
            continue;

        if (ch == '\r' || ch == '\n')
        {
            if (used > 0)
            {
                line[used] = '\0';
                process_command(line);
                used = 0;
                printf("\r\n> ");
            }
        }
        else if ((ch == 8 || ch == 127) && used > 0)
        {
            used--;
        }
        else if (ch >= 32 && ch <= 126 &&
                 used + 1 < sizeof(line))
        {
            line[used++] = (char)ch;
        }
    }

    return 0;
}
