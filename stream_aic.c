#define DEBUG(...) printf(__VA_ARGS__)

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

#include "tusb.h"

#include "nfc.h"
#include "aime.h"
#include "bana.h"
#include "pn532.h"
#include "board_defs.h"
#include "mode.h"

#include "pico/multicore.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

const int reader_intf = 1;

static struct {
    uint8_t buf[64];
    int pos;
} reader;

typedef volatile struct {
    bool debug;
    bool touch;
    reader_mode_t mode;
} aic_runtime_t;

aic_runtime_t aic_runtime;

typedef struct __attribute__((packed)) {
    struct {
        uint8_t level_idle;
        uint8_t level_active;
        bool rgb;
        bool led;
    } light;
    struct {
        bool virtual_aic;
        uint8_t mode;
    } reader;
    struct {
        uint8_t backlight;
    } lcd;
    struct {
        bool pn5180_tx;
    } tweak;
    uint32_t reserved;
} aic_cfg_t;

static aic_cfg_t aic_cfg = {
    .light = { .level_idle = 24, .level_active = 128, .rgb = true, .led = true },
    .reader = { .virtual_aic = true, .mode = MODE_AUTO },
    .lcd = { .backlight = 200, },
    .tweak = { .pn5180_tx = false },
};

void reader_poll_data()
{
    if (tud_cdc_n_available(reader_intf)) {
        int count = tud_cdc_n_read(reader_intf, reader.buf + reader.pos,
                                   sizeof(reader.buf) - reader.pos);
        if (count > 0) {
            uint32_t now = time_us_32();
            DEBUG("\n\033[32m%6ld>>", now / 1000);
            for (int i = 0; i < count; i++) {
                DEBUG(" %02X", reader.buf[reader.pos + i]);
            }
            DEBUG("\033[0m");
            reader.pos += count;
        }
    }
}

void wait_loop() {
    tud_task();
    reader_poll_data();
}

void card_name_update_cb(nfc_card_name card_name)
{
    printf("CardSignal\n");
}

static void cdc_reader_putc(uint8_t byte)
{
    tud_cdc_n_write(reader_intf, &byte, 1);
    tud_cdc_n_write_flush(reader_intf);
}

static void reader_detect_mode()
{
    if (aic_cfg.reader.mode == MODE_AUTO) {
        static bool was_active = true; // so first time mode will be cleared
        bool is_active = aime_is_active() || bana_is_active();
        if (was_active && !is_active) {
            aic_runtime.mode = MODE_NONE;
        }
        was_active = is_active;
    } else {
        aic_runtime.mode = aic_cfg.reader.mode;
    }

    if (aic_runtime.mode == MODE_NONE) {
        cdc_line_coding_t coding;
        tud_cdc_n_get_line_coding(reader_intf, &coding);
        aic_runtime.mode = mode_detect(reader.buf, reader.pos, coding.bit_rate);
        if ((reader.pos > 10) && (aic_runtime.mode == MODE_NONE)) {
            reader.pos = 0; // drop the buffer
        }
    }

}

static void reader_run()
{
    reader_poll_data();
    reader_detect_mode();

    if (reader.pos > 0) {
        uint8_t buf[64];
        memcpy(buf, reader.buf, reader.pos);
        int count = reader.pos;
        switch (aic_runtime.mode) {
            case MODE_AIME0:
            case MODE_AIME1:
                reader.pos = 0;
                aime_sub_mode(aic_runtime.mode == MODE_AIME0 ? 0 : 1);
                for (int i = 0; i < count; i++) {
                    aime_feed(buf[i]);
                }
                break;
            case MODE_BANA:
                reader.pos = 0;
                for (int i = 0; i < count; i++) {
                    bana_feed(buf[i]);
                }
                break;
            default:
                break;
        }
    }
}

static struct {
    uint8_t current[9];
    uint8_t reported[9];
    uint64_t report_time;
} hid_cardio;


enum {
    REPORT_ID_EAMU = 1,
    REPORT_ID_FELICA = 2,
    REPORT_ID_LIGHTS = 3,
};

static void update_cardio(nfc_card_t *card)
{
    switch (card->card_type) {
        case NFC_CARD_MIFARE:
            hid_cardio.current[0] = REPORT_ID_EAMU;
            hid_cardio.current[1] = 0xe0;
            hid_cardio.current[2] = 0x04;
            if (card->len == 4) {
                memcpy(hid_cardio.current + 3, card->uid, 4);
                memcpy(hid_cardio.current + 7, card->uid, 2);
            } else if (card->len == 7) {
                memcpy(hid_cardio.current + 3, card->uid + 1, 6);
            }
            break;
        case NFC_CARD_FELICA:
            hid_cardio.current[0] = REPORT_ID_FELICA;
            memcpy(hid_cardio.current + 1, card->uid, 8);
           break;
        case NFC_CARD_VICINITY:
            hid_cardio.current[0] = REPORT_ID_EAMU;
            memcpy(hid_cardio.current + 1, card->uid, 8);
            break;
        default:
            memset(hid_cardio.current, 0, 9);
            return;
    }

    printf("CardIO ");
    for (int i = 1; i < 9; i++) {
        printf("%02X", hid_cardio.current[i]);
    }
    printf("\n");
}


static void cardio_run()
{
    if (aime_is_active() || bana_is_active()) {
        memset(hid_cardio.current, 0, 9);
        return;
    }

    static nfc_card_t old_card = { 0 };

    nfc_rf_field(true);
    nfc_card_t card = nfc_detect_card();
    if (card.card_type != NFC_CARD_NONE) {
        nfc_identify_last_card();
    }
    nfc_rf_field(false);

    if (memcmp(&old_card, &card, sizeof(old_card)) == 0) {
        return;
    }

    old_card = card;


    display_card(&card);
    update_cardio(&card);
}


int main()
{
    stdio_init_all();

    tusb_init();

    uint32_t freq = clock_get_hz(clk_sys);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, freq, freq);

    nfc_init_i2c(I2C_PORT, I2C_SCL, I2C_SDA, I2C_FREQ);
    nfc_init_spi(SPI_PORT, SPI_MISO, SPI_SCK, SPI_MOSI, SPI_RST, SPI_NSS, SPI_BUSY);
    nfc_init();
    nfc_set_wait_loop(wait_loop);
    nfc_set_card_name_listener(card_name_update_cb);

    aime_init(cdc_reader_putc);
    aime_virtual_aic(true);
    bana_init(cdc_reader_putc);

    while(1) {
        tud_task();

        reader_run();
        cardio_run();

        sleep_ms(1);
    }

    /*
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    
    #ifdef PICO_DEFAULT_LED_PIN
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    #else
    blink_pin_forever(pio, 0, offset, 6, 3);
    #endif
    // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }

    */
}
