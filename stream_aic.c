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
}
