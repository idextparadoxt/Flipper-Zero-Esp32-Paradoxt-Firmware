#include "furi_hal_subghz.h"
#include "furi_hal_subghz_i.h"

#include <stdio.h>
#include <string.h>

#include <driver/rmt_tx.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <furi.h>
#include <furi_hal_spi.h>
#include <furi_hal_resources.h>
#include <furi_hal_power.h>
#include <furi_hal_aw9523.h>
#include <cc1101.h>
#include "boards/board.h"

static const char* TAG = "FuriHalSubGhz";

#if defined(BOARD_HAS_AW9523) && BOARD_HAS_AW9523
#define FURI_HAL_SUBGHZ_HAS_AW9523 1
#else
#define FURI_HAL_SUBGHZ_HAS_AW9523 0
#endif

typedef enum {
    FuriHalSubGhzStateInit,
    FuriHalSubGhzStateIdle,
    FuriHalSubGhzStateAsyncRx,
    FuriHalSubGhzStateAsyncTx,
} FuriHalSubGhzState;

typedef enum {
    FuriHalSubGhzRegulationOnlyRx,
    FuriHalSubGhzRegulationTxRx,
} FuriHalSubGhzRegulation;

typedef enum {
    FuriHalSubGhzAsyncTxMiddlewareStateIdle,
    FuriHalSubGhzAsyncTxMiddlewareStateReset,
    FuriHalSubGhzAsyncTxMiddlewareStateRun,
} FuriHalSubGhzAsyncTxMiddlewareState;

typedef struct {
    FuriHalSubGhzAsyncTxMiddlewareState state;
    bool is_odd_level;
    uint32_t adder_duration;
} FuriHalSubGhzAsyncTxMiddleware;

typedef struct {
    FuriHalSubGhzAsyncTxCallback callback;
    void* callback_context;
    FuriHalSubGhzAsyncTxMiddleware middleware;
    uint64_t duty_high;
    uint64_t duty_low;
    uint32_t remaining_duration;
    bool remaining_level;
    bool sequence_complete;
    volatile bool stop_requested;
    bool sleep_inhibited;
    rmt_channel_handle_t channel;
    bool channel_enabled;
    rmt_encoder_handle_t encoder;
} FuriHalSubGhzAsyncTx;

typedef struct {
    FuriHalSubGhzState state;
    FuriHalSubGhzRegulation regulation;
    const GpioPin* async_mirror_pin;
    int32_t rolling_counter_mult;
    uint32_t frequency;
    bool ext_leds_and_amp;
    bool dangerous_frequency_i;
    bool connected;
    volatile bool async_tx_complete;
    FuriHalSubGhzCaptureCallback async_rx_callback;
    void* async_rx_context;
    uint32_t async_rx_last_timestamp;
    bool async_rx_last_level;
    FuriHalSubGhzAsyncTx async_tx;
} FuriHalSubGhzContext;

typedef struct {
    uint8_t partnumber;
    uint8_t version;
    CC1101Status reset_status;
    CC1101Status status;
} FuriHalSubGhzProbe;

static FuriHalSubGhzContext furi_hal_subghz = {
    .state = FuriHalSubGhzStateInit,
    .regulation = FuriHalSubGhzRegulationTxRx,
    .async_mirror_pin = NULL,
    .rolling_counter_mult = 1,
    .frequency = 433920000,
    .ext_leds_and_amp = true,
    .dangerous_frequency_i = false,
    .connected = false,
    .async_tx_complete = true,
    .async_rx_callback = NULL,
    .async_rx_context = NULL,
    .async_rx_last_timestamp = 0,
    .async_rx_last_level = false,
    .async_tx =
        {
            .callback = NULL,
            .callback_context = NULL,
            .middleware = {0},
            .duty_high = 0,
            .duty_low = 0,
            .remaining_duration = 0,
            .remaining_level = true,
            .sequence_complete = true,
            .stop_requested = false,
            .sleep_inhibited = false,
            .channel = NULL,
            .encoder = NULL,
        },
};

#define FURI_HAL_SUBGHZ_RMT_RESOLUTION_HZ     (1000000u)
#define FURI_HAL_SUBGHZ_RMT_MEM_BLOCK_SYMBOLS (64u)
#define FURI_HAL_SUBGHZ_RMT_TX_QUEUE_DEPTH    (1u)
/* rmt_symbol_word_t durations are 15-bit fields. A larger chunk is silently
 * truncated (mod 32768): long gaps shrink, and an exact multiple of 32768
 * becomes 0, which the RMT treats as the end marker and stops the TX early. */
#define FURI_HAL_SUBGHZ_RMT_MAX_DURATION_TICKS (0x7FFFu)

static volatile uint32_t furi_hal_subghz_rx_irq_count = 0;

static void furi_hal_subghz_capture_gpio_callback(void* context) {
    UNUSED(context);
    furi_hal_subghz_rx_irq_count++;

    FuriHalSubGhzCaptureCallback callback = furi_hal_subghz.async_rx_callback;
    if(!callback) return;

    const uint32_t now = (uint32_t)esp_timer_get_time();
    const bool current_level = furi_hal_gpio_read(&gpio_cc1101_g0);
    const bool previous_level = furi_hal_subghz.async_rx_last_level;
    uint32_t duration = 0;

    if(furi_hal_subghz.async_rx_last_timestamp != 0) {
        duration = now - furi_hal_subghz.async_rx_last_timestamp;
    }

    furi_hal_subghz.async_rx_last_timestamp = now;
    furi_hal_subghz.async_rx_last_level = current_level;

    if(duration > 0) {
        callback(previous_level, duration, furi_hal_subghz.async_rx_context);
    }
}

static void furi_hal_subghz_async_tx_middleware_idle(FuriHalSubGhzAsyncTxMiddleware* middleware) {
    middleware->state = FuriHalSubGhzAsyncTxMiddlewareStateIdle;
    middleware->is_odd_level = false;
    middleware->adder_duration = 0;
}

static uint32_t furi_hal_subghz_async_tx_middleware_get_duration(FuriHalSubGhzAsyncTx* async_tx) {
    FuriHalSubGhzAsyncTxMiddleware* middleware = &async_tx->middleware;
    uint32_t ret = 0;

    if(middleware->state == FuriHalSubGhzAsyncTxMiddlewareStateReset) {
        return 0;
    }

    while(1) {
        LevelDuration ld = async_tx->callback(async_tx->callback_context);
        if(level_duration_is_reset(ld)) {
            middleware->state = FuriHalSubGhzAsyncTxMiddlewareStateReset;
            return middleware->is_odd_level ? middleware->adder_duration : 0;
        } else if(level_duration_is_wait(ld)) {
            middleware->is_odd_level = !middleware->is_odd_level;
            ret = middleware->adder_duration + FURI_HAL_SUBGHZ_ASYNC_TX_GUARD_TIME;
            middleware->adder_duration = 0;
            return ret;
        }

        const bool is_level = level_duration_get_level(ld);

        if(middleware->state == FuriHalSubGhzAsyncTxMiddlewareStateIdle) {
            if(is_level != middleware->is_odd_level) {
                middleware->state = FuriHalSubGhzAsyncTxMiddlewareStateRun;
                middleware->is_odd_level = is_level;
                middleware->adder_duration = 0;
            } else {
                continue;
            }
        }

        if(middleware->state == FuriHalSubGhzAsyncTxMiddlewareStateRun) {
            if(is_level == middleware->is_odd_level) {
                middleware->adder_duration += level_duration_get_duration(ld);
                continue;
            }

            middleware->is_odd_level = is_level;
            ret = middleware->adder_duration;
            middleware->adder_duration = level_duration_get_duration(ld);
            return ret;
        }
    }
}

static void furi_hal_subghz_async_tx_prepare(FuriHalSubGhzAsyncTx* async_tx) {
    furi_hal_subghz_async_tx_middleware_idle(&async_tx->middleware);
    async_tx->duty_high = 0;
    async_tx->duty_low = 0;
    async_tx->remaining_duration = 0;
    async_tx->remaining_level = true;
    async_tx->sequence_complete = false;
    async_tx->stop_requested = false;
}

static void furi_hal_subghz_async_tx_release_sleep_inhibit(FuriHalSubGhzAsyncTx* async_tx) {
    if(async_tx->sleep_inhibited) {
        async_tx->sleep_inhibited = false;
        furi_hal_power_insomnia_exit();
    }
}

static bool furi_hal_subghz_async_tx_next_chunk(
    FuriHalSubGhzAsyncTx* async_tx,
    bool* level,
    uint16_t* duration) {
    furi_check(level);
    furi_check(duration);

    while(async_tx->remaining_duration == 0 && !async_tx->sequence_complete) {
        if(async_tx->stop_requested) {
            async_tx->sequence_complete = true;
            break;
        }

        async_tx->remaining_duration = furi_hal_subghz_async_tx_middleware_get_duration(async_tx);
        if(async_tx->remaining_duration == 0) {
            async_tx->sequence_complete = true;
            break;
        }
    }

    if(async_tx->sequence_complete) {
        return false;
    }

    *level = async_tx->remaining_level;
    const uint32_t chunk =
        MIN(async_tx->remaining_duration, (uint32_t)FURI_HAL_SUBGHZ_RMT_MAX_DURATION_TICKS);
    *duration = (uint16_t)chunk;
    async_tx->remaining_duration -= chunk;

    if(*level) {
        async_tx->duty_high += chunk;
    } else {
        async_tx->duty_low += chunk;
    }

    if(async_tx->remaining_duration == 0) {
        async_tx->remaining_level = !async_tx->remaining_level;
    }

    return true;
}

static size_t IRAM_ATTR furi_hal_subghz_async_tx_encoder_callback(
    const void* data,
    size_t data_size,
    size_t symbols_written,
    size_t symbols_free,
    rmt_symbol_word_t* symbols,
    bool* done,
    void* arg) {
    UNUSED(data);
    UNUSED(data_size);
    UNUSED(symbols_written);

    FuriHalSubGhzAsyncTx* async_tx = arg;
    size_t encoded = 0;

    while(encoded < symbols_free) {
        bool level0 = false;
        uint16_t duration0 = 0;
        if(!furi_hal_subghz_async_tx_next_chunk(async_tx, &level0, &duration0)) {
            *done = true;
            break;
        }

        bool level1 = false;
        uint16_t duration1 = 0;
        if(!furi_hal_subghz_async_tx_next_chunk(async_tx, &level1, &duration1)) {
            level1 = false;
            duration1 = 1;
            *done = true;
        }

        symbols[encoded].level0 = level0;
        symbols[encoded].duration0 = duration0;
        symbols[encoded].level1 = level1;
        symbols[encoded].duration1 = duration1;
        encoded++;

        if(*done) {
            break;
        }
    }

    return encoded;
}

static bool IRAM_ATTR furi_hal_subghz_async_tx_done_callback(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t* event_data,
    void* context) {
    UNUSED(channel);
    UNUSED(event_data);
    UNUSED(context);

    furi_hal_subghz.async_tx_complete = true;
    return false;
}

static esp_err_t furi_hal_subghz_async_tx_alloc_backend(FuriHalSubGhzAsyncTx* async_tx) {
    if(async_tx->channel) {
        return ESP_OK;
    }

    const rmt_tx_channel_config_t channel_config = {
        .gpio_num = gpio_cc1101_g0.pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = FURI_HAL_SUBGHZ_RMT_RESOLUTION_HZ,
        .mem_block_symbols = FURI_HAL_SUBGHZ_RMT_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = FURI_HAL_SUBGHZ_RMT_TX_QUEUE_DEPTH,
        .intr_priority = 0,
        .flags =
            {
                .invert_out = 0,
                .with_dma = 0,
                .io_loop_back = 0,
                .io_od_mode = 0,
                .allow_pd = 0,
            },
    };
    esp_err_t error = rmt_new_tx_channel(&channel_config, &async_tx->channel);
    if(error != ESP_OK) {
        return error;
    }

    const rmt_simple_encoder_config_t encoder_config = {
        .callback = furi_hal_subghz_async_tx_encoder_callback,
        .arg = async_tx,
        .min_chunk_size = 1,
    };
    error = rmt_new_simple_encoder(&encoder_config, &async_tx->encoder);
    if(error != ESP_OK) {
        rmt_del_channel(async_tx->channel);
        async_tx->channel = NULL;
        return error;
    }

    const rmt_tx_event_callbacks_t callbacks = {
        .on_trans_done = furi_hal_subghz_async_tx_done_callback,
    };
    error = rmt_tx_register_event_callbacks(async_tx->channel, &callbacks, NULL);
    if(error != ESP_OK) {
        rmt_del_encoder(async_tx->encoder);
        rmt_del_channel(async_tx->channel);
        async_tx->encoder = NULL;
        async_tx->channel = NULL;
        return error;
    }

    error = rmt_enable(async_tx->channel);
    async_tx->channel_enabled = (error == ESP_OK);
    return error;
}

static void furi_hal_subghz_async_tx_free_backend(FuriHalSubGhzAsyncTx* async_tx) {
    if(async_tx->channel && async_tx->channel_enabled) {
        esp_err_t error = rmt_disable(async_tx->channel);
        if(error != ESP_OK) {
            ESP_LOGW(TAG, "Failed to disable RMT TX channel: %s", esp_err_to_name(error));
        }
        async_tx->channel_enabled = false;
    }

    if(async_tx->encoder) {
        esp_err_t error = rmt_del_encoder(async_tx->encoder);
        if(error != ESP_OK) {
            ESP_LOGW(TAG, "Failed to delete RMT encoder: %s", esp_err_to_name(error));
        }
        async_tx->encoder = NULL;
    }

    if(async_tx->channel) {
        esp_err_t error = rmt_del_channel(async_tx->channel);
        if(error != ESP_OK) {
            ESP_LOGW(TAG, "Failed to delete RMT TX channel: %s", esp_err_to_name(error));
        }
        async_tx->channel = NULL;
    }
}

/* Leaving SPWD (or right after power-up) the CC1101 holds SO high (CHIP_RDYn)
 * until its crystal is stable, typically ~150 us. The datasheet expects the
 * master to wait for SO low after CSn; the ESP32 SPI master can't, so anything
 * clocked in earlier is lost and reads back as 0xFF. At 200 kHz the slow
 * transfers hid this; at 2 MHz the boot probe could finish before the chip
 * woke up. Poll the status byte (each SNOP also pulls CSn low) until ready. */
static bool furi_hal_subghz_wait_chip_ready_locked(void) {
    for(uint32_t i = 0; i < 20; i++) {
        if(cc1101_status_is_ready(cc1101_get_status(&furi_hal_spi_bus_handle_subghz))) {
            return true;
        }
        esp_rom_delay_us(100);
    }
    return false;
}

static bool furi_hal_subghz_probe_read(FuriHalSubGhzProbe* probe, bool reset_radio) {
    furi_check(probe);
    memset(probe, 0, sizeof(*probe));

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    furi_hal_subghz_wait_chip_ready_locked();
    if(reset_radio) {
        probe->reset_status = cc1101_reset(&furi_hal_spi_bus_handle_subghz);
        furi_hal_subghz_wait_chip_ready_locked(); // SRES drops CHIP_RDYn again
    }
    probe->status = cc1101_get_status(&furi_hal_spi_bus_handle_subghz);
    probe->partnumber = cc1101_get_partnumber(&furi_hal_spi_bus_handle_subghz);
    probe->version = cc1101_get_version(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);

    return (probe->partnumber != 0x00 && probe->partnumber != 0xFF) ||
           (probe->version != 0x00 && probe->version != 0xFF);
}

static bool furi_hal_subghz_wait_for_state_impl(
    CC1101State state,
    uint32_t timeout_us,
    bool spi_locked) {
    const int64_t deadline_us = esp_timer_get_time() + timeout_us;

    do {
        if(!spi_locked) furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
        const CC1101Status status = cc1101_get_status(&furi_hal_spi_bus_handle_subghz);
        if(!spi_locked) furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
        if(cc1101_status_state(status) == state) return true;

        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if(remaining_us <= 0) break;
        const uint32_t delay_us = (remaining_us < 20) ? (uint32_t)remaining_us : 20;
        esp_rom_delay_us(delay_us);
    } while(true);

    return false;
}

bool furi_hal_subghz_wait_for_state_locked(CC1101State state, uint32_t timeout_us) {
    return furi_hal_subghz_wait_for_state_impl(state, timeout_us, true);
}

bool furi_hal_subghz_wait_for_state(CC1101State state, uint32_t timeout_us) {
    return furi_hal_subghz_wait_for_state_impl(state, timeout_us, false);
}

static void furi_hal_subghz_log_probe(const FuriHalSubGhzProbe* probe, bool connected) {
    furi_check(probe);

    if(connected) {
        ESP_LOGD(
            TAG,
            "CC1101 probe OK: part=0x%02X version=0x%02X status=0x%02X reset_status=0x%02X",
            probe->partnumber,
            probe->version,
            probe->status.raw,
            probe->reset_status.raw);
        return;
    }

    const char* hint = "unexpected signature";
    if(probe->partnumber == 0x00 && probe->version == 0x00) {
        hint = "all-zero reply, check power/CSN/SCK/MOSI";
    } else if(probe->partnumber == 0xFF && probe->version == 0xFF) {
        hint = "all-ones reply, check MISO wiring and CSN polarity";
    } else if(probe->partnumber == 0x00 && probe->version == 0xFF) {
        hint = "mixed reply, likely unstable SPI wiring";
    }

    ESP_LOGW(
        TAG,
        "CC1101 probe failed: part=0x%02X version=0x%02X status=0x%02X reset_status=0x%02X (%s)",
        probe->partnumber,
        probe->version,
        probe->status.raw,
        probe->reset_status.raw,
        hint);
}

int32_t furi_hal_subghz_get_rolling_counter_mult(void) {
    return furi_hal_subghz.rolling_counter_mult;
}

void furi_hal_subghz_set_rolling_counter_mult(int32_t mult) {
    furi_hal_subghz.rolling_counter_mult = mult;
}

void furi_hal_subghz_set_dangerous_frequency(bool state_i) {
    furi_hal_subghz.dangerous_frequency_i = state_i;
}

void furi_hal_subghz_set_async_mirror_pin(const GpioPin* pin) {
    furi_hal_subghz.async_mirror_pin = pin;
}

void furi_hal_subghz_set_ext_leds_and_amp(bool enabled) {
    furi_hal_subghz.ext_leds_and_amp = enabled;
}

bool furi_hal_subghz_get_ext_leds_and_amp(void) {
    return furi_hal_subghz.ext_leds_and_amp;
}

const GpioPin* furi_hal_subghz_get_data_gpio(void) {
    return &gpio_cc1101_g0;
}

void furi_hal_subghz_init(void) {
    /* Note: BOARD_PIN_PWR_EN is set in furi_hal_init_early() */

#if defined(BOARD_CC1101_SPI_HOST) && defined(BOARD_PIN_RADIO_SPI_MOSI) && \
    defined(BOARD_PIN_RADIO_SPI_MISO) && defined(BOARD_PIN_RADIO_SPI_SCK)
    ESP_LOGI(
        TAG,
        "Dedicated radio SPI: host=%d MOSI=%u MISO=%u SCK=%u CC1101_CSN=%u NRF24_CSN=%u",
        BOARD_CC1101_SPI_HOST,
        (unsigned)BOARD_PIN_RADIO_SPI_MOSI,
        (unsigned)BOARD_PIN_RADIO_SPI_MISO,
        (unsigned)BOARD_PIN_RADIO_SPI_SCK,
        (unsigned)BOARD_PIN_CC1101_CSN,
        (unsigned)BOARD_PIN_NRF24_CSN);
#endif

#if defined(BOARD_PIN_CC1101_SW0) && defined(BOARD_PIN_CC1101_SW1) && \
    BOARD_PIN_CC1101_SW0 != UINT16_MAX && BOARD_PIN_CC1101_SW1 != UINT16_MAX
    /* Initialize RF band-select switch GPIOs */
    static const GpioPin sw0_init = {.port = NULL, .pin = BOARD_PIN_CC1101_SW0};
    static const GpioPin sw1_init = {.port = NULL, .pin = BOARD_PIN_CC1101_SW1};
    furi_hal_gpio_init_simple(&sw0_init, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(&sw1_init, GpioModeOutputPushPull);
    /* Default to 433 MHz band (SW1=H SW0=H) */
    furi_hal_gpio_write(&sw1_init, true);
    furi_hal_gpio_write(&sw0_init, true);
#endif

    FuriHalSubGhzProbe probe = {0};
    furi_hal_spi_bus_handle_init(&furi_hal_spi_bus_handle_subghz);
#if FURI_HAL_SUBGHZ_HAS_AW9523
    furi_hal_aw9523_init();
#endif
    furi_hal_subghz.connected = furi_hal_subghz_probe_read(&probe, true);
    furi_hal_subghz_log_probe(&probe, furi_hal_subghz.connected);
    furi_hal_subghz.state = FuriHalSubGhzStateIdle;

    /* Park the radio in SPWD right away. The probe above leaves the CC1101 in
     * IDLE (roughly 1.5-2 mA) and, until a Sub-GHz app is opened and closed,
     * nothing else ever puts it to sleep. Every Sub-GHz app already calls
     * furi_hal_subghz_sleep() on exit, so this is the same known-good state the
     * chip is in after any app; apps reconfigure it from sleep on entry. The
     * PWR_EN rail itself must stay up because it also feeds the fuel gauge. */
    if(furi_hal_subghz.connected) {
#if FURI_HAL_SUBGHZ_HAS_AW9523
        if(!furi_hal_aw9523_set_idle()) {
            ESP_LOGW(TAG, "AW9523 front-end idle state could not be verified");
        }
#endif
        furi_hal_subghz_sleep();
    }
}

void furi_hal_subghz_sleep(void) {
    if(furi_hal_subghz.state == FuriHalSubGhzStateInit) furi_hal_subghz_init();

    /* Sleep is also used by application teardown. Stop asynchronous
     * peripherals first so no GPIO/RMT callback can access the CC1101 while
     * it is being powered down. */
    if(furi_hal_subghz.state == FuriHalSubGhzStateAsyncRx) {
        furi_hal_subghz_stop_async_rx();
    } else if(furi_hal_subghz.state == FuriHalSubGhzStateAsyncTx) {
        furi_hal_subghz_stop_async_tx();
    }

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    /* The chip may already be in SLEEP (app teardown, then shutdown): strobes
     * clocked in before CHIP_RDYn goes low are lost, so the SPWD below would
     * silently not happen and the radio would stay in IDLE (~1.7 mA). */
    furi_hal_subghz_wait_chip_ready_locked();
    cc1101_switch_to_idle(&furi_hal_spi_bus_handle_subghz);
    /* SWRS061I section 4.4: the 0.2 uA SLEEP current is specified with every GDO
     * programmed to 0x2F (HW to 0). GDO0/GDO2 then drive low in SLEEP, matching
     * the MCU-side pull-down, instead of floating (0x2E) or toggling. IOCFG1 is
     * left alone: GDO1 is the shared SPI MISO and must stay 3-state. IOCFGx are
     * retained in SLEEP; apps reload them after SRES (furi_hal_subghz_reset). */
    cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, CC1101_IOCFG0, CC1101IocfgHW);
    cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, CC1101_IOCFG2, CC1101IocfgHW);
    /* SPWD takes effect when CSn goes high, i.e. on the release below. */
    cc1101_shutdown(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
#if FURI_HAL_SUBGHZ_HAS_AW9523
    (void)furi_hal_aw9523_set_idle();
#endif
    furi_hal_subghz.state = FuriHalSubGhzStateIdle;
}

void furi_hal_subghz_dump_state(void) {
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    printf(
        "[furi_hal_subghz] cc1101 chip %u, version %u\r\n",
        cc1101_get_partnumber(&furi_hal_spi_bus_handle_subghz),
        cc1101_get_version(&furi_hal_spi_bus_handle_subghz));
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_load_custom_preset(const uint8_t* preset_data) {
    furi_check(preset_data);
    furi_hal_subghz_reset();

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    uint32_t i = 0;
    uint8_t pa[8] = {0};
    while(preset_data[i]) {
        cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, preset_data[i], preset_data[i + 1]);
        i += 2;
    }
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);

    memcpy(pa, &preset_data[i + 2], sizeof(pa));
    furi_hal_subghz_load_patable(pa);
}

void furi_hal_subghz_load_registers(const uint8_t* data) {
    furi_check(data);
    furi_hal_subghz_reset();

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    uint32_t i = 0;
    while(data[i]) {
        cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, data[i], data[i + 1]);
        i += 2;
    }
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_load_patable(const uint8_t data[8]) {
    furi_check(data);
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_set_pa_table(&furi_hal_spi_bus_handle_subghz, data);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_write_packet(const uint8_t* data, uint8_t size) {
    furi_check(data);
    furi_check(size);
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_flush_tx(&furi_hal_spi_bus_handle_subghz);
    cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, CC1101_FIFO, size);
    cc1101_write_fifo(&furi_hal_spi_bus_handle_subghz, data, size);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

bool furi_hal_subghz_rx_pipe_not_empty(void) {
    CC1101RxBytes status = {0};
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_read_reg(
        &furi_hal_spi_bus_handle_subghz,
        (CC1101_STATUS_RXBYTES) | CC1101_BURST,
        &status.raw);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    return cc1101_rx_bytes_available(status) > 0;
}

bool furi_hal_subghz_is_rx_data_crc_valid(void) {
    uint8_t data = 0;
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_read_reg(&furi_hal_spi_bus_handle_subghz, CC1101_STATUS_LQI | CC1101_BURST, &data);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    return ((data >> 7) & 0x01) != 0;
}

void furi_hal_subghz_read_packet(uint8_t* data, uint8_t* size) {
    furi_check(data);
    furi_check(size);
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_read_fifo(&furi_hal_spi_bus_handle_subghz, data, size);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_flush_rx(void) {
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_flush_rx(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_flush_tx(void) {
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_flush_tx(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
}

void furi_hal_subghz_shutdown(void) {
    furi_hal_subghz_sleep();
}

void furi_hal_subghz_reset(void) {
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    furi_hal_subghz_wait_chip_ready_locked();
    cc1101_switch_to_idle(&furi_hal_spi_bus_handle_subghz);
    cc1101_reset(&furi_hal_spi_bus_handle_subghz);
    furi_hal_subghz_wait_chip_ready_locked(); // SRES drops CHIP_RDYn again
    cc1101_write_reg(&furi_hal_spi_bus_handle_subghz, CC1101_IOCFG0, CC1101IocfgHighImpedance);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    FuriHalSubGhzProbe probe = {0};
    furi_hal_subghz.connected = furi_hal_subghz_probe_read(&probe, false);
}

void furi_hal_subghz_idle(void) {
#if FURI_HAL_SUBGHZ_HAS_AW9523
    (void)furi_hal_aw9523_set_idle();
#endif
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_switch_to_idle(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    if(!furi_hal_subghz_wait_for_state(CC1101StateIDLE, 10000)) {
        ESP_LOGW(TAG, "CC1101 did not reach IDLE");
    }
    furi_hal_subghz.state = FuriHalSubGhzStateIdle;
}

bool furi_hal_subghz_rx(void) {
    if(!furi_hal_subghz.connected) {
        ESP_LOGE(TAG, "Cannot enter RX: CC1101 is not connected");
        return false;
    }
#if FURI_HAL_SUBGHZ_HAS_AW9523
    const bool rf_frontend_ok = furi_hal_aw9523_set_rx();
    if(!rf_frontend_ok) {
        ESP_LOGE(TAG, "Refusing CC1101 RX: AW9523 RXEN was not verified");
        return false;
    }
#endif
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_switch_to_rx(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    const bool radio_ok = furi_hal_subghz_wait_for_state(CC1101StateRX, 10000);
#if FURI_HAL_SUBGHZ_HAS_AW9523
    ESP_LOGI(TAG, "CC1101 RX %s (AW9523 RXEN verified)", radio_ok ? "active" : "failed");
#else
    ESP_LOGI(TAG, "CC1101 RX %s", radio_ok ? "active" : "failed");
#endif
#if FURI_HAL_SUBGHZ_HAS_AW9523
    if(!radio_ok) (void)furi_hal_aw9523_set_idle();
#endif
    return radio_ok;
}

bool furi_hal_subghz_tx(void) {
    if(furi_hal_subghz.regulation != FuriHalSubGhzRegulationTxRx) return false;
    if(!furi_hal_subghz.connected) {
        ESP_LOGE(TAG, "Cannot enter TX: CC1101 is not connected");
        return false;
    }

#if FURI_HAL_SUBGHZ_HAS_AW9523
    const bool rf_frontend_ok = furi_hal_aw9523_set_tx();
    if(!rf_frontend_ok) {
        ESP_LOGE(TAG, "Refusing CC1101 TX: AW9523 TXEN was not verified");
        return false;
    }
#endif
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_switch_to_tx(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    const bool ok = furi_hal_subghz_wait_for_state(CC1101StateTX, 10000);
#if FURI_HAL_SUBGHZ_HAS_AW9523
    ESP_LOGI(TAG, "CC1101 TX %s (AW9523 TXEN verified)", ok ? "active" : "failed");
#else
    ESP_LOGI(TAG, "CC1101 TX %s", ok ? "active" : "failed");
#endif
#if FURI_HAL_SUBGHZ_HAS_AW9523
    if(!ok) (void)furi_hal_aw9523_set_idle();
#endif
    return ok;
}

float furi_hal_subghz_get_rssi(void) {
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    int32_t rssi_dec = cc1101_get_rssi(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);

    float rssi = rssi_dec;
    if(rssi_dec >= 128) {
        rssi = ((rssi - 256.0f) / 2.0f) - 74.0f;
    } else {
        rssi = (rssi / 2.0f) - 74.0f;
    }
    return rssi;
}

uint8_t furi_hal_subghz_get_lqi(void) {
    uint8_t data = 0;
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_read_reg(&furi_hal_spi_bus_handle_subghz, CC1101_STATUS_LQI | CC1101_BURST, &data);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    return data & 0x7F;
}

bool furi_hal_subghz_is_frequency_valid(uint32_t value) {
    return ((value >= 281000000 && value <= 361000000) ||
            (value >= 378000000 && value <= 481000000) ||
            (value >= 749000000 && value <= 962000000));
}

uint32_t furi_hal_subghz_set_frequency_and_path(uint32_t value) {
    value = furi_hal_subghz_set_frequency(value);
    if(value >= 281000000 && value <= 361000000) {
        furi_hal_subghz_set_path(FuriHalSubGhzPath315);
    } else if(value >= 378000000 && value <= 481000000) {
        furi_hal_subghz_set_path(FuriHalSubGhzPath433);
    } else if(value >= 749000000 && value <= 962000000) {
        furi_hal_subghz_set_path(FuriHalSubGhzPath868);
    } else {
        furi_hal_subghz_set_path(FuriHalSubGhzPathIsolate);
    }
    return value;
}

bool furi_hal_subghz_is_tx_allowed(uint32_t value) {
    if(!furi_hal_subghz.dangerous_frequency_i &&
       !(value >= 299999755 && value <= 350000335) &&
       !(value >= 386999938 && value <= 467750000) &&
       !(value >= 778999847 && value <= 928000000)) {
        return false;
    }

    if(furi_hal_subghz.dangerous_frequency_i && !furi_hal_subghz_is_frequency_valid(value)) {
        return false;
    }

    return true;
}

uint32_t furi_hal_subghz_set_frequency(uint32_t value) {
    furi_hal_subghz.regulation = furi_hal_subghz_is_tx_allowed(value) ?
                                     FuriHalSubGhzRegulationTxRx :
                                     FuriHalSubGhzRegulationOnlyRx;

    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    uint32_t real_frequency = cc1101_set_frequency(&furi_hal_spi_bus_handle_subghz, value);
    cc1101_calibrate(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    if(!furi_hal_subghz_wait_for_state(CC1101StateIDLE, 10000)) {
        ESP_LOGW(TAG, "CC1101 calibration did not reach IDLE");
    }

    ESP_LOGD(TAG, "set_frequency: requested=%lu real=%lu", (unsigned long)value, (unsigned long)real_frequency);
    furi_hal_subghz.frequency = real_frequency;
    return real_frequency;
}

void furi_hal_subghz_set_path(FuriHalSubGhzPath path) {
#if defined(BOARD_PIN_CC1101_SW0) && defined(BOARD_PIN_CC1101_SW1) && \
    BOARD_PIN_CC1101_SW0 != UINT16_MAX && BOARD_PIN_CC1101_SW1 != UINT16_MAX
    /* T-Embed CC1101 band-select RF switch:
     * SW1=H SW0=L → 315 MHz
     * SW1=L SW0=H → 868/915 MHz
     * SW1=H SW0=H → 434 MHz */
    static const GpioPin sw0 = {.port = NULL, .pin = BOARD_PIN_CC1101_SW0};
    static const GpioPin sw1 = {.port = NULL, .pin = BOARD_PIN_CC1101_SW1};
    static FuriHalSubGhzPath current_path = FuriHalSubGhzPathIsolate;
    bool changed = (path != current_path);
    switch(path) {
    case FuriHalSubGhzPath315:
        furi_hal_gpio_write(&sw1, true);
        furi_hal_gpio_write(&sw0, false);
        break;
    case FuriHalSubGhzPath868:
        furi_hal_gpio_write(&sw1, false);
        furi_hal_gpio_write(&sw0, true);
        break;
    case FuriHalSubGhzPath433:
        furi_hal_gpio_write(&sw1, true);
        furi_hal_gpio_write(&sw0, true);
        break;
    default:
        furi_hal_gpio_write(&sw1, false);
        furi_hal_gpio_write(&sw0, false);
        break;
    }
    if(changed) {
        /* 10ms settlement delay for antenna impedance matching */
        furi_delay_ms(10);
        current_path = path;
    }
#else
    UNUSED(path);
#endif
}

void furi_hal_subghz_start_async_rx(FuriHalSubGhzCaptureCallback callback, void* context) {
    furi_check(callback);
    if(furi_hal_subghz.state != FuriHalSubGhzStateIdle) {
        ESP_LOGW(TAG, "Cannot start async RX while Sub-GHz state is %d", furi_hal_subghz.state);
        return;
    }
    if(!furi_hal_subghz.connected) {
        ESP_LOGE(TAG, "Cannot start async RX: CC1101 is not connected");
        return;
    }

    /* Hold insomnia for the whole RX session. The CC1101 GDO0 capture is a plain
     * GPIO interrupt that would NOT fire during light sleep (it is not a
     * configured wake source), so we must keep the SoC awake, and at full clock,
     * while receiving. Paired with the exit in stop_async_rx. */
    furi_hal_power_insomnia_enter();

    ESP_LOGD(TAG, "start_async_rx: GDO0=GPIO%d freq=%lu connected=%d",
        gpio_cc1101_g0.pin, (unsigned long)furi_hal_subghz.frequency, furi_hal_subghz.connected);

    furi_hal_subghz.async_rx_callback = callback;
    furi_hal_subghz.async_rx_context = context;
    furi_hal_subghz.async_rx_last_level = furi_hal_gpio_read(&gpio_cc1101_g0);
    furi_hal_subghz.async_rx_last_timestamp = (uint32_t)esp_timer_get_time();
    furi_hal_subghz.state = FuriHalSubGhzStateAsyncRx;

    furi_hal_gpio_init(&gpio_cc1101_g0, GpioModeInterruptRiseFall, GpioPullNo, GpioSpeedLow);
    furi_hal_gpio_add_int_callback(
        &gpio_cc1101_g0, furi_hal_subghz_capture_gpio_callback, NULL);
    if(!furi_hal_subghz_rx()) {
        furi_hal_gpio_remove_int_callback(&gpio_cc1101_g0);
        furi_hal_gpio_init(&gpio_cc1101_g0, GpioModeInput, GpioPullNo, GpioSpeedLow);
        furi_hal_subghz.async_rx_callback = NULL;
        furi_hal_subghz.async_rx_context = NULL;
        furi_hal_subghz.state = FuriHalSubGhzStateIdle;
        furi_hal_power_insomnia_exit();
        ESP_LOGE(TAG, "Async RX start aborted: radio or RF front-end setup failed");
        return;
    }

    /* Read CC1101 status to verify RX mode */
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    CC1101Status status = cc1101_get_status(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);
    ESP_LOGD(TAG, "start_async_rx: CC1101 status=0x%02X state=%d gdo0_level=%d irq_count=%lu",
        status.raw, cc1101_status_state(status),
        furi_hal_gpio_read(&gpio_cc1101_g0),
        (unsigned long)furi_hal_subghz_rx_irq_count);
}

void furi_hal_subghz_stop_async_rx(void) {
    if(furi_hal_subghz.state != FuriHalSubGhzStateAsyncRx) return;

    furi_hal_gpio_remove_int_callback(&gpio_cc1101_g0);
    furi_hal_gpio_init(&gpio_cc1101_g0, GpioModeInput, GpioPullNo, GpioSpeedLow);
    furi_hal_subghz.async_rx_callback = NULL;
    furi_hal_subghz.async_rx_context = NULL;
    furi_hal_subghz.async_rx_last_timestamp = 0;
    furi_hal_subghz.async_rx_last_level = false;
    furi_hal_subghz.state = FuriHalSubGhzStateIdle;
    furi_hal_subghz_idle();

    furi_hal_power_insomnia_exit();
}

bool furi_hal_subghz_start_async_tx(FuriHalSubGhzAsyncTxCallback callback, void* context) {
    furi_check(furi_hal_subghz.state == FuriHalSubGhzStateIdle);
    furi_check(callback);

    if(furi_hal_subghz.regulation != FuriHalSubGhzRegulationTxRx) return false;

    FuriHalSubGhzAsyncTx* async_tx = &furi_hal_subghz.async_tx;
    furi_hal_power_insomnia_enter();
    async_tx->sleep_inhibited = true;
    async_tx->callback = callback;
    async_tx->callback_context = context;
    furi_hal_subghz_async_tx_prepare(async_tx);
    furi_hal_subghz.async_tx_complete = false;
    furi_hal_subghz.state = FuriHalSubGhzStateAsyncTx;

    furi_hal_gpio_remove_int_callback(&gpio_cc1101_g0);

    if(furi_hal_subghz.async_mirror_pin) {
        furi_hal_gpio_init(
            furi_hal_subghz.async_mirror_pin, GpioModeOutputPushPull, GpioPullNo, GpioSpeedLow);
        furi_hal_gpio_write(furi_hal_subghz.async_mirror_pin, false);
    }

    esp_err_t error = furi_hal_subghz_async_tx_alloc_backend(async_tx);
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RMT TX backend: %s", esp_err_to_name(error));
        furi_hal_subghz_async_tx_free_backend(async_tx);
        async_tx->callback = NULL;
        async_tx->callback_context = NULL;
        furi_hal_subghz.async_tx_complete = true;
        furi_hal_subghz.state = FuriHalSubGhzStateIdle;
        furi_hal_subghz_async_tx_release_sleep_inhibit(async_tx);
        return false;
    }

    if(!furi_hal_subghz_tx()) {
        async_tx->callback = NULL;
        async_tx->callback_context = NULL;
        furi_hal_subghz.async_tx_complete = true;
        furi_hal_subghz.state = FuriHalSubGhzStateIdle;
        furi_hal_subghz_async_tx_free_backend(async_tx);
        furi_hal_subghz_async_tx_release_sleep_inhibit(async_tx);
        return false;
    }

    error = rmt_encoder_reset(async_tx->encoder);
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset RMT encoder: %s", esp_err_to_name(error));
        furi_hal_subghz_idle();
        furi_hal_subghz_async_tx_free_backend(async_tx);
        async_tx->callback = NULL;
        async_tx->callback_context = NULL;
        furi_hal_subghz.async_tx_complete = true;
        furi_hal_subghz.state = FuriHalSubGhzStateIdle;
        furi_hal_subghz_async_tx_release_sleep_inhibit(async_tx);
        return false;
    }

    /* rmt_transmit() validates the source payload before invoking the
     * encoder. Keep the marker in internal RAM; a static const object would
     * normally be placed in flash and is rejected by the RMT driver. */
    static DRAM_ATTR uint8_t tx_marker = 0xA5;
    const rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags =
            {
                .eot_level = 0,
                .queue_nonblocking = 0,
            },
    };
    error = rmt_transmit(async_tx->channel, async_tx->encoder, &tx_marker, sizeof(tx_marker), &tx_config);
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT transmission: %s", esp_err_to_name(error));
        furi_hal_subghz_idle();
        furi_hal_subghz_async_tx_free_backend(async_tx);
        async_tx->callback = NULL;
        async_tx->callback_context = NULL;
        furi_hal_subghz.async_tx_complete = true;
        furi_hal_subghz.state = FuriHalSubGhzStateIdle;
        furi_hal_subghz_async_tx_release_sleep_inhibit(async_tx);
        return false;
    }

    return true;
}

bool furi_hal_subghz_is_async_tx_complete(void) {
    return furi_hal_subghz.async_tx_complete;
}

void furi_hal_subghz_stop_async_tx(void) {
    if(furi_hal_subghz.state != FuriHalSubGhzStateAsyncTx) {
        return;
    }

    FuriHalSubGhzAsyncTx* async_tx = &furi_hal_subghz.async_tx;
    async_tx->stop_requested = true;

    /* Leave TX before letting go of GDO0: in asynchronous TX the CC1101 keys
       the carrier from that pin, so releasing it first could transmit
       whatever level the floating input settles at until IDLE is reached. */
    furi_hal_spi_acquire(&furi_hal_spi_bus_handle_subghz);
    cc1101_switch_to_idle(&furi_hal_spi_bus_handle_subghz);
    furi_hal_spi_release(&furi_hal_spi_bus_handle_subghz);

    /* free_backend() disables the channel, which also aborts a transmission
       still in flight — no separate abort call (it would fail with
       INVALID_STATE and make the RMT driver log an error). */
    furi_hal_subghz_async_tx_free_backend(async_tx);

    /* Pull down so the next TX entry starts with the carrier keyed off. */
    furi_hal_gpio_init(&gpio_cc1101_g0, GpioModeInput, GpioPullDown, GpioSpeedLow);
    if(furi_hal_subghz.async_mirror_pin) {
        furi_hal_gpio_write(furi_hal_subghz.async_mirror_pin, false);
    }

    async_tx->callback = NULL;
    async_tx->callback_context = NULL;
    async_tx->sequence_complete = true;
    async_tx->remaining_duration = 0;
    async_tx->remaining_level = true;
    async_tx->stop_requested = false;

    if((async_tx->duty_low + async_tx->duty_high) > 0) {
        float duty_cycle = 100.0f * (float)async_tx->duty_high /
                           ((float)async_tx->duty_low + (float)async_tx->duty_high);
        ESP_LOGD(
            TAG,
            "Async TX radio stats: on %.0fus, off %.0fus, duty %.0f%%",
            (double)async_tx->duty_high,
            (double)async_tx->duty_low,
            (double)duty_cycle);
    }

    furi_hal_subghz.async_tx_complete = true;
    furi_hal_subghz.state = FuriHalSubGhzStateIdle;
    furi_hal_subghz_idle();
    furi_hal_subghz_async_tx_release_sleep_inhibit(async_tx);
}
