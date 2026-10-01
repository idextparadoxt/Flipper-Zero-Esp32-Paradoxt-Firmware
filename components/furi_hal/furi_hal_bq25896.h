/**
 * @file furi_hal_bq25896.h
 * BQ25896 charger driver (I2C) — ported from STM32 Flipper firmware
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool furi_hal_bq25896_init(void);
bool furi_hal_bq25896_is_present(void);

bool furi_hal_bq25896_is_charging(void);
bool furi_hal_bq25896_is_charging_done(void);

void furi_hal_bq25896_enable_charging(void);
void furi_hal_bq25896_disable_charging(void);

uint16_t furi_hal_bq25896_get_vbus_voltage_mv(void);
uint16_t furi_hal_bq25896_get_vbat_voltage_mv(void);
uint16_t furi_hal_bq25896_get_vbat_current_ma(void);
int32_t furi_hal_bq25896_get_temperature_mc(void); /* milli-celsius */

uint16_t furi_hal_bq25896_get_vreg_voltage_mv(void);
void furi_hal_bq25896_set_vreg_voltage_mv(uint16_t vreg_mv);

void furi_hal_bq25896_enable_otg(void);
void furi_hal_bq25896_disable_otg(void);
bool furi_hal_bq25896_is_otg_enabled(void);

/** True if the charger is currently powered from VBUS (USB attached). */
bool furi_hal_bq25896_is_vbus_present(void);

/** Read VBUS state. Returns false on I2C/read error; *present is valid only
 * when the return value is true. */
bool furi_hal_bq25896_get_vbus_present(bool* present);

/** Read only VBUS_GD (one I2C transaction). Returns false on I2C error;
 * *present is valid only when the return value is true. */
bool furi_hal_bq25896_read_vbus_good(bool* present);

/** Battery monitor ADC: true = 1 s continuous conversion, false = off.
 * The ADC keeps REGN powered and raises quiescent current, so it is switched
 * off while the device light-sleeps. VBUS_GD stays valid with the ADC off. */
bool furi_hal_bq25896_set_adc_continuous(bool enable);

/** Enter ship mode (BATFET off): physically disconnect the battery for a real
 *  power-off. Only effective on battery — with USB attached SYS stays powered. */
bool furi_hal_bq25896_poweroff(void);

/** Read live ICO/DPM status (REG13/REG14). Returns false on I2C error or if
 * the charger is not present; all outputs are valid only when true.
 * vdpm/idpm: charger is currently regulating VBUS/IINDPM (source is weak).
 * iinlim_ma: effective input current limit (post-ICO if ico_done). */
bool furi_hal_bq25896_get_dpm_status(bool* vdpm, bool* idpm, uint16_t* iinlim_ma, bool* ico_done);

/** Enable/disable Input Current Optimizer (REG02 bit 4). Enabled by default
 * in furi_hal_bq25896_init(); exposed for runtime diagnostics/override. */
bool furi_hal_bq25896_set_ico(bool enable);

/** Configure the charger to keep charging autonomously while the MCU is in
 * deep sleep (CHG_CONFIG=1, OTG off, EN_HIZ=0, watchdog disabled), verified
 * by register read-back. Call immediately before deep sleep, before any I2C
 * pin is isolated/held. Returns false if not present or verification failed —
 * caller should log a warning but may still proceed to sleep. */
bool furi_hal_bq25896_prepare_for_sleep(void);

#ifdef __cplusplus
}
#endif
