#pragma once

#ifdef AUDIO_ENABLE
#define STARTUP_SONG SONG(PLANCK_SOUND)
#endif

#define MIDI_BASIC

#define ENCODER_RESOLUTION 4

/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/

#define ORYX_CONFIGURATOR
#undef DEBOUNCE
#define DEBOUNCE 5

#undef TAPPING_TERM
#define TAPPING_TERM 147

#define COMBO_TERM 30

#undef RGB_MATRIX_TIMEOUT
#define RGB_MATRIX_TIMEOUT 180000

#define USB_SUSPEND_WAKEUP_DELAY 0
#undef MOUSEKEY_INTERVAL
#define MOUSEKEY_INTERVAL 10

#undef MOUSEKEY_WHEEL_DELAY
#define MOUSEKEY_WHEEL_DELAY 100

#undef MOUSEKEY_MAX_SPEED
#define MOUSEKEY_MAX_SPEED 3

#undef MOUSEKEY_TIME_TO_MAX
#define MOUSEKEY_TIME_TO_MAX 45

#undef MOUSEKEY_WHEEL_INTERVAL
#define MOUSEKEY_WHEEL_INTERVAL 120

#undef MOUSEKEY_WHEEL_MAX_SPEED
#define MOUSEKEY_WHEEL_MAX_SPEED 2

#define CAPS_LOCK_STATUS
#define FIRMWARE_VERSION u8"Jq07z/bW6p7"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define LAYER_STATE_16BIT
#define HCS(report) host_consumer_send(record->event.pressed ? report : 0); return false

#define RGB_MATRIX_STARTUP_SPD 60

