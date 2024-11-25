#include QMK_KEYBOARD_H
#ifdef AUDIO_ENABLE
#include "muse.h"
#endif
#include "eeprom.h"

enum planck_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  MAC_LOCK,
};


enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
};

enum planck_layers {
  _LAYER0,
  _BASE,
  _LOWER,
  _RAISE,
  _ADJUST,
  _LAYER5,
  _LAYER6,
  _LAYER7,
  _LAYER8,
};

#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

#define THUMB0 GUI_T(KC_BSPC)
#define THUMB1 CTL_T(KC_SPACE)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [_LAYER0] = LAYOUT_planck_grid(
    KC_Q,         KC_W,  KC_E,           KC_R,                KC_T,            KC_0,          KC_3,     KC_Y,        KC_U,              KC_I,           KC_O,           KC_P,
    KC_A,         KC_S,  LT(6,KC_D),     LT(7,KC_F),          KC_G,            KC_1,          KC_4,     KC_H,        GUI_T(KC_J),       CTL_T(KC_K),    ALT_T(KC_L),    KC_SCLN,
    SFT_T(KC_Z),  KC_X,  KC_C,           ALT_T(KC_V),         KC_B,            KC_2,          KC_5,     KC_N,        KC_M,              TG(5),          KC_DOT,         KC_RIGHT_SHIFT, 
    TG(3),        TG(8), LT(2,KC_ESCAPE),THUMB0,              THUMB1,          KC_9,          KC_NO,    RAISE,       KC_ENTER,          C(KC_SPACE),    KC_CAPS,        TG(1)
  ),

  [_BASE] = LAYOUT_planck_grid(
    KC_Q,           KC_Z,           KC_F,           KC_P,           KC_V,           _______,  _______, KC_J,           KC_L,           KC_U,           KC_Y,           KC_X,
    KC_A,           KC_R,           LT(6,KC_S),     LT(7,KC_T),     KC_G,           _______,  _______, KC_K,           GUI_T(KC_N),    CTL_T(KC_E),    ALT_T(KC_I),    KC_O,
    KC_LSFT,        KC_W,           KC_C,           ALT_T(KC_D),    KC_B,           _______,  _______, KC_M,           KC_H,           TG(5),          KC_DOT,         KC_RSFT,
    _______,        _______,        _______,        _______,        _______,        _______,  KC_NO,   RAISE,          LT(_LOWER, KC_ENTER),_______,   _______,        _______
  ),

  [_LOWER] = LAYOUT_planck_grid(
    KC_SCRL,        KC_INSERT,      KC_F11,         KC_F12,         KC_PSCR,        _______, _______, _______, _______, _______, _______, _______, 
    KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          _______, _______, _______, _______, _______, _______, _______, 
    KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         _______, _______, _______, _______, _______, _______, _______, 
    _______,        _______,        _______,        KC_DELETE,      _______,        _______, KC_NO,   _______, _______, _______, _______, _______
  ),

  [_RAISE] = LAYOUT_planck_grid(
    KC_LBRC,        KC_RBRC,        KC_LPRN,        KC_RPRN,        KC_DQUO,        _______, _______, KC_SCLN,        KC_7,           KC_8,           KC_9,           KC_BSLS,        
    KC_QUOTE,       KC_DOT,         KC_COMMA,       KC_0,           KC_LCBR,        _______, _______, KC_RCBR,        KC_1,           KC_2,           KC_3,           KC_COLN,        
    KC_LSFT,        KC_UNDS,        KC_MINUS,       KC_EQUAL,       KC_GRAVE,       _______, _______, KC_SLASH,       KC_4,           KC_5,           KC_6,           KC_RSFT, 
    _______, _______, KC_TAB,         _______, _______, _______, KC_NO,          _______, _______, _______, _______, TO(1)
  ),

  [_ADJUST] = LAYOUT_planck_grid(
    RGB_TOG,        RGB_MODE_FORWARD,RGB_VAD,        RGB_VAI,        _______, _______, _______, KC_AUDIO_MUTE,  KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,LED_LEVEL,      MAC_LOCK,       
    _______, _______, _______, _______, _______, _______, _______, KC_MEDIA_PREV_TRACK,KC_MEDIA_PLAY_PAUSE,KC_MEDIA_NEXT_TRACK,KC_BRIGHTNESS_DOWN,KC_BRIGHTNESS_UP,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_SYSTEM_SLEEP,
    _______, _______, _______, _______, _______, _______, KC_NO,          _______, _______, _______, _______, _______
  ),

  // MOUSE
  [_LAYER5] = LAYOUT_planck_grid(
    KC_NO,          KC_NO,          KC_MS_UP,       KC_NO,          KC_NO,          _______, _______, KC_NO,          KC_NO,          KC_MS_WH_UP,    KC_NO,          KC_NO,          
    KC_NO,          KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_NO,          _______, _______, KC_NO,          KC_MS_BTN1,     KC_MS_WH_DOWN,  KC_MS_BTN2,     KC_NO,          
    KC_LEFT_SHIFT,  KC_LEFT_ALT,    KC_NO,          KC_NO,          KC_NO,          _______, _______, KC_NO,          KC_MS_WH_LEFT,  _______, KC_MS_WH_RIGHT, KC_NO,          
    _______, _______, _______, KC_LEFT_CTRL,   KC_LEFT_ALT,    _______, KC_NO,          KC_MS_ACCEL2,   _______, _______, _______, TO(1)
  ),

  // ARROWS
  [_LAYER6] = LAYOUT_planck_grid(
    KC_NO,          KC_NO,          KC_NO,     KC_NO,    KC_NO,          _______, _______, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_LGUI,        KC_LALT,        _______,   KC_LSFT,  KC_NO,          _______, _______, KC_NO,          KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       
    KC_NO,          KC_NO,          KC_NO,     KC_NO,    KC_NO,          _______, _______, KC_NO,          KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         
    TO(0),          _______,        _______,   _______,  KC_LCTL,  _______, KC_NO,          _______, _______, _______, _______, TO(1)
  ),

  // SYMBOLS
  [_LAYER7] = LAYOUT_planck_grid(
    KC_LBRC,        KC_RBRC,        KC_LPRN,        KC_RPRN,        KC_QUOTE,       _______, _______, KC_SCLN,        KC_AMPR,        KC_ASTR,        KC_NO,          KC_PIPE,        
    KC_DQUO,        KC_DOT,         KC_COMMA,       _______, _______, _______, _______, KC_RCBR,        KC_EXLM,        KC_AT,          KC_HASH,        KC_COLN,        
    _______, KC_UNDS,        KC_MINUS,       _______, _______, _______, _______, KC_QUES,        KC_DLR,         KC_PERC,        KC_CIRC,        _______, 
    _______, _______, _______, _______, _______, _______, KC_NO,          _______, _______, _______, _______, _______
  ),

  // WITHMOUSE
  [_LAYER8] = LAYOUT_planck_grid(
    KC_F1,          TD(DANCE_0),    TD(DANCE_1),    KC_F4,          KC_F5,          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         KC_F12,         
    KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           _______,        _______,        _______,        KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       
    SFT_T(KC_6),    KC_7,           KC_8,           ALT_T(KC_9),    KC_0,           _______,        _______,        _______,        KC_HOME,        KC_PGDN,        KC_PGUP,        KC_END,         
    _______,        _______,        _______,        KC_LCTL,        KC_LGUI,        _______,        KC_NO,          _______,        _______,        _______,        _______,        _______         
  ),

};


const uint16_t PROGMEM tab_combo[] = {KC_F, LT(7,KC_T), COMBO_END};
const uint16_t PROGMEM left_tab_combo[] = {KC_C, THUMB0, COMBO_END};
const uint16_t PROGMEM right_tab_combo[] = {MT(MOD_LALT,KC_D), THUMB0, COMBO_END};
const uint16_t PROGMEM left_workspace_combo[] = {LT(6,KC_S), MT(MOD_LALT,KC_D), THUMB0, COMBO_END};
const uint16_t PROGMEM right_workspace_combo[] = {LT(6,KC_S), MT(MOD_LALT,KC_D), THUMB1, COMBO_END};
const uint16_t PROGMEM mission_control_combo[] = {KC_F, LT(7,KC_T), THUMB0, COMBO_END};

combo_t key_combos[] = {
    COMBO(tab_combo, KC_TAB),
    COMBO(left_tab_combo, S(C(KC_TAB))),
    COMBO(right_tab_combo, C(KC_TAB)),
    COMBO(left_workspace_combo, G(C(KC_LEFT))),
    COMBO(right_workspace_combo, G(C(KC_RIGHT))),
    COMBO(mission_control_combo, KC_MISSION_CONTROL),
};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {0,86,209}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234}, {243,222,234} },

    [1] = { {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {84,110,178}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184}, {85,205,184} },

    [3] = { {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {31,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255}, {10,255,255} },

    [5] = { {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,18,251}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204}, {195,212,204} },

    [8] = { {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,255,255}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137}, {131,240,137} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 5:
      set_layer_color(5);
      break;
    case 8:
      set_layer_color(8);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case MAC_LOCK:
      HCS(0x19E);

    case RGB_SLD:
        if (rawhid_state.rgb_control) {
            return false;
        }
        if (record->event.pressed) {
            rgblight_mode(1);
        }
        return false;
  }
  return true;
}

#ifdef AUDIO_ENABLE
bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
    if (muse_mode) {
        if (IS_LAYER_ON(_RAISE)) {
            if (clockwise) {
                muse_offset++;
            } else {
                muse_offset--;
            }
        } else {
            if (clockwise) {
                muse_tempo+=1;
            } else {
                muse_tempo-=1;
            }
        }
    } else {
        if (clockwise) {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_DOWN);
            unregister_code(KC_MS_WH_DOWN);
        #else
            register_code(KC_PGDN);
            unregister_code(KC_PGDN);
        #endif
        } else {
        #ifdef MOUSEKEY_ENABLE
            register_code(KC_MS_WH_UP);
            unregister_code(KC_MS_WH_UP);
        #else
            register_code(KC_PGUP);
            unregister_code(KC_PGUP);
        #endif
        }
    }
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
    switch (keycode) {
    case RAISE:
    case LOWER:
        return false;
    default:
        return true;
    }
}
#endif

uint16_t layer_state_set_user(uint16_t state) {
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[2];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
        tap_code16(KC_GRAVE);
    }
    if(state->count > 3) {
        tap_code16(KC_GRAVE);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_GRAVE); break;
        case SINGLE_HOLD: register_code16(KC_F2); break;
        case DOUBLE_TAP: register_code16(KC_GRAVE); register_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_GRAVE); register_code16(KC_GRAVE);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_GRAVE); break;
        case SINGLE_HOLD: unregister_code16(KC_F2); break;
        case DOUBLE_TAP: unregister_code16(KC_GRAVE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_GRAVE); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case SINGLE_HOLD: register_code16(KC_F3); break;
        case DOUBLE_TAP: register_code16(KC_TAB); register_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case SINGLE_HOLD: unregister_code16(KC_F3); break;
        case DOUBLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[1].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
};
