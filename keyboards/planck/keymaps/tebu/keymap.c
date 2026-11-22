#include QMK_KEYBOARD_H
#include "eeprom.h"

enum custom_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  MAC_LCK,  // macOS lock screen
  JA_SW,    // toggle: when on, LG_SPC sends Alt+` (Windows IME switch) instead of Gui+Space
  LG_SPC,   // Gui+Space, or Alt+` while JA_SW is on
};

enum layers { Base, Qwerty, Lower, Raise, Adjust, Mouse, Arrows, Symbols, WithMous };

enum tap_dances { TD_GRV, TD_TAB };

// Tap-hold keys: hold for modifier/layer, tap for the key
#define GUI_BSP GUI_T(KC_BSPC)
#define CTL_SPC CTL_T(KC_SPC)
#define ALT_C   ALT_T(KC_C)
#define ALT_COM ALT_T(KC_COMM)
#define LWR_ENT LT(Lower, KC_ENT)
#define ARR_S   LT(Arrows, KC_S)
#define SYM_T   LT(Symbols, KC_T)
#define GUI_N   GUI_T(KC_N)
#define CTL_E   CTL_T(KC_E)
#define ALT_W   ALT_T(KC_W)
// same keys on the Qwerty layer
#define SFT_Z   SFT_T(KC_Z)
#define ARR_D   LT(Arrows, KC_D)
#define SYM_F   LT(Symbols, KC_F)
#define GUI_J   GUI_T(KC_J)
#define CTL_K   CTL_T(KC_K)
#define ALT_X   ALT_T(KC_X)
// WithMous layer
#define SFT_6   SFT_T(KC_6)
#define ALT_7   ALT_T(KC_7)
#define GRV_F2  TD(TD_GRV)  // tap: `, hold: F2
#define TAB_F3  TD(TD_TAB)  // tap: Tab, hold: F3

// Layer switching
#define RAISE   MO(Raise)
#define TG_QWRT TG(Qwerty)    // Colemak-DH <-> Qwerty
#define TG_MOUS TG(Mouse)
#define TG_WMOU TG(WithMous)
#define TO_BASE TO(Base)
#define TO_QWRT TO(Qwerty)
#define LED_LVL LED_LEVEL

// Base (Colemak-DH) is the default layer; Qwerty is toggled on top of it with TG_QWRT.
// The bottom-row gap (XXXXXXX) is the second half of the 2u key.
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [Base] = LAYOUT_planck_grid(
    KC_Q,    KC_Z,    KC_F,    KC_P,    KC_V,      _______, _______,   KC_J,    KC_L,    KC_U,    KC_Y,    KC_X,
    KC_A,    KC_R,    ARR_S,   SYM_T,   KC_G,      _______, JA_SW,     KC_K,    GUI_N,   CTL_E,   KC_I,    KC_O,
    KC_LSFT, ALT_W,   ALT_C,   KC_D,    KC_B,      _______, _______,   KC_M,    KC_H,    ALT_COM, KC_DOT,  KC_RSFT,
    TG_WMOU, KC_ESC,  KC_TAB,  GUI_BSP, CTL_SPC,   TG_MOUS, XXXXXXX,   RAISE,   LWR_ENT, LG_SPC,  KC_CAPS, TG_QWRT
  ),
  [Qwerty] = LAYOUT_planck_grid(
    KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,      _______, _______,   KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,
    KC_A,    KC_S,    ARR_D,   SYM_F,   KC_G,      _______, JA_SW,     KC_H,    GUI_J,   CTL_K,   KC_L,    KC_SCLN,
    SFT_Z,   ALT_X,   ALT_C,   KC_V,    KC_B,      _______, _______,   KC_N,    KC_M,    ALT_COM, KC_DOT,  KC_RSFT,
    TG_WMOU, KC_ESC,  KC_TAB,  GUI_BSP, CTL_SPC,   TG_MOUS, XXXXXXX,   RAISE,   LWR_ENT, LG_SPC,  KC_CAPS, TG_QWRT
  ),
  [Lower] = LAYOUT_planck_grid(
    KC_SCRL, KC_INS,  KC_F11,  KC_F12,  KC_PSCR,   _______, _______,   _______, _______, _______, _______, KC_PAUS,
    KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,     _______, _______,   _______, _______, _______, _______, _______,
    KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,    _______, _______,   _______, _______, _______, _______, _______,
    _______, _______, _______, KC_DEL,  _______,   _______, XXXXXXX,   _______, _______, _______, _______, _______
  ),
  [Raise] = LAYOUT_planck_grid(
    KC_LBRC, KC_RBRC, KC_LPRN, KC_RPRN, KC_DQUO,   _______, _______,   KC_SCLN, KC_7,    KC_8,    KC_9,    KC_BSLS,
    KC_QUOT, KC_DOT,  KC_COMM, KC_0,    KC_LCBR,   _______, _______,   KC_RCBR, KC_1,    KC_2,    KC_3,    KC_COLN,
    KC_LSFT, KC_UNDS, KC_MINS, KC_EQL,  KC_GRV,    _______, _______,   KC_SLSH, KC_4,    KC_5,    KC_6,    KC_RSFT,
    _______, _______, KC_TAB,  _______, _______,   _______, XXXXXXX,   _______, _______, KC_0,    _______, TO_BASE
  ),
  // Adjust: on while Lower + Raise are both held
  [Adjust] = LAYOUT_planck_grid(
    RGB_TOG, RGB_MOD, RGB_VAD, RGB_VAI, _______,   _______, _______,   KC_MUTE, KC_VOLD, KC_VOLU, LED_LVL, MAC_LCK,
    _______, _______, _______, _______, _______,   _______, _______,   KC_MPRV, KC_MPLY, KC_MNXT, KC_BRID, KC_BRIU,
    _______, _______, _______, _______, _______,   _______, _______,   _______, _______, _______, _______, KC_SLEP,
    _______, _______, _______, _______, _______,   _______, XXXXXXX,   _______, _______, _______, _______, _______
  ),
  [Mouse] = LAYOUT_planck_grid(
    XXXXXXX, XXXXXXX, KC_MS_U, XXXXXXX, XXXXXXX,   _______, _______,   XXXXXXX, XXXXXXX, KC_WH_U, XXXXXXX, XXXXXXX,
    XXXXXXX, KC_MS_L, KC_MS_D, KC_MS_R, XXXXXXX,   _______, _______,   XXXXXXX, KC_BTN1, KC_WH_D, KC_BTN2, XXXXXXX,
    KC_LSFT, KC_LALT, XXXXXXX, XXXXXXX, XXXXXXX,   _______, _______,   XXXXXXX, KC_WH_L, _______, KC_WH_R, XXXXXXX,
    _______, _______, _______, _______, KC_LCTL,   _______, XXXXXXX,   KC_ACL2, _______, _______, _______, TO_BASE
  ),
  [Arrows] = LAYOUT_planck_grid(
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,   _______, _______,   XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_PAUS,
    KC_LGUI, KC_LALT, _______, KC_LSFT, XXXXXXX,   _______, _______,   XXXXXXX, KC_LEFT, KC_UP,   KC_DOWN, KC_RGHT,
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,   _______, _______,   XXXXXXX, KC_HOME, KC_PGUP, KC_PGDN, KC_END,
    TO_QWRT, _______, KC_LALT, KC_LGUI, KC_LCTL,   _______, XXXXXXX,   _______, _______, _______, _______, TO_BASE
  ),
  [Symbols] = LAYOUT_planck_grid(
    KC_LBRC, KC_RBRC, KC_LPRN, KC_RPRN, KC_DQUO,   _______, _______,   KC_SCLN, KC_AMPR, KC_ASTR, KC_GRV,  KC_PIPE,
    KC_QUOT, KC_DOT,  KC_COMM, _______, KC_LCBR,   _______, _______,   KC_RCBR, KC_EXLM, KC_AT,   KC_HASH, KC_COLN,
    KC_LSFT, KC_UNDS, KC_MINS, KC_EQL,  KC_TILD,   _______, _______,   KC_QUES, KC_DLR,  KC_PERC, KC_CIRC, KC_PLUS,
    _______, _______, _______, _______, _______,   _______, XXXXXXX,   _______, _______, _______, _______, _______
  ),
  [WithMous] = LAYOUT_planck_grid(
    KC_F1,   GRV_F2,  TAB_F3,  KC_F4,   KC_F5,     KC_F6,   KC_F7,     KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,
    KC_1,    KC_2,    KC_3,    KC_4,    KC_5,      _______, _______,   _______, KC_LEFT, KC_UP,   KC_DOWN, KC_RGHT,
    SFT_6,   ALT_7,   KC_8,    KC_9,    KC_0,      _______, _______,   _______, KC_HOME, KC_PGUP, KC_PGDN, KC_END,
    _______, _______, KC_ESC,  GUI_BSP, CTL_SPC,   _______, XXXXXXX,   _______, _______, _______, _______, _______
  ),
};

const uint16_t PROGMEM tab_combo[]             = {GUI_BSP, CTL_SPC, COMBO_END};
const uint16_t PROGMEM left_tab_combo[]        = {ALT_C, GUI_BSP, COMBO_END};
const uint16_t PROGMEM right_tab_combo[]       = {KC_D, GUI_BSP, COMBO_END};
// note: no key is ALT_T(KC_D), so the two workspace combos can't currently fire
const uint16_t PROGMEM left_workspace_combo[]  = {ARR_S, ALT_T(KC_D), GUI_BSP, COMBO_END};
const uint16_t PROGMEM right_workspace_combo[] = {ARR_S, ALT_T(KC_D), CTL_SPC, COMBO_END};
const uint16_t PROGMEM mission_control_combo[] = {KC_F, SYM_T, GUI_BSP, COMBO_END};

combo_t key_combos[] = {
  COMBO(tab_combo,             KC_TAB),
  COMBO(left_tab_combo,        S(C(KC_TAB))),
  COMBO(right_tab_combo,       C(KC_TAB)),
  COMBO(left_workspace_combo,  G(C(KC_LEFT))),
  COMBO(right_workspace_combo, G(C(KC_RIGHT))),
  COMBO(mission_control_combo, KC_MCTL),
};

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

// Per-layer colours (HSV): the 36 LEDs of the top three rows, then the 11 of the bottom row.
#define X4(...)  {__VA_ARGS__}, {__VA_ARGS__}, {__VA_ARGS__}, {__VA_ARGS__}
#define X11(...) X4(__VA_ARGS__), X4(__VA_ARGS__), {__VA_ARGS__}, {__VA_ARGS__}, {__VA_ARGS__}
#define X36(...) X4(__VA_ARGS__), X4(__VA_ARGS__), X4(__VA_ARGS__), X4(__VA_ARGS__), X4(__VA_ARGS__), \
                 X4(__VA_ARGS__), X4(__VA_ARGS__), X4(__VA_ARGS__), X4(__VA_ARGS__)
#define LAYER_COLORS(h, s, v, bottom_h, bottom_s, bottom_v) { X36(h, s, v), X11(bottom_h, bottom_s, bottom_v) }

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
  //                         keys            bottom row
  [Base]     = LAYER_COLORS( 84, 110, 178,     85, 205, 184),
  [Qwerty]   = LAYER_COLORS(  0,  86, 209,    243, 222, 234),
  [Raise]    = LAYER_COLORS( 31, 255, 255,     10, 255, 255),
  [Mouse]    = LAYER_COLORS(195,  18, 251,    195, 212, 204),
  [WithMous] = LAYER_COLORS(131, 255, 255,    131, 240, 137),
};

static void set_layer_color(uint8_t layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
      rgb_matrix_set_color(i, 0, 0, 0);
    } else {
      RGB   rgb = hsv_to_rgb(hsv);
      float f   = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
      rgb_matrix_set_color(i, f * rgb.r, f * rgb.g, f * rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) return false;
  if (keyboard_config.disable_layer_led) return false;

  uint8_t layer = get_highest_layer(layer_state);
  switch (layer) {
  case Base:
  case Qwerty:
  case Raise:
  case Mouse:
  case WithMous:
    set_layer_color(layer);
    break;
  default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
    break;
  }
  return true;
}

static bool     ja_sw       = false;
static uint16_t lg_spc_sent = KC_NO;  // what LG_SPC registered on press, so release matches even if toggled meanwhile

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  if (keycode == LG_SPC) {
    if (record->event.pressed) {
      lg_spc_sent = ja_sw ? A(KC_GRV) : G(KC_SPC);
      register_code16(lg_spc_sent);
    } else {
      unregister_code16(lg_spc_sent);
    }
    return false;
  }

  // all other custom handling happens on key-down
  if (!record->event.pressed) return true;

  switch (keycode) {
  case JA_SW:
    ja_sw = !ja_sw;
    return false;

  case MAC_LCK:
    host_consumer_send(0x19E);
    return false;

  case RGB_SLD:
    if (!rawhid_state.rgb_control) {
      rgblight_mode(1);
    }
    return false;
  }
  return true;
}

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, Lower, Raise, Adjust);
}

// Tap dances: tap for one key, hold for another; triple tap sends the tap key three times.
typedef struct {
  uint16_t tap;
  uint16_t hold;
  uint8_t  step;
} dance_t;

static dance_t dances[] = {
  [TD_GRV] = {.tap = KC_GRV, .hold = KC_F2},
  [TD_TAB] = {.tap = KC_TAB, .hold = KC_F3},
};

enum { SINGLE_TAP = 1, SINGLE_HOLD, DOUBLE_TAP, DOUBLE_HOLD, DOUBLE_SINGLE_TAP, MORE_TAPS };

static uint8_t dance_step(tap_dance_state_t *state) {
  if (state->count == 1) {
    return (state->interrupted || !state->pressed) ? SINGLE_TAP : SINGLE_HOLD;
  }
  if (state->count == 2) {
    if (state->interrupted) return DOUBLE_SINGLE_TAP;
    return state->pressed ? DOUBLE_HOLD : DOUBLE_TAP;
  }
  return MORE_TAPS;
}

static void dance_each_tap(tap_dance_state_t *state, void *user_data) {
  dance_t *d = user_data;
  if (state->count == 3) {
    tap_code16(d->tap);
    tap_code16(d->tap);
    tap_code16(d->tap);
  }
  if (state->count > 3) {
    tap_code16(d->tap);
  }
}

static void dance_finished(tap_dance_state_t *state, void *user_data) {
  dance_t *d = user_data;
  d->step    = dance_step(state);
  switch (d->step) {
  case SINGLE_TAP:        register_code16(d->tap); break;
  case SINGLE_HOLD:       register_code16(d->hold); break;
  case DOUBLE_TAP:        register_code16(d->tap); register_code16(d->tap); break;
  case DOUBLE_SINGLE_TAP: tap_code16(d->tap); register_code16(d->tap); break;
  }
}

static void dance_reset(tap_dance_state_t *state, void *user_data) {
  dance_t *d = user_data;
  wait_ms(10);
  switch (d->step) {
  case SINGLE_TAP:
  case DOUBLE_TAP:
  case DOUBLE_SINGLE_TAP: unregister_code16(d->tap); break;
  case SINGLE_HOLD:       unregister_code16(d->hold); break;
  }
  d->step = 0;
}

#define DANCE(i) { .fn = {dance_each_tap, dance_finished, dance_reset, NULL}, .user_data = &dances[i] }

tap_dance_action_t tap_dance_actions[] = {
  [TD_GRV] = DANCE(TD_GRV),
  [TD_TAB] = DANCE(TD_TAB),
};
