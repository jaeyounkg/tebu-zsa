#include QMK_KEYBOARD_H
#include "version.h"
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  JA_SW,   // toggle: when on, LG_SPC sends Alt+` (Windows IME switch) instead of Gui+Space
  LG_SPC,  // Gui+Space, or Alt+` while JA_SW is on
};

enum layers { Base, Qwerty, Raise, Lower, Arrows, Mouse, Symbols, WithMous };

// Tap-hold keys: hold for modifier/layer, tap for the key
#define GUI_N   LGUI_T(KC_N)
#define CTL_E   LCTL_T(KC_E)
#define ALT_CMA LALT_T(KC_COMM)
#define ALT_W   LALT_T(KC_W)
#define ALT_C   LALT_T(KC_C)
#define ALT_TAB LALT_T(KC_TAB)
#define GUI_BSP LGUI_T(KC_BSPC)
#define CTL_SPC LCTL_T(KC_SPC)
#define SFT_6   LSFT_T(KC_6)
#define LWR_ENT LT(Lower, KC_ENT)
#define ARR_S   LT(Arrows, KC_S)
#define SYM_T   LT(Symbols, KC_T)
// same keys on the Qwerty layer
#define GUI_J   LGUI_T(KC_J)
#define CTL_K   LCTL_T(KC_K)
#define ALT_X   LALT_T(KC_X)
#define SFT_Z   LSFT_T(KC_Z)
#define ARR_D   LT(Arrows, KC_D)
#define SYM_F   LT(Symbols, KC_F)
#define LED_LVL LED_LEVEL

// The halves are used swapped: the board's right half sits on the left of the desk.
// LAYOUT_swapped takes keys in desk order (left block = board's right half) and
// forwards them to LAYOUT_moonlander in the board's own order.
// The number row is unused, so it is left out and filled with KC_NO.
#define LAYOUT_swapped( \
    r10, r11, r12, r13, r14, r15, r16,   l10, l11, l12, l13, l14, l15, l16, \
    r20, r21, r22, r23, r24, r25, r26,   l20, l21, l22, l23, l24, l25, l26, \
         r31, r32, r33, r34, r35, r36,   l30, l31, l32, l33, l34, l35,      \
         r41, r42, r43, r44, r45, r46,   l40, l41, l42, l43, l44, l45,      \
    r50, r51, r52,                                      l50, l51, l52       \
) LAYOUT_moonlander( \
    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,   XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, \
    l10, l11, l12, l13, l14, l15, l16,   r10, r11, r12, r13, r14, r15, r16, \
    l20, l21, l22, l23, l24, l25, l26,   r20, r21, r22, r23, r24, r25, r26, \
    l30, l31, l32, l33, l34, l35,             r31, r32, r33, r34, r35, r36, \
    l40, l41, l42, l43, l44, l45,             r41, r42, r43, r44, r45, r46, \
                        l50, l51, l52,   r50, r51, r52 \
)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [Base] = LAYOUT_swapped(
    _______, KC_Q,    KC_Z,    KC_F,    KC_P,    KC_V,    _______,          _______,  KC_J,      KC_L,    KC_U,    KC_Y,    KC_X,    _______,
    _______, KC_A,    KC_R,    ARR_S,   SYM_T,   KC_G,    TG(Qwerty),       JA_SW,    KC_K,      GUI_N,   CTL_E,   KC_I,    KC_O,    _______,
             KC_LSFT, ALT_W,   ALT_C,   KC_D,    KC_B,    TG(WithMous),     _______,  KC_M,      KC_H,    ALT_CMA, KC_DOT,  KC_RSFT,
             _______, KC_ESC,  ALT_TAB, GUI_BSP, CTL_SPC, TO(Mouse),        _______,  MO(Raise), LWR_ENT, LG_SPC,  KC_CAPS, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Qwerty] = LAYOUT_swapped(
    _______, KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    _______,          _______,  KC_Y,      KC_U,    KC_I,    KC_O,    KC_P,    _______,
    _______, KC_A,    KC_S,    ARR_D,   SYM_F,   KC_G,    _______,          _______,  KC_H,      GUI_J,   CTL_K,   KC_L,    KC_SCLN, _______,
             SFT_Z,   ALT_X,   ALT_C,   KC_V,    KC_B,    _______,          _______,  KC_N,      KC_M,    ALT_CMA, KC_DOT,  KC_RSFT,
             _______, KC_ESC,  ALT_TAB, GUI_BSP, CTL_SPC, TO(Mouse),        TO(Base), MO(Raise), LWR_ENT, LG_SPC,  KC_CAPS, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Raise] = LAYOUT_swapped(
    _______, KC_LBRC, KC_RBRC, KC_LPRN, KC_RPRN, KC_DQUO, _______,          _______,  KC_SCLN,   KC_7,    KC_8,    KC_9,    KC_BSLS, _______,
    _______, KC_QUOT, KC_DOT,  KC_COMM, KC_0,    KC_LCBR, _______,          _______,  KC_RCBR,   KC_1,    KC_2,    KC_3,    KC_COLN, _______,
             KC_LSFT, KC_UNDS, KC_MINS, KC_EQL,  KC_GRV,  _______,          _______,  KC_SLSH,   KC_4,    KC_5,    KC_6,    KC_RSFT,
             _______, _______, _______, _______, _______, _______,          TO(Base), _______,   _______, KC_0,    _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Lower] = LAYOUT_swapped(
    _______, KC_SCRL, KC_NUM,  KC_F11,  KC_F12,  KC_PSCR, _______,          _______,  KC_MUTE,   KC_VOLD, KC_VOLU, RGB_VAD, RGB_VAI, _______,
    _______, KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   _______,          _______,  KC_MPRV,   KC_MPLY, KC_MNXT, KC_BRID, KC_BRIU, _______,
             KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  _______,          _______,  _______,   _______, _______, LED_LVL, RGB_TOG,
             _______, _______, _______, KC_DEL,  KC_INS,  _______,          TO(Base), _______,   _______, _______, _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Arrows] = LAYOUT_swapped(
    _______, _______, _______, _______, _______, _______, _______,          _______,  _______,   _______, _______, _______, _______, _______,
    _______, KC_LGUI, KC_LALT, _______, KC_LSFT, _______, _______,          _______,  _______,   KC_LEFT, KC_UP,   KC_DOWN, KC_RGHT, _______,
             _______, _______, _______, _______, _______, _______,          _______,  _______,   KC_HOME, KC_PGUP, KC_PGDN, KC_END,
             _______, _______, KC_LALT, KC_LGUI, KC_LCTL, _______,          TO(Base), _______,   _______, _______, _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Mouse] = LAYOUT_swapped(
    _______, _______, _______, KC_MS_U, _______, _______, _______,          _______,  _______,   _______, KC_WH_U, _______, _______, _______,
    _______, _______, KC_MS_L, KC_MS_D, KC_MS_R, _______, _______,          _______,  _______,   KC_BTN1, KC_WH_D, KC_BTN2, _______, _______,
             KC_LSFT, _______, _______, _______, _______, _______,          _______,  _______,   KC_WH_L, _______, KC_WH_R, _______,
             _______, _______, KC_LALT, KC_LGUI, KC_LCTL, _______,          TO(Base), KC_ACL2,   _______, _______, _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [Symbols] = LAYOUT_swapped(
    _______, KC_LBRC, KC_RBRC, KC_LPRN, KC_RPRN, KC_DQUO, _______,          _______,  KC_SCLN,   KC_AMPR, KC_ASTR, KC_PIPE, KC_BSLS, _______,
    _______, KC_QUOT, KC_DOT,  KC_COMM, _______, _______, _______,          _______,  KC_RCBR,   KC_EXLM, KC_AT,   KC_HASH, KC_COLN, _______,
             _______, KC_UNDS, KC_MINS, KC_EQL,  KC_GRV,  _______,          _______,  KC_SLSH,   KC_DLR,  KC_PERC, KC_CIRC, _______,
             _______, _______, _______, _______, _______, _______,          TO(Base), _______,   _______, _______, _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
  [WithMous] = LAYOUT_swapped(
    _______, _______, _______, KC_TAB,  KC_ENT,  _______, _______,          _______,  _______,   _______, _______, _______, _______, _______,
    _______, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    _______,          _______,  _______,   KC_LEFT, KC_UP,   KC_DOWN, KC_RGHT, _______,
             SFT_6,   KC_7,    KC_8,    KC_9,    KC_0,    _______,          _______,  _______,   KC_HOME, KC_PGUP, KC_PGDN, KC_END,
             _______, _______, _______, _______, _______, _______,          TO(Base), _______,   _______, _______, _______, _______,
    _______, _______, _______,                                                                                     _______, _______, _______
  ),
};

static bool     ja_sw = false;
static uint16_t lg_spc_sent = KC_NO;  // what LG_SPC registered on press, so release matches even if toggled meanwhile

extern bool is_launching;  // moonlander.c: true while the startup LED animation runs

// Status LEDs (board halves, not desk halves):
//   board right half, LEDs 4-6: highest active layer in binary, LED 4 = bit 0
//   board left half:  LED 1 = JA_SW on, LED 2 unused, LED 3 = Caps Lock
// `state` is passed in because layer_state_set_user runs before layer_state is updated.
static void update_status_leds(layer_state_t state) {
  if (is_launching) return;
  bool    on    = keyboard_config.led_level;
  uint8_t layer = get_highest_layer(state);
  STATUS_LED_1(on && ja_sw);
  STATUS_LED_2(false);
  STATUS_LED_3(on && host_keyboard_led_state().caps_lock);
  STATUS_LED_4(on && (layer & 1));
  STATUS_LED_5(on && (layer & 2));
  STATUS_LED_6(on && (layer & 4));
}

layer_state_t layer_state_set_user(layer_state_t state) {
  update_status_leds(state);
  return state;
}

bool led_update_user(led_t led_state) {
  update_status_leds(layer_state);
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
  case JA_SW:
    if (record->event.pressed) {
      ja_sw = !ja_sw;
      update_status_leds(layer_state);
    }
    return false;

  case LED_LEVEL:  // normally handled by moonlander.c, but MOONLANDER_USER_LEDS disables that
    if (record->event.pressed) {
      keyboard_config.led_level ^= 1;
      eeconfig_update_kb(keyboard_config.raw);
      update_status_leds(layer_state);
    }
    return false;

  case LG_SPC:
    if (record->event.pressed) {
      lg_spc_sent = ja_sw ? LALT(KC_GRV) : LGUI(KC_SPC);
      register_code16(lg_spc_sent);
    } else {
      unregister_code16(lg_spc_sent);
    }
    return false;

  case QK_MODS ... QK_MODS_MAX:
    // Mouse and consumer keys (volume, media) with modifiers work inconsistently across operating systems,
    // this makes sure that modifiers are always applied to the key that was pressed.
    if (IS_MOUSE_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode)) || IS_CONSUMER_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode))) {
      if (record->event.pressed) {
        add_mods(QK_MODS_GET_MODS(keycode));
        send_keyboard_report();
        wait_ms(2);
        register_code(QK_MODS_GET_BASIC_KEYCODE(keycode));
        return false;
      } else {
        wait_ms(2);
        del_mods(QK_MODS_GET_MODS(keycode));
      }
    }
    break;

    case RGB_SLD:
        if (record->event.pressed) {
            rgblight_mode(1);
        }
        return false;
  }
  return true;
}

