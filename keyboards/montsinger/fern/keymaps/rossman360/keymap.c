#include QMK_KEYBOARD_H
#include "rossman360.h"

#define PGMOD LT(_NUM, KC_PGDN)
#define TABMOD LT(_FN1, KC_TAB)
#define SPCMOD LT(_FN1, KC_SPACE)
#define ENTMOD LT(_FN2, KC_ENTER)
#define ESCMOD LT(_NUM, KC_ESC)
#define RSMOD LT(_FN1, KC_RSFT)
#

enum layer_names {
  _BASE,
  _DEL,
  _FN1,
  _FN2,
  _NUM,
  _BLANK
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_BASE] = LAYOUT(
      KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,      KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,
     KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,        KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSLS,
    KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,          KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT,
   KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,   QK_BOOT, KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_MINS,
    KC_BSPC, ALTDEL,           CTRLSP,  KC_TAB,  PGMOD,         KC_ENT,  SPCMOD,  MO(_DEL),         KC_BSPC, KC_DEL
),

[_FN1] = LAYOUT(
      _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______, 
     _______, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______,        KC_HOME, KC_LEFT, KC_UP,   KC_RIGHT,KC_END,  _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, KC_DOWN, _______, _______, _______,
    _______, _______,          _______, LWORD,   RWORD,          _______, _______, _______,           _______, _______
),

[_NUM] = LAYOUT(
      KC_GRAVE,_______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______, 
     _______, _______, _______, _______, _______, _______,      _______, KC_7,    KC_8,    KC_9, _______, _______,
    _______, _______, _______, _______, _______, _______,        _______, KC_4,    KC_5,    KC_6, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, KC_1,    KC_2,    KC_3, _______, _______,
    _______, _______,          _______, _______, _______,        _______, _______, KC_0,          _______, _______
),

[_DEL] = LAYOUT(
      _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______, 
     QK_BOOT, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______,        _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______,          BLINE, KC_BSPC, BWORD,            _______, _______, _______,           _______, _______
),

[_BLANK] = LAYOUT(
      _______, _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______, 
     _______, _______, _______, _______, _______, _______,      _______, _______, _______, _______, _______, _______,
    _______, _______, _______, _______, _______, _______,        _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
    _______, _______,          _______, _______, _______,        _______, _______, _______,           _______, _______
)
};

bool encoder_update_user(uint8_t index, bool clockwise) {
    if (clockwise) {
      tap_code16(S(KC_VOLD));
    } else {
      tap_code16(KC_VOLU);
    }
    return true;
};
