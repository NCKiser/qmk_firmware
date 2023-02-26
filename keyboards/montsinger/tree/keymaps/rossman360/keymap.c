#include QMK_KEYBOARD_H
#include "rossman360.h"

#define PGMOD LT(NUM, KC_PGDN)
#define TABMOD LT(_FN1, KC_TAB)
#define SPCMOD LT(FN1, KC_SPACE)
#define ENTMOD LT(_FN2, KC_ENTER)
#define ESCMOD LT(_NUM, KC_ESC)
#define RSMOD LT(FN1, KC_RSHIFT)
#define NUMNTR LT(NUM, KC_ENTER)
#define CTLDEL MT(MOD_LCTL, KC_DEL)

enum layer_names {
  BASE,
  FN1,
  NUM,
  BLANK,
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[BASE] = LAYOUT(
   KC_1,    KC_2,    KC_3,    KC_4,    KC_5,   KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_BSPC,
   KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,   KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_BSLS, QK_BOOT,
   KC_CAPS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,   KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_ENT,
   KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,   KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_UP,
   KC_BSPC, CTLDEL,  KC_TAB,  NUMNTR,            KC_ENT, SPCMOD,  KC_MINS,                   KC_LEFT, KC_DOWN, KC_RGHT
),

[FN1] = LAYOUT(
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, KC_HOME, KC_LEFT, KC_UP,   KC_RIGHT,KC_END,  _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, KC_DOWN, _______, _______, _______, 
   _______, _______, _______, _______, _______, _______, _______,                   _______, _______, _______
),

[NUM] = LAYOUT(
   KC_GRV,  _______, _______, _______, _______, _______, _______, KC_F8,   _______, _______, KC_MINS, KC_EQUAL,
   _______, _______, _______, _______, _______, _______, _______, KC_7,    KC_8,    KC_9,    _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, KC_4,    KC_5,    KC_6,    _______,_______, _______,
   _______, _______, _______, _______, _______, _______, _______, KC_1,    KC_2,    KC_3,    _______,_______, 
   _______, _______, _______, _______, KC_0,    _______,    _______,                   _______, _______, _______
),

[BLANK] = LAYOUT(
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,_______, _______,
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,_______, 
   _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
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
