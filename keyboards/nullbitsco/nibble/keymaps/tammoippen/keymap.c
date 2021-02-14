/* Copyright 2020 Jay Greco
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H

#define _BL 0
#define _FN 1

#define KC_COPY LCTL(KC_C)
#define KC_PASTE LCTL(KC_V)
#define KC_FIND LCTL(KC_F)

enum custom_keycodes {
  KC_CUST = SAFE_RANGE,
};

// Tap Dance declarations
enum {
    TD_CTL_A,
};
// Tap Dance definitions
qk_tap_dance_action_t tap_dance_actions[] = {
    // Tap once for CTL-A, twice for Caps Lock
    [TD_CTL_A] = ACTION_TAP_DANCE_DOUBLE(LCTL(KC_A), KC_CAPS),
};



const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  // _BL: Base  Layer
  [_BL] = LAYOUT_iso(
             KC_GRAVE,     KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_MINS, KC_EQL,  KC_BSPC, KC_WWW_SEARCH,
    KC_MUTE, KC_TAB,       KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_LBRC, KC_RBRC,          KC_DEL,
    KC_ESC,  TD(TD_CTL_A), KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_QUOT, KC_NUHS, KC_ENT,  KC_COPY,
    KC_NO,   KC_LSFT,      KC_NUBS, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT, KC_UP,   KC_PASTE,
    KC_NO,   KC_LCTL,      KC_LGUI, KC_LALT,                            KC_SPC,                    KC_ALGR, KC_FIND, MO(_FN), KC_LEFT, KC_DOWN, KC_RGHT
  ),
  // _FN: Funktion Layer
  [_FN] = LAYOUT_iso(
             _______,   KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F11,  KC_F12,  KC_F13,      KC_SYSTEM_SLEEP,
    RGB_TOG, _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______,              KC_F14,
    KC_F17,  _______,  _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______, KC_KP_ENTER, KC_F15,
    _______, _______,  _______, _______, _______, _______, _______, _______, KC_MSTP, KC_MPLY, KC_MPRV,  KC_MNXT, _______, _______, KC_PGUP,     KC_F16,
    _______, _______,  _______, _______,                            _______,                   _______,  _______, _______, KC_HOME, KC_PGDN,     KC_END
  )

};

#ifdef OLED_DRIVER_ENABLE
oled_rotation_t oled_init_user(oled_rotation_t rotation) { return OLED_ROTATION_90; }

void oled_task_user(void) {

  switch (get_highest_layer(layer_state)) {
    case _BL:
      oled_write_ln_P(PSTR("Base"), false);
      break;
    case _FN:
      oled_write_ln_P(PSTR(" FN"), false);
      break;
    default:
      oled_write_ln_P(PSTR("Undefined"), false);
  }

  oled_write_ln(0, false);
  oled_write_ln(0, false);
  oled_write_ln(0, false);
  oled_write_ln(0, false);
  oled_write_ln(0, false);
  oled_write_ln(0, false);
  oled_write_ln(0, false);

  oled_write_ln_P(PSTR("RGB"), false);
  static char status[5] = {0};
  snprintf(status, sizeof(status), "  %2d", rgblight_get_mode());
  oled_write_ln(status, false);

  oled_write_ln_P(PSTR("HUE"), false);
  snprintf(status, sizeof(status), " %3d", rgblight_get_hue());
  oled_write_ln(status, false);

  oled_write_ln_P(PSTR("SAT"), false);
  snprintf(status, sizeof(status), " %3d", rgblight_get_sat());
  oled_write_ln(status, false);

  oled_write_ln_P(PSTR("VAL"), false);
  snprintf(status, sizeof(status), " %3d", rgblight_get_val());
  oled_write_ln(status, false);
  // char wpm_str[5];
	// sprintf(wpm_str, "WPM = %i\n", get_current_wpm());
	// //oled_write_P(PSTR("\n"), false);
	// oled_write(wpm_str, false);
  //   led_t led_state = host_keyboard_led_state();
  //   oled_write_ln_P(led_state.caps_lock ? PSTR("[ C A P S   L O C K ]") : PSTR("[caps off]"), false);

}
#endif

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  // Send keystrokes to host keyboard, if connected (see readme)
  process_record_remote_kb(keycode, record);
  switch(keycode) {
    case KC_CUST: //custom macro
      if (record->event.pressed) {
      }
    break;

    case RM_1: //remote macro 1
      if (record->event.pressed) {
      }
    break;

    case RM_2: //remote macro 2
      if (record->event.pressed) {
      }
    break;

    case RM_3: //remote macro 3
      if (record->event.pressed) {
      }
    break;

    case RM_4: //remote macro 4
      if (record->event.pressed) {
      }
    break;

  }
return true;
}

// RGB config, for changing RGB settings on non-VIA firmwares
void change_RGB(bool clockwise) {
    bool shift = get_mods() & MOD_MASK_SHIFT;
    bool alt = get_mods() & MOD_MASK_ALT;
    bool ctrl = get_mods() & MOD_MASK_CTRL;

    if (clockwise) {
        if (alt) {
            rgblight_increase_hue_noeeprom();
        } else if (ctrl) {
            rgblight_increase_val_noeeprom();
        } else if (shift) {
            rgblight_increase_sat_noeeprom();
        } else {
            rgblight_step_noeeprom();
        }

  } else {
      if (alt) {
            rgblight_decrease_hue_noeeprom();
        } else if (ctrl) {
            rgblight_decrease_val_noeeprom();
        } else if (shift) {
            rgblight_decrease_sat_noeeprom();
        } else {
            rgblight_step_reverse_noeeprom();
        }
    }
}

void encoder_update_kb(uint8_t index, bool clockwise) {
  if (layer_state_is(_FN)) {
    //change RGB settings
    change_RGB(clockwise);
  }
  else {
    if (clockwise) {
      tap_code(KC_VOLU);
  } else {
      tap_code(KC_VOLD);
    }
  }
}

void matrix_init_user(void) {
  // Initialize remote keyboard, if connected (see readme)
  matrix_init_remote_kb();
}

void matrix_scan_user(void) {
  // Scan and parse keystrokes from remote keyboard, if connected (see readme)
  matrix_scan_remote_kb();
}
