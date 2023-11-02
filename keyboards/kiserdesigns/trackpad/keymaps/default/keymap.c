/* Copyright 2021 Noah Kiser (NCKiser)
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

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
    [0] = LAYOUT(
        KC_Q,   KC_W
    )
};

#define SCROLL_AVERAGE 10
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    static int16_t h       = 0;
    static uint8_t h_count = 0;
    static int16_t v       = 0;
    static uint8_t v_count = 0;

    if (mouse_report.h != 0) {
        h_count++;
        h += mouse_report.h;
        mouse_report.h = 0;
        if (h_count == SCROLL_AVERAGE) {
            h_count = 0;
            if (h < 0) {
                mouse_report.h = -1;
            } else {
                mouse_report.h = 1;
            }
            h = 0;
        }
    } else {
        if (h_count) {
            if (h < 0) {
                mouse_report.h = -1;
            } else {
                mouse_report.h = 1;
            }
        }
        h_count = 0;
        h       = 0;
    }

    if (mouse_report.v != 0) {
        v_count++;
        v += mouse_report.v;
        mouse_report.v = 0;
        if (v_count == SCROLL_AVERAGE) {
            v_count = 0;
            if (v < 0) {
                mouse_report.v = -1;
            } else {
                mouse_report.v = 1;
            }
            v = 0;
        }
    } else {
        if (v_count) {
            if (v < 0) {
                mouse_report.v = -1;
            } else {
                mouse_report.v = 1;
            }
        }
        v_count = 0;
        v       = 0;
    }

    return mouse_report;
}