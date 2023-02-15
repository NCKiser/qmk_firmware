

#include QMK_KEYBOARD_H
#include "i2c_master.h"

extern i2c_status_t tca9555_status;
#undef I2C_TIMEOUT
#define I2C_TIMEOUT 1000

// I2C address:
// All address pins of the tca9555 are connected to the ground
// | 0  | 1  | 0  | 0  | A2 | A1 | A0 |
// | 0  | 1  | 0  | 0  | 0  | 0  | 0  |
#define I2C_ADDR (0b0100000 << 1)

// Register addresses
#define IODIRA 0x06  // i/o direction register
#define IODIRB 0x07
#define IREGP0 0x00  // GPIO pull-up resistor register
#define IREGP1 0x01
#define OREGP0 0x02  // general purpose i/o port register (write modifies OLAT)
#define OREGP1 0x03

bool         i2c_initialized = 0;
i2c_status_t tca9555_status = I2C_ADDR;

uint8_t init_tca9555(void) {
    print("starting init");
    tca9555_status = I2C_ADDR;

    // I2C subsystem
    if (i2c_initialized == 0) {
        i2c_init();
        i2c_initialized = true;
        wait_ms(I2C_TIMEOUT);
    }

    // set pin direction
    // - unused  : input  : 1
    // - input   : input  : 1
    // - driving : output : 0
    uint8_t conf[2] = {
        // This means: write on IO 7 and IO3, read on the rest
        0b01110111,
        // This means: write on IO 7,IO5 and IO0, read on the rest
        0b01011110,
    };
    tca9555_status = i2c_writeReg(I2C_ADDR, IODIRA, conf, 2, I2C_TIMEOUT);

    return tca9555_status;
}

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];      // debounced values

static matrix_row_t read_cols(uint8_t row);
static void         init_cols(void);
static void         unselect_rows(void);
static void         select_row(uint8_t row);

static uint8_t tca9555_reset_loop;

void matrix_init_custom(void) {
    // initialize row and col

    tca9555_status = init_tca9555();

    unselect_rows();
    init_cols();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        matrix[i]     = 0;
    }
}

void matrix_power_up(void) {
    tca9555_status = init_tca9555();

    unselect_rows();
    init_cols();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        matrix[i] = 0;
    }
}

// Reads and stores a row, returning
// whether a change occurred.
static inline bool store_matrix_row(matrix_row_t current_matrix[], uint8_t index) {
    matrix_row_t temp = read_cols(index);
    if (current_matrix[index] != temp) {
        current_matrix[index] = temp;
        return true;
    }
    return false;
}

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    if (tca9555_status) {  // if there was an error
        if (++tca9555_reset_loop == 0) {
            // since tca9555_reset_loop is 8 bit - we'll try to reset once in 255 matrix scans
            // this will be approx bit more frequent than once per second
            dprint("trying to reset tca9555\n");
            tca9555_status = init_tca9555();
            if (tca9555_status) {
                dprint("right side not responding\n");
            } else {
                dprint("right side attached\n");
            }
        }
    }

    bool changed = false;
    
    for (uint8_t i = 0; i < MATRIX_ROWS-MATRIX_ROWS_NP; i++) {
        // select rows from MCU
        uint8_t alpha_index  = i;
        select_row(alpha_index);

        // we don't need a 30us delay anymore, because selecting a
        // left-hand row requires more than 30us for i2c.

        changed |= store_matrix_row(current_matrix, alpha_index);

        unselect_rows();
    }
    for(uint8_t i = MATRIX_ROWS-MATRIX_ROWS_NP; i < MATRIX_ROWS; i++) {
        uint8_t np_index  = i;
        select_row(np_index);
        
        changed |= store_matrix_row(current_matrix, np_index);

        unselect_rows();
    }
    return changed;
}

static void init_cols(void) {
    // init on tca9555
    // not needed, already done as part of init_tca9555()

    // init on mcu
    pin_t matrix_col_pins_mcu[MATRIX_COLS] = MATRIX_COL_PINS;
    for (int pin_index = 0; pin_index < MATRIX_COLS-MATRIX_COLS_NP; pin_index++) {
        pin_t pin = matrix_col_pins_mcu[pin_index];
        setPinInput(pin);
        writePinHigh(pin);
    }
}

//COLS
static matrix_row_t read_cols(uint8_t row) {
    if (row < MATRIX_ROWS-MATRIX_ROWS_NP) {
        pin_t        matrix_col_pins_mcu[MATRIX_COLS] = MATRIX_COL_PINS;
        matrix_row_t current_row_value                         = 0;
        // For each col...
        for (uint8_t col_index = 0; col_index < MATRIX_COLS-MATRIX_COLS_NP; col_index++) {
            // Select the col pin to read (active low)
            uint8_t pin_state = readPin(matrix_col_pins_mcu[col_index]);

            // Populate the matrix row with the state of the col pin
            current_row_value |= pin_state ? 0 : (MATRIX_ROW_SHIFTER << col_index);
        }
        return current_row_value;
    } else {
        if (tca9555_status) {  // if there was an error
            return 0;
        } else {
            uint8_t data    = 0;
            uint8_t ports[2]   = {0};
            tca9555_status  = i2c_readReg(I2C_ADDR, IREGP0, &ports[0], 2, I2C_TIMEOUT);
            if (tca9555_status) {  // if there was an error
                // do nothing
                return 0;
            } else {
                data |= (ports[1] & 0x40) >> 6;
                data |= (ports[0] & 0x07) << 1;
                data = ~data;
                tca9555_status = I2C_STATUS_SUCCESS;
                return data;
            }
        }
    }
}

static void unselect_rows(void) {
    // no need to unselect on tca9555, because the select step sets all
    // the other row bits high, and it's not changing to a different
    // direction

    // unselect rows on microcontroller
    pin_t matrix_row_pins_mcu[MATRIX_ROWS] = MATRIX_ROW_PINS;
    for (int pin_index = 0; pin_index < MATRIX_ROWS-MATRIX_ROWS_NP; pin_index++) {
        pin_t pin = matrix_row_pins_mcu[pin_index];
        setPinInput(pin);
        writePinLow(pin);
    }
}

//ROWS
static void select_row(uint8_t row) {
    uint8_t ports[2];
    ports[0] = 0xff;
    ports[1] = 0xff;

    if (row < MATRIX_ROWS-MATRIX_ROWS_NP) {
        // select on atmega32u4
        pin_t matrix_row_pins_mcu[MATRIX_ROWS] = MATRIX_ROW_PINS;
        pin_t pin                              = matrix_row_pins_mcu[row];
        setPinOutput(pin);
        writePinLow(pin);
    } else {
        // select on tca9555
        if (tca9555_status) {  // if there was an error
                                // do nothing
        } else {
            switch(row) {
                case 5: ports[0] &= ~(1 << 3); break;
                case 6: ports[1] &= ~(1 << 5); break;
                case 7: ports[1] &= ~(1 << 7); break;
                case 8: ports[1] &= ~(1 << 0); break;
                case 9: ports[0] &= ~(1 << 7); break;
                default:                    break;
            }

            tca9555_status = i2c_writeReg(I2C_ADDR, OREGP0, &ports[0], 2, I2C_TIMEOUT);
            // Select the desired row by writing a byte for the entire GPIOB bus where only the bit representing the row we want to select is a zero (write instruction) and every other bit is a one.
            // Note that the row - MATRIX_ROWS reflects the fact that being on the right hand, the columns are numbered from MATRIX_ROWS to MATRIX_ROWS, but the pins we want to write to are indexed from zero up on the GPIOB bus.
        }
    }
}
