/*
 * ============================================================
 *  MORSE COMMUNICATOR — Bare-Metal AVR C
 *  CO321 Embedded Systems Project (2026)
 *  Department of Computer Engineering
 *  University of Peradeniya
 * ============================================================
 *  Target:  ATmega328P (Arduino Uno) — Tinkercad Simulation
 *  Language: Pure C (no Arduino libraries)
 *  Constraint: No system delays — all timing via Timer interrupts
 *
 *  HARDWARE WIRING:
 *  LED          -> D13 (PB5, with 220 ohm to GND)
 *  Buzzer       -> D9  (PB1/OC1A, to GND)
 *  Push Button  -> D2  (PD2, other leg to GND, internal pull-up)
 *  LCD RS       -> D12 (PB4)
 *  LCD EN       -> D11 (PB3)
 *  LCD D4       -> D4  (PD4)
 *  LCD D5       -> D5  (PD5)
 *  LCD D6       -> D6  (PD6)
 *  LCD D7       -> D7  (PD7)
 *  LCD RW       -> GND
 *  LCD VSS      -> GND
 *  LCD VDD      -> 5V
 *  LCD V0       -> Potentiometer wiper (contrast)
 *  LCD A        -> 5V via 220 ohm (backlight)
 *  LCD K        -> GND (backlight)
 * ============================================================
 */

/* ═══════════════════════════════════════════════════════════════
 *  CONFIGURATION
 * ═══════════════════════════════════════════════════════════════ */

#include <avr/io.h>
#include <avr/interrupt.h>

/* Pin Definitions */
#define LED_BIT   5   /* PB5 — Arduino D13 */
#define BUZ_BIT   1   /* PB1 — Arduino D9 (OC1A) */
#define BTN_BIT   2   /* PD2 — Arduino D2 */
#define LCD_RS_BIT 4  /* PB4 — Arduino D12 */
#define LCD_EN_BIT 3  /* PB3 — Arduino D11 */
#define LCD_DATA_MASK 0xF0  /* PD7:PD4 */

/* Morse Timing (ms) — 300ms dot for human-readable speed */
#define DOT_TIME       300
#define DASH_TIME      900
#define INTRA_GAP      300
#define INTER_GAP      900
#define WORD_GAP       2100

/* Decoding Thresholds (ms) */
#define DOT_THRESHOLD  500
#define CHAR_TIMEOUT   900
#define WORD_TIMEOUT   2100
#define DEBOUNCE_MS    20

/* Buzzer frequency */
#define BUZZER_FREQ    800

/* UART */
#define BAUD_RATE      9600
#define UBRR_VAL       ((F_CPU / (16UL * BAUD_RATE)) - 1)

/* Buffers */
#define RX_BUF_SIZE    64
#define MSG_BUF_SIZE   64
#define DEC_MSG_SIZE   33
#define MORSE_BUF_SIZE 8

/* System modes */
#define MODE_ENCODE 0
#define MODE_DECODE 1

/* Encoding states */
#define ENC_IDLE      0
#define ENC_FETCH     1
#define ENC_PLAY_ON   2
#define ENC_PLAY_OFF  3
#define ENC_CHAR_GAP  4
#define ENC_WORD_GAP  5
#define ENC_DONE      6

/* Decoding states */
#define DEC_IDLE      0
#define DEC_PRESSED   1
#define DEC_RELEASED  2
#define DEC_CHAR_DONE 3
#define DEC_DONE      4


/* ═══════════════════════════════════════════════════════════════
 *  INTERNATIONAL MORSE CODE TABLE
 *  Index 0-25: A-Z, Index 26-35: 0-9
 * ═══════════════════════════════════════════════════════════════ */

char morse_A[] = ".-";
char morse_B[] = "-...";
char morse_C[] = "-.-.";
char morse_D[] = "-..";
char morse_E[] = ".";
char morse_F[] = "..-.";
char morse_G[] = "--.";
char morse_H[] = "....";
char morse_I[] = "..";
char morse_J[] = ".---";
char morse_K[] = "-.-";
char morse_L[] = ".-..";
char morse_M[] = "--";
char morse_N[] = "-.";
char morse_O[] = "---";
char morse_P[] = ".--.";
char morse_Q[] = "--.-";
char morse_R[] = ".-.";
char morse_S[] = "...";
char morse_T[] = "-";
char morse_U[] = "..-";
char morse_V[] = "...-";
char morse_W[] = ".--";
char morse_X[] = "-..-";
char morse_Y[] = "-.--";
char morse_Z[] = "--..";
char morse_0[] = "-----";
char morse_1[] = ".----";
char morse_2[] = "..---";
char morse_3[] = "...--";
char morse_4[] = "....-";
char morse_5[] = ".....";
char morse_6[] = "-....";
char morse_7[] = "--...";
char morse_8[] = "---..";
char morse_9[] = "----.";

char* morse_table[] = {
    morse_A, morse_B, morse_C, morse_D, morse_E, morse_F,
    morse_G, morse_H, morse_I, morse_J, morse_K, morse_L,
    morse_M, morse_N, morse_O, morse_P, morse_Q, morse_R,
    morse_S, morse_T, morse_U, morse_V, morse_W, morse_X,
    morse_Y, morse_Z,
    morse_0, morse_1, morse_2, morse_3, morse_4, morse_5,
    morse_6, morse_7, morse_8, morse_9
};


/* ═══════════════════════════════════════════════════════════════
 *  GLOBAL STATE
 * ═══════════════════════════════════════════════════════════════ */

/* Millisecond tick counter */
volatile unsigned long millis_count = 0;

/* UART RX ring buffer */
volatile unsigned char rx_buf[RX_BUF_SIZE];
volatile unsigned char rx_head = 0;
volatile unsigned char rx_tail = 0;

/* System mode */
volatile unsigned char current_mode = MODE_ENCODE;

/* Encoding state */
unsigned char enc_state = ENC_IDLE;
char* enc_pattern = 0;
unsigned char enc_sym_idx = 0;
unsigned long enc_timer = 0;
char enc_msg[MSG_BUF_SIZE];
unsigned char enc_msg_len = 0;
unsigned char enc_msg_idx = 0;
unsigned long enc_last_rx_time = 0;

/* Decoding state */
unsigned char dec_state = DEC_IDLE;
unsigned long dec_press_time = 0;
unsigned long dec_release_time = 0;
char dec_morse[MORSE_BUF_SIZE];
unsigned char dec_morse_len = 0;
char dec_message[DEC_MSG_SIZE];
unsigned char dec_msg_len = 0;
unsigned long dec_debounce = 0;
unsigned char dec_word_gap_sent = 1;


/* ═══════════════════════════════════════════════════════════════
 *  TIMER0: 1ms TICK (millis)
 * ═══════════════════════════════════════════════════════════════ */

ISR(TIMER0_COMPA_vect) {
    millis_count++;
}

unsigned long millis_now() {
    unsigned long m;
    unsigned char sreg = SREG;
    cli();
    m = millis_count;
    SREG = sreg;
    return m;
}

void timer0_init() {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}


/* ═══════════════════════════════════════════════════════════════
 *  TIMER1: BUZZER TONE (800 Hz on OC1A / PB1)
 * ═══════════════════════════════════════════════════════════════ */

void timer1_init() {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A  = 1249;
}

void buzzer_on() {
    TCCR1A |= (1 << COM1A0);
}

void buzzer_off() {
    TCCR1A &= ~(1 << COM1A0);
    PORTB &= ~(1 << BUZ_BIT);
}


/* ═══════════════════════════════════════════════════════════════
 *  MICROSECOND DELAYS (simple loop for Tinkercad compatibility)
 *  At 16MHz, ~4 cycles per loop iteration ≈ 0.25µs
 * ═══════════════════════════════════════════════════════════════ */

void delay_us(unsigned int us) {
    while (us > 0) {
        volatile unsigned char i;
        for (i = 0; i < 4; i++);
        us--;
    }
}

void delay_ms_blocking(unsigned int ms) {
    unsigned long start = millis_now();
    while (millis_now() - start < ms)
        ;
}


/* ═══════════════════════════════════════════════════════════════
 *  UART: 9600 BAUD, RX INTERRUPT, RING BUFFER
 * ═══════════════════════════════════════════════════════════════ */

ISR(USART_RX_vect) {
    unsigned char data = UDR0;
    unsigned char next = (rx_head + 1) % RX_BUF_SIZE;
    if (next != rx_tail) {
        rx_buf[rx_head] = data;
        rx_head = next;
    }
}

void uart_init() {
    UBRR0H = (unsigned char)(UBRR_VAL >> 8);
    UBRR0L = (unsigned char)(UBRR_VAL);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

unsigned char uart_available() {
    return (rx_head != rx_tail);
}

unsigned char uart_read_byte() {
    while (rx_head == rx_tail)
        ;
    unsigned char data = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return data;
}

void uart_write(unsigned char c) {
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void uart_print(const char* s) {
    while (*s)
        uart_write(*s++);
}

void uart_flush_rx() {
    rx_head = 0;
    rx_tail = 0;
}


/* ═══════════════════════════════════════════════════════════════
 *  LCD: 16x2 HD44780, 4-BIT MODE
 * ═══════════════════════════════════════════════════════════════ */

void lcd_pulse_en() {
    PORTB |= (1 << LCD_EN_BIT);
    delay_us(1);
    PORTB &= ~(1 << LCD_EN_BIT);
    delay_us(50);
}

void lcd_send_nibble(unsigned char nibble) {
    PORTD = (PORTD & ~LCD_DATA_MASK) | (nibble & LCD_DATA_MASK);
    lcd_pulse_en();
}

void lcd_send_byte(unsigned char data, unsigned char rs) {
    if (rs)
        PORTB |= (1 << LCD_RS_BIT);
    else
        PORTB &= ~(1 << LCD_RS_BIT);

    lcd_send_nibble(data & 0xF0);
    lcd_send_nibble((data << 4) & 0xF0);
}

void lcd_command(unsigned char cmd) {
    lcd_send_byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02)
        delay_ms_blocking(2);
    else
        delay_us(50);
}

void lcd_data(unsigned char data) {
    lcd_send_byte(data, 1);
}

void lcd_print(const char* s) {
    while (*s)
        lcd_data(*s++);
}

void lcd_clear() {
    lcd_command(0x01);
}

void lcd_set_cursor(unsigned char col, unsigned char row) {
    unsigned char addr;
    if (row == 0)
        addr = col;
    else
        addr = col + 0x40;
    lcd_command(0x80 | addr);
}

void lcd_init() {
    DDRB  |= (1 << LCD_RS_BIT);
    DDRB  |= (1 << LCD_EN_BIT);
    DDRD  |= LCD_DATA_MASK;

    delay_ms_blocking(50);

    PORTB &= ~(1 << LCD_RS_BIT);

    lcd_send_nibble(0x30);
    delay_ms_blocking(5);
    lcd_send_nibble(0x30);
    delay_us(200);
    lcd_send_nibble(0x30);
    delay_us(200);
    lcd_send_nibble(0x20);
    delay_us(200);

    lcd_command(0x28);
    lcd_command(0x08);
    lcd_command(0x01);
    lcd_command(0x06);
    lcd_command(0x0C);
}


/* ═══════════════════════════════════════════════════════════════
 *  LED & GPIO
 * ═══════════════════════════════════════════════════════════════ */

void led_on()  { PORTB |=  (1 << LED_BIT); }
void led_off() { PORTB &= ~(1 << LED_BIT); }

void gpio_init() {
    DDRB |= (1 << LED_BIT);
    led_off();

    DDRB |= (1 << BUZ_BIT);
    buzzer_off();

    DDRD  &= ~(1 << BTN_BIT);
    PORTD |=  (1 << BTN_BIT);
}

unsigned char button_pressed() {
    return !(PIND & (1 << BTN_BIT));
}


/* ═══════════════════════════════════════════════════════════════
 *  MORSE CODE HELPERS
 * ═══════════════════════════════════════════════════════════════ */

int morse_lookup_index(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a';
    if (c >= '0' && c <= '9') return 26 + (c - '0');
    return -1;
}

char morse_decode_char(const char* code) {
    unsigned char i;
    for (i = 0; i < 36; i++) {
        const char* m = morse_table[i];
        const char* c = code;
        unsigned char match = 1;
        while (*m && *c) {
            if (*m != *c) { match = 0; break; }
            m++;
            c++;
        }
        if (match && *m == '\0' && *c == '\0') {
            if (i < 26) return 'A' + i;
            else return '0' + (i - 26);
        }
    }
    return '?';
}


/* ═══════════════════════════════════════════════════════════════
 *  ENCODING MODE — State Machine
 *  Serial input -> Morse code -> LED + Buzzer
 * ═══════════════════════════════════════════════════════════════ */

void process_encode() {
    unsigned long now = millis_now();

    switch (enc_state) {

    case ENC_IDLE:
        if (uart_available()) {
            unsigned char c = uart_read_byte();
            uart_write(c);

            if (c == '\r' || c == '\n') {
                if (enc_msg_len > 0) {
                    enc_msg[enc_msg_len] = '\0';
                    enc_msg_idx = 0;
                    enc_state = ENC_FETCH;
                    uart_print("\r\nEncoding: ");
                    uart_print(enc_msg);
                    uart_print("\r\n");
                }
            } else if (enc_msg_len < MSG_BUF_SIZE - 1) {
                enc_msg[enc_msg_len++] = c;
            }
        }
        break;

    case ENC_FETCH:
        if (enc_msg_idx >= enc_msg_len) {
            enc_state = ENC_DONE;
            break;
        }
        {
            char ch = enc_msg[enc_msg_idx++];
            int idx;

            if (ch == ' ') {
                lcd_set_cursor(0, 1);
                lcd_print("[SPACE]         ");
                enc_timer = now;
                enc_state = ENC_WORD_GAP;
                break;
            }

            idx = morse_lookup_index(ch);
            if (idx < 0) {
                break;
            }

            enc_pattern = morse_table[idx];
            enc_sym_idx = 0;

            lcd_set_cursor(0, 1);
            if (ch >= 'a' && ch <= 'z') ch = ch - 32;
            lcd_data(ch);
            lcd_data(' ');
            lcd_print(enc_pattern);
            lcd_print("          ");

            led_on();
            buzzer_on();
            enc_timer = now;
            enc_state = ENC_PLAY_ON;
        }
        break;

    case ENC_PLAY_ON:
        {
            unsigned int dur;
            if (enc_pattern[enc_sym_idx] == '.')
                dur = DOT_TIME;
            else
                dur = DASH_TIME;

            if (now - enc_timer >= dur) {
                led_off();
                buzzer_off();
                enc_sym_idx++;

                if (enc_pattern[enc_sym_idx] == '\0') {
                    enc_timer = now;
                    enc_state = ENC_CHAR_GAP;
                } else {
                    enc_timer = now;
                    enc_state = ENC_PLAY_OFF;
                }
            }
        }
        break;

    case ENC_PLAY_OFF:
        if (now - enc_timer >= INTRA_GAP) {
            led_on();
            buzzer_on();
            enc_timer = now;
            enc_state = ENC_PLAY_ON;
        }
        break;

    case ENC_CHAR_GAP:
        if (now - enc_timer >= INTER_GAP) {
            enc_state = ENC_FETCH;
        }
        break;

    case ENC_WORD_GAP:
        if (now - enc_timer >= WORD_GAP) {
            enc_state = ENC_FETCH;
        }
        break;

    case ENC_DONE:
        uart_print("Encoding complete.\r\n");
        uart_print("Switching to DECODE mode.\r\n\r\n");

        enc_msg_len = 0;
        enc_state = ENC_IDLE;

        current_mode = MODE_DECODE;

        dec_state = DEC_IDLE;
        dec_morse_len = 0;
        dec_msg_len = 0;
        dec_message[0] = '\0';
        dec_word_gap_sent = 1;
        dec_release_time = millis_now();

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("DECODE MODE");
        lcd_set_cursor(0, 1);
        lcd_print("Press button...");

        uart_print("DECODE MODE\r\n");
        uart_print("Button: dots & dashes\r\n");
        uart_print("AR (.-.-.) to end\r\n");
        break;
    }
}


/* ═══════════════════════════════════════════════════════════════
 *  DECODING MODE — State Machine
 *  Button presses -> dots/dashes -> decoded text on LCD
 * ═══════════════════════════════════════════════════════════════ */

void dec_display_message() {
    unsigned char start = 0;
    unsigned char i;
    lcd_set_cursor(0, 1);
    if (dec_msg_len > 16) start = dec_msg_len - 16;
    for (i = start; i < dec_msg_len; i++)
        lcd_data(dec_message[i]);
    for (i = dec_msg_len - start; i < 16; i++)
        lcd_data(' ');
}

void dec_display_morse_input() {
    unsigned char i;
    lcd_set_cursor(0, 0);
    lcd_print("IN: ");
    for (i = 0; i < dec_morse_len; i++)
        lcd_data(dec_morse[i]);
    for (i = dec_morse_len; i < 12; i++)
        lcd_data(' ');
}

void process_decode() {
    unsigned long now = millis_now();
    unsigned char btn = button_pressed();

    switch (dec_state) {

    case DEC_IDLE:
        if (btn && (now - dec_debounce > DEBOUNCE_MS)) {
            dec_press_time = now;
            dec_debounce = now;
            dec_word_gap_sent = 0;
            led_on();
            buzzer_on();
            dec_state = DEC_PRESSED;
        }
        if (!btn && dec_morse_len > 0 && (now - dec_release_time >= CHAR_TIMEOUT)) {
            dec_state = DEC_CHAR_DONE;
        }
        if (!btn && dec_morse_len == 0 && dec_msg_len > 0
            && !dec_word_gap_sent
            && (now - dec_release_time >= WORD_TIMEOUT)) {
            if (dec_msg_len < DEC_MSG_SIZE - 1) {
                dec_message[dec_msg_len++] = ' ';
                dec_message[dec_msg_len] = '\0';
                uart_write(' ');
                dec_display_message();
            }
            dec_word_gap_sent = 1;
        }
        break;

    case DEC_PRESSED:
        if (!btn && (now - dec_debounce > DEBOUNCE_MS)) {
            unsigned long press_duration;
            led_off();
            buzzer_off();
            dec_release_time = now;
            dec_debounce = now;

            press_duration = now - dec_press_time;

            if (press_duration < DOT_THRESHOLD) {
                if (dec_morse_len < MORSE_BUF_SIZE - 1)
                    dec_morse[dec_morse_len++] = '.';
            } else {
                if (dec_morse_len < MORSE_BUF_SIZE - 1)
                    dec_morse[dec_morse_len++] = '-';
            }
            dec_morse[dec_morse_len] = '\0';

            dec_display_morse_input();
            dec_state = DEC_RELEASED;
        }
        break;

    case DEC_RELEASED:
        if (btn && (now - dec_debounce > DEBOUNCE_MS)) {
            dec_press_time = now;
            dec_debounce = now;
            led_on();
            buzzer_on();
            dec_state = DEC_PRESSED;
        }
        if (!btn && (now - dec_release_time >= CHAR_TIMEOUT)) {
            dec_state = DEC_CHAR_DONE;
        }
        break;

    case DEC_CHAR_DONE:
        {
            char decoded;

            if (dec_morse_len == 5
                && dec_morse[0] == '.' && dec_morse[1] == '-'
                && dec_morse[2] == '.' && dec_morse[3] == '-'
                && dec_morse[4] == '.') {
                dec_state = DEC_DONE;
                break;
            }

            decoded = morse_decode_char(dec_morse);

            if (dec_msg_len < DEC_MSG_SIZE - 1) {
                dec_message[dec_msg_len++] = decoded;
                dec_message[dec_msg_len] = '\0';
            }

            dec_display_message();
            uart_write(decoded);

            lcd_set_cursor(0, 0);
            lcd_print("DECODE MODE     ");

            dec_morse_len = 0;
            dec_morse[0] = '\0';
            dec_state = DEC_IDLE;
        }
        break;

    case DEC_DONE:
        uart_print("\r\n--- AR received ---\r\n");
        uart_print("Decoded: ");
        uart_print(dec_message);
        uart_print("\r\n\r\n");
        uart_print("Switching to ENCODE mode.\r\n\r\n");

        dec_morse_len = 0;
        dec_msg_len = 0;
        dec_message[0] = '\0';
        dec_state = DEC_IDLE;
        dec_word_gap_sent = 1;

        current_mode = MODE_ENCODE;
        uart_flush_rx();

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("ENCODE MODE");
        lcd_set_cursor(0, 1);
        lcd_print("Type message...");

        uart_print("ENCODE MODE\r\n");
        uart_print("Type message + Enter\r\n");
        break;
    }
}


/* ═══════════════════════════════════════════════════════════════
 *  ENTRY POINTS
 *  Uses setup()/loop() for Tinkercad compatibility.
 *  NO Arduino library functions are called — everything is
 *  direct AVR register manipulation.
 * ═══════════════════════════════════════════════════════════════ */

void setup() {
    gpio_init();
    timer0_init();
    timer1_init();
    uart_init();

    sei();

    lcd_init();

    lcd_set_cursor(0, 0);
    lcd_print("MORSE TERMINAL");
    lcd_set_cursor(0, 1);
    lcd_print("CO321 Project");

    uart_print("\r\n");
    uart_print("================================\r\n");
    uart_print("  MORSE COMMUNICATOR TERMINAL\r\n");
    uart_print("  CO321 Embedded Systems 2026\r\n");
    uart_print("  University of Peradeniya\r\n");
    uart_print("================================\r\n\r\n");

    delay_ms_blocking(2000);

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ENCODE MODE");
    lcd_set_cursor(0, 1);
    lcd_print("Type message...");

    uart_print("ENCODE MODE\r\n");
    uart_print("Type message + Enter\r\n\r\n");
}

void loop() {
    if (current_mode == MODE_ENCODE) {
        process_encode();
    } else {
        process_decode();
    }
}
