/*
 * File:   main.c
 * Target: PIC16F1719 on Explorer 8
 * Compiler: XC8
 *
 * Project: YOLO adaptive traffic light (RED/GREEN) + LCD UI + pedestrian button
 *
 * Explorer 8 LCD is connected via MCP23S17:
 *   GPA6 -> E
 *   GPA7 -> RS
 *   GPB0..GPB7 -> DB0..DB7 (8-bit mode)
 *
 * Jumpers required for LCD:
 *   J59: RA2 -> MCP23S17 CS
 *   J60: RB5 -> MCP23S17 RESET
 *   J61: LCD power
 *
 * USB-UART:
 *   Ensure J54 is installed (USB-to-UART routing).
 *   Python sends once per second: C:<n>\\n  (e.g. C:07\\n)
 *
 * LEDs:
 *   Explorer 8 D1..D4 are on RD0..RD3. We use:
 *     D1 (RD0) = RED
 *     D2 (RD1) = GREEN
 *
 * Button:
 *   S1 is RB0 (active-low).
 */

#include <xc.h>
#include <stdint.h>

// Configuration Bits (PIC16F1719 - internal oscillator, no WDT, no LVP)
#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config BOREN = OFF
#pragma config CLKOUTEN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config WRT = OFF
#pragma config PPS1WAY = OFF
#pragma config ZCDDIS = ON
#pragma config PLLEN = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config LVP = OFF

// System clock used by __delay_ms/us
#define _XTAL_FREQ 8000000

// MCP23S17 Registers (SPI I/O expander for LCD)
#define MCP_IODIRA   0x00
#define MCP_IODIRB   0x01
#define MCP_IOCON    0x0A
#define MCP_GPIOA    0x12
#define MCP_GPIOB    0x13
#define MCP_OLATA    0x14
#define MCP_OLATB    0x15

// MCP23S17 SPI Address (A0=A1=A2=0 on Explorer 8)
#define MCP_WRITE    0x40
#define MCP_READ     0x41

// LCD mapping on MCP23S17 (Explorer 8 schematic)
#define LCD_E_BIT    0x40  // GPA6
#define LCD_RS_BIT   0x80  // GPA7

// Chip Select + Reset (via jumpers J59/J60)
#define CS_LOW()     LATAbits.LATA2 = 0
#define CS_HIGH()    LATAbits.LATA2 = 1
#define RESET_LOW()  LATBbits.LATB5 = 0
#define RESET_HIGH() LATBbits.LATB5 = 1

// Delay helpers (used for LCD timing and SPI settle time)
void delay_ms(unsigned int ms) {
    while(ms--) {
        __delay_ms(1);
    }
}

void delay_us(unsigned int us) {
    while(us--) {
        __delay_us(1);
    }
}

// ---------------------------------------------------------------------------
// SPI Functions (SSP1) -> MCP23S17
// ---------------------------------------------------------------------------
void SPI_Init(void) {
    // PPS mapping for SPI signals
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    RC3PPS = 0x10;      // RC3 = SCK
    RC5PPS = 0x11;      // RC5 = SDO
    SSPDATPPS = 0x14;   // RC4 = SDI
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    // SPI pins direction
    TRISCbits.TRISC3 = 0;  // SCK output
    TRISCbits.TRISC4 = 1;  // SDI input
    TRISCbits.TRISC5 = 0;  // SDO output
    
    // CS + RESET pins
    TRISAbits.TRISA2 = 0;  // CS output (J59)
    TRISBbits.TRISB5 = 0;  // RESET output (J60)
    CS_HIGH();
    RESET_HIGH();
    
    // SPI Master mode, Fosc/16 (stable for MCP23S17)
    SSP1CON1 = 0x21;  // SPI Master, Fosc/16, enabled
    SSP1STAT = 0x40;  // CKE=1
}

unsigned char SPI_Transfer(unsigned char data) {
    SSP1BUF = data;
    while(!SSP1STATbits.BF);
    return SSP1BUF;
}

// ---------------------------------------------------------------------------
// MCP23S17 Functions
// ---------------------------------------------------------------------------
void MCP_Write(unsigned char reg, unsigned char data) {
    CS_LOW();
    delay_us(1);
    SPI_Transfer(MCP_WRITE);
    SPI_Transfer(reg);
    SPI_Transfer(data);
    delay_us(1);
    CS_HIGH();
}

void MCP_Init(void) {
    // Reset expander (J60)
    RESET_LOW();
    delay_ms(2);
    RESET_HIGH();
    delay_ms(2);
    
    // Enable HAEN (address pins) in IOCON
    MCP_Write(MCP_IOCON, 0x08);
    
    // Set Port A and Port B as outputs (LCD uses all pins)
    MCP_Write(MCP_IODIRA, 0x00);
    MCP_Write(MCP_IODIRB, 0x00);
    MCP_Write(MCP_OLATA, 0x00);
    MCP_Write(MCP_OLATB, 0x00);
}

// ---------------------------------------------------------------------------
// LCD Functions (8-bit via MCP23S17)
// ---------------------------------------------------------------------------
unsigned char lcd_porta = 0;
unsigned char lcd_portb = 0;

void LCD_Pulse(void) {
    lcd_porta |= LCD_E_BIT;
    MCP_Write(MCP_OLATA, lcd_porta);
    delay_us(10);
    lcd_porta &= ~LCD_E_BIT;
    MCP_Write(MCP_OLATA, lcd_porta);
    delay_us(100);
}

void LCD_Write8(unsigned char value) {
    lcd_portb = value;           // DB0..DB7
    MCP_Write(MCP_OLATB, lcd_portb);
    LCD_Pulse();
}

void LCD_Command(unsigned char cmd) {
    lcd_porta &= ~LCD_RS_BIT;    // RS = 0 (command)
    MCP_Write(MCP_OLATA, lcd_porta);
    LCD_Write8(cmd);
    delay_ms(2);
}

void LCD_Char(char c) {
    lcd_porta |= LCD_RS_BIT;     // RS = 1 (data)
    MCP_Write(MCP_OLATA, lcd_porta);
    LCD_Write8((unsigned char)c);
    delay_ms(1);
}

void LCD_Init(void) {
    delay_ms(100);
    
    lcd_porta = 0;
    lcd_portb = 0;
    MCP_Write(MCP_OLATA, lcd_porta);
    MCP_Write(MCP_OLATB, lcd_portb);
    delay_ms(50);
    
    // 8-bit mode init sequence
    LCD_Command(0x38);  // Function set: 8-bit, 2-line
    delay_ms(5);
    LCD_Command(0x38);
    delay_ms(1);
    LCD_Command(0x38);
    delay_ms(1);
    
    LCD_Command(0x0C);  // Display ON, cursor OFF
    LCD_Command(0x06);  // Entry mode
    LCD_Command(0x01);  // Clear display
    delay_ms(5);
}

void LCD_String(const char *s) {
    while(*s) {
        LCD_Char(*s++);
    }
}

void LCD_SetCursor(unsigned char row, unsigned char col) {
    LCD_Command(row == 0 ? (0x80 + col) : (0xC0 + col));
}

// ---------------------------------------------------------------------------
// UART (EUSART) - 9600 baud @ 8MHz
// Protocol: "C:<n>\\n" where n is 00..99 (vehicle density)
// ---------------------------------------------------------------------------
static void UART_Init(void) {
    // RX/TX pins are fixed on PIC16F1719 (RC7=RX, RC6=TX).
    ANSELC = 0;
    TRISCbits.TRISC7 = 1; // RX
    TRISCbits.TRISC6 = 0; // TX

    // Baud: BRG16=1, BRGH=1 -> Baud = Fosc / (4*(SPBRG+1))
    // SPBRG ~= (8,000,000/(4*9600)) - 1 ~= 207.33 -> 207
    BAUD1CONbits.BRG16 = 1;
    TX1STAbits.BRGH = 1;
    SP1BRGH = 0;
    SP1BRGL = 207;

    RC1STAbits.SPEN = 1;
    TX1STAbits.SYNC = 0;
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;
}

static inline uint8_t UART_Available(void) {
    return PIR1bits.RCIF;
}

static uint8_t UART_Read(void) {
    // Clear overrun to keep RX alive
    if (RC1STAbits.OERR) {
        RC1STAbits.CREN = 0;
        RC1STAbits.CREN = 1;
    }
    return (uint8_t)RC1REG;
}

static void UART_Write(uint8_t b) {
    while (!PIR1bits.TXIF) {;}
    TX1REG = b;
}

static void UART_WriteStr(const char* s) {
    while (*s) UART_Write((uint8_t)*s++);
}

// ---------------------------------------------------------------------------
// 1-second tick via Timer1 (10ms base)
// Fosc=8MHz => Fosc/4=2MHz, prescale 1:8 => tick=4us
// 10ms => 2500 ticks, preload = 65536-2500 = 63036 = 0xF63C
// This keeps the main loop non-blocking and enables clean state transitions.
// ---------------------------------------------------------------------------
volatile uint8_t g_tick_10ms = 0;
volatile uint8_t g_tick_1s = 0;

void __interrupt() isr(void) {
    if (PIR1bits.TMR1IF) {
        TMR1H = 0xF6;
        TMR1L = 0x3C;
        PIR1bits.TMR1IF = 0;

        g_tick_10ms++;
        if (g_tick_10ms >= 100) {
            g_tick_10ms = 0;
            g_tick_1s = 1;
        }
    }
}

static void Timer1_Init_10ms(void) {
    // Timer1 internal clock, prescaler 1:8
    T1CONbits.TMR1CS = 0;    // Fosc/4
    T1CONbits.T1CKPS1 = 1;
    T1CONbits.T1CKPS0 = 1;   // 1:8

    TMR1H = 0xF6;
    TMR1L = 0x3C;

    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    T1CONbits.TMR1ON = 1;
}

// ---------------------------------------------------------------------------
// Traffic logic + UI
// RED/GREEN only (no yellow). RED can be extended when density is high.
// ---------------------------------------------------------------------------
typedef enum { PH_RED = 0, PH_GREEN = 1 } Phase;

// LED mapping (Explorer 8: D1=RD0, D2=RD1)
#define LED_RED_LAT   LATDbits.LATD0
#define LED_GRN_LAT   LATDbits.LATD1

static void set_phase(Phase ph) {
    if (ph == PH_RED) {
        LED_RED_LAT = 1;
        LED_GRN_LAT = 0;
    } else {
        LED_RED_LAT = 0;
        LED_GRN_LAT = 1;
    }
}

// Globals updated by UART (vehicle density 0..99)
volatile uint8_t vehicle_count = 0;

static void lcd_print_2d(uint8_t v) {
    LCD_Char('0' + (v / 10));
    LCD_Char('0' + (v % 10));
}

static void lcd_print_phase(Phase ph) {
    if (ph == PH_RED) {
        LCD_String("RED");
    } else {
        LCD_String("GRN");
    }
}

static void lcd_fill_spaces(uint8_t n) {
    while (n--) LCD_Char(' ');
}

static void lcd_draw(Phase ph, uint8_t remaining, uint8_t cars, uint8_t ped, uint8_t ext_on) {
    // Line1: PH:RED T:12
    LCD_SetCursor(0, 0);
    LCD_String("PH:");
    lcd_print_phase(ph);
    LCD_String(" T:");
    lcd_print_2d(remaining);
    lcd_fill_spaces(16 - 3 - 3 - 3 - 2); // rough pad

    // Line2: CAR:07 PED:1 EXT
    LCD_SetCursor(1, 0);
    LCD_String("CAR:");
    lcd_print_2d(cars);
    LCD_String(" PED:");
    LCD_Char(ped ? '1' : '0');
    LCD_String(" ");
    LCD_String(ext_on ? "EXT" : "   ");
    lcd_fill_spaces(16 - 4 - 2 - 5 - 1 - 3); // pad
}

// UART line parser: expects "C:<n>" where n is 00..99
static void parse_line_and_update(const char* line) {
    if (line[0] != 'C' || line[1] != ':') return;
    int val = 0;
    if (line[2] >= '0' && line[2] <= '9') {
        val = line[2] - '0';
    } else return;
    if (line[3] >= '0' && line[3] <= '9') {
        val = val * 10 + (line[3] - '0');
    }
    if (val < 0) val = 0;
    if (val > 99) val = 99;
    vehicle_count = (uint8_t)val;

    // Optional ACK for debug (enable if needed)
    // UART_WriteStr("ACK\n");
}

int main(void) {
    OSCCON = 0x72;  // 8 MHz
    
    // All digital
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    
    // LEDs on PORTD (D1/D2)
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    LED_RED_LAT = 0;
    LED_GRN_LAT = 0;

    // Button S1 on RB0 (active-low)
    TRISBbits.TRISB0 = 1;

    // Init peripherals (UART -> Timer1 -> SPI/LCD)
    UART_Init();
    Timer1_Init_10ms();
    SPI_Init();
    MCP_Init();
    LCD_Init();

    UART_WriteStr("PIC READY\n");

    // Traffic constants (tune here)
    const uint8_t GREEN_BASE = 10;
    const uint8_t RED_BASE = 10;
    const uint8_t RED_MAX = 30;
    const uint8_t EXT_STEP = 3;      // seconds to add per decision
    const uint8_t THRESHOLD = 20;    // cars threshold to extend red
    const uint8_t PED_HOLD = 5;      // seconds forced red on button press

    Phase phase = PH_GREEN;
    uint8_t remaining = GREEN_BASE;
    uint8_t red_extended_total = 0;

    uint8_t ped_active = 0;
    uint8_t ped_remaining = 0;

    // UART line buffer (small, line-based)
    char line[16];
    uint8_t li = 0;

    set_phase(phase);
    lcd_draw(phase, remaining, vehicle_count, 0, 0);

    while (1) {
        // Non-blocking UART receive + line parsing
        while (UART_Available()) {
            char c = (char)UART_Read();
            if (c == '\r') continue;
            if (c == '\n') {
                line[li] = '\0';
                parse_line_and_update(line);
                li = 0;
            } else if (li < (sizeof(line) - 1)) {
                line[li++] = c;
            } else {
                li = 0; // overflow -> reset
            }
        }

        if (!g_tick_1s) continue;
        g_tick_1s = 0;

        // Debounced pedestrian (simple): trigger when pressed
        static uint8_t last_btn = 1;
        uint8_t btn = PORTBbits.RB0;
        uint8_t ped_req = 0;
        if (last_btn == 1 && btn == 0) {
            delay_ms(10);
            if (PORTBbits.RB0 == 0) ped_req = 1;
        }
        last_btn = btn;

        if (ped_req && !ped_active) {
            // Force RED immediately for pedestrian request
            ped_active = 1;
            ped_remaining = PED_HOLD;
            phase = PH_RED;
            remaining = (remaining < PED_HOLD) ? PED_HOLD : remaining;
            set_phase(phase);
        }

        // Tick down
        if (remaining > 0) remaining--;
        if (ped_active && ped_remaining > 0) ped_remaining--;
        if (ped_active && ped_remaining == 0) {
            ped_active = 0;
        }

        // Adaptive: extend RED only while in RED and above threshold
        uint8_t ext_on = 0;
        if (phase == PH_RED && vehicle_count >= THRESHOLD && remaining < RED_MAX) {
            uint8_t add = EXT_STEP;
            while (add-- && remaining < RED_MAX) {
                remaining++;
                red_extended_total++;
            }
            ext_on = 1;
        }

        // Phase transitions when remaining time reaches 0
        if (remaining == 0) {
            if (phase == PH_GREEN) {
                phase = PH_RED;
                remaining = RED_BASE;
                red_extended_total = 0;
            } else {
                phase = PH_GREEN;
                remaining = GREEN_BASE;
            }
            set_phase(phase);
        }

        lcd_draw(phase, remaining, vehicle_count, ped_active, (ext_on || red_extended_total > 0));
    }
}
