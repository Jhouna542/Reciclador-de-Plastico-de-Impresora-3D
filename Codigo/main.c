#define F_CPU 32000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>

/* ---- Parámetros del termistor ---- */
#define BETA        3950.0
#define T0_K        298.15       // 25 ºC en Kelvin
#define R0          100000.0     // 100kΩ a 25 ºC
#define SERIES_R    100000.0     // resistencia en serie 100kΩ
#define ADC_MAX     4095.0       // 12bit resolución
#define VREF_VOLTS  3.3          // referencia ADC

/* ---- Histéresis ---- */
#define TEMP_LOWER  195.0
#define TEMP_UPPER  205.0

/* ---- Pines hardware ---- */
#define TEMP_ADC_CH     0       // ADC0 -> PORTA.0
#define SPEED_ADC_CH    1       // ADC1 -> PORTA.1
#define PIN_HEAT_bm     PIN0_bm // PORTC.0
#define PIN_EN_bm       PIN2_bm // PORTC.2
#define PIN_STEP_bm     PIN3_bm // PORTC.3
#define PIN_DIR_bm      PIN4_bm // PORTC.4
#define PIN_LED_bm      PIN5_bm // PORTC.5
#define PIN_BUT1_bm     PIN7_bm // PORTE.7

volatile bool motor_active = false;
bool heater_on = false;
float set_temperature = 200.0;
float temperature_read = 0.0;

/* ---- Prototipos ---- */
void system_clock_init(void);
void adc_init(void);
uint16_t adc_read(uint8_t ch);
float read_temperature(void);
void io_init(void);
void motor_init(void);
void motor_update_speed(uint16_t adc_val);
void button_check(void);
void heat_control(float temp);

/* ---------------------------------- */
int main(void) {
    system_clock_init();
    adc_init();
    io_init();
    motor_init();

    while (1) {
        button_check();                        // Verificar botón
        temperature_read = read_temperature(); // Leer temperatura
        heat_control(temperature_read);        // Control calefactor

        if (motor_active) {
            uint16_t adc_speed = adc_read(SPEED_ADC_CH);
            motor_update_speed(adc_speed);
        }

        _delay_ms(200);
    }
}

/* ---------------------------------- */
void system_clock_init(void) {
    // Si ya tenés el reloj configurado a 32MHz, no hace falta tocar nada.
}

/* ---- Inicialización ADC ---- */
void adc_init(void) {
    ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;           // referencia interna 1V
    ADCA.PRESCALER = ADC_PRESCALER_DIV512_gc;
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;
    ADCA.CTRLA = ADC_ENABLE_bm;
    ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    _delay_ms(1);
}

/* ---- Lectura de canal ADC ---- */
uint16_t adc_read(uint8_t ch) {
    ADCA.CH0.MUXCTRL = ch;
    ADCA.CH0.CTRL |= ADC_CH_START_bm;
    while (!(ADCA.CH0.INTFLAGS & ADC_CH_IF_bm));
    ADCA.CH0.INTFLAGS = ADC_CH_IF_bm;
    return ADCA.CH0.RES;
}

/* ---- Conversión lectura → temperatura ---- */
float read_temperature(void) {
    uint16_t raw = adc_read(TEMP_ADC_CH);
    float voltage = (raw * VREF_VOLTS) / ADC_MAX;

    if (voltage <= 0.0f)
        voltage = 0.0001f;

    float resistance = SERIES_R * ((VREF_VOLTS / voltage) - 1.0f);
    float tempK = 1.0f / ((1.0f / T0_K) + (1.0f / BETA) * logf(resistance / R0));
    return tempK - 273.15f;
}

/* ---- Inicialización de pines ---- */
void io_init(void) {
    PORTC.DIRSET = PIN_HEAT_bm | PIN_EN_bm | PIN_STEP_bm | PIN_DIR_bm | PIN_LED_bm;
    PORTC.OUTCLR = PIN_HEAT_bm | PIN_EN_bm | PIN_STEP_bm | PIN_LED_bm;

    PORTE.DIRCLR = PIN_BUT1_bm; // botón entrada
    PORTE.PIN7CTRL = PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm;
}

/* ---- Motor ---- */
void motor_init(void) {
    PORTC.OUTCLR = PIN_EN_bm; // deshabilitado al inicio
}

/* ---- Actualizar velocidad motor ---- */
void motor_update_speed(uint16_t adc_val) {
    uint32_t speed = adc_val;
    if (speed < 1000) speed = 1000;
    if (speed > 5000) speed = 5000;

    PORTC.OUTSET = PIN_EN_bm;   // habilitar driver
    PORTC.OUTSET = PIN_LED_bm;  // LED encendido
    PORTC.OUTSET = PIN_STEP_bm;
    _delay_us(speed);
    PORTC.OUTCLR = PIN_STEP_bm;
    _delay_us(speed);
}

/* ---- Leer botón ---- */
void button_check(void) {
    static bool last_state = true;
    bool pressed = !(PORTE.IN & PIN_BUT1_bm);

    if (pressed && last_state) {
        motor_active = !motor_active;
        last_state = false;
    } else if (!pressed && !last_state) {
        last_state = true;
    }
}

/* ---- Control calefactor (sin PWM) ---- */
void heat_control(float temp) {
    if (heater_on) {
        if (temp >= TEMP_UPPER) {
            PORTC.OUTCLR = PIN_HEAT_bm;
            heater_on = false;
        }
    } else {
        if (temp <= TEMP_LOWER) {
            PORTC.OUTSET = PIN_HEAT_bm;
            heater_on = true;
        }
    }
}
