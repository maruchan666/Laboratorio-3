#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "driver/uart.h"

/* ── Definiciones de pines y parámetros ── */
#define SERIAL_PORT       UART_NUM_0
#define ADC_INTERVAL_US   1000
#define PIN_BULB          23
#define PIN_PWM_LED       14
#define PIN_A1            19
#define PIN_A2            18
#define PIN_B1            5
#define PIN_B2            17

volatile bool dir_motor = 0;
adc_oneshot_unit_handle_t adc_handle;

uint8_t seq[8][4] = {
    {1, 0, 1, 0},
    {1, 0, 0, 0},
    {1, 0, 0, 1},
    {0, 0, 0, 1},
    {0, 1, 0, 1},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0}
};

void avanzar_paso(bool sentido) {
    static int paso = 0;
    paso = (sentido == 0) ? (paso + 1) % 8 : (paso + 7) % 8;
    gpio_set_level(PIN_A1, seq[paso][0]);
    gpio_set_level(PIN_A2, seq[paso][1]);
    gpio_set_level(PIN_B1, seq[paso][2]);
    gpio_set_level(PIN_B2, seq[paso][3]);
}

static bool IRAM_ATTR isr_timer(void *arg) {
    avanzar_paso(dir_motor);
    return false;
}

static inline void motor_off(void) {
    gpio_set_level(PIN_A1, 0);
    gpio_set_level(PIN_A2, 0);
    gpio_set_level(PIN_B1, 0);
    gpio_set_level(PIN_B2, 0);
}

void app_main(void) {

    /* 1. UART — primero para poder imprimir mensajes de depuración desde el inicio */
    uart_config_t cfg_uart = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(SERIAL_PORT, &cfg_uart);
    uart_driver_install(SERIAL_PORT, 1024, 1024, 0, NULL, 0);

    /* Mensaje de bienvenida justo después de tener UART listo */
    char *bienvenida = "\nSistema de control de temperatura e iluminación\n";
    uart_write_bytes(SERIAL_PORT, bienvenida, strlen(bienvenida));

    /* 2. ADC — sensores de temperatura y luz */
    adc_oneshot_unit_init_cfg_t cfg_adc_unit = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&cfg_adc_unit, &adc_handle);

    adc_oneshot_chan_cfg_t cfg_canal = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &cfg_canal); // GPIO34 — LDR
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_7, &cfg_canal); // GPIO35 — LM35

    /* 3. PWM — control de brillo del LED de potencia */
    ledc_timer_config_t cfg_pwm_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&cfg_pwm_timer);

    ledc_channel_config_t cfg_pwm_canal = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_PWM_LED,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&cfg_pwm_canal);

    /* 4. GPIO — pines de salida digital */
    gpio_config_t cfg_gpio = {
        .pin_bit_mask = (1ULL << PIN_BULB) |
                        (1ULL << PIN_A1)   |
                        (1ULL << PIN_A2)   |
                        (1ULL << PIN_B1)   |
                        (1ULL << PIN_B2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_gpio);

    gpio_set_level(PIN_BULB, 0);
    motor_off();

    /* 5. Timers del grupo 0 — consola y muestreo ADC */
    timer_config_t cfg_timer = {
        .divider     = 80,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_DIS,
        .auto_reload = false,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &cfg_timer);
    timer_init(TIMER_GROUP_0, TIMER_1, &cfg_timer);

    /* 6. Timer del grupo 1 — interrupción para el motor paso a paso */
    timer_config_t cfg_isr = {
        .divider     = 80,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(TIMER_GROUP_1, TIMER_0, &cfg_isr);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    uint64_t alarma_inicial = 1000000;
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, alarma_inicial);
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, isr_timer, NULL, 0);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_start(TIMER_GROUP_1, TIMER_0);

    /* Arranque de timers de visualización y muestreo */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_start(TIMER_GROUP_0, TIMER_1);

    /* ── Variables de control ── */
    uint64_t tick        = 0;
    uint64_t tick_serial = 0;
    uint64_t tick_adc    = 0;
    int raw_ldr = 0, raw_lm35 = 0;
    int duty_val = 0;
    int duty_pct = 0;
    int luz_pct  = 0;
    int vel_pasos    = 100;
    int vel_anterior = -1;
    int temp_med  = 0;
    int temp_set  = 0;

    /* ── Bucle principal ── */
    while (1) {

        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &tick);
        timer_get_counter_value(TIMER_GROUP_0, TIMER_1, &tick_adc);

        /* Recepción de temperatura objetivo — va primero, independiente del ADC */
        static char buf_rx[16];
        static int  pos = 0;
        uint8_t byte_rx;
        if (uart_read_bytes(SERIAL_PORT, &byte_rx, 1, 0) > 0) {
            if (byte_rx == '\n' || byte_rx == '\r') {
                buf_rx[pos] = '\0';
                int val = atoi(buf_rx);
                if (val > 0 && val < 100) temp_set = val;
                pos = 0;
            } else if (pos < (int)sizeof(buf_rx) - 1) {
                buf_rx[pos++] = byte_rx;
            }
        }

        /* Lectura periódica de sensores cada ADC_INTERVAL_US */
        if (tick_adc >= ADC_INTERVAL_US) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &raw_ldr);
            adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &raw_lm35);
            luz_pct = (raw_ldr * 100) / 4095;
            timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
        }

        /* Ajuste del duty cycle del LED según nivel de luz ambiente */
        if      (luz_pct <= 20)                    duty_pct = 100;
        else if (luz_pct > 20 && luz_pct <= 30)    duty_pct = 80;
        else if (luz_pct > 30 && luz_pct <= 40)    duty_pct = 60;
        else if (luz_pct > 40 && luz_pct <= 60)    duty_pct = 50;
        else if (luz_pct > 60 && luz_pct <= 80)    duty_pct = 30;
        else                                        duty_pct = 0;

        duty_val = (duty_pct * 4095) / 100;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_val);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        /* LM35: 10 mV/°C → escala entera: (raw × 330) / 4095 */
        temp_med = (raw_lm35 * 330) / 4095;

        /* Lógica de control: bombilla y velocidad/dirección del motor */
        if (temp_med >= temp_set - 1 && temp_med <= temp_set + 1) {
            gpio_set_level(PIN_BULB, 0);
            motor_off();
        } else if (temp_med < temp_set - 1) {
            gpio_set_level(PIN_BULB, 1);
            vel_pasos = 100;
            dir_motor = 0;
        } else if (temp_med > temp_set + 1 && temp_med < temp_set + 3) {
            gpio_set_level(PIN_BULB, 0);
            vel_pasos = 100;
            dir_motor = 1;
        } else if (temp_med >= temp_set + 3 && temp_med <= temp_set + 5) {
            gpio_set_level(PIN_BULB, 0);
            vel_pasos = 300;
            dir_motor = 1;
        } else if (temp_med > temp_set + 5) {
            gpio_set_level(PIN_BULB, 0);
            vel_pasos = 600;
            dir_motor = 1;
        }

        /* Actualización del timer solo si cambió la velocidad */
        if (vel_pasos != vel_anterior) {
            timer_disable_intr(TIMER_GROUP_1, TIMER_0);
            if (vel_pasos > 0) {
                uint64_t periodo = 1000000ULL / ((uint64_t)vel_pasos * 2);
                timer_pause(TIMER_GROUP_1, TIMER_0);
                timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
                timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, periodo);
                timer_start(TIMER_GROUP_1, TIMER_0);
            } else {
                timer_pause(TIMER_GROUP_1, TIMER_0);
            }
            timer_enable_intr(TIMER_GROUP_1, TIMER_0);
            vel_anterior = vel_pasos;
        }

        /* Refresco de consola cada 2 s — al final, es solo presentación */
        if (tick - tick_serial >= 2000000) {
            char linea[32];

            uart_write_bytes(SERIAL_PORT, "Temp. objetivo : ", 17);
            sprintf(linea, "%d °C\n", temp_set);
            uart_write_bytes(SERIAL_PORT, linea, strlen(linea));

            uart_write_bytes(SERIAL_PORT, "Temp. medida   : ", 17);
            sprintf(linea, "%d °C\n", temp_med);
            uart_write_bytes(SERIAL_PORT, linea, strlen(linea));

            uart_write_bytes(SERIAL_PORT, "Luz ambiente   : ", 17);
            sprintf(linea, "%d %%\n", luz_pct);
            uart_write_bytes(SERIAL_PORT, linea, strlen(linea));

            tick_serial = tick;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}