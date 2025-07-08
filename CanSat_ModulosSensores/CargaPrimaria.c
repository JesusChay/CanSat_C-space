#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include <math.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <stdbool.h>

// Configuracion para el sensor de viento Wind Sensor Rev. C
#define WIND_ADC_CHANNEL ADC1_CHANNEL_0  // GPIO36
#define V_REF 3.3f
#define ADC_MAX 4095.0f
#define MPH_TO_KPH 1.60934f

float zero_wind_voltage = 1.25f;   // Calibracion en condiciones de viento cero
float wind_factor = 0.2300f;       // Ajustar para mayor sensibilidad
float wind_exponent = 2.7265f;     
float threshold = 0.01f;           // Umbral mínimo de detección (en voltios)     0.01f es más sensible, 0.03f más estable

// Configuracion de I2C para el IMU GY-87
#define I2C_MASTER_SCL_IO 22        // GPIO para SCL
#define I2C_MASTER_SDA_IO 21        // GPIO para SDA
#define I2C_MASTER_FREQ_HZ 100000   // Frecuencia de I2C
#define I2C_MASTER_PORT_NUM I2C_NUM_0

// Direcciones I2C
#define MPU6050_ADDR 0x68           // Dirección del MPU6050
#define HMC5883L_ADDR 0x1E          // Dirección del HMC5883L
#define BMP180_ADDR 0x77            // Dirección del BMP180

// Registros MPU6050
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_ACCEL_CONFIG 0x1C   // Registro de configuración del acelerómetro
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// Registros HMC5883L
#define HMC5883L_CONFIG_A 0x00
#define HMC5883L_CONFIG_B 0x01
#define HMC5883L_MODE 0x02
#define HMC5883L_DATA_X_H 0x03
#define HMC5883L_DATA_X_L 0x04
#define HMC5883L_DATA_Z_H 0x05
#define HMC5883L_DATA_Z_L 0x06
#define HMC5883L_DATA_Y_H 0x07
#define HMC5883L_DATA_Y_L 0x08
#define HMC5883L_STATUS 0x09
#define HMC5883L_ID_A 0x0A

// Registros BMP180
#define BMP180_CAL_AC1 0xAA
#define BMP180_CAL_AC2 0xAC
#define BMP180_CAL_AC3 0xAE
#define BMP180_CAL_AC4 0xB0
#define BMP180_CAL_AC5 0xB2
#define BMP180_CAL_AC6 0xB4
#define BMP180_CAL_B1 0xB6
#define BMP180_CAL_B2 0xB8
#define BMP180_CAL_MB 0xBA
#define BMP180_CAL_MC 0xBC
#define BMP180_CAL_MD 0xBE
#define BMP180_CONTROL 0xF4
#define BMP180_TEMPDATA 0xF6
#define BMP180_PRESSUREDATA 0xF6

// Comandos BMP180
#define BMP180_READTEMPCMD 0x2E
#define BMP180_READPRESSURECMD 0x34
#define PRESSURE_READINGS 5

// Modos de operación BMP180 (oversampling)
#define BMP180_ULTRALOWPOWER 0
#define BMP180_STANDARD 1
#define BMP180_HIGHRES 2
#define BMP180_ULTRAHIGHRES 3

// Variables de calibración BMP180
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

// Configuración de UART para el GT-U7
#define UART_PORT UART_NUM_2
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define BUF_SIZE 256
#define FILTER_WINDOW_SIZE 10 // Tamaño de la ventana del filtro

//static const char *WS = "WindSensor";
//static const char *GY = "GY87";
//static const char *GT = "GPS";
static const char *ESPNOW = "ESPNOW_SEND_PRIMARY";

// Factores de conversión GY-87
const float ACCEL_SCALE_2G = 16384.0f;    // LSB/g para ±2g
const float GYRO_SCALE_250DPS = 131.0f;   // LSB/°/s para ±250°/s
const float HMC5883L_GAIN_1_3GA = 1090.0f; // LSB/Gauss para ±1.3Ga
const float GAUSS_TO_MICROTESLA = 100.0f;  // 1 Gauss = 100 μT

// Configuracion del GY-87
uint8_t who_am_i;
int16_t accel[3], gyro[3], mag[3];
float accel_scale = ACCEL_SCALE_2G; // Valor por defecto (±2g)
float gyro_scale = GYRO_SCALE_250DPS;

// Variables globales
float speed = 0;
float last_speed = 0;
const float response_factor = 0.5f;  // Factor de respuesta (0.1-0.5) 0.3 es estándar, 0.5 es más rápido

float mag_bias[3] = {0, 0, 0};
float mag_scale[3] = {1.0, 1.0, 1.0};

float accelx = 0, accely = 0, accelz = 0;
float gyrox = 0, gyroy = 0, gyroz = 0;
float magx = 0, magy = 0, magz = 0; 
float mag_cal[3];

float latitude, longitude, altitudee, filtered_altitudee;

// Variables para el filtro de media móvil
float altitude_buffer[FILTER_WINDOW_SIZE] = {0};
int buffer_index = 0;
float altitude_sum = 0;

// Filtro de media movil
float pressure_history[PRESSURE_READINGS] = {0};
int pressure_index = 0;

// Filtro de presion
#define PRESSURE_WINDOW_SIZE 10
float pressure_buffer[PRESSURE_WINDOW_SIZE] = {0};
float pressure_sum = 0.0f;
int pressure_index_presion = 0;

// Variables para cálculo de altitud BMP del GY-87
#define INITIAL_READINGS_BMP 5
float initial_pressures_bmp[INITIAL_READINGS_BMP] = {0};
int pressure_readings_count_bmp = 0;
float prom_press_bmp = 97700.00; // Valor inicial por defecto
float filtered_altitude_bmp = 0.0f;
float altitud = 0.0f;

#define ALTITUDE_WINDOW_SIZE 10
int altitude_index = 0;
#define MAX_ALTITUDE_JUMP 10.0f  // máximo cambio permitido entre lecturas


// Variables de desacoplamiento
bool decoupling_bmp = false;
bool decoupling_status = false;


// Rele
#define LED_PIN GPIO_NUM_27

// Identificador para el receptor
typedef enum {
    PRIMARY_LOAD = 1,
    SECONDARY_LOAD = 2
} load_type_t;

// Estructura de datos a enviar
typedef struct {
	load_type_t load_type;
    float speed;
    float accelx, accely, accelz;
    float gyrox, gyroy, gyroz;
    float magx, magy, magz;
    float altitud;
    float latitude;
    float longitude;
    bool decoupling_status;
} primary_load_data_t;

// MAC del receptor
static uint8_t peer_mac[] =  {0x68, 0x25, 0xDD, 0xF0, 0x25, 0xD0};

void init_led_gpio() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  Wind Sensor Rev. C                                                         //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(WIND_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

float read_wind_voltage() {
    return (adc1_get_raw(WIND_ADC_CHANNEL) / ADC_MAX) * V_REF;
}

float calculate_wind_speed(float voltage) {
    float voltage_diff = voltage - zero_wind_voltage;
    
    // Aplicar umbral de detección
    if (voltage_diff < threshold) return 0.0f;
    
    // Fórmula de conversión optimizada
    return powf((voltage_diff / wind_factor), wind_exponent) * MPH_TO_KPH;
}

void calibrate_zero_wind() {
    //ESP_LOGI(WS, "Iniciando calibración (mantener el sensor sin viento)...");
    float sum = 0;
    const int samples = 20;
    
    for (int i = 0; i < samples; i++) {
        sum += read_wind_voltage();
        vTaskDelay(pdMS_TO_TICKS(50));  // Muestras rápidas
    }
    
    zero_wind_voltage = sum / samples;
    //ESP_LOGI(WS, "Calibración completada. Voltaje base: %.3fV", zero_wind_voltage);
}

void read_WindSensorRevC_data(){
	float voltage = read_wind_voltage();
    float current_speed = calculate_wind_speed(voltage);
    
    // Aplicar filtro de respuesta rápida (no promedio móvil)
    speed = last_speed + (current_speed - last_speed) * response_factor;
    last_speed = speed;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                       IMU GY-87                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT_NUM, conf.mode, 0, 0, 0));
}

esp_err_t i2c_register_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_register_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true));
    if (len > 1) {
        ESP_ERROR_CHECK(i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void enable_bypass_mode() {
    // 1. Deshabilitar el control del I2C maestro (USER_CTRL register)
    i2c_register_write(MPU6050_ADDR, MPU6050_USER_CTRL, 0x00);
    
    // 2. Habilitar el modo bypass (INT_PIN_CFG register)
    i2c_register_write(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02);
    
    // 3. Configurar el reloj (PWR_MGMT_1 register)
    i2c_register_write(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
}

void mpu6050_init() {
    // Despertar el MPU6050
    i2c_register_write(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    
    // Configurar el rango del acelerómetro (±2g)
    i2c_register_write(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void hmc5883l_init() {
    // Configurar el magnetómetro
    i2c_register_write(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x78); // 75Hz, 8 muestras promedio
    i2c_register_write(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20); // Ganancia ±1.3Ga (1090 LSB/Gauss)
    i2c_register_write(HMC5883L_ADDR, HMC5883L_MODE, 0x00);     // Modo de medición continua
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void bmp180_read_calibration_data() {
    uint8_t cal_data[22];
    
    if (i2c_register_read(BMP180_ADDR, BMP180_CAL_AC1, cal_data, 22) == ESP_OK) {
        ac1 = (cal_data[0] << 8) | cal_data[1];
        ac2 = (cal_data[2] << 8) | cal_data[3];
        ac3 = (cal_data[4] << 8) | cal_data[5];
        ac4 = (cal_data[6] << 8) | cal_data[7];
        ac5 = (cal_data[8] << 8) | cal_data[9];
        ac6 = (cal_data[10] << 8) | cal_data[11];
        b1 = (cal_data[12] << 8) | cal_data[13];
        b2 = (cal_data[14] << 8) | cal_data[15];
        mb = (cal_data[16] << 8) | cal_data[17];
        mc = (cal_data[18] << 8) | cal_data[19];
        md = (cal_data[20] << 8) | cal_data[21];
    }
}

bool bmp180_is_ready() {
    uint8_t status;
    if(i2c_register_read(BMP180_ADDR, 0xF4, &status, 1) == ESP_OK) {
        return (status & 0x20) == 0; // Bit 5 debe ser 0 cuando está listo
    }
    return false;
}

bool bmp180_begin() {
    uint8_t id;
    if(i2c_register_read(BMP180_ADDR, 0xD0, &id, 1) != ESP_OK || id != 0x55) {
        return false;
    }
    bmp180_read_calibration_data();
    return true;
}

int32_t bmp180_read_raw_temp() {
    // Esperar ready
    int timeout = 100;
    while(!bmp180_is_ready() && timeout-- > 0) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // Enviar comando
    if(i2c_register_write(BMP180_ADDR, BMP180_CONTROL, BMP180_READTEMPCMD) != ESP_OK) {
        return -1;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // 4.5ms según datasheet

    // Leer datos
    uint8_t data[2];
    if(i2c_register_read(BMP180_ADDR, BMP180_TEMPDATA, data, 2) != ESP_OK) {
        return -1;
    }

    return (data[0] << 8) | data[1];
}

int32_t bmp180_read_raw_pressure(uint8_t oss) {
    // Esperar hasta que el sensor esté listo
    int timeout = 100;
    while(!bmp180_is_ready() && timeout-- > 0) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // Escribir comando
    uint8_t cmd = BMP180_READPRESSURECMD + (oss << 6);
    if(i2c_register_write(BMP180_ADDR, BMP180_CONTROL, cmd) != ESP_OK) {
        return -1;
    }

    // Esperar según oversampling
    uint16_t delay_ms;
    switch(oss) {
        case BMP180_ULTRALOWPOWER: delay_ms = 10; break;
        case BMP180_STANDARD:      delay_ms = 15; break;
        case BMP180_HIGHRES:       delay_ms = 25; break;
        case BMP180_ULTRAHIGHRES:  delay_ms = 45; break;
        default:                   delay_ms = 10;
    }
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);

    // Leer datos
    uint8_t data[3];
    if(i2c_register_read(BMP180_ADDR, BMP180_PRESSUREDATA, data, 3) != ESP_OK) {
        return -1;
    }

    int32_t raw = (((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2]) >> (8 - oss);
    return raw;
}

float bmp180_calculate_temp(int32_t ut) {
    int32_t x1, x2;
    
    x1 = ((ut - ac6) * ac5) >> 15;
    x2 = (mc << 11) / (x1 + md);
    
    int32_t b5 = x1 + x2;
    float temp = ((b5 + 8) >> 4) / 10.0f;
    
    return temp;
}

int32_t bmp180_calculate_pressure(int32_t up, uint8_t oss) {
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;
    
    // Calcular b5 (necesario para presión)
    int32_t ut = bmp180_read_raw_temp();
    x1 = ((ut - ac6) * ac5) >> 15;
    x2 = (mc << 11) / (x1 + md);
    b5 = x1 + x2;
    
    // Cálculos de presión
    b6 = b5 - 4000;
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
    
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
    if (b4 == 0) {  // Evitar división por cero
	    return 0;    // o manejar el error adecuadamente
	}
    b7 = ((uint32_t)(up - b3) * (50000 >> oss));
    
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    
    return p;
}

void mpu6050_read_accel_gyro(int16_t *accel, int16_t *gyro, float *accel_scale) {
    uint8_t raw_data[14];
    if (i2c_register_read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, raw_data, 14) == ESP_OK) {
        accel[0] = (raw_data[0] << 8) | raw_data[1];
        accel[1] = (raw_data[2] << 8) | raw_data[3];
        accel[2] = (raw_data[4] << 8) | raw_data[5];
        gyro[0] = (raw_data[8] << 8) | raw_data[9];
        gyro[1] = (raw_data[10] << 8) | raw_data[11];
        gyro[2] = (raw_data[12] << 8) | raw_data[13];
        
        // Leer la configuración actual del acelerómetro para determinar el rango
        uint8_t accel_config;
        if (i2c_register_read(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, &accel_config, 1) == ESP_OK) {
            *accel_scale = ACCEL_SCALE_2G;
        }
    }
}

bool hmc5883l_read_mag(int16_t *mag) {
    uint8_t raw_data[6];
    uint8_t status;
    
    // Leer registro de estado
    if (i2c_register_read(HMC5883L_ADDR, HMC5883L_STATUS, &status, 1) != ESP_OK) {
        return false;
    }
    
    // Verificar si hay datos nuevos (bit 0 del registro de estado)
    if (!(status & 0x01)) {
        return false;
    }
    
    // Leer datos del magnetómetro
    if (i2c_register_read(HMC5883L_ADDR, HMC5883L_DATA_X_H, raw_data, 6) != ESP_OK) {
        return false;
    }
    
    // El HMC5883L devuelve los datos en orden X, Z, Y
    mag[0] = (raw_data[0] << 8) | raw_data[1]; // X
    mag[2] = (raw_data[2] << 8) | raw_data[3]; // Z
    mag[1] = (raw_data[4] << 8) | raw_data[5]; // Y
    
    return true;
}

void calibrate_magnetometer() {
    int16_t mag[3];
    int16_t mag_min[3] = {2047, 2047, 2047};
    int16_t mag_max[3] = {-2048, -2048, -2048};
    
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    for (int i = 0; i < 300; i++) {
        if (hmc5883l_read_mag(mag)) {
            for (int j = 0; j < 3; j++) {
                if (mag[j] < mag_min[j]) mag_min[j] = mag[j];
                if (mag[j] > mag_max[j]) mag_max[j] = mag[j];
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    // Calcular bias y escala
    for (int j = 0; j < 3; j++) {
        mag_bias[j] = (mag_max[j] + mag_min[j]) / 2.0;
        mag_scale[j] = (mag_max[j] - mag_min[j]) / 2.0;
    }
    
    // Normalizar escalas
    float avg_scale = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0;
    for (int j = 0; j < 3; j++) {
        mag_scale[j] = avg_scale / mag_scale[j];
    }
    
    //ESP_LOGI(GY, "Calibración completada:");
    //ESP_LOGI(GY, "Bias: X=%.2f, Y=%.2f, Z=%.2f", mag_bias[0], mag_bias[1], mag_bias[2]);
    //ESP_LOGI(GY, "Scale: X=%.2f, Y=%.2f, Z=%.2f", mag_scale[0], mag_scale[1], mag_scale[2]);
}

float filtered_pressure(int32_t raw_pressure) {
    pressure_history[pressure_index] = raw_pressure / 100.0f;
    pressure_index = (pressure_index + 1) % PRESSURE_READINGS;
    
    float sum = 0;
    for(int i = 0; i < PRESSURE_READINGS; i++) {
        sum += pressure_history[i];
    }
    return sum / PRESSURE_READINGS;
}

void read_GY87_data(){
	// Leer sensores
    mpu6050_read_accel_gyro(accel, gyro, &accel_scale);
    hmc5883l_read_mag(mag);
    
    accelx = (float)accel[0] / accel_scale;
    accely = (float)accel[1] / accel_scale;
    accelz = (float)accel[2] / accel_scale;
    
    gyrox = (float)gyro[0] / gyro_scale; 
    gyroy = (float)gyro[1] / gyro_scale; 
    gyroz = (float)gyro[2] / gyro_scale;
    
    // Aplicar calibración y convertir a microteslas
    for (int i = 0; i < 3; i++) {
            mag_cal[i] = (float)(mag[i] - mag_bias[i]) * mag_scale[i] / HMC5883L_GAIN_1_3GA * GAUSS_TO_MICROTESLA;
    }
    
    magx = mag_cal[0];
    magy = mag_cal[1];
    magz = mag_cal[2];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                         GT-U7                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init_UART_GTU7(){
	// Configuración del UART
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //ESP_LOGI(GT, "UART2 configurado. Esperando datos del GPS...");
}

// Función para aplicar el filtro de media móvil
float apply_moving_average(float new_value) {
    // Restar el valor más antiguo de la suma
    altitude_sum -= altitude_buffer[buffer_index];
    // Añadir el nuevo valor a la suma
    altitude_sum += new_value;
    // Almacenar el nuevo valor en el buffer
    altitude_buffer[buffer_index] = new_value;
    // Actualizar el índice del buffer
    buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;
    // Calcular la media
    return altitude_sum / FILTER_WINDOW_SIZE;
}

float convert_to_decimal_degrees(float raw_degrees, char direction) {
    // Extraer grados y minutos (formato GGMM.MMMMM)
    int degrees = (int)(raw_degrees / 100);
    float minutes = raw_degrees - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0f);
    
    // Ajustar según la dirección (N/S/E/W)
    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    
    return decimal_degrees;
}

// Función para procesar el mensaje GPGGA y extraer la altitud
void process_gpgga(const char *message) {
    char time[10], lat[12], lat_dir, lon[12], lon_dir, fix_quality, num_satellites[3], hdop[6];
    char altitude[10], altitude_units, geoid_height[10], geoid_units;

    // Extraer datos del mensaje GPGGA
    int parsed = sscanf(message, "$GPGGA,%[^,],%[^,],%c,%[^,],%c,%c,%[^,],%[^,],%[^,],%c,%[^,],%c",
                        time, lat, &lat_dir, lon, &lon_dir, &fix_quality, num_satellites, hdop, altitude, &altitude_units, geoid_height, &geoid_units);

    // Verificar si se extrajeron correctamente los datos
    if (parsed == 12 && fix_quality != '0') { // '0' significa sin fix
    	latitude = convert_to_decimal_degrees(atof(lat), lat_dir); // Convertir las coordenadas a cartesianas
        longitude = convert_to_decimal_degrees(atof(lon), lon_dir); // Convertir las coordenadas a cartesianas
        altitudee = atof(altitude); // Convertir la altitud a float
        filtered_altitudee = apply_moving_average(altitudee); // Aplicar el filtro
                
        // Imprimir los valores
        //ESP_LOGI(GT, "Latitud: %.6f, Longitud: %.6f", latitude, longitude);
        //ESP_LOGI(GT, "Altitud: %.2f %c (Filtrada: %.2f %c)", altitud, altitude_units, filtered_altitude, altitude_units);
    } else {
        //ESP_LOGW(GT, "Datos no validos o sin fix GPS");
    }
}

void read_GTU7_data(){
		uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
	    char buffer[512] = {0}; // Buffer para acumular datos
	    size_t buffer_index = 0;
    
	    int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Añadir los datos recibidos al buffer
            if (buffer_index + len < sizeof(buffer)) {
                memcpy(buffer + buffer_index, data, len);
                buffer_index += len;
            } else {
                //ESP_LOGE(GT, "Buffer lleno. Reiniciando...");
                buffer_index = 0; // Reiniciar el buffer si está lleno
            }

            // Buscar mensajes completos en el buffer
            char *start = buffer;
            while (1) {
                char *end = strstr(start, "\r\n"); // Buscar el final del mensaje
                if (!end) break; // No hay mensajes completos

                *end = '\0'; // Terminar el mensaje actual
                //ESP_LOGI(TAG, "Mensaje recibido: %s", start);

                // Procesar el mensaje si es GPGGA
                if (strstr(start, "$GPGGA")) {
                    process_gpgga(start);
                }

                // Mover el inicio del buffer al siguiente mensaje
                start = end + 2; // Saltar "\r\n"
            }

            // Mover los datos restantes al inicio del buffer
            if (start != buffer) {
                size_t remaining = buffer + buffer_index - start;
                memmove(buffer, start, remaining);
                buffer_index = remaining;
            }
        }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                        ESP-NOW                                                              //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Callback al enviar
void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(ESPNOW, "Enviado a %02X:%02X:%02X:%02X:%02X:%02X | Estado: %s",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5],
             status == ESP_NOW_SEND_SUCCESS ? "Exito" : "Fallo");
}

void wifi_init() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Modo largo alcance
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
}

void espnow_init() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    esp_now_peer_info_t peer = {
        .channel = 1,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, peer_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void send_ESPNOW_data(){
	primary_load_data_t data = {
		.load_type = PRIMARY_LOAD,
        .speed = speed,
        .accelx = accelx, .accely = accely, .accelz = accelz,
        .gyrox = gyrox, .gyroy = gyroy, .gyroz = gyroz,
        .magx = magx, .magy = magy, .magz = magz,
        .altitud = altitud,
        .latitude = latitude,
        .longitude = longitude,
        .decoupling_status = decoupling_status
    };
    
    ESP_ERROR_CHECK(esp_now_send(peer_mac, (uint8_t *)&data, sizeof(data))); //Enviar datos
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                      DECOUPLING                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Altitud por BMP180 del GY-87
float calculate_altitude_bmp(float pressure, float reference_pressure) {
    if (isnan(pressure)) return 0.0f;
    if (reference_pressure <= 0) return 0.0f;
    
    float altitude = 44330.0f * (1.0f - powf(pressure / reference_pressure, 0.1903f));
    return (altitude > 0) ? altitude : 0.0f;
}

void update_altitude_bmp() {
    int32_t raw_pressure = bmp180_read_raw_pressure(BMP180_STANDARD);
    if (raw_pressure < 0) return; // Error en lectura
    
    float pressure = bmp180_calculate_pressure(raw_pressure, BMP180_STANDARD) / 100.0f; // Convertir a hPa
    
    // Sistema de calibración inicial
    if (pressure_readings_count_bmp < INITIAL_READINGS_BMP) {
        initial_pressures_bmp[pressure_readings_count_bmp] = pressure;
        pressure_readings_count_bmp++;
        
        if (pressure_readings_count_bmp == INITIAL_READINGS_BMP) {
            float sum = 0;
            for (int i = 0; i < INITIAL_READINGS_BMP; i++) {
                sum += initial_pressures_bmp[i];
            }
            prom_press_bmp = sum / INITIAL_READINGS_BMP;
        }
        altitud = 0.0f; // Altitud cero durante calibración
    } else {
		// Filtro de media móvil en presión
		pressure_sum -= pressure_buffer[pressure_index];
		pressure_buffer[pressure_index] = pressure;
		pressure_sum += pressure;
		pressure_index = (pressure_index + 1) % PRESSURE_WINDOW_SIZE;
        
        float filtered_pressure = pressure_sum / PRESSURE_WINDOW_SIZE;
		float new_altitude = calculate_altitude_bmp(filtered_pressure, prom_press_bmp);
		
		// Evitando salto por mas de 20 metros de diferencia
		if (fabsf(new_altitude - filtered_altitude_bmp) > MAX_ALTITUDE_JUMP) {
	        return; // no se actualiza la altitud
	    }
	    
	    // De ser valida, se guarda la nueva altura
	    altitud = new_altitude;
	    
		// Aplicar filtro de media móvil
        altitude_sum -= altitude_buffer[buffer_index];
        altitude_buffer[buffer_index] = altitud;
        altitude_sum += altitud;
        buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;
        filtered_altitude_bmp = altitude_sum / FILTER_WINDOW_SIZE;
    }
}

void activatingRele() {
	for (int i = 0; i < 2; i++) {
		gpio_set_level(LED_PIN, 0);      // Rele encendido
		vTaskDelay(pdMS_TO_TICKS(68));   // Espera 68 ms
		gpio_set_level(LED_PIN, 1);      // Rele apagado
		vTaskDelay(pdMS_TO_TICKS(700));  // Espera 700 ms
	}
}

void decoupling() {	
	update_altitude_bmp();
	
	if (altitud >= 100) {
        decoupling_bmp = true;
    }
	
	if((decoupling_bmp && altitud <= 60 && altitud >= 40 && !decoupling_status)) {
        activatingRele();
        decoupling_status = true;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                         MAIN                                                                //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main() {
	init_led_gpio();
	gpio_set_level(LED_PIN, 1);  // Rele apagado
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay de 5 segundos
    adc_init(); // Inicializar lectura en pines ADC
    calibrate_zero_wind(); //Calibrar el sensor Rev. C
    
    i2c_master_init(); // Iniciar el I2C del GY-87
    enable_bypass_mode(); // Habilitar modo bypass para acceder directamente al HMC5883L
    mpu6050_init(); // Inicializar MPU6050
    hmc5883l_init(); // Inicializar HMC5883L
    bmp180_begin(); // Iniciar BMP180
    //calibrate_magnetometer(); // Calibrar magnetómetro (opcional)

    init_UART_GTU7();   // Inicializar el sensor GT-U7
	        
    esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	    ESP_ERROR_CHECK(nvs_flash_erase());
	    ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	wifi_init();
	espnow_init();

    while(1) {
		read_WindSensorRevC_data();
		read_GY87_data();
		read_GTU7_data();
		printf("%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%s\n",
            speed, accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz, altitud, latitude, longitude, decoupling_status ? "true" : "false"
        );
        send_ESPNOW_data(); // Enviar datos
        decoupling(); // Desacoplaje
        vTaskDelay(500 / portTICK_PERIOD_MS); // Cada 0.5 segundos
    }
}