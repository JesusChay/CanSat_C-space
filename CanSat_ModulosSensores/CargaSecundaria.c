#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <math.h>

// Configuracion de I2C para el BME280
#define SCL_BME280 26                  // GPIO 26 para SCL
#define SDA_BME280 27                  // GPIO 27 para SDA
#define FREQ_HZ 100000                 // Frecuencia de I2C (100 kHz)
#define MASTER_NUM_BME280 I2C_NUM_0    // Puerto I2C, por default 0x76
#define BME280_ADDR 0x76

// Configuracion de I2C para el MPU6050
#define SCL_MPU6050 22                 // GPIO para SCL
#define SDA_MPU6050 21                 // GPIO para SDA
#define FREQ_HZ 100000                 // Frecuencia de I2C
#define MASTER_NUM_MPU6050 I2C_NUM_1
#define MPU6050_ADDR 0x68              // Dirección del MPU6050

// Registros MPU6050
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_ACCEL_CONFIG 0x1C     // Registro de configuración del acelerómetro
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// Registros del BME280
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_PRESS_MSB 0xF7
#define BME280_REG_TEMP_MSB 0xFA
#define BME280_REG_HUM_MSB 0xFD

// Coeficientes de calibracion del BME280
#define BME280_REG_CALIB00 0x88
#define BME280_REG_CALIB26 0xE1

// Configuración de UART para el GT-U7
#define UART_PORT UART_NUM_2
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16
#define BUF_SIZE 256
#define FILTER_WINDOW_SIZE 10 // Tamaño de la ventana del filtro

//static const char *BME = "BME280";
//static const char *ICM = "MPU6050";
//static const char *GT = "GPS";
static const char *ESPNOW = "ESPNOW_SEND_SECUNDARY";

// Variables globales
float temperature = 0.0, humidity = 0.0, pressure = 0.0;  // BME280
int16_t accel[3], gyro[3];
float latitude, longitude, altitud, altitudee, filtered_altitudee;
float accelx = 0, accely = 0, accelz = 0;
float gyrox = 0, gyroy = 0, gyroz = 0;
const float ACCEL_SCALE_2G = 16384.0f;    // LSB/g para ±2g
const float GYRO_SCALE_250DPS = 131.0f;   // LSB/°/s para ±250°/s
float accel_scale = ACCEL_SCALE_2G; // Valor por defecto (±2g)
float gyro_scale = GYRO_SCALE_250DPS;

// Variables para el filtro de media móvil
float altitude_buffer[FILTER_WINDOW_SIZE] = {0};
int buffer_index = 0;
float altitude_sum = 0;

//Altura por presion
#define INITIAL_READINGS 5
float initial_pressures[INITIAL_READINGS];
int pressure_readings_count = 0;
float prom_press = 98250.00f;
float filtered_altitude = 0.0f;

// Identificador para el receptor
typedef enum {
    PRIMARY_LOAD = 1,
    SECONDARY_LOAD = 2
} load_type_t;

// Estructura de datos a enviar
typedef struct {
	load_type_t load_type;
    float temperature;
    float humidity;
    float pressure;
    float accelx, accely, accelz;
    float gyrox, gyroy, gyroz;
    float altitud;
    float latitude;
    float longitude;
} secondary_load_data_t;

// MAC del receptor
static uint8_t peer_mac[] = {0x68, 0x25, 0xDD, 0xF0, 0x25, 0xD0}; 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                        BME280                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Estructura para almacenar los coeficientes de calibración
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t dig_H1, dig_H3;
    int16_t dig_H2, dig_H4, dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
} bme280_calib_data;

static bme280_calib_data calib_data;

// Función para escribir un registro del BME280
esp_err_t bme280_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MASTER_NUM_BME280, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Función para leer registros del BME280
esp_err_t bme280_read_registers(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MASTER_NUM_BME280, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Función para leer los coeficientes de calibración
void bme280_read_calibration_data() {
    uint8_t calib_data1[25];
    uint8_t calib_data2[7];
    
    // Leer primera parte de los coeficientes (0x88-0xA1)
    bme280_read_registers(BME280_REG_CALIB00, calib_data1, 24);
    
    // Leer segunda parte de los coeficientes (0xE1-0xE7)
    bme280_read_registers(BME280_REG_CALIB26, calib_data2, 7);
    
    // Parsear los coeficientes de temperatura
    calib_data.dig_T1 = (calib_data1[1] << 8) | calib_data1[0];
    calib_data.dig_T2 = (calib_data1[3] << 8) | calib_data1[2];
    calib_data.dig_T3 = (calib_data1[5] << 8) | calib_data1[4];
    
    // Parsear los coeficientes de presión
    calib_data.dig_P1 = (calib_data1[7] << 8) | calib_data1[6];
    calib_data.dig_P2 = (calib_data1[9] << 8) | calib_data1[8];
    calib_data.dig_P3 = (calib_data1[11] << 8) | calib_data1[10];
    calib_data.dig_P4 = (calib_data1[13] << 8) | calib_data1[12];
    calib_data.dig_P5 = (calib_data1[15] << 8) | calib_data1[14];
    calib_data.dig_P6 = (calib_data1[17] << 8) | calib_data1[16];
    calib_data.dig_P7 = (calib_data1[19] << 8) | calib_data1[18];
    calib_data.dig_P8 = (calib_data1[21] << 8) | calib_data1[20];
    calib_data.dig_P9 = (calib_data1[23] << 8) | calib_data1[22];
    
    // Parsear los coeficientes de humedad
    calib_data.dig_H1 = calib_data1[24];
    calib_data.dig_H2 = (calib_data2[1] << 8) | calib_data2[0];
    calib_data.dig_H3 = calib_data2[2];
    calib_data.dig_H4 = (calib_data2[3] << 4) | (calib_data2[4] & 0x0F);
    calib_data.dig_H5 = (calib_data2[5] << 4) | (calib_data2[4] >> 4);
    calib_data.dig_H6 = calib_data2[6];
}

// Función para inicializar el BME280
esp_err_t init_BME280() {
    // Configurar I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_BME280,
        .scl_io_num = SCL_BME280,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(MASTER_NUM_BME280, &conf);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al configurar I2C");
        return ret;
    }
    
    ret = i2c_driver_install(MASTER_NUM_BME280, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al instalar el driver I2C");
        return ret;
    }
    
    // Verificar ID del dispositivo
    uint8_t id;
    ret = bme280_read_registers(BME280_REG_ID, &id, 1);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al leer el ID del BME280");
        return ret;
    }
    
    if (id != 0x60) {  // El ID del BME280 es 0x60
        //ESP_LOGE(BME, "Dispositivo no reconocido (ID: 0x%02X)", id);
        return ESP_FAIL;
    }
    
    // Resetear el dispositivo
    ret = bme280_write_register(BME280_REG_RESET, 0xB6);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al resetear el BME280");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Leer datos de calibración
    bme280_read_calibration_data();
    
    // Configurar el sensor
    // Configurar humedad (oversampling x1)
    ret = bme280_write_register(BME280_REG_CTRL_HUM, 0x01);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al configurar humedad");
        return ret;
    }
    
    // Configurar modo normal, oversampling temperatura y presión
    ret = bme280_write_register(BME280_REG_CTRL_MEAS, 0x27);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al configurar medicion");
        return ret;
    }
    
    // Configurar standby time y filtro
    ret = bme280_write_register(BME280_REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        //ESP_LOGE(BME, "Error al configurar standby y filtro");
        return ret;
    }
    
    // Inicializar el buffer de altitud
	for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
	    altitude_buffer[i] = 0.0f;
	}
    
    //ESP_LOGI(BME, "BME280 inicializado correctamente");
    return ESP_OK;
}

// Compensación de temperatura (devuelve temperatura en °C)
float bme280_compensate_temperature(int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    
    calib_data.t_fine = var1 + var2;
    T = (calib_data.t_fine * 5 + 128) >> 8;
    
    // Compensacion adicional basada en presion si es necesario
    float temp_compensated = (float)T / 100.0;
    
    // Compensacion adicional por desviaciones, hay que declarar el valor
    // temp_compensated += your_calibration_offset;
    
    return temp_compensated;
}

// Compensación de presión (devuelve presión en Pa)
float bme280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)calib_data.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    
    if (var1 == 0) {
        return 0; // Evitar división por cero
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (float)p / 256.0;
}

// Compensación de humedad (devuelve humedad en %RH)
float bme280_compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    
    v_x1_u32r = (calib_data.t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + 
                  ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) * 
                  (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * 
                  ((int32_t)calib_data.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    
    return (float)(v_x1_u32r >> 12) / 1024.0;
}

// Leer temperatura
esp_err_t bme280_read_temperature(float *temperature) {
    uint8_t data[3];
    esp_err_t ret = bme280_read_registers(BME280_REG_TEMP_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int32_t adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    *temperature = bme280_compensate_temperature(adc_T);
    return ESP_OK;
}

// Leer presión
esp_err_t bme280_read_pressure(float *pressure) {
    uint8_t data[3];
    esp_err_t ret = bme280_read_registers(BME280_REG_PRESS_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    *pressure = bme280_compensate_pressure(adc_P);
    return ESP_OK;
}

// Leer humedad
esp_err_t bme280_read_humidity(float *humidity) {
    uint8_t data[2];
    esp_err_t ret = bme280_read_registers(BME280_REG_HUM_MSB, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int32_t adc_H = (data[0] << 8) | data[1];
    *humidity = bme280_compensate_humidity(adc_H);
    return ESP_OK;
}

// Calculo de altitud
float calculate_altitude(float pressure, float reference_pressure) {
	if (isnan(pressure) || isnan(reference_pressure)) return 0.0f;
    if (reference_pressure <= 0) return 0.0f; // Evitar división por cero
    
    // Fórmula barométrica
    float altitude = 44330.0f * (1.0f - powf(pressure / reference_pressure, 0.1903f));
    
    // Solo valores positivos
    return (altitude > 0) ? altitude : 0.0f;
}

void read_BME280_data() {
	// Leer temperatura
        if (bme280_read_temperature(&temperature) == ESP_OK) {
            //ESP_LOGI(BME, "Temperatura: %.2f °C", temperature);
        } else {
            //ESP_LOGE(BME, "Error al leer la temperatura");
        }

        // Leer humedad
        if (bme280_read_humidity(&humidity) == ESP_OK) {
            //ESP_LOGI(BME, "Humedad: %.2f %%", humidity);
        } else {
            //ESP_LOGE(BME, "Error al leer la humedad");
        }

        // Leer presión
	    if (bme280_read_pressure(&pressure) == ESP_OK) {
	        // Calcular altitud
	        if (pressure_readings_count < INITIAL_READINGS) {
	            // Guardar las primeras lecturas
	            initial_pressures[pressure_readings_count] = pressure;
	            pressure_readings_count++;
	            
	            if (pressure_readings_count == INITIAL_READINGS) {
	                // Calcular el promedio de las primeras 5 lecturas
	                float sum = 0;
	                for (int i = 0; i < INITIAL_READINGS; i++) {
	                    sum += initial_pressures[i];
	                }
	                prom_press = sum / (float)INITIAL_READINGS;
	                //ESP_LOGI(BME, "Presión de referencia calculada: %.2f Pa", prom_press);
	            }
	            // Durante las primeras lecturas, altitud = 0
	            altitud = 0.0f;
	        } else {
	            // Calcular altitud usando la presión de referencia
	            altitud = calculate_altitude(pressure, prom_press);
	            
	            // Aplicar filtro de media móvil si es necesario
	            altitude_sum -= altitude_buffer[buffer_index];
	            altitude_buffer[buffer_index] = altitud;
	            altitude_sum += altitud;
	            buffer_index = (buffer_index + 1) % FILTER_WINDOW_SIZE;
	            filtered_altitude = altitude_sum / FILTER_WINDOW_SIZE;
	        }
	    } else {
	        //ESP_LOGE(BME, "Error al leer la presión");
	        // En caso de error, usar presión estándar
	        altitud = calculate_altitude(prom_press, prom_press); // Esto dará 0
	    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                       MPU6050                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Función para inicializar el sensor MPU6050
void init_MPU6050() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_MPU6050,
        .scl_io_num = SCL_MPU6050,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(MASTER_NUM_MPU6050, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(MASTER_NUM_MPU6050, conf.mode, 0, 0, 0));
}

esp_err_t i2c_register_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(MASTER_NUM_MPU6050, cmd, 1000 / portTICK_PERIOD_MS);
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
    esp_err_t ret = i2c_master_cmd_begin(MASTER_NUM_MPU6050, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void mpu6050_init() {
    // Despertar el MPU6050
    i2c_register_write(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
    
    // Configurar el rango del acelerómetro (±2g, ±4g, ±8g o ±16g)
    // 0x00 = ±2g (por defecto)
    // 0x08 = ±4g
    // 0x10 = ±8g
    // 0x18 = ±16g
    i2c_register_write(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00); // ±2g
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
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

void read_MPU6050_data(){
	mpu6050_read_accel_gyro(accel, gyro, &accel_scale);
	
	// Variables finales compensadas
	accelx = (float)accel[0] / accel_scale, accely = (float)accel[1] / accel_scale, accelz = (float)accel[2] / accel_scale;
	gyrox = (float)gyro[0] / gyro_scale, gyroy = (float)gyro[1] / gyro_scale, gyroz = (float)gyro[2] / gyro_scale;
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
	secondary_load_data_t data = {
		.load_type = SECONDARY_LOAD,
        .temperature = temperature,
        .humidity = humidity,
        .pressure = pressure,
        .accelx = accelx, .accely = accely, .accelz = accelz,
        .gyrox = gyrox, .gyroy = gyroy, .gyroz = gyroz,
        .altitud = altitud,
        .latitude = latitude,
        .longitude = longitude
    };
    
    ESP_ERROR_CHECK(esp_now_send(peer_mac, (uint8_t *)&data, sizeof(data))); //Enviar datos
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                         MAIN                                                                //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Función principal
void app_main(void) {
    init_BME280();      // Inicializar el sensor BME280
    init_MPU6050();     // Inicializar el sensor MPU6050
    init_UART_GTU7();   // Inicializar el sensor GT-U7
    mpu6050_init();
    
    esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
	    ESP_ERROR_CHECK(nvs_flash_erase());
	    ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	wifi_init();
	espnow_init();
    
	while (1) {  
        read_BME280_data();
        read_MPU6050_data();
        read_GTU7_data();

        printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f\n",
            temperature, accelx, accely, accelz, gyrox, gyroy, gyroz, humidity, pressure, altitud, latitude, longitude
        );
        send_ESPNOW_data(); // Enviar datos
        vTaskDelay(500 / portTICK_PERIOD_MS); // Cada 0.5 segundos
	}
}
