#include "i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "I2C_DRIVER";

esp_err_t i2c_bus_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_bus_probe_peripherals(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Critical IMU sensor (QMI8658) not found on I2C bus!");
        return err;
    }
    ESP_LOGI(TAG, "QMI8658 IMU successfully detected.");

    // Probe light sensor
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Optional Light Sensor (TSL2561) not found. Proceeding with fallback theme.");
    } else {
        ESP_LOGI(TAG, "TSL2561 Light Sensor successfully detected.");
    }

    return ESP_OK;
}

esp_err_t i2c_sensor_init_imu(void)
{
    // 1. Unlocks basic continuous configuration access
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMI8658_REG_CTRL1, true);
    i2c_master_write_byte(cmd, 0x60, true); 
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10)); // Crucial hardware settling windows

    // 2. Configure CTRL2: Force standard +/- 8g operation window explicitly
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMI8658_REG_CTRL2, true);
    i2c_master_write_byte(cmd, 0x32, true); 
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Configure CTRL7: Power on Accelerometer and Gyroscope cores fully
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMI8658_REG_CTRL7, true);
    i2c_master_write_byte(cmd, 0x03, true); 
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(20)); // Allow internal data state lines to clear
    return err;
}

esp_err_t i2c_sensor_read_imu(i2c_imu_data_t *imu_data)
{
    if (imu_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw_data[12];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMI8658_REG_DATA_OUT_BASE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMI8658_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 11, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &raw_data[11], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    int16_t raw_acc_x  = (int16_t)((raw_data[1] << 8)  | raw_data[0]);
    int16_t raw_acc_y  = (int16_t)((raw_data[3] << 8)  | raw_data[2]);
    int16_t raw_acc_z  = (int16_t)((raw_data[5] << 8)  | raw_data[4]);
    int16_t raw_gyro_x = (int16_t)((raw_data[7] << 8)  | raw_data[6]);
    int16_t raw_gyro_y = (int16_t)((raw_data[9] << 8)  | raw_data[8]);
    int16_t raw_gyro_z = (int16_t)((raw_data[11] << 8) | raw_data[10]);

    imu_data->acc_x  = ((float)raw_acc_x  / QMI8658_ACCEL_SCALE_4096G) * 2.0f + ACC_X_OFFSET;
    imu_data->acc_y  = ((float)raw_acc_y  / QMI8658_ACCEL_SCALE_4096G) * 2.0f + ACC_Y_OFFSET;
    imu_data->acc_z  = ((float)raw_acc_z  / QMI8658_ACCEL_SCALE_4096G) * 2.0f + ACC_Z_OFFSET;

    imu_data->gyro_x = (float)raw_gyro_x / QMI8658_GYRO_SCALE_64DPS;
    imu_data->gyro_y = (float)raw_gyro_y / QMI8658_GYRO_SCALE_64DPS;
    imu_data->gyro_z = (float)raw_gyro_z / QMI8658_GYRO_SCALE_64DPS;

    return ESP_OK;
}

esp_err_t i2c_sensor_read_light(float *lux_value)
{
    if (lux_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, TSL2561_CMD_SELECT_BIT | 0x00, true); 
    i2c_master_write_byte(cmd, 0x03, true); 
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    uint8_t ch0_data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, TSL2561_CMD_SELECT_BIT | TSL2561_CMD_WORD_PROTOCOL | TSL2561_REG_CHAN0_BASE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, ch0_data, 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &ch0_data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    uint8_t ch1_data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, TSL2561_CMD_SELECT_BIT | TSL2561_CMD_WORD_PROTOCOL | TSL2561_REG_CHAN1_BASE, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, ch1_data, 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &ch1_data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) return err;

    uint16_t ch0 = (ch0_data[1] << 8) | ch0_data[0];
    uint16_t ch1 = (ch1_data[1] << 8) | ch1_data[0];

    if (ch0 == 0) {
        *lux_value = 0.0f;
        return ESP_OK;
    }

    float ratio = (float)ch1 / (float)ch0;
    float lux = 0.0f;

    if (ratio <= TSL2561_RATIO_K1) {
        lux = (0.0304f * ch0) - (0.062f * ch0 * powf(ratio, 1.4f));
    } else if (ratio <= TSL2561_RATIO_K2) {
        lux = (0.0224f * ch0) - (0.031f * ch1);
    } else if (ratio <= TSL2561_RATIO_K3) {
        lux = (0.0128f * ch0) - (0.0153f * ch1);
    } else if (ratio <= TSL2561_RATIO_K4) {
        lux = (0.00146f * ch0) - (0.00112f * ch1);
    } else {
        lux = 0.0f; 
    }

    *lux_value = (lux < 0.0f) ? 0.0f : lux;
    return ESP_OK;
}
