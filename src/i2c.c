#include "i2c.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

// Static device handles preserved across the life cycle of the application context
static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t imu_handle = NULL;
static i2c_master_dev_handle_t light_handle = NULL;

esp_err_t i2c_bus_master_init(void) {
    // Configure the master bus properties
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, // Hardware signal noise filtering threshold
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Register the individual IMU device onto the active bus instance
    i2c_device_config_t imu_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMI8658_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(bus_handle, &imu_dev_config, &imu_handle);
    if (err != ESP_OK) {
        return err;
    }

    // Register the external light sensor device object on the same line
    i2c_device_config_t light_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TSL2561_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    return i2c_master_bus_add_device(bus_handle, &light_dev_config, &light_handle);
}

esp_err_t i2c_bus_probe_peripherals(void) {
    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Native diagnostic ping check
    esp_err_t err = i2c_master_probe(bus_handle, QMI8658_I2C_ADDRESS, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }

    return i2c_master_probe(bus_handle, TSL2561_I2C_ADDRESS, pdMS_TO_TICKS(50));
}

esp_err_t i2c_sensor_read_imu(i2c_imu_data_t *imu_data) {
    if (imu_data == NULL || imu_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = QMI8658_REG_DATA_OUT_BASE;
    uint8_t raw_data[12];

    esp_err_t err = i2c_master_transmit_receive(imu_handle, &reg_addr, 1, raw_data, 12, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t ay = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t az = (int16_t)((raw_data[5] << 8) | raw_data[4]);
    
    int16_t gx = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    int16_t gy = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    int16_t gz = (int16_t)((raw_data[11] << 8) | raw_data[10]);

    imu_data->acc_x = (float)ax / QMI8658_ACCEL_SCALE_4096G;
    imu_data->acc_y = (float)ay / QMI8658_ACCEL_SCALE_4096G;
    imu_data->acc_z = (float)az / QMI8658_ACCEL_SCALE_4096G;
    
    imu_data->gyro_x = (float)gx / QMI8658_GYRO_SCALE_64DPS;
    imu_data->gyro_y = (float)gy / QMI8658_GYRO_SCALE_64DPS;
    imu_data->gyro_z = (float)gz / QMI8658_GYRO_SCALE_64DPS;

    return ESP_OK;
}

esp_err_t i2c_sensor_read_light(float *lux_value) {
    if (lux_value == NULL || light_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ch0_cmd = TSL2561_CMD_SELECT_BIT | TSL2561_CMD_WORD_PROTOCOL | TSL2561_REG_CHAN0_BASE;
    uint8_t ch1_cmd = TSL2561_CMD_SELECT_BIT | TSL2561_CMD_WORD_PROTOCOL | TSL2561_REG_CHAN1_BASE;
    
    uint8_t ch0_data[2];
    uint8_t ch1_data[2];

    esp_err_t err = i2c_master_transmit_receive(light_handle, &ch0_cmd, 1, ch0_data, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }
    
    err = i2c_master_transmit_receive(light_handle, &ch1_cmd, 1, ch1_data, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        return err;
    }

    uint16_t ch0 = (ch0_data[1] << 8) | ch0_data[0];
    uint16_t ch1 = (ch1_data[1] << 8) | ch1_data[0];

    if (ch0 == 0) {
        *lux_value = 0.0f;
        return ESP_OK;
    }

    float ratio = (float)ch1 / (float)ch0;
    
    if (ratio > 0.0f && ratio <= TSL2561_RATIO_K1) {
        *lux_value = (0.0304f * ch0) - (0.062f * ch0 * powf(ratio, 1.4f));
    } else if (ratio > TSL2561_RATIO_K1 && ratio <= TSL2561_RATIO_K2) {
        *lux_value = (0.0224f * ch0) - (0.031f * ch1);
    } else if (ratio > TSL2561_RATIO_K2 && ratio <= TSL2561_RATIO_K3) {
        *lux_value = (0.0128f * ch0) - (0.0153f * ch1);
    } else if (ratio > TSL2561_RATIO_K3 && ratio <= TSL2561_RATIO_K4) {
        *lux_value = (0.00146f * ch0) - (0.00112f * ch1);
    } else {
        *lux_value = 0.0f; 
    }

    return ESP_OK;
}
