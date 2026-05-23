#ifndef I2C_H
#define I2C_H

#include "esp_err.h"

// I2C Bus Configuration
#define I2C_MASTER_NUM              0
#define I2C_MASTER_SDA_IO           15
#define I2C_MASTER_SCL_IO           7
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// I2C Devices Addresses
#define QMI8658_I2C_ADDRESS         0x6B
#define TSL2561_I2C_ADDRESS         0x39

// QMI8658 REGISTERS & COEFFICIENTS
#define QMI8658_REG_STATUS0         0x2C       // Output Data Status Register
#define QMI8658_REG_CTRL1           0x02       
#define QMI8658_REG_CTRL2           0x03       
#define QMI8658_REG_CTRL7           0x08       
#define QMI8658_REG_CTRL8           0x09       // DSP Command Register
#define QMI8658_REG_DATA_OUT_BASE   0x35       
#define QMI8658_ACCEL_SCALE_4096G   4096.0f    
#define QMI8658_GYRO_SCALE_64DPS    64.0f      

// TSL2561 REGISTERS, COMMANDS & COEFFICIENTS
#define TSL2561_CMD_SELECT_BIT      0x80       
#define TSL2561_CMD_WORD_PROTOCOL   0x20       
#define TSL2561_REG_CHAN0_BASE      0x0C       
#define TSL2561_REG_CHAN1_BASE      0x0E       

#define TSL2561_RATIO_K1            0.50f
#define TSL2561_RATIO_K2            0.61f
#define TSL2561_RATIO_K3            0.80f
#define TSL2561_RATIO_K4            1.30f

typedef struct {
    float acc_x;  
    float acc_y;  
    float acc_z;  
    float gyro_x; 
    float gyro_y; 
    float gyro_z; 
} i2c_imu_data_t;

#define ACC_X_OFFSET -0.03f
#define ACC_Y_OFFSET -0.08f
#define ACC_Z_OFFSET -0.02f

// Function Prototypes
esp_err_t i2c_bus_master_init(void);
esp_err_t i2c_bus_probe_peripherals(void);

esp_err_t i2c_sensor_init_imu(void);
esp_err_t i2c_sensor_read_imu(i2c_imu_data_t *imu_data);

esp_err_t i2c_sensor_wakeup_light(void);
esp_err_t i2c_sensor_read_light(float *lux_value, uint16_t *raw_ch0, uint16_t *raw_ch1);

#endif // I2C_H
