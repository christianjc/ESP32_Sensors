

#pragma once
#include <stdio.h>
#include "esp_types.h"
#include "esp_err.h"

/** BNO055 Address Alternative **/
#define BNO055_ADDRESS_A (0x28)                     // This requires the ADR pin to be low
/** BNO055 Address Default **/
#define BNO055_ADDRESS_DEFAULT (0x29)               // This requires the ADR pin to be high
/** BNO055 Adress being used **/
#define BNO055_ADDRESS BNO055_ADDRESS_DEFAULT
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)
#define NUM_BNO055_QUATERNION_REGISTERS (8)
#define NUM_BNO055_EULER_REGISTERS (6)

/** I2C configuration settings **/
#define I2C_MASTER_SCL_IO           (19)//CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           (18)//CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                        /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000                       /*!< I2C master timeout */
#define I2C_CONNECTION_TO_TRY       10                          /*!< I2C number of times to try to send a message */


/** BNO055 Registers Adress **/
typedef enum {
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    /* PAGE0 REGISTERS */

    /* Chip IDs */
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Acceleration data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

    /* Magnetometer data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

    /* Gyroscope data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

    /* Euler angels data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_EULER_R_LSB_ADDR = 0X1C,
    BNO055_EULER_R_MSB_ADDR = 0X1D,
    BNO055_EULER_P_LSB_ADDR = 0X1E,
    BNO055_EULER_P_MSB_ADDR = 0X1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    /* System Statues registers */
    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
} bno055_reg_t;


/** BNO055 power settings */
typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
} bno055_powermode_t;


/** Operation mode settings **/
typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} bno055_opmode_t;


/** Remap settings **/
typedef enum {
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

/** Remap Signs **/
typedef enum {
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;


/** Vector Mappings **/
typedef enum {
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
} bno055_vector_type_t;

typedef enum {
    UNITS_MS2 = 0x00,   /**< Meters per second squared */
    UNITS_MG = 0x01,    /**< Mass times gravity */
} bno055_accel_unit_t;

typedef enum {
    UNITS_DPS = 0x00,   /**< Degrees per second */
    UNITS_RPS = 0x02,   /**< Radians per second */
} bno055_angular_rate_unit_t;

typedef enum {
    UNITS_DEGREES = 0x00,   /**< Degrees */
    UNITS_RADIANS = 0x04,   /**< Rdians */
} bno055_euler_unit_t;

typedef enum {
    UNITS_CELSIUS = 0x00,       /**< Degrees Celsius */
    UNITS_FAHRENHEIT = 0x10,    /**< Degrees Fahrenheit */
} bno055_temp_unit_t;

/** A structure to configure the units **/
typedef struct {
    bno055_accel_unit_t accel;
    bno055_angular_rate_unit_t angular_rate;
    bno055_euler_unit_t euler_angel;
    bno055_temp_unit_t temperature;
} bno055_units_config_t;

/** A structure to represent offsets **/
typedef struct {
    int16_t accel_offset_x; /**< x acceleration offset */
    int16_t accel_offset_y; /**< y acceleration offset */
    int16_t accel_offset_z; /**< z acceleration offset */

    int16_t mag_offset_x; /**< x magnetometer offset */
    int16_t mag_offset_y; /**< y magnetometer offset */
    int16_t mag_offset_z; /**< z magnetometer offset */

    int16_t gyro_offset_x; /**< x gyroscrope offset */
    int16_t gyro_offset_y; /**< y gyroscrope offset */
    int16_t gyro_offset_z; /**< z gyroscrope offset */

    int16_t accel_radius; /**< acceleration radius */

    int16_t mag_radius; /**< magnetometer radius */
} bno055_offsets_t;


/** A structure to represent revisions **/
typedef struct {
    uint8_t accel_rev; /**< acceleration rev */
    uint8_t mag_rev;   /**< magnetometer rev */
    uint8_t gyro_rev;  /**< gyroscrope rev */
    uint16_t sw_rev;   /**< SW rev */
    uint8_t bl_rev;    /**< bootloader rev */
} bno055_rev_info_t;



/*!
 *  @brief  Initial set up
 *  @return ESP_OK: if operatin was succesful
 *          ESP_FAIL: if operation fail
 */
esp_err_t bno055_begin();


esp_err_t set_opmode(bno055_opmode_t op_mode);
bno055_opmode_t get_opmode(void);

esp_err_t set_powermode(bno055_powermode_t power_mode);
bno055_powermode_t get_powermode(void);

esp_err_t set_axis_remap(bno055_axis_remap_config_t config);
bno055_axis_remap_config_t get_axis_remap(void);

esp_err_t set_axis_sign(bno055_axis_remap_sign_t config);
bno055_axis_remap_sign_t get_axis_sign(void);

esp_err_t unit_config(bno055_units_config_t* config);

/**
 *  @brief  Gets current calibration state.  Each value should be a uint8_t
 *          pointer and it will be set to 0 if not calibrated and 3 if
 *          fully calibrated.
 *          See section 34.3.54
 *  @param  sys
 *          Current system calibration status, depends on status of all sensors,
 * read-only
 *  @param  gyro
 *          Current calibration status of Gyroscope, read-only
 *  @param  accel
 *          Current calibration status of Accelerometer, read-only
 *  @param  mag
 *          Current calibration status of Magnetometer, read-only
 */
void get_calibration_state(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

/**
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
bool isFullyCalibrated(void);

/**
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
esp_err_t get_sensor_offsets(uint8_t* calibData);

/**
 *  @brief  Reads the sensor's offset registers into an offset struct to work with raw data
 *  @param  offsets type of offsets
 *  @return true if read is successful
 */
esp_err_t get_sensor_offsets_struct(bno055_offsets_t* offsets);

/**
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t get_temp(void);

/**
 *  @brief   Gets a vector reading from the specified source
 *  @param   vector_type
 *           possible vector type values
 *           [VECTOR_ACCELEROMETER
 *            VECTOR_MAGNETOMETER
 *            VECTOR_GYROSCOPE
 *            VECTOR_EULER
 *            VECTOR_LINEARACCEL
 *            VECTOR_GRAVITY]
 *  @param xyz pointer to a array of size 3
 *  @return  vector from specified source
 */
esp_err_t get_vector(bno055_vector_type_t vector_type, int16_t* xyz);

/**
 *  @brief  Gets a quaternion reading from the specified source
 *  @param quat pointer to an array of int16 of size 8
 *  @return quaternion reading
 */
esp_err_t get_quat(int16_t* quat);


esp_err_t print_calib_profile_from_nvs(void);

/**
 * @brief   Resets the bno055 chip
*/
esp_err_t bno055_reset(void);

// void setMode(bno055_opmode_t mode);


// void setAxisRemap(bno055_axis_remap_config_t remapcode);


// void setAxisSign(bno055_axis_remap_sign_t remapsign);


// void getRevInfo(bno055_rev_info_t*);


// void setExtCrystalUse(boolean usextal);



// int8_t getTemp();


// void getSystemStatus(uint8_t* system_status, uint8_t* self_test_result, uint8_t* system_error);


// void getCalibration(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);



// /* Adafruit_Sensor implementation */
// bool getEvent(sensors_event_t*);
// bool getEvent(sensors_event_t*, adafruit_vector_type_t);
// void getSensor(sensor_t*);

// /* Functions to deal with raw calibration data */
// bool getSensorOffsets(uint8_t* calibData);
// bool getSensorOffsets(bno055_offsets_t& offsets_type);
// void setSensorOffsets(const uint8_t* calibData);
// void setSensorOffsets(const no055_offsets_t& offsets_type);
// bool isFullyCalibrated();

// /* Power managments functions */
// void enterSuspendMode();
// void enterNormalMode();






// esp_err_t readLen(bno055_reg_t reg, uint8_t* buffer, size_t len);







// Adafruit_I2CDevice* i2c_dev = NULL; ///< Pointer to I2C bus interface

// int32_t _sensorID;
// bno055_opmode_t _mode;





// Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A,
//     TwoWire* theWire = &Wire);


// imu::Vector<3> getVector(adafruit_vector_type_t vector_type);
// imu::Quaternion getQuat();