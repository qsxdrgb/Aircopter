//#ifndef __MPU6050_H
//#define __MPU6050_H
//
//#include "stm32f4xx_hal.h"
//
///** TO DO
// * DEFINE THESE REGISTERS.
// * THE ADDRESS AND CONFIG REGISTERS HAVE BEEN DONE FOR YOU AS AN EXAMPLE.
// *
// * You will need to look at the register map.
// */
//
//#define ADDRESS                  0xD1   //the slave address of the IMU
//#define INT_STATUS               0x3A   //the interrupt status register
//#define PWR_MGMT_1               0x6B   //power management 1
//#define SMPLRT_DIV				 0x19   //sample rate dividend. Set sample rate to 1 khz.
//#define CONFIG                   0x1A   //the IMU config register
//#define GYRO_CONFIG              0x1B   //the gyroscope config register
//#define ACCEL_CONFIG             0x1C   //the accelerometer config register
//#define GYRO_X                   0x43   //the register that stores the first bit of the gyroscope x value
//#define ACCEL_X                  0x3C   //the register that stores the first bit of the accelerometer x value
//
//class MPU6050 {
//
//public:
//
//    /**
//     * Creates an MPU6050 object
//     *
//     * hi2c: private variable for pointer to hi2c object created for I2C initialization by MX_I2CX_Init to initialize i2c channel that interfaces with the IMU.
//     */
//    MPU6050(I2C_HandleTypeDef *hi2c);
//
//    /**
//     * Called before using the MPU6050.
//     *
//     * Sets up the I2C by configuring registers
//     * as described in the spec (calls write_reg and read_reg).
//     */
//    void start(void);
//
//    /**
//     * Gets the readings from the gyroscope and accelerometer (calls read_reg_seq).
//     *
//     * Parameters: pointers to addresses where these readings should be stored.
//     *
//     * Returns true upon success, and false upon failure.
//     */
//    bool read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az);
//
//    /**
//     * Reads the interrupt status register (calls read_reg).
//     *
//     * Returns true if data is ready, false otherwise.
//     */
//    bool data_ready(void);
//
//
//private:
//
//    //I2C i2c_object; //instance of an i2c class
//
//    /**
//     * Writes 1 byte of data to a specific address
//     *
//     * addr: the i2c address of the slave device
//     * reg: the register number to write to
//     * buf: the data to write
//     *
//     * Returns true upon success, and false upon failure.
//     */
//    bool write_reg(uint8_t reg, uint8_t buf);
//
//    /**
//     * Reads a byte data from a specific address.
//     *
//     * reg: the register address to read from
//     * buf: an array to store the data read
//     *
//     * Returns true upon success, and false upon failure.
//     */
//    bool read_reg(uint8_t reg, uint8_t *buf);
//
//    /**
//     * Reads a sequence of data from a specific address, up to 8 bytes
//     *
//     * reg: the register address to read from
//     * buf: an array to store the data read
//     * length: length of buf (how many bytes to read)
//     *
//     * Returns true upon success, and false upon failure.
//     */
//
//    bool read_reg_seq(uint8_t reg, uint8_t *buf, uint8_t length);
//
//    /**
//     * Converts raw data to g and degree/s
//     */
//    void convert_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az);
//
//    /**
//     *Private variable for pointer to hi2c object created for I2C initialization by MX_I2CX_Init
//     */
//
//    I2C_HandleTypeDef *m_hi2c;
//};
//
//#endif
