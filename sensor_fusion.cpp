//#include "stm32f4xx_hal.h"
//#include "sensor_fusion.h"
//#include <stdio.h>
//
//MPU6050::MPU6050(I2C_HandleTypeDef *hi2c): m_hi2c(hi2c) {
//}
//
//void MPU6050::start(void) {
//    /** TO DO
//     *
//     * CONFIGURE THE FOLLOWING REGISTERS ACCORDING TO THE DATASHEET:
//     *
//     * PWR_MGMT_1 register to take the IMU out of sleep mode
//     * ACCEL_CONFIG register to the smallest possible full-scale range (why might we want to do that?)
//     * GYRO_CONFIG register to the largest possible full-scale range to enable the detection of high-velocity rotations
//     * CONFIG register to the largest possible bandwidth.
//     *
//     * Refer to the spec for more detail.
//     */
//
//    /** YOUR CODE GOES BELOW */
//
//    //the PWR_MGMT_1 config is done for you as example. do the rest
//	uint8_t oldVal = '\0';
//	uint8_t newVal = '\0';
//
//	read_reg((uint8_t)PWR_MGMT_1, &oldVal);
//	newVal = (oldVal & 0b00011000);
//	write_reg((uint8_t)PWR_MGMT_1, newVal);
//
//	//ACCEL_CONFIG
//	read_reg((uint8_t)ACCEL_CONFIG, &oldVal);
//	newVal = (oldVal & 0b00000111);
//	write_reg((uint8_t)ACCEL_CONFIG, newVal);
//
//	//GYRO_CONFIG
//	read_reg((uint8_t)GYRO_CONFIG, &oldVal);
//	newVal = ((oldVal & 0b00011111) | 0b00011000);
//	write_reg((uint8_t)GYRO_CONFIG, newVal);
//
//	//CONFIG
//	read_reg((uint8_t)CONFIG, &oldVal);
//	newVal = (oldVal & 0b11000000);
//	write_reg((uint8_t)CONFIG, newVal);
//
//	newVal = 0x07;
//	write_reg((uint8_t)SMPLRT_DIV, newVal);
//
//
//}
//
//bool MPU6050::read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az) {
//
//	/** TO DO
//     *
//     * GET THE RAW READINGS FROM THE ACCELEROMETER/GYRSCOPE
//     *
//     * Store the readings in the floats pointed to by the given float pointers.
//     *
//     * Your implementation should use functions defined below.
//     */
//
//    /** YOUR CODE GOES BELOW */
//
//	if (!data_ready())
//		return false;
//
//	uint8_t accel_data[6];
//	uint8_t gyro_data[6];
//
//	if (!read_reg_seq(ACCEL_X, accel_data, 6))
//		return false;
//
//	if(!read_reg_seq(GYRO_X, gyro_data, 6))
//		return false;
//
//	*ax = (int16_t) (accel_data[0] << 8 | accel_data[1]);
//	*ay = (int16_t) (accel_data[2] << 8 | accel_data[3]);
//	*az = (int16_t) (accel_data[4] << 8 | accel_data[5]);
//
//	*gx = (int16_t) (gyro_data[0] << 8 | gyro_data[1]);
//	*gy = (int16_t) (gyro_data[2] << 8 | gyro_data[3]);
//	*gz = (int16_t) (gyro_data[4] << 8 | gyro_data[5]);
//
//	convert_raw(gx, gy, gz, ax, ay, az);
//
//	return true;
//
//
//}
//
//bool MPU6050::data_ready(void) {
//    /** TO DO
//     *
//     * CHECK THE INT_STATUS REGISTER TO DETERMINE IF DATA IS READY
//     *
//     * Return true if it is ready, false otherwise.
//     */
//
//    /** YOUR CODE GOES BELOW */
//
//	uint8_t status = '\0';
//
//	bool ret = read_reg(INT_STATUS, &status);
//
//	if((status & 0b00000001) && ret)
//		return true;
//
//	return false;
//
//}
//
//bool MPU6050::write_reg(uint8_t reg, uint8_t buf) {
//
//	/** TO DO
//     *
//     * IMPELEMENT THIS FUNCTION
//     *
//     * See the documentation in sensor_fusion.h for detail.
//     */
//
//    /** YOUR CODE GOES BELOW */
//	uint8_t data[2];
//	data[0] = reg;
//	data[1] = buf;
//
//	if (HAL_I2C_Master_Transmit(m_hi2c, ADDRESS, data, 2, HAL_MAX_DELAY) != HAL_OK)
//		return false;
//
//	return true;
//
//
//
//
//}
//
//bool MPU6050::read_reg(uint8_t reg, uint8_t *buf) {
//
//    /** TO DO
//     *
//     * IMPLEMENT THIS FUNCTION
//     *
//     * See the documentation in sensor_fusion.h for detail.
//     */
//
//    /** YOUR CODE GOES BELOW */
//
//
//
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(m_hi2c, ADDRESS, &reg, 1, HAL_MAX_DELAY);
//
//	if (ret != HAL_OK)
//		return false;
//
//	ret = HAL_I2C_Master_Receive(m_hi2c, ADDRESS, buf, 1, HAL_MAX_DELAY);
//
//	if (ret != HAL_OK)
//		return false;
//
//	return true;
//
//}
//
//bool MPU6050::read_reg_seq(uint8_t reg, uint8_t *buf, uint8_t length) {
//    /** TO DO
//     *
//     * IMPLEMENT THIS FUNCTION
//     *
//     * See the documentation in sensor_fusion.h for detail.
//     */
//
//    /** YOUR CODE GOES BELOW */
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(m_hi2c, ADDRESS, &reg, 1, HAL_MAX_DELAY);
//
//	if (ret != HAL_OK)
//		return false;
//
//	ret = HAL_I2C_Master_Receive(m_hi2c, ADDRESS, buf, length, HAL_MAX_DELAY);
//
//	if (ret != HAL_OK)
//		return false;
//
//	return true;
//}
//
//void MPU6050::convert_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az)
//{
//    /** TO DO
//     *
//     * IMPLEMENT THIS FUNCTION
//     *
//     * See the documentation in sensor_fusion.h for detail.
//     */
//
//    /** YOUR CODE GOES BELOW */
//
//	//16384 = 2^16 / 4
//	//16 bits, +-2 g range
//	*ax /= 16384.0;
//	*ay /= 16384.0;
//	*az /= 16384.0;
//
//	//16.4 = 2^16 / 4000
//	//16 bits, +-2000 degree range
//	*gx /= 16.4;
//	*gy /= 16.4;
//	*gz /= 16.4;
//
//}
