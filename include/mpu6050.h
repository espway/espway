#ifndef MPU6050_H
#define MPU6050_H

// Register addresses and bits as per the MPU-6050 datasheet
// http://43zrtwysvxb2gf29r5o0athu.wpengine.netdna-cdn.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define MPU_PWR_MGMT_1 0x6B
#define MPU_TEMP_DIS (1 << 3)
#define MPU_CLK_PLL_ZGYRO 3

#define MPU_CONFIG 0x1A
#define MPU_SMPRT_DIV 0x19

#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C

#define MPU_INT_ENABLE 0x38
#define MPU_DATA_RDY_EN 1
#define MPU_MOT_EN (1 << 6)

#define MPU_INT_STATUS 0x3A
#define MPU_DATA_RDY_INT 1

#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_XOUT_L 0x3C
#define MPU_ACCEL_YOUT_H 0x3D
#define MPU_ACCEL_YOUT_L 0x3E
#define MPU_ACCEL_ZOUT_H 0x3F
#define MPU_ACCEL_ZOUT_L 0x40

#define MPU_TEMP_OUT_H 0x41
#define MPU_TEMP_OUT_L 0x42

#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_XOUT_L 0x44
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_YOUT_L 0x46
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_GYRO_ZOUT_L 0x48

#define MPU_WHO_AM_I 0x75

#define MPU_ACC_X 0
#define MPU_ACC_Y 1
#define MPU_ACC_Z 2
#define MPU_TEMP 3
#define MPU_GYRO_X 4
#define MPU_GYRO_Y 5
#define MPU_GYRO_Z 6

typedef struct {
    bool disableTemp;
    // Lowpass filter bandwidth
    // 0 = 260 Hz
    // 1 = 184 Hz
    // 2 = 94 Hz
    // 3 = 44 Hz
    // 4 = 21 Hz
    // 5 = 10 Hz
    // 6 = 5 Hz
    uint8_t lowpass;
    uint8_t sampleRateDivider;
    // Gyro full scale range
    // 0 = +- 250 deg/s
    // 1 = +- 500 deg/s
    // 2 = +- 1000 deg/s
    // 3 = +- 2000 deg/s
    uint8_t gyroRange;
    // Accelerometer full scale range
    // 0 = +- 2 g
    // 1 = +- 4 g
    // 2 = +- 8 g
    // 3 = +- 16 g
    uint8_t accelRange;
    bool enableInterrupt;
} mpuconfig;

typedef struct {
    int16_t gThresh;
    int16_t g2;
    int16_t gyroDivider;
    int16_t alpha;
    int16_t alphaComplement;
} mpufilter;

/* mpuconfig mpuDefaultConfig = {
    .disableTemp = true,
    .lowpass = 3,
    .sampleRateDivider = 4,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true
}; */

int mpuReadIntStatus(const uint8_t addr);
int mpuReadRawData(const uint8_t addr, int16_t * const data);
void mpuApplyOffsets(int16_t * const data, const int16_t * const offsets);
int mpuSetup(const uint8_t addr, const mpuconfig * const config);
void mpuSetupFilter(const mpuconfig * const config, mpufilter * const filter,
    const int16_t alpha);
void mpuUpdatePitch(mpufilter * const filter, int16_t * const data,
    int16_t * const pitch);

#endif

