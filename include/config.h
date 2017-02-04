// Angle PID == the PID controller which regulates motor output signal to reach
// the target angle
const float ANGLE_KP = 5.0, ANGLE_KI = 10.0, ANGLE_KD = 0.1;
const float ANGLE_HIGH_KP = 20.0, ANGLE_HIGH_KI = ANGLE_KI, ANGLE_HIGH_KD = ANGLE_KD;
// Velocity PID == the PID controller which regulates target angle to reach
// the target velocity
const float VEL_KP = 2.0, VEL_KI = 0.5, VEL_KD = 0.002;

const float STEERING_FACTOR = 0.13,
            SPEED_CONTROL_FACTOR = 0.67;

const float TRAVEL_SPEED_SMOOTHING = 0.002,
            TARGET_SPEED_SMOOTHING = 0.001;

// Madgwick beta == accelerometer correction factor. Should be fine-ish as is
const float MADGWICK_BETA = 0.1;

const unsigned long ORIENTATION_STABILIZE_DURATION = 6000;

const float FALL_LIMIT = 0.75,
            RECOVER_LIMIT = 0.2,
            HIGH_PID_LIMIT = 0.2;

const float STABLE_ANGLE = -0.2;

const int16_t GYRO_OFFSETS[] = { -27, -89, 14 };

// Undervoltage cutoff check
const float BATTERY_THRESHOLD = 7.4;
const float BATTERY_CALIBRATION_FACTOR = 102.4;
const unsigned long BATTERY_CHECK_INTERVAL = 500;
const bool ENABLE_BATTERY_CHECK = true;

const bool REVERSE_LEFT_MOTOR = true, REVERSE_RIGHT_MOTOR = true;

