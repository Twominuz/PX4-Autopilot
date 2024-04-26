uint64 timestamp                                 # time since system start (microseconds)


uint16 counter                                          # incremented by the writing thread everytime new data is stored
uint8 count                                         # number of connected ESCs
uint8 connectiontype                                # how ESCs connected to the system

uint8 info
uint16[4] failure_flags					#{ "failure_flags", NULL, MAVLINK_TYPE_UINT16_T, 4, 26, offsetof(mavlink_servo_info_t, failure_flags) }, \
uint32[4] error_count					#{ "error_count", NULL, MAVLINK_TYPE_UINT32_T, 4, 8, offsetof(mavlink_servo_info_t, error_count) }, \
uint16[4] temperature					#{ "temperature", NULL, MAVLINK_TYPE_INT16_T, 4, 34, offsetof(mavlink_servo_info_t, temperature) }, \






uint32 esc_errorcount                                   # Number of reported errors by ESC - if supported



uint8 FAILURE_NONE = 0
uint8 FAILURE_OVER_CURRENT = 1
uint8 FAILURE_OVER_VOLTAGE = 2
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 4
uint8 FAILURE_OVER_RPM = 8
uint8 FAILURE_INCONSISTENT_CMD = 16
uint8 FAILURE_MOTOR_STUCK = 32
uint8 FAILURE_GENERIC = 64


uint8 CONNECTED_ESC_MAX = 8                             # The number of ESCs supported. Current (Q2/2013) we support 8 ESCs


uint8 ESC_CONNECTION_TYPE_PPM = 0                       # Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1                    # Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2                   # One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3                       # I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4                       # CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5                     # DShot
