# Proposed pin map (pin-map-power Decision in docs is still open — confirm
# before wiring). Constraints: UART0 for CRSF, UART1 for GPS, I2C0 for the
# MPU6050, servos on one PWM slice (both 50 Hz), ESC on its own slice.

CRSF_UART_ID = 0
CRSF_TX_PIN = 0   # GP0, Pico -> ELRS RX (telemetry wire)
CRSF_RX_PIN = 1   # GP1, ELRS TX -> Pico

GPS_UART_ID = 1
GPS_TX_PIN = 4    # GP4, Pico -> BZ-251 RX (UBX config)
GPS_RX_PIN = 5    # GP5, BZ-251 TX -> Pico

IMU_I2C_ID = 0
IMU_SDA_PIN = 8   # GP8
IMU_SCL_PIN = 9   # GP9
# The BZ-251's onboard QMC5883 compass shares this same I2C0 bus (wire its
# SDA/SCL pads to GP8/GP9): fixed address 0x0D vs the MPU6050's 0x68.

SERVO_LEFT_PIN = 2   # GP2, PWM slice 1 channel A
SERVO_RIGHT_PIN = 3  # GP3, PWM slice 1 channel B (same slice, same 50 Hz)

ESC_PIN = 6  # GP6, PWM slice 3 channel A (Oneshot125)

# GP23/24/25/29 belong to the Pico W's wireless chip and are never exposed.
_RESERVED = (23, 24, 25, 29)
_CLAIMED = (
    CRSF_TX_PIN, CRSF_RX_PIN, GPS_TX_PIN, GPS_RX_PIN,
    IMU_SDA_PIN, IMU_SCL_PIN, SERVO_LEFT_PIN, SERVO_RIGHT_PIN, ESC_PIN,
)
# Unclaimed GPIOs, available to user logic as digital in/out or PWM; 26-28
# are additionally ADC-capable. Computed here, from the map above, so the
# list cannot drift from the wiring. Kept in this module (which imports
# nothing) so both the hardware layer and the web server can use it.
FREE_PINS = tuple(
    gp for gp in list(range(0, 23)) + [26, 27, 28]
    if gp not in _CLAIMED and gp not in _RESERVED
)
ADC_PINS = (26, 27, 28)
