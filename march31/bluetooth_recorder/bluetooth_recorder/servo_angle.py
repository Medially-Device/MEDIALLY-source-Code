import time
import board
import busio
from adafruit_pca9685 import PCA9685

# Create the I2C bus interface.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the PCA9685 at I2C address 0x40.
pca = PCA9685(i2c, address=0x40)
pca.frequency = 50  # Set frequency to 50 Hz (typical for servos).

# Define pulse length limits for the servo (values are out of 4096).
servo_min = 150  # Minimum pulse length (approximately 1 ms).
servo_max = 600  # Maximum pulse length (approximately 2 ms).

def set_servo_angle(channel, angle):
    """
    Set the servo at the specified channel to the given angle (0 to 180).
    This function converts the angle to a pulse value between servo_min and servo_max,
    then scales that value to the 16-bit duty_cycle expected by the library.
    """
    # Compute pulse length in 12-bit resolution.
    pulse = int(servo_min + (angle / 180.0) * (servo_max - servo_min))
    # Convert 12-bit pulse (0-4095) to 16-bit duty cycle (0-65535).
    duty_cycle = int(pulse * (65535 / 4096))
    pca.channels[channel].duty_cycle = duty_cycle

# Set the servo on channel 0 to a fixed angle (e.g., 90°).
set_servo_angle(15, 180)
print("Servo on channel 0 set to 90°.")

# Keep the program running so the servo remains at that position.
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Exit gracefully on Ctrl+C.
    pass
finally:
    pca.deinit()
