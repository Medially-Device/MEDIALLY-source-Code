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
# These values might need adjustment depending on your servo's requirements.
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

def sweep_channel(channel):
    """Sweep the servo on the specified channel from 0° to 180° and back to 0°."""
    # Sweep from 0° to 180°.
    for angle in range(0, 90, 2):
        set_servo_angle(channel, angle)
        time.sleep(0.02)
    # Sweep from 180° back to 0°.


channels = [0]

try:
    while True:
        for ch in channels:
            print(f"Sweeping servo on channel {ch}")
            sweep_channel(ch)
            time.sleep(1)  # Pause for 1 second before moving to the next channel.
except KeyboardInterrupt:
    # Exit gracefully on Ctrl+C.
    pass
finally:
    pca.deinit()
