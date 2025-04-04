import lgpio
import time

BUTTON_PIN = 27
CHIP = 4  # Usually 0 for Raspberry Pi

# Setup GPIO using lgpio
h = lgpio.gpiochip_open(CHIP)
lgpio.gpio_claim_input(h, BUTTON_PIN)

try:
    while True:
        button_state = lgpio.gpio_read(h, BUTTON_PIN)  # Read GPIO state
        if button_state == 1:
            print("Button is pressed.")
        else:
            print("Button is not pressed.")
        time.sleep(0.1)  # Small delay to avoid excessive CPU usage

except KeyboardInterrupt:
    lgpio.gpiochip_close(h)
    print("\nGPIO cleanup done. Exiting.")
