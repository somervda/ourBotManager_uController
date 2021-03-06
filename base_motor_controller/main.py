from base_motor_controller import Base_Motor_Controller
import utime
import util
from machine import Pin

led = Pin(25, Pin.OUT)
bmc = Base_Motor_Controller(20, 19, 18, 16, 11, 12,
                            13,  10, motorCruisingSpeed=35)

# Start the motor controller processing - once started it will
#  respond to serial commands
bmc.start(True)
print("UART serial started")
# Flash led 5 times to indicate things are running
for blink_loop in range(5):
    led.on()
    utime.sleep_ms(200)
    led.off()
    utime.sleep_ms(200)
