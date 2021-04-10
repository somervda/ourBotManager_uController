from base_motor_controller import Base_Motor_Controller
import utime
from machine import Pin

led = Pin(25, Pin.OUT)
bmc = Base_Motor_Controller(20, 19, 18, 10, 11, 12,
                            13,  16, motorCruisingSpeed=35)
# Flash led 25 times (10 sec delay) to indicate things are running and
# also use to break out of the startup before bmc is started
for blink_loop in range(25):
    led.on()
    utime.sleep_ms(200)
    led.off()
    utime.sleep_ms(200)

# Start the motor controller processing - once started it will
#  respond to serial commands
bmc.start(True)
print("UART serial started")
