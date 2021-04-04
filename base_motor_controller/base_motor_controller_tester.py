from base_motor_controller import Base_Motor_Controller
import utime

bmc = Base_Motor_Controller(20, 19, 18, 10, 11, 12,
                            13,  16, motorCruisingSpeed=35)
# Start the motor controller processing - once started it will
#  respond to serial commands
bmc.start(True)
print("UART serial started")
