from base_motor_controller import Base_Motor_Controller
import utime

bmc = Base_Motor_Controller(20, 19, 18, 11, 12, 13, 10, 16)

bmc.run(90)
utime.sleep_ms(3000)
print("distL:" + str(bmc.motorEncoderCntTotalL) +
      " distR:" + str(bmc.motorEncoderCntTotalR))
bmc.stop()
utime.sleep_ms(2000)
bmc.turn(-40)
utime.sleep_ms(3000)
print("distL:" + str(bmc.motorEncoderCntTotalL) +
      " distR:" + str(bmc.motorEncoderCntTotalR))
bmc.stop()
