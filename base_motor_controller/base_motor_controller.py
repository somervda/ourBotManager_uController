from machine import Pin, I2C, Timer
import utime
from hbridge import HBridge
import gc


class Base_Motor_Controller():

    # Set up motor encoder interface
    motorEncoderCntR = 0
    motorEncoderCntTotalR = 0
    motorEncoderCntL = 0
    motorEncoderCntTotalL = 0

    # Current speed setting
    speed = 0

    # dFactor represents the coefficient for the amount that overall distance
    # corrections should be applied on one monitoring cycle. Values less than 1
    # will insure distance corrections are done slowly over multiple monitoring
    # cycles
    dFactor = 0.2

    def __init__(self, pin_num_pwm_l, pin_num_in1_l, pin_num_in2_l, pin_num_encoder_l, pin_num_pwm_r, pin_num_in1_r, pin_num_in2_r,  pin_num_encoder_r, freq=1000):
        # setup hbridge motor controller pin assignments
        self.motorLeft = HBridge(
            pin_num_pwm_l,  pin_num_in1_l, pin_num_in2_l, freq)
        self.motorRight = HBridge(
            pin_num_pwm_r,  pin_num_in1_r, pin_num_in2_r, freq)
        self.motorEncoderL = Pin(pin_num_encoder_l, Pin.IN)
        self.motorEncoderR = Pin(pin_num_encoder_r, Pin.IN)
        gc.collect()
        gc.disable()

    def run(self, speed):
        self.speed = speed
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0
        self.motorEncoderR.irq(trigger=Pin.IRQ_FALLING,
                               handler=self.motorEncoderCallbackR)
        self.motorEncoderL.irq(trigger=Pin.IRQ_FALLING,
                               handler=self.motorEncoderCallbackL)

        self.statusTimer = Timer()
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self.monitorRunStatus)
        if (self.speed > 0):
            self.motorLeft.forward(abs(self.speed))
            self.motorRight.forward(abs(self.speed))
        if (self.speed < 0):
            self.motorLeft.reverse(abs(self.speed))
            self.motorRight.reverse(abs(self.speed))
        if (self.speed == 0):
            self.stop()

    def turn(self, speed):
        # rotate the body at the velocity defined by speed
        self.speed = speed
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0
        self.motorEncoderR.irq(trigger=Pin.IRQ_FALLING,
                               handler=self.motorEncoderCallbackR)
        self.motorEncoderL.irq(trigger=Pin.IRQ_FALLING,
                               handler=self.motorEncoderCallbackL)

        self.statusTimer = Timer()
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self.monitorTurnStatus)
        if (self.speed > 0):
            self.motorLeft.reverse(abs(self.speed))
            self.motorRight.forward(abs(self.speed))
        if (self.speed < 0):
            self.motorLeft.forward(abs(self.speed))
            self.motorRight.reverse(abs(self.speed))
        if (self.speed == 0):
            self.stop()

    def stop(self):
        # Use stop when the motor control should be re-initialized
        # otherwise the next run() will continue to compensate for
        # existing distance discrepancies
        self.motorLeft.stop()
        self.motorRight.stop()
        self.statusTimer.deinit()
        self.motorEncoderR.irq(handler=None)
        self.motorEncoderL.irq(handler=None)
        self.motorEncoderCntTotalR = 0
        self.motorEncoderCntTotalL = 0
        gc.collect()

    def monitorRunStatus(self, timer):
        self.adjustLRSpeed(True)
        gc.collect()
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0

    def monitorTurnStatus(self, timer):
        self.adjustLRSpeed(False)
        gc.collect()
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0

    def adjustLRSpeed(self, isRun):
        # Checks the the distance moved on each Tick and
        # reduce the faster motor speed if needed
        speedL = self.speed
        speedR = self.speed
        if (self.motorEncoderCntR > 0 and self.motorEncoderCntL > 0 and self.motorEncoderCntTotalL > 0 and self.motorEncoderCntTotalR > 0):
            speedDiff = abs((self.motorEncoderCntL -
                             self.motorEncoderCntR) / ((self.motorEncoderCntR + self.motorEncoderCntL)/2))
            #  Distance difference is based on overall difference in distance divided by average delta distance
            distDiff = abs((self.motorEncoderCntTotalL -
                            self.motorEncoderCntTotalR) / ((self.motorEncoderCntR + self.motorEncoderCntL)/2))

            #  Set the difference to reduce faster motor speed based on if positive or negative speed
            if (self.motorEncoderCntL < self.motorEncoderCntR):
                speedL -= round(speedL * speedDiff)
            else:
                speedR -= round(speedR * speedDiff)

            # If speeds must be adjusted to compensate for over all distance
            # then do it gradually based on dFactor
            if (self.motorEncoderCntTotalL < self.motorEncoderCntTotalR):
                speedL -= round(speedL * distDiff * self.dFactor)
            else:
                speedR -= round(speedR * distDiff * self.dFactor)

        # Set motor directions depending on if it is a RUN or Turn action
        # and if direction is positive or negative
        if (isRun):
            if (self.speed > 0):
                self.motorLeft.forward(abs(speedL))
                self.motorRight.forward(abs(speedR))
            else:
                self.motorLeft.reverse(abs(speedL))
                self.motorRight.reverse(abs(speedR))
        else:
            if (self.speed > 0):
                self.motorLeft.reverse(abs(speedL))
                self.motorRight.forward(abs(speedR))
            else:
                self.motorLeft.forward(abs(speedL))
                self.motorRight.reverse(abs(speedR))

    def motorEncoderCallbackR(self, pin):
        self.motorEncoderCntR += 1
        self.motorEncoderCntTotalR += 1

    def motorEncoderCallbackL(self, pin):
        self.motorEncoderCntL += 1
        self.motorEncoderCntTotalL += 1
