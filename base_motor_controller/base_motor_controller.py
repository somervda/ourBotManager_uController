from machine import Pin, I2C, Timer, UART
# import machine
import utime
from hbridge import HBridge
import gc


class Base_Motor_Controller():

    debug = False

    uart = UART(0, 9600)  # uart0 pins 1 (tx) and 2 (rx)
    uartData = ""

    # Set up motor encoder interface
    motorMode = "L"  # R=Run T=Turn L=Listen
    baseTargetDistance = 0  # In average encoder ticks
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
                               handler=self._motorEncoderCallbackR)
        self.motorEncoderL.irq(trigger=Pin.IRQ_FALLING,
                               handler=self._motorEncoderCallbackL)

        self.statusTimer = Timer()
        self.motorMode = "R"
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self._monitorStatus)
        if (self.speed > 0):
            self.motorLeft.forward(abs(self.speed))
            self.motorRight.forward(abs(self.speed))
        if (self.speed < 0):
            self.motorLeft.reverse(abs(self.speed))
            self.motorRight.reverse(abs(self.speed))
        if (self.speed == 0):
            self.stop()

    def runDistance(self, speed, distance):
        self.baseTargetDistance = distance
        self.run(speed)

    def turn(self, speed):
        # rotate the body at the velocity defined by speed
        self.speed = speed
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0
        self.motorEncoderR.irq(trigger=Pin.IRQ_FALLING,
                               handler=self._motorEncoderCallbackR)
        self.motorEncoderL.irq(trigger=Pin.IRQ_FALLING,
                               handler=self._motorEncoderCallbackL)

        self.statusTimer = Timer()
        self.motorMode = "T"
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self._monitorStatus)
        if (self.speed > 0):
            self.motorLeft.reverse(abs(self.speed))
            self.motorRight.forward(abs(self.speed))
        if (self.speed < 0):
            self.motorLeft.forward(abs(self.speed))
            self.motorRight.reverse(abs(self.speed))
        if (self.speed == 0):
            self.stop()

    def turnDistance(self, speed, distance):
        self.baseTargetDistance = distance
        self.turn(speed)

    def stop(self):
        # Use stop when the motor control should be re-initialized
        # otherwise the next run() will continue to compensate for
        # existing distance discrepancies
        self.speed = 0
        self.motorLeft.stop()
        self.motorRight.stop()
        self.baseTargetDistance = 0
        self.motorEncoderR.irq(handler=None)
        self.motorEncoderL.irq(handler=None)
        self.motorEncoderCntTotalR = 0
        self.motorEncoderCntTotalL = 0
        self.motorMode = "L"
        gc.collect()

    def start(self, debug=False):
        self.motorMode = "L"
        self.debug = debug
        self.statusTimer = Timer()
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self._monitorStatus)

    def quit(self):
        try:
            self.statusTimer.deinit()
            print("Quit OK")
        except:
            print("Could not deinit status timer!")

    def _monitorStatus(self, timer):
        if self.motorMode != "L":
            self._adjustLRSpeed(self.motorMode == "R")
            self.motorEncoderCntR = 0
            self.motorEncoderCntL = 0
        self._uartListen()
        gc.collect()

    def _adjustLRSpeed(self, isRun):
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
        if self.debug:
            print("speedL:" + str(speedL) + " speedL:" + str(speedR) + " distL:" +
                  str(self.motorEncoderCntTotalL) + " distR:" + str(self.motorEncoderCntTotalR) + " speedDiff:" + str(speedDiff) + " distDiff:" + str(distDiff) + " deltaL:" +
                  str(self.motorEncoderCntL) + " deltaR:" + str(self.motorEncoderCntR))
        # If we are moving a certain distance then check the distance the base has moved (average of the two motor encoders)
        # vs the  baseTargetDistance
        if self.baseTargetDistance > 0 and ((self.motorEncoderCntTotalR + self.motorEncoderCntTotalL)/2) > self.baseTargetDistance:
            if self.debug:
                print("Distance exceeded, stopped:" +
                      str((self.motorEncoderCntR + self.motorEncoderCntL)/2))
            self.stop()
        else:
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

    def _motorEncoderCallbackR(self, pin):
        self.motorEncoderCntR += 1
        self.motorEncoderCntTotalR += 1

    def _motorEncoderCallbackL(self, pin):
        self.motorEncoderCntL += 1
        self.motorEncoderCntTotalL += 1

    def _uartListen(self):
        while (self.uart.any()):
            uartIn = self.uart.read(1).decode("utf-8")
            if uartIn == "\n":
                self._processData()
                self.uartData = ""
            else:
                self.uartData += uartIn

    def _processData(self):
        # print(str(self.uart.any()))
        command = ""
        velocity = 0
        distance = 0
        # print("_processData:" + self.uartData + " " + str(len(self.uartData)))
        if len(self.uartData) == 2:
            command = self.uartData
            if command not in ["SP", "RB", "QT"]:
                command = ""
        if len(self.uartData) == 6:
            command = self.uartData[0:2]
            if command not in ["RV", "TV"]:
                command = ""
            else:
                try:
                    velocity = int(self.uartData[2:6])
                except:
                    command = ""
                    velocity = 0
        if len(self.uartData) == 11:
            command = self.uartData[0:2]
            if command not in ["RD", "TD"]:
                command = ""
            else:
                try:
                    velocity = int(self.uartData[2:6])
                    distance = int(self.uartData[6:11])
                except:
                    command = ""
                    velocity = 0
                    distance = 0
        if len(command) > 0:
            if self.debug:
                print("commandMsg:", self.uartData + " command:" + command +
                      " velocity:" + str(velocity) + " distance:" + str(distance))
            if command == "QT":
                self.stop()
                self.quit()
            elif command == "SP":
                self.stop()
            elif command == "RV":
                self.run(velocity)
            elif command == "TV":
                self.turn(velocity)
            elif command == "RD":
                self.runDistance(velocity, distance)
            elif command == "TD":
                self.turnDistance(velocity, distance)
