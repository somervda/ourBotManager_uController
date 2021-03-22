from machine import Pin, I2C, Timer, UART
# import machine
import utime
from hbridge import HBridge
import gc


class Base_Motor_Controller():

    debug = False

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
    # motorSpeedMin is the minimum a speed can be set to or adjusted
    motorSpeedMin = 10

    # dFactor represents the coefficient for the amount that overall distance
    # corrections should be applied on one monitoring cycle. Values less than 1
    # will insure distance corrections are done slowly over multiple monitoring
    # cycles
    dFactor = 0.2

    def __init__(self, pin_pwm_l, pin_in1_l, pin_in2_l, pin_encoder_l, pin_pwm_r, pin_in1_r, pin_in2_r,  pin_encoder_r, freq=1000, uart_id=0):
        # setup hbridge motor controller pin assignments
        self.motorLeft = HBridge(
            pin_pwm_l,  pin_in1_l, pin_in2_l, freq)
        self.motorRight = HBridge(
            pin_pwm_r,  pin_in1_r, pin_in2_r, freq)
        self.motorEncoderL = Pin(pin_encoder_l, Pin.IN)
        self.motorEncoderR = Pin(pin_encoder_r, Pin.IN)
        # PI pico - not tested with esp32 (UART implementations seem to vary)
        self.uart = UART(uart_id, 9600)  # uart0 uses pins 1 (tx) and 2 (rx)
        gc.collect()
        gc.disable()

    def run(self, speed):
        self.speed = speed
        if self.speed < self.motorSpeedMin:
            self.speed = self.motorSpeedMin
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
        if self.speed < self.motorSpeedMin:
            self.speed = self.motorSpeedMin
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
                print("speedDiff:" + str(speedDiff) + " distDiff:" + str(distDiff) + " deltaL:" +
                      str(self.motorEncoderCntL) + " deltaR:" + str(self.motorEncoderCntR))
        if self.debug:
            print("speedL:" + str(speedL) + " speedR:" + str(speedR) + " distL:" +
                  str(self.motorEncoderCntTotalL) + " distR:" + str(self.motorEncoderCntTotalR))
        # If we are moving a certain distance then check the distance the base has moved (average of the two motor encoders)
        # vs the  baseTargetDistance
        if self.baseTargetDistance > 0:
            baseDistance = ((self.motorEncoderCntTotalR +
                             self.motorEncoderCntTotalL)/2)
            baseSpeed = ((self.motorEncoderCntR + self.motorEncoderCntL)/2)
            # Stop after distance exceeded
            if baseDistance > self.baseTargetDistance:
                if self.debug:
                    print("Distance exceeded, stopped:" + str(baseDistance))
                self.stop()
            # slowdown when distance will be exceeded at current speed
            if baseSpeed + baseDistance > self.baseTargetDistance and baseSpeed > 0:
                slowing = (self.baseTargetDistance - baseDistance) / baseSpeed
                speedL *= slowing
                speedR *= slowing
                if self.debug:
                    print("Slowing down speedL:" +
                          str(speedL) + "  speedR:" + str(speedR))
        if self.speed > 0:
            # Set motor directions depending on if it is a RUN or Turn action
            # and if direction is positive or negative
            if speedL < self.motorSpeedMin:
                speedL = self.motorSpeedMin
            if speedR < self.motorSpeedMin:
                speedR = self.motorSpeedMin
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
