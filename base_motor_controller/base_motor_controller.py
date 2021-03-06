from machine import Pin, I2C, Timer, UART
import machine
import utime
from hbridge import HBridge
import gc
#


class Base_Motor_Controller():

    debug = False

    uartData = ""

    # Set up motor encoder interface
    motorMode = "S"  # R=Run T=Turn S=Stopped
    baseTargetDistance = 0  # In average encoder ticks
    motorEncoderCntR = 0
    motorEncoderCntTotalR = 0
    motorEncoderCntL = 0
    motorEncoderCntTotalL = 0

    # Current speed setting
    speed = 0
    # motorSpeedMin is the minimum a speed can be set to or adjusted
    motorSpeedMin = 50

    motorCruisingSpeed = 50
    motorCruisingDistance = 700

    # dFactor represents the coefficient for the amount that overall distance
    # corrections should be applied on one monitoring cycle. Values less than 1
    # will insure distance corrections are done slowly over multiple monitoring
    # cycles
    dFactor = 0.2

    # srFactor and stFactor is how much speed adjustment is applied
    # normally 1 to get speeds adjusted on next cycle but I found turning speed
    # adjustments is a bit picky
    srFactor = 1.0
    stFactor = 0.5

    # Adjustment to convert millimeters to encoder ticks
    distanceRunTickRatio = 1.71393

    # Adjustment to convert degrees to encoder ticks
    distanceTurnTickRatio = 3.6111

    def __init__(self, pin_pwm_l, pin_in1_l, pin_in2_l, pin_encoder_l, pin_pwm_r, pin_in1_r, pin_in2_r, pin_encoder_r,
                 freq=1000, uart_id=0, motorCruisingSpeed=30, motorCruisingDistance=120):
        # setup hbridge motor controller pin assignments
        self.motorLeft = HBridge(
            pin_pwm_l,  pin_in1_l, pin_in2_l, freq)
        self.motorRight = HBridge(
            pin_pwm_r,  pin_in1_r, pin_in2_r, freq)
        self.motorEncoderL = Pin(pin_encoder_l, Pin.IN)
        self.motorEncoderR = Pin(pin_encoder_r, Pin.IN)
        # PI pico - not tested with esp32 (UART implementations seem to vary)
        self.uart = UART(uart_id, 9600)  # uart0 uses pins 1 (tx) and 2 (rx)
        self.motorCruisingSpeed = motorCruisingSpeed
        self.motorCruisingDistance = motorCruisingDistance
        gc.collect()
        gc.disable()

    def run(self, speed):
        self._setSpeed(speed)
        if self.motorMode != "R":
            self.motorMode = "R"
            #  Only restart encoders and update motors
            #  after change in mode
            self.motorEncoderCntR = 0
            self.motorEncoderCntL = 0
            if (self.speed > 0):
                self.motorLeft.forward(abs(self.speed))
                self.motorRight.forward(abs(self.speed))
            if (self.speed < 0):
                self.motorLeft.reverse(abs(self.speed))
                self.motorRight.reverse(abs(self.speed))
            if (self.speed == 0):
                self.stop()

    def runDistance(self, speed, distance):
        # convert distance to move into target number of ticks
        self.baseTargetDistance = int(distance * self.distanceRunTickRatio)
        self.run(speed)

    def turn(self, speed):
        # rotate the body at the velocity defined by speed
        self._setSpeed(speed)
        if self.motorMode != "T":
            self.motorMode = "T"
            #  Only restart encoders and update motors after change in mode
            self.motorEncoderCntR = 0
            self.motorEncoderCntL = 0
            if (self.speed > 0):
                self.motorLeft.reverse(abs(self.speed))
                self.motorRight.forward(abs(self.speed))
            if (self.speed < 0):
                self.motorLeft.forward(abs(self.speed))
                self.motorRight.reverse(abs(self.speed))
            if (self.speed == 0):
                self.stop()

    def turnDistance(self, speed, distance):
        self.baseTargetDistance = int(distance * self.distanceTurnTickRatio)
        self.turn(speed)

    def stop(self):
        # Use stop when the motor control should be re-initialized
        # otherwise the next run() will continue to compensate for
        # existing distance discrepancies
        self.motorLeft.stop()
        self.motorRight.stop()
        self.baseTargetDistance = 0
        #  Report back if a movement happened
        if self.motorMode in ["R", "T"]:
            # Send back info about average base movement in the form
            # of an R value (after a run) or a T value
            totalMovement = round(
                (self.motorEncoderCntTotalL + self.motorEncoderCntTotalR)/2)
            if (self.speed < 0):
                totalMovement *= -1
            movementString = self.motorMode + "%+06d" % totalMovement + "\n"
            self.uart.write(movementString.encode())
        #  Clean up movement info.
        self.speed = 0
        self.motorEncoderCntR = 0
        self.motorEncoderCntL = 0
        self.motorEncoderCntTotalR = 0
        self.motorEncoderCntTotalL = 0
        self.motorMode = "S"
        gc.collect()

    def start(self, debug=False):
        self.motorMode = "S"
        self.debug = debug
        self.statusTimer = Timer()
        self.statusTimer.init(freq=10, mode=Timer.PERIODIC,
                              callback=self._monitorStatus)
        self.motorEncoderR.irq(trigger=Pin.IRQ_FALLING,
                               handler=self._motorEncoderCallbackR)
        self.motorEncoderL.irq(trigger=Pin.IRQ_FALLING,
                               handler=self._motorEncoderCallbackL)

    def _monitorStatus(self, timer):
        if self.motorMode != "S":
            self._adjustLRSpeed()
            self.motorEncoderCntR = 0
            self.motorEncoderCntL = 0
        self._uartListen()
        gc.collect()

    def quit(self):
        try:
            self.statusTimer.deinit()
            print("Quit OK")
        except:
            print("Could not deinit status timer!")
        try:
            self.motorEncoderR.irq(handler=None)
            self.motorEncoderL.irq(handler=None)
        except:
            print("Could not release encoder interupts!")

    def reboot(self):
        # Hardware reboot to get everything restarted
        machine.reset()

    def _setSpeed(self, speed):
        # Set the initial speed after a run or turn request
        self.speed = speed
        if abs(self.speed) < self.motorSpeedMin:
            if speed >= 0:
                self.speed = self.motorSpeedMin
            else:
                self.speed = self.motorSpeedMin * -1

    def _adjustLRSpeed(self):
        # Checks the the distance moved on each Tick and
        # reduce the faster motor speed if needed

        # Slow down to cruising speed if within cruising distance
        if self.baseTargetDistance > 0:
            baseDistance = ((self.motorEncoderCntTotalR +
                             self.motorEncoderCntTotalL)/2)
            if (self.speed != 0 and (self.baseTargetDistance - baseDistance < self.motorCruisingDistance) and self.motorCruisingSpeed < self.speed) and self.motorMode == "R":
                self.speed = self.motorCruisingSpeed
                if self.debug:
                    print("Crusing!")

        speedL = abs(self.speed)
        speedR = abs(self.speed)
        # Start adjusting speeds after first encoder feedback has occurred
        if (self.motorEncoderCntR > 0 and self.motorEncoderCntL > 0 and self.motorEncoderCntTotalL > 0 and self.motorEncoderCntTotalR > 0):
            speedDiff = abs((self.motorEncoderCntL -
                             self.motorEncoderCntR) / ((self.motorEncoderCntR + self.motorEncoderCntL)/2))
            #  Distance difference is based on overall difference in distance divided by average delta distance
            distDiff = abs((self.motorEncoderCntTotalL -
                            self.motorEncoderCntTotalR) / ((self.motorEncoderCntR + self.motorEncoderCntL)/2))

            #  Set the difference to reduce faster motor speed based on if positive or negative speed
            # Apply the srFactor or stFactor depending on the movement type.
            sFactor = 1.0
            if self.motorMode == "R":
                sFactor = self.srFactor
            else:
                sFactor = self.stFactor

            if (self.motorEncoderCntL < self.motorEncoderCntR):
                speedR -= round(speedR * speedDiff * sFactor)
            else:
                speedL -= round(speedL * speedDiff * sFactor)

            # If speeds must be adjusted to compensate for over all distance
            # then do it gradually based on dFactor
            if (self.motorEncoderCntTotalL < self.motorEncoderCntTotalR):
                speedR -= round(speedR * distDiff * self.dFactor)
            else:
                speedL -= round(speedL * distDiff * self.dFactor)
            if self.debug:
                print("speedDiff:" + str(speedDiff) + " distDiff:" + str(distDiff) + " encoderL:" +
                      str(self.motorEncoderCntL) + " encoderR:" + str(self.motorEncoderCntR))

        # If we are moving a certain distance then check the distance the base has moved (average of the two motor encoders)
        # vs the  baseTargetDistance
        if self.baseTargetDistance > 0:
            baseDistance = ((self.motorEncoderCntTotalR +
                             self.motorEncoderCntTotalL)/2)
            baseSpeed = ((self.motorEncoderCntR + self.motorEncoderCntL)/2)
            # slowdown when distance will be exceeded at current speed
            if self.speed != 0 and (baseSpeed + baseDistance > self.baseTargetDistance) and baseSpeed > 0:
                slowing = (self.baseTargetDistance -
                           baseDistance) / baseSpeed
                speedL *= slowing
                speedR *= slowing
                if self.debug:
                    print("Slowing down speedL:" +
                          str(speedL) + "  speedR:" + str(speedR))
            # Stop if distance exceeded
            if baseDistance > self.baseTargetDistance:
                if self.debug:
                    print("Distance exceeded, stopped  baseDist:" + str(baseDistance) + " distL:" +
                          str(self.motorEncoderCntTotalL) + " distR:" + str(self.motorEncoderCntTotalR))
                self.stop()
        if self.motorMode in ["R", "T"]:
            # Set motor directions depending on if it is a Run or Turn action
            # and if direction is positive or negative
            if speedL < self.motorSpeedMin:
                speedL = self.motorSpeedMin
            if speedR < self.motorSpeedMin:
                speedR = self.motorSpeedMin
            if self.debug:
                print("speedL:" + str(speedL) + " speedR:" + str(speedR) + " distL:" +
                      str(self.motorEncoderCntTotalL) + " distR:" + str(self.motorEncoderCntTotalR))
                print("")
            if (self.motorMode == "R"):
                #  Is motorMode=R and going forward or back in straight line
                if self.speed > 0:
                    self.motorLeft.forward(speedL)
                    self.motorRight.forward(speedR)
                else:
                    self.motorLeft.reverse(speedL)
                    self.motorRight.reverse(speedR)
            else:
                # otherwise is a turn - motorMode = T
                if self.speed > 0:
                    self.motorLeft.reverse(speedL)
                    self.motorRight.forward(speedR)
                else:
                    self.motorLeft.forward(speedL)
                    self.motorRight.reverse(speedR)

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
            elif command == "RB":
                self.reboot()
            elif command == "RV":
                self.run(velocity)
            elif command == "TV":
                self.turn(velocity)
            elif command == "RD":
                self.runDistance(velocity, distance)
            elif command == "TD":
                self.turnDistance(velocity, distance)
