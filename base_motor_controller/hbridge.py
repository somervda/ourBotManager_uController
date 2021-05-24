# A HBridge motor controller class for managing L298 and TB6612FNG motor controllers

from machine import Pin, PWM


class HBridge():
    max_duty_u16 = 65535

    def __init__(self, pin_num_pwm, pin_num_in1, pin_num_in2, freq=1000):
        # setup pin assignments and frequency
        self._pwm = PWM(Pin(pin_num_pwm))
        self._pin_in1 = Pin(pin_num_in1, Pin.OUT)
        self._pin_in2 = Pin(pin_num_in2, Pin.OUT)
        self._pwm.freq(freq)

    #  functions to set motor controller signals
    def setForward(self):
        self._pin_in1.off()
        self._pin_in2.on()

    def setReverse(self):
        self._pin_in1.on()
        self._pin_in2.off()

    def setBrake(self):
        self._pin_in1.off()
        self._pin_in2.off()

    # Functions to set motor speed (As a percentage of max speed) and direction
    def forward(self, speed):
        # Speed %
        self.setForward()
        self._pwm.duty_u16(self.speedToU16(abs(speed)))

    def reverse(self, speed):
        self.setReverse()
        self._pwm.duty_u16(self.speedToU16(abs(speed)))

    def stop(self):
        self.setBrake()
        self._pwm.duty_u16(0)

    @classmethod
    def speedToU16(cls, speed):
        # Return a duty_u16 value based on percentage speed
        return int(speed * cls.max_duty_u16/100)
