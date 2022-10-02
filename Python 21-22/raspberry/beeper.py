import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class Beeper:
    pin = 0

    def __init__(self, pin=23):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)

    def setHIGH(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def setLOW(self):
        GPIO.output(self.pin, GPIO.LOW)
