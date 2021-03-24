import RPi.GPIO as GPIO
import time
LED_PIN = 32
PRINT_LED = 33
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(PRINT_LED, GPIO.OUT)

while True:
	GPIO.output(PRINT_LED, 1)
	GPIO.output(LED_PIN, 1)
	time.sleep(2)
	GPIO.output(PRINT_LED, 0)
	GPIO.output(LED_PIN, 0)
	time.sleep(2)
GPIO.output(LED_PIN, 1)
GPIO.output(PRINT_LED, 1)
time.sleep(10)
