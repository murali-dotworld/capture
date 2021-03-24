from apds9930 import APDS9930
import time
sensor = APDS9930(bus=1)  # initialize sensor by providing i2c bus number
sensor_timeout = 20
warm_time = 1
sensor_thresh = 1023
sensor = APDS9930(1)
sensor.enable_proximity_sensor(interrupt=False)
sensor.get_mode(2)
while True:
    val = sensor.proximity
    print(val)
    time.sleep(0.1)
