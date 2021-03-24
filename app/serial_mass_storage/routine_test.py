#!/usr/bin/python
# -*- coding: utf-8 -*-

import traceback
import sys
import time
import timeit
import multiprocessing
import subprocess
import os
import re
from shutil import copyfile
from subprocess import call
import logging as log
from apds9930 import APDS9930
import RPi.GPIO as GPIO
import os.path
from os import path
import pathlib
import shutil
import traceback
import serial

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)  # Open port with baud rate
ser.write('captureid'.encode())

LED_PIN = 32  # FLASH PIN
#PRINT_LED = 33  #printer led

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)
#GPIO.setup(PRINT_LED, GPIO.OUT)
flash = GPIO.PWM(LED_PIN, 500)  # channel=12 frequency=5kHz
flash.start(0)
blink_thresh = 20

# Sensor initialize

sensor = APDS9930(bus=1)  # initialize sensor by providing i2c bus number
sensor.enable_proximity_sensor(interrupt=False)  # disable interrupt
sensor.get_mode(2)  # set in proximity mode
sensor_timeout = 20
warm_time = 1
sensor_thresh = 1023

# --------------------------------------


LED_BLINK_DELAY = 0.2

mnt_path = '/mnt/usb_share'
capture_fname = 'capture.txt'
shutdown_fname = 'shutdown.txt'
restart_fname = 'restart.txt'
transfer_fname = 'image.jpg'
fdelete_fname = 'received.txt'
cmd0 = 'raspistill -w 3280 -h 2464 -n -t 1 -o raw.jpg -cs 0'

clear_cache_cmd = 'echo 3 | sudo tee /proc/sys/vm/drop_caches'

umount_cmd = 'sudo modprobe -r g_mass_storage'
sync_cmd = 'sync'
mount_cmd = \
    'sudo sudo modprobe g_mass_storage file=/piusb.bin removable=1 ro=0 stall=0'
chmod_cmd = 'sudo mount -o remount,rw /mnt/usb_share/'
devnull = open(os.devnull, 'w')


def main():
    initCaptureid()
    print ('flash refreshed')
    operation()


def initCaptureid():
    #refreshFlashDrive()

    # subprocess.call(clear_cache_cmd, shell=True)

    if path.exists(os.path.join(mnt_path, 'image.jpg')):
        print ('image file deletion in mnt')
        os.remove(os.path.join(mnt_path, 'image.jpg'))
    if path.exists('raw.jpg'):
        print ('image file deletion in local')
        os.remove('raw.jpg')
    flash.ChangeDutyCycle(100)
    time.sleep(1)
    flash.ChangeDutyCycle(0)


def operation():
    PRINT_FLAG = False
    PRINT_TIMER = 0
    PRINT_LED_DELAY = 0.2
    PRINT_LED_COUNT = 0
    PRINT_LED_MAX = 8      #4secs
    PRINT_LED_PREV = False
    PRINT_FLAG = False
    while True:
        try:
            #if PRINT_FLAG == False:
                #GPIO.output(PRINT_LED,0)
            #GPIO.output(LED_PIN,0)
            #serData = readMaster()
            time.sleep(25)
            initCaptureid()
            time.sleep(5)
            serData = 'capok'
            if serData != None:

                # print (tcpData)

                if 'capok' in serData:
                    print ('capture method called')

                    # log.INFO('capture method called')

                    capture_start_time = timeit.default_timer()
                    CAPTURE_STATE = capture()

                    # GPIO.output(LED_PIN, 0)

                    flash.ChangeDutyCycle(0)
                    if CAPTURE_STATE == True:
                        sendImage('raw.jpg')
                        print ('captureing & transfering time: ' \
                            + str(timeit.default_timer()
                                  - capture_start_time))
                    elif CAPTURE_STATE == 'TIMEOUT':
                        writeMaster('0')
                        print ('timeout file writed')
                    elif CAPTURE_STATE == 'ERROR':

                        # log.INFO('operation cancelled')

                        writeMaster('2')
                        print ('error file writed')
                    elif CAPTURE_STATE == 'CANCEL':

                                                # writeMaster("cancel")

                        print ('canceled by master')
                        #ser.write('1'.encode())
                    elif CAPTURE_STATE == False:
                        break
                elif 'shtdn' in serData:
                    print ('INITIALIZING SHUTDDOWN...')

                    # log.INFO('INITIALIZING SHUTDDOWN')

                    subprocess.call('sudo shutdown now', shell=True)
                elif 'rstrt' in serData:
                    print ('INITIALIZING RESTART...')

                    # #log.INFO('INITIALIZING RESTART...')

                    subprocess.call('sudo reboot', shell=True)
                elif 'received' in serData:
                    print ('deleting images')
                    if path.exists(os.path.join(mnt_path, 'image.jpg')):
                        print ('image file deletion in mnt')
                        os.remove(os.path.join(mnt_path, 'image.jpg'))
                    if path.exists('raw.jpg'):
                        print ('image file deletion in local')
                        os.remove('raw.jpg')
                    ser.write('4'.encode())
                    refreshFlashDrive()
                elif 'print' in serData:
                    print ('printer signal')
                    PRINT_FLAG = True
                    PRINT_TIMER = timeit.default_timer()
            if PRINT_FLAG == True and (timeit.default_timer()-PRINT_TIMER) > PRINT_LED_DELAY:
                 print (PRINT_LED_PREV,PRINT_LED_COUNT)
                 PRINT_TIMER = timeit.default_timer()
                 #GPIO.output(PRINT_LED, PRINT_LED_PREV)
                 PRINT_LED_COUNT +=1
                 PRINT_LED_PREV = not(PRINT_LED_PREV)
                 if PRINT_LED_COUNT>PRINT_LED_MAX: 
                     PRINT_FLAG = False
                     PRINT_LED_COUNT = 0
                     #GPIO.output(PRINT_LED,0)
                     PRINT_LED_PREV = False

        except:

            print ('ERR11 moved to critical')
            ser.write('2'.encode())

            # log.INFO('ERR11 moved to critical')

            print ('<<<>>>')

            # log.INFO('<<<>>>')
            # print (e)

            traceback.print_exc()

            # log.INFO(str(e))

            time.sleep(2)
            os.execl(sys.executable, sys.executable, *sys.argv)


            # operation()
            # break

def writeMaster(msg):
    ser.write(msg.encode())


def readMaster():

    data = ser.readline()
    if len(data) > 0:
        print ('serial data->' + str(len(data)))
        if 'capture' in data:
            ser.write('1'.encode())
            return 'capok'
        elif 'restart' in data:
            ser.write('1'.encode())
            return 'rstrt'
        elif 'shutdown' in data:
            ser.write('1'.encode())
            return 'shtdn'
        elif 'cancel' in data:
            return 'cancel'
        elif 'received' in data:
            return 'received'
        elif 'print' in data:
            return 'print'
        elif '5' in data:
            ser.write('5'.encode())
    else:

        return 'null'


def sendImage(img_name):
    shutil.move(img_name, os.path.join(mnt_path, transfer_fname))
    subprocess.call(sync_cmd, shell=True)

    # subprocess.call(clear_cache_cmd, shell=True)

    refreshFlashDrive()


    # os.remove(img_name)

def refreshFlashDrive():
    subprocess.call(umount_cmd, shell=True)
    subprocess.call(sync_cmd, shell=True)
    subprocess.call(mount_cmd, shell=True)


    # subprocess.call(chmod_cmd, shell=True)

def sensorSignal():
    try:
        return 'OK'
        print ('start')
        sensor = APDS9930(1)
        sensor.enable_proximity_sensor(interrupt=False)
        sensor.get_mode(2)
        STATE = False
        START_TIME_1 = timeit.default_timer()
        blink_start = timeit.default_timer()
        blink_pwm = blink_thresh
        while True:

                # GPIO.output(LED_PIN, 1)

            val = sensor.proximity

                # GPIO.output(LED_PIN, 0)
                # print (">: "+ str(val))
                # ------------blink----------------

            if timeit.default_timer() - blink_start > LED_BLINK_DELAY:
                blink_start = timeit.default_timer()
                flash.ChangeDutyCycle(blink_pwm)
                blink_pwm = blink_thresh - blink_pwm

                        # print ("blink_state: "+str(blink_state))

            if val >= sensor_thresh:

            # GPIO.output(LED_PIN, 1)

                flash.ChangeDutyCycle(0)
                print ('sensed')
                START_TIME_2 = timeit.default_timer()
                while True:

                                # print ("timer : "+str(timeit.default_timer()-START_TIME_2))

                    val = sensor.proximity
                    if val < 1020:
                        break
                    if timeit.default_timer() - START_TIME_2 \
                        >= warm_time:
                        sensor.close()
                        flash.ChangeDutyCycle(100)

                                        # GPIO.output(LED_PIN, 1)

                        return 'OK'
            if timeit.default_timer() - START_TIME_1 >= sensor_timeout:
                sensor.close()

            # GPIO.output(LED_PIN, 0)

                return 'TIMEOUT'

                # time.sleep(0.05)

            serData = readMaster()

        # print (serData)

            if serData != None:
                if serData == 'cancel':
                    return 'CANCEL'
    except Exception as e:
        print ('------------EXCEPTION--------------')
        print (e)
        print ('-----------------------------------')
        return 'ERROR'


def capture():
    try:
        data = '0'
        recv = sensorSignal()
        print ('recv : ' + str(recv))
        if 'OK' in recv:
            start = timeit.default_timer()
            p1 = multiprocessing.Process(target=cheez, args=(cmd0, ))
            p1.start()
            p1.join()
            print ('TOtal time : ' + str(timeit.default_timer() - start))

            # log.INFO("image captured")

            return True
        return recv
    except Exception as e:

        # ....os.execl(sys.executable, sys.executable, *sys.argv)

        # c.send(("cancell".encode('utf-8')))
        # log.INFO("Exception capture method")
        # log.INFO(e)

        print (e)
        print ('ERR05 moved to critical')
        return False


def cheez(cmd):
    try:
        STATUS = 1
        trial = 0
        while STATUS != 0:
            STATUS = subprocess.call(cmd, shell=True)
            trial += 1
            if trial == 3:
                raise CameraError
    except Exception as e:

        # copyfile(SOURCE_LOG_FILENAME,SENDTO_LOG_FILENAME)
        # c.send(("cancell".encode('utf-8')))
        # log.INFO("exception in cheez")

        print ('exception in Cheez')
        ser.write('3'.encode())
        print (e)

        # log.INFO(e)

        print ('ERR07 moved to critical')


class Error(Exception):

    """Base class for other exception"""

    pass


class CameraError(Error):

    """Check camera cable"""

    pass


if __name__ == '__main__':
    main()
