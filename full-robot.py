#!/usr/bin/env python3
import serial
import time
import numpy as np
import RPi.GPIO as GPIO
import asyncio

def sendString(port,baud,input,waitTime):
    ser=serial.Serial(port,baud)
    
    for x in input:
        ser.write(bytes(x,'utf-8'))
        # ser.write("%s"%(x).encode())
        time.sleep(waitTime)

sensor = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor,GPIO.IN)
shoot = False

def shootCommand(shootInternal,desVelLF,desVelLR,desVelRF,desVelRR,duration):
    sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shootInternal)+'>',0.0001)

old_speed = 0
def lpf(input_speed):
    global old_speed
    alpha = 0.04
    new_speed = alpha*input_speed + (1-alpha)*old_speed
    old_speed = new_speed
    return new_speed

async def IRcheck():
    await asyncio.sleep(0.2)
    irValue = GPIO.input(sensor)
    return not irValue

async def main():
    ser = serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer() # clears anything the arduino has been sending while the Rpi isnt prepared to recieve.

    duration = 10
    shoot = "false"
    detection = False

    # try:

    #     driveForward = True
    #     while driveForward:

            # new_speed = lpf(30)
            # print(old_speed)
            # desVelRF = new_speed
            # desVelLR = new_speed
            # desVelLF = new_speed
            # desVelRR = new_speed
            # #desVelLF, desVelLR, desVelRF, desVelRR = new_speed, new_speed, new_speed, new_speed
            # sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
            # time.sleep(0.01)

    #         # DRIVE COMMANDS
    #         desVelLF = 30
    #         desVelRR = 30
    #         desVelRF = 30
    #         desVelLR = 30
    #         sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
    #         time.sleep(1)

    #         desVelLF = 0
    #         desVelRR = 0
    #         desVelRF = 0
    #         desVelLR = 0
    #         sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
    #         time.sleep(2)

    #         shootCommand("true",desVelLF,desVelLR,desVelRF,desVelRR,duration)
    #         time.sleep(5)
    #         shootCommand("false",desVelLF,desVelLR,desVelRF,desVelRR,duration)
    #         time.sleep(5)
            
    #         desVelLF = -30
    #         desVelRR = -30
    #         desVelRF = -30
    #         desVelLR = -30
    #         sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
    #         time.sleep(1)

    #         desVelLF = 0
    #         desVelRR = 0
    #         desVelRF = 0
    #         desVelLR = 0
    #         sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
    #         time.sleep(2)

    # except KeyboardInterrupt:
    #     GPIO.cleanup()
    
    try:
        # drive forward
        desVelLF = 40
        desVelRR = 40
        desVelRF = 40
        desVelLR = 40
        sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
        time.sleep(1)

        # stop
        desVelLF = 0
        desVelRR = 0
        desVelRF = 0
        desVelLR = 0
        sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
        time.sleep(2)
        
        # rotate + check IR + shoot
        CheckIRStatus = True
        currentTime = time.time()
        turn = True
        while CheckIRStatus:
            
            sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)      
            detection = await IRcheck()
            if detection:
                print('DETECTED')
                desVelLF = 0
                desVelLR = 0
                desVelRF = 0
                desVelRR = 0
                sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)      
                time.sleep(1)
                shootCommand("true",desVelLF,desVelLR,desVelRF,desVelRR,duration)
                time.sleep(5)
                shootCommand("false",desVelLF,desVelLR,desVelRF,desVelRR,duration)
                time.sleep(7)
            else:
                print('NOT DETECTED')
                if(not turn):
                    desVelLF = 30
                    desVelLR = 30
                    desVelRF = -30
                    desVelRR = -30
                    sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001) 
                    turn = True
                    currentTime = time.time()
                elif(time.time()-currentTime > 0.4):
                    turn = False
                elif(time.time()-currentTime > 0.2 ):
                    desVelLF = 0
                    desVelLR = 0
                    desVelRF = 0
                    desVelRR = 0
                    sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)

    except KeyboardInterrupt:
        # Turn off Motors Before Exit
        desVelLF = 0
        desVelLR = 0
        desVelRF = 0
        desVelRR = 0
        sendString('/dev/ttyACM0',115200,'<'+str(desVelLF)+','+str(desVelLR)+','+str(desVelRF)+','+str(desVelRR)+','+str(duration)+','+str(shoot)+'>',0.0001)
        time.sleep(1)
        GPIO.cleanup()

if __name__ == '__main__':
	asyncio.run(main())