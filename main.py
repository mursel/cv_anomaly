import time, utime, pyb, sensor, image, os, tf
from pyb import Pin
from array import array

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((295,0,70,420))
sensor.skip_frames(time = 2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 2000)
current_exposure_time_in_microseconds = sensor.get_exposure_us()
sensor.set_auto_exposure(False, \
    exposure_us = int(current_exposure_time_in_microseconds * 0.6))

labels=None
net=None

try:
    labels, net = tf.load_builtin_model('trained')
except Exception as e:
    raise Exception(e)

pinDir0 = pyb.Pin('P3', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
pinDir1 = pyb.Pin('P2', pyb.Pin.OUT_PP, pyb.Pin.PULL_NONE)
pinDir0.value(1)
pinDir1.value(0)

tim = pyb.Timer(4, freq=50)
motor_ch = tim.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P7"), pulse_width_percent=50)
servo_ch = tim.channel(3, pyb.Timer.PWM, pin=pyb.Pin("P9"), pulse_width_percent=100)

trig_pin = Pin('P5', Pin.OUT_PP, Pin.PULL_DOWN)
echo_pin = Pin('P4', Pin.IN, Pin.PULL_NONE)

start_time = 0
end_time = 0
distance = 0
initServo = 0
top = 0
scoreBad = 0
scoreGood = 0
scoreNeutral = 0
scoreBroken=0
finalScore = 0
c=0
f=0

print(os.listdir())

def startScan():
    trig_pin.value(0)
    pyb.udelay(5)
    trig_pin.value(1)
    pyb.udelay(10)
    trig_pin.value(0)

def getDistance():
    global end_time, distance, start_time
    while(echo_pin.value()==0):
        start_time = pyb.micros()
    while(echo_pin.value() == 1):
        end_time = pyb.elapsed_micros(start_time)
    distance = end_time * 0.5 * 0.0343
    return distance

def servoSignalBad():
    time.sleep_ms(1300)
    servo_ch.pulse_width(1500)
    moveMotorForward()
    motorStart()
    pyb.delay(1280)
    servo_ch.pulse_width(3000)
def servoSignalGood():
    time.sleep_ms(1200)
    servo_ch.pulse_width(3000)
    moveMotorForward()
    motorStart()
    pyb.delay(1250)
    servo_ch.pulse_width(1500)

def motorStart():
    motor_ch.pulse_width_percent(30)
def motorStop():
    motor_ch.pulse_width_percent(0)

def initServoPos():
    global initServo
    if(initServo == 0):
        servo_ch.pulse_width(1300)
        initServo = 1

def moveMotorBack():
    pinDir0.value(0)
    pinDir1.value(1)
    motorStart()

def moveMotorForward():
    pinDir0.value(1)
    pinDir1.value(0)
    motorStart()

def doShowImage():
    time.sleep_ms(1000)
    img = sensor.snapshot()
    time.sleep_ms(1000)
    img = sensor.snapshot()

def do_classification(imgObj):
    global top, scoreBad, scoreBroken, scoreGood, scoreNeutral, c, f
    scoreBad = 0
    scoreGood = 0
    scoreNeutral = 0
    scoreBroken = 0
    top = 0
    c = 0
    while(top <= imgObj.height()-70):
        croppedImage = imgObj.copy(roi=(0, top, 70, 70))
        #croppedImage.save("img"+str(top)+"-"+str(time.ticks_us())+".jpg")
        obj = net.classify(croppedImage, min_scale=1.0, scale_mul=0.8, x_overlap=0.5, y_overlap=0.5)

        scoreBad += obj[0].output()[0]
        scoreBroken += obj[0].output()[1]
        scoreGood += obj[0].output()[2]
        scoreNeutral += obj[0].output()[3]

        tempScores = [scoreBad,scoreBroken,scoreGood,scoreNeutral]

        # neutral?
        if(tempScores.index(max(tempScores)) == 3):
            #croppedImage.save("neutral"+str(top)+str(c)+".jpg")
            break

        # broken?
        if(tempScores.index(max(tempScores)) == 1):
            #croppedImage.save("broken"+str(top)+str(c)+".jpg")
            break

        # good?
        #if(tempScores.index(max(tempScores)) == 2):
            #croppedImage.save("good"+str(top)+str(c)+".jpg")

        # bad?
        #if(tempScores.index(max(tempScores)) == 0):
            #croppedImage.save("bad"+str(top)+str(c)+".jpg")

        print(str(top))
        top+=70
        c+=1

    scores = [scoreBad,scoreBroken,scoreGood,scoreNeutral]
    print(' bad: '+str(scoreBad)+'\n broken: '+str(scoreBroken)+'\n good: '+str(scoreGood)+'\n neutral: '+str(scoreNeutral))
    max_value = max(scores)
    max_index = scores.index(max_value)
    label = labels[max_index]
    print('result: '+label)

    return label

while(1):
    if(initServo == 0):
        img = sensor.snapshot()
    initServoPos()
    startScan()
    d = getDistance()
    r = -1
    if(d > 4 and d < 6):
        time.sleep_ms(620)
        motorStop()
        doShowImage()

        r = do_classification(img)

        while(r == "neutral"):
            if(c > 3):
                moveMotorForward()
                time.sleep_ms(95)
                motorStop()
                r = do_classification(img)
                doShowImage()
            else:
                moveMotorBack()
                time.sleep_ms(90)
                motorStop()
                r = do_classification(img)
                doShowImage()

        if(r == "bad" or r == "broken"):
            servoSignalBad()
        if(r == "good"):
            servoSignalGood()

        moveMotorForward()

    else:
        motorStart()
    time.sleep_ms(100)
