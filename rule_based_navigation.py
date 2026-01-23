"""
Rule-Based Obstacle Avoidance
-----------------------------
This script implements a basic obstacle avoidance system using
hard-coded rules based on ultrasonic sensor distance thresholds.

Purpose:
- Establish baseline autonomous navigation behavior
- Serve as a comparison against learning-based approaches

This version does not use machine learning.
"""

import RPi.GPIO as GPIO
import time


time.sleep(15)
# Ultrasonic pin definitions
EchoPin = 0
TrigPin = 1

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor pin definitions
L1 = 20
L2 = 21
R1 = 19
R2 = 26
ENA = 16
ENB = 13

global pwm_ENA
global pwm_ENB

GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(EchoPin, GPIO.IN)
GPIO.setup(TrigPin, GPIO.OUT)

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

# Motor pins
pins = [L1, L2, R1, R2]
for p in pins:
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

# ---- MOTOR FUNCTIONS ----
def stop():
    GPIO.output(L1, 0)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 0)

def run(leftspeed, rightspeed):
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def back(leftspeed, rightspeed):
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

def turn_left(leftspeed, rightspeed):
    # Left wheels backward, right wheels forward
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    time.sleep(0.4)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

    stop()
    time.sleep(0.001)

def turn_right(leftspeed, rightspeed):
    # Left wheels forward, right wheels backward
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    time.sleep(0.4)

    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

    stop()
    time.sleep(0.001)

def spin_left(leftspeed, rightspeed):
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
    stop()
    time.sleep(0.001)

def spin_right(leftspeed, rightspeed):
    # Turn right in place
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)
    stop()
    time.sleep(0.001)

# ---- ULTRASONIC ----
def Distance():
    GPIO.output(TrigPin, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)

    t0 = time.time()
    while not GPIO.input(EchoPin):
        if time.time() - t0 > 0.03:
            return -1
    t1 = time.time()

    while GPIO.input(EchoPin):
        if time.time() - t1 > 0.03:
            return -1
    t2 = time.time()

    return ((t2 - t1) * 340 / 2) * 100  # cm

def Distance_test():
    valid_Ultrasonics = []
    for _ in range(5):
        d = Distance()
        while d == -1 or d >= 500 or d == 0:
            d = Distance()
        valid_Ultrasonics.append(d)
        time.sleep(0.01)
    return sum(valid_Ultrasonics[1:4]) / 3

# ---- MAIN LOOP ----
while True:
    distance = Distance_test()

    if distance >= 60:
        run(40, 40)

    elif 40 <= distance < 60:
        run(25, 25)

    else:  # distance < 40
        stop()
        time.sleep(0.05)

        # Try turning right first
        turn_right(25, 25)
        distance = Distance_test()

        # Emergency back only if really close
        if distance < 20:
            stop()
            time.sleep(0.1)
            back_start = time.time()
            while time.time() - back_start < 2:
                back(40, 40)
                distance = Distance_test()
                if distance >= 40:
                    break
                time.sleep(0.05)
            stop()
            time.sleep(0.1)

        # Otherwise try turning left if still blocked
        elif distance < 40:
            turn_left(25, 25)
            distance = Distance_test()
            if distance >= 40:
                run(40, 40)
            else:
                stop()
                time.sleep(0.1)
                back_start = time.time()
                while time.time() - back_start < 2:
                    back(40, 40)
                    distance = Distance_test()
                    if distance >= 40:
                        break
                    time.sleep(0.05)
                stop()
                time.sleep(0.1)

        # Optional small corrective turn
        if distance < 40:
            turn_left(40, 40)
        else:
            turn_right(40, 40)
        time.sleep(0.3)
        stop()



