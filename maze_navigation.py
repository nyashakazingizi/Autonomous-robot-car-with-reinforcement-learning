import RPi.GPIO as GPIO
import time

time.sleep(15)

# ---------------- CONSTANTS ----------------
SAFE_DISTANCE = 28
CRITICAL_DISTANCE = 10

COMMIT_TURN_SPEED = 18
COMMIT_TIME_SCALE = 1.0
LOCK_IN_TIME = 0.15
COMMIT_FORWARD_TIME = 0.35   # FIXED (was too aggressive)

FWD_FAST = 22
FWD_SLOW = 12
TURN_SPEED = 25

# --- TURN SCANNING PARAMETERS ---
STEP_TURN_TIME = 0.07        # FIXED (stronger rotation)
MAX_TURN_STEPS = 13
MIN_ACCEPT_STEPS = 9         # FIXED (~90° minimum)
SENSOR_SETTLE = 0.05

# --- CALIBRATION ---
TURN_TIME_LEFT_BOOST = 0.02
LEFT_BOOST = 3
LEFT_FWD_BOOST = 3

# ---------------- GPIO SETUP ----------------
EchoPin = 0
TrigPin = 1

L1 = 20
L2 = 21
R1 = 19
R2 = 26
ENA = 16
ENB = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([L1, L2, R1, R2, ENA, ENB, TrigPin], GPIO.OUT)
GPIO.setup(EchoPin, GPIO.IN)

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

# ---------------- MOTOR FUNCTIONS ----------------
def stop():
    GPIO.output([L1, L2, R1, R2], 0)
    pwm_ENA.ChangeDutyCycle(0)
    pwm_ENB.ChangeDutyCycle(0)

def run(ls, rs):
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(ls + LEFT_FWD_BOOST)
    pwm_ENB.ChangeDutyCycle(rs)

def spin_right_step(speed):
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(speed)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(STEP_TURN_TIME)
    stop()

def spin_left_step(speed):
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(speed + LEFT_BOOST)
    pwm_ENB.ChangeDutyCycle(speed)
    time.sleep(STEP_TURN_TIME + TURN_TIME_LEFT_BOOST)
    stop()

def spin_right_commit():
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(COMMIT_TURN_SPEED)
    pwm_ENB.ChangeDutyCycle(COMMIT_TURN_SPEED)
    time.sleep(STEP_TURN_TIME * COMMIT_TIME_SCALE)
    stop()

def spin_left_commit():
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(COMMIT_TURN_SPEED + LEFT_BOOST)
    pwm_ENB.ChangeDutyCycle(COMMIT_TURN_SPEED)
    time.sleep((STEP_TURN_TIME + TURN_TIME_LEFT_BOOST) * COMMIT_TIME_SCALE)
    stop()

# ---------------- ULTRASONIC ----------------
def Distance():
    GPIO.output(TrigPin, 0)
    time.sleep(0.000002)
    GPIO.output(TrigPin, 1)
    time.sleep(0.000015)
    GPIO.output(TrigPin, 0)

    start = time.time()
    while not GPIO.input(EchoPin):
        if time.time() - start > 0.03:
            return -1

    start = time.time()
    while GPIO.input(EchoPin):
        if time.time() - start > 0.03:
            return -1

    return ((time.time() - start) * 340 / 2) * 100

def Distance_test():
    vals = []
    for _ in range(3):
        d = Distance()
        if 0 < d < 500:
            vals.append(d)
        time.sleep(0.01)
    return sum(vals) / len(vals) if vals else 999

# ---------------- SAFE TURN SCAN ----------------
def explore_direction(turn_step_func):
    steps = 0

    for _ in range(MAX_TURN_STEPS):
        turn_step_func(TURN_SPEED)
        steps += 1

        time.sleep(SENSOR_SETTLE)
        d = Distance_test()

        if d < CRITICAL_DISTANCE:
            return False, steps, True

        if steps >= MIN_ACCEPT_STEPS and d >= SAFE_DISTANCE:
            return True, steps, False

    return False, steps, False

# ---------------- POST-TURN VERIFICATION ----------------
def verify_forward():
    time.sleep(0.1)
    return Distance_test() >= SAFE_DISTANCE

# ---------------- MAIN LOOP ----------------
try:
    print("Maze navigation started (TURN-CORRECTED MODE)")

    while True:
        front = Distance_test()

        # -------- CLEAR PATH --------
        if front > SAFE_DISTANCE:
            run(FWD_FAST + 1, FWD_FAST)
            continue

        # -------- MOVE INTO CORNER --------
        run(FWD_SLOW, FWD_SLOW)
        time.sleep(0.25)
        stop()
        time.sleep(0.1)

        path_found = False
        chosen_steps = 0
        chosen_dir = None

        # ===== SCAN RIGHT =====
        found, steps, _ = explore_direction(spin_right_step)
        for _ in range(steps):
            spin_left_step(TURN_SPEED)

        if found:
            path_found = True
            chosen_steps = steps
            chosen_dir = "right"

        # ===== SCAN LEFT =====
        if not path_found:
            found, steps, _ = explore_direction(spin_left_step)
            for _ in range(steps):
                spin_right_step(TURN_SPEED)

            if found:
                path_found = True
                chosen_steps = steps
                chosen_dir = "left"

        # ===== COMMIT OR STOP =====
        if path_found:
            if chosen_dir == "right":
                for _ in range(chosen_steps):
                    spin_right_commit()
            else:
                for _ in range(chosen_steps):
                    spin_left_commit()

            run(FWD_SLOW, FWD_SLOW)
            time.sleep(COMMIT_FORWARD_TIME)

            if not verify_forward():
                stop()
                time.sleep(0.2)

        else:
            stop()
            print("Dead end detected — stopping safely")
            break

except KeyboardInterrupt:
    stop()
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.cleanup()
    print("Stopped safely")


