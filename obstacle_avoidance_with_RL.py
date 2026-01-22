import RPi.GPIO as GPIO
import time
import random
import numpy as np

# ================= RL PARAMETERS =================
NUM_STATES = 4
NUM_ACTIONS = 5

Q = np.zeros((NUM_STATES, NUM_ACTIONS))

alpha = 0.1      # learning rate
gamma = 0.9      # discount factor
epsilon = 0.8    # exploration rate
EPSILON_MIN = 0.05
EPSILON_DECAY = 0.995

ACTION_DURATION = 0.25  # seconds

# ================= HARDWARE SETUP =================
time.sleep(15)

EchoPin = 0
TrigPin = 1

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

L1 = 20
L2 = 21
R1 = 19
R2 = 26
ENA = 16
ENB = 13

GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(EchoPin, GPIO.IN)
GPIO.setup(TrigPin, GPIO.OUT)

pwm_ENA = GPIO.PWM(ENA, 2000)
pwm_ENB = GPIO.PWM(ENB, 2000)
pwm_ENA.start(0)
pwm_ENB.start(0)

pins = [L1, L2, R1, R2]
for p in pins:
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

# ================= MOTOR FUNCTIONS =================
def stop():
    GPIO.output(L1, 0)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 0)

def run(ls, rs):
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(ls)
    pwm_ENB.ChangeDutyCycle(rs)

def back(ls, rs):
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(ls)
    pwm_ENB.ChangeDutyCycle(rs)

def turn_left(ls, rs):
    GPIO.output(L1, 0)
    GPIO.output(L2, 1)
    GPIO.output(R1, 1)
    GPIO.output(R2, 0)
    pwm_ENA.ChangeDutyCycle(ls)
    pwm_ENB.ChangeDutyCycle(rs)

def turn_right(ls, rs):
    GPIO.output(L1, 1)
    GPIO.output(L2, 0)
    GPIO.output(R1, 0)
    GPIO.output(R2, 1)
    pwm_ENA.ChangeDutyCycle(ls)
    pwm_ENB.ChangeDutyCycle(rs)

# ================= ULTRASONIC =================
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

    return ((t2 - t1) * 340 / 2) * 100

def Distance_test():
    values = []
    for _ in range(5):
        d = Distance()
        while d <= 0 or d > 500:
            d = Distance()
        values.append(d)
        time.sleep(0.01)
    return sum(values[1:4]) / 3

# ================= RL FUNCTIONS =================
def get_state(distance):
    if distance < 20:
        return 0   # VERY CLOSE
    elif distance < 40:
        return 1   # CLOSE
    elif distance < 60:
        return 2   # SAFE
    else:
        return 3   # FAR

def get_reward(distance, collision):
    if collision:
        return -20
    if distance >= 60:
        return 2
    elif distance >= 40:
        return 1
    elif distance >= 20:
        return -1
    else:
        return -5

def choose_action(state):
    if random.random() < epsilon:
        return random.randint(0, NUM_ACTIONS - 1)
    return np.argmax(Q[state])

def perform_action(action):
    if action == 0:
        run(40, 40)
    elif action == 1:
        turn_left(30, 30)
    elif action == 2:
        turn_right(30, 30)
    elif action == 3:
        back(40, 40)
    elif action == 4:
        stop()

    time.sleep(ACTION_DURATION)
    stop()

# ================= MAIN LOOP =================
step_count = 0

while True:
    distance = Distance_test()

    # ---- HARD SAFETY LAYER (NOT RL) ----
    if distance < 8:
        stop()
        back(40, 40)
        time.sleep(0.3)
        stop()
        continue

    state = get_state(distance)
    action = choose_action(state)
    perform_action(action)

    new_distance = Distance_test()
    new_state = get_state(new_distance)

    collision = new_distance < 10
    reward = get_reward(new_distance, collision)

    Q[state, action] += alpha * (
        reward + gamma * np.max(Q[new_state]) - Q[state, action]
    )

    epsilon = max(EPSILON_MIN, epsilon * EPSILON_DECAY)

    step_count += 1
    if step_count % 20 == 0:
        print(f"State:{state} Action:{action} Reward:{reward} Epsilon:{epsilon:.2f}")
