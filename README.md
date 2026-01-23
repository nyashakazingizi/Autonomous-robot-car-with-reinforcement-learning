AUTONOMOUS ROBOT CAR WITH REINFORCEMENT LEARNING
==============================================

STATUS: Work in Progress (Final Year Project)


OVERVIEW
--------
This project focuses on the design and implementation of an autonomous
robot car capable of self-navigation, obstacle avoidance, and efficient
pathfinding using reinforcement learning.

The system is built on a Raspberry Pi–powered Yahboom 4WD smart car
equipped with ultrasonic and camera sensors, and integrates an IoT
telemetry dashboard for real-time monitoring of robot behavior and
learning performance.

The project serves as a small-scale, low-cost experimental platform for
studying autonomous navigation, reinforcement learning, and intelligent
robotic systems.


KEY FEATURES
------------
- Autonomous navigation in controlled or dynamic environments
- Reinforcement learning–based decision making (Q-Learning, with
  possible extension to Deep Q-Networks)
- Real-time sensor feedback using ultrasonic and camera sensors
- IoT dashboard for live telemetry visualization
- Obstacle detection and avoidance
- Continuous learning through trial-and-error interaction


HARDWARE PLATFORM
-----------------
- Yahboom 4WD Smart Car
- Raspberry Pi
- Ultrasonic distance sensors
- Camera module (2-DOF movement)
- DC motors and motor driver
- Onboard power supply

The Raspberry Pi acts as the central controller, handling sensor input,
learning logic, decision-making, motor control, and telemetry
communication.


SYSTEM ARCHITECTURE
-------------------
The robot follows a classical Perception -> Decision -> Control
architecture.

1. PERCEPTION
   - Ultrasonic sensors measure distances to nearby obstacles
   - Camera sensor provides visual feedback for lane or obstacle detection
   - Sensor data is continuously collected and pre-processed

2. DECISION-MAKING
   - Reinforcement learning agent evaluates possible actions
   - Actions include moving forward, turning left, right, or stopping
   - Rewards and penalties guide the learning process

3. CONTROL
   - Selected actions are executed through motor commands
   - Continuous feedback ensures smooth and accurate movement

PROJECT EVOLUTION
-----------------
This project was developed incrementally through multiple
implementations to study autonomous navigation techniques:

1. Rule-Based Obstacle Avoidance
   - Hard-coded logic using sensor thresholds
   - Fast but inflexible behavior

2. Reinforcement Learning-Based Navigation
   - Q-Learning agent learns actions through rewards and penalties
   - Improved adaptability to dynamic environments

3. Maze Navigation Environment
   - More complex state space and decision-making
   - Focus on pathfinding and long-term planning


REINFORCEMENT LEARNING APPROACH
-------------------------------
The robot learns navigation behavior using Reinforcement Learning (RL),
primarily based on Q-Learning.

- States:
  Derived from sensor readings (e.g., distance to obstacles)

- Actions:
  Movement commands (forward, left, right, stop)

- Rewards:
  * Positive rewards for safe and efficient movement
  * Penalties for collisions or unsafe proximity to obstacles

The learning process follows the Bellman Optimality Equation, enabling
the robot to improve its policy over time through experience.

Future extensions may include Deep Q-Networks (DQN) to handle more
complex state spaces.


IOT TELEMETRY DASHBOARD
----------------------
A key component of the project is the IoT-based telemetry dashboard,
designed to:

- Display live sensor readings (distance, navigation status)
- Monitor robot actions and learning behavior in real time
- Assist with debugging, performance analysis, and optimization
- Provide visibility into training progress

This bridges robotics with web and IoT technologies, making system
behavior observable and easier to analyze.


PROJECT SCOPE AND LIMITATIONS
-----------------------------
- Focused on small-scale autonomous navigation
- Operates in controlled environments
- Reinforcement learning targets pathfinding and obstacle avoidance
- Does not include full SLAM, high-speed navigation, or cloud-based
  training


TECHNOLOGIES USED
-----------------
Programming:
- Python
- C++

AI / Machine Learning:
- Reinforcement Learning
- Q-Learning

Robotics:
- Sensor integration
- Motor control

Web and IoT:
- HTML
- CSS
- JavaScript
- IoT telemetry dashboards

Hardware:
- Raspberry Pi
- Ultrasonic sensors
- Camera module


PROJECT STATUS
--------------
Work in Progress

This project is currently under active development as part of a Final
Year BSc (Hons) Software Engineering project.

Ongoing work includes:
- Refining reward functions
- Improving navigation stability
- Expanding telemetry visualization
- Exploring advanced reinforcement learning techniques


AUTHOR
------
Nyasha Kazingizi
BSc (Hons) Software Engineering
University of Mauritius

Aspiring Software Developer with interests in Robotics, Automation,
and AI-driven systems
