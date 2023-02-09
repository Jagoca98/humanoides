"""teo_plantilla_py controller."""

import controller
import math
import time

l0 = 0.329
l1 = 0.215+0.090
A = l0+l1

def fwdKin(q0, q1):
    # __1__
    print("fwdKin: input (q1 q2):", q0, q1)
    u = ...
    v = ...
    print("fwdKin: intermediate (u v):", u, v)
    x = ...
    z = ...
    print("fwdKin: output (x z):", x, z)
    return [x, z]

def invKin(x, z):
    # __2__
    print("invKin: input (x z):", x, z)
    u = ...
    v = ...
    print("invKin: intermediate (u v):", u, v)
    q1 = ...
    q0 = ...
    print("invKin: output (q0 q1):", math.degrees(q0), math.degrees(q1))
    return [q0, q1]

# create the Robot instance.

robot = controller.Robot()

motor_q0 = robot.getDevice("r_shoulder_pitch")
motor_q1 = robot.getDevice("r_elbow_pitch")

# target 0

## target 0: setPosition

# __3__
target0_q0 = ... # use helper: math.radians()
target0_q1 = ... # use helper: math.radians()
motor_q0.setPosition(target0_q0)
motor_q1.setPosition(target0_q1)
robot.step(1000)
time.sleep(1)

## target 0: perform fwdKin

[target0_x, target0_z] = fwdKin(target0_q0, target0_q1)

## target 0: check if invKin works to recover original joint space targets

[recovered0_q0, recovered0_q1] = invKin(target0_x, target0_z)

# target 1

## target 1: generate in Cartesian space, adding offset from target 0

# __4a__
target1_x = ...
target1_z = ...

## target 1: compute in joint space, via invKin

[target1_q0, target1_q1] = invKin(target1_x, target1_z)

# target 2

## target 2: generate in Cartesian space

# __4b__
target2_x = ...
target2_z = ...

## target 2: compute in joint space, via invKin

[target2_q0, target2_q1] = invKin(target2_x, target2_z)

# target 3

## target 3: generate in Cartesian space, removing offset from target 0

# __4c__
target3_x = ...
target3_z = ...

## target 3: compute in joint space, via invKin

[target3_q0, target3_q1] = invKin(target3_x, target3_z)

# Main loop:
# - toggle between target 1, target 2, target 3 and back again
# - perform simulation steps until Webots is stopping the controller
while True:
    # __5__
    motor_q0.setPosition(target1_q0)
    motor_q1.setPosition(target1_q1)
    if -1 == robot.step(1000):
        break
    time.sleep(1)
    motor_q0.setPosition(...)
    motor_q1.setPosition(...)
    if -1 == robot.step(1000):
        break
    time.sleep(1)
    motor_q0.setPosition(...)
    motor_q1.setPosition(...)
    if -1 == robot.step(1000):
        break
    time.sleep(1)
    motor_q0.setPosition(target2_q0)
    motor_q1.setPosition(target2_q1)
    if -1 == robot.step(1000):
        break
    time.sleep(1)
