########################################################
## you need python3 and pip3
## -> sudo apt install python3 python3-pip
## then pygame and pyserial
## -> sudo pip3 install pygame pyserial
########################################################

import pygame
import serial

from enum import Enum
class States(Enum):
    INIT = 1
    READY = 2
    GO = 3

state = States.INIT
pygame.init()
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

joy = pygame.joystick.Joystick(0)
joy.init()

# -------- Main Program Loop -----------
while not done:
    # --- Main event loop
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True

    # --- Game logic should go here
    axis_yaw   = joy.get_axis(0)
    left_trigger  = joy.get_axis(2)
    right_trigger = joy.get_axis(5)

    if state == States.INIT:
        if left_trigger > 0.9 and right_trigger > 0.9:
            state = States.READY
    elif state == States.READY:
        if left_trigger < -0.9 and right_trigger < -0.9:
            state = States.GO
    elif state == States.GO:
        print('GO')
        X_order = int(min((axis_yaw + 1) * 128, 255))
        Throttle = int(min(((-(left_trigger + 1) + (right_trigger + 1) + 2) * 64), 255))
        ser.write(bytes([12, 13, X_order, Throttle]))
        print(X_order, Throttle)

    # --- Limit to 60 frames per second
    clock.tick(10)

# Close the window and quit.
ser.close()
pygame.quit()

