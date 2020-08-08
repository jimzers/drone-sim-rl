import airsim
import numpy as np
import cv2
from random import randint

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()

direction_mod_offset = 0

direction_arr = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]


class Drone:
    def __init__(self, logging=False):
        self.velocity = 5
        self.direction_mod_offset = 0
        self.direction_arr = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
        self.quad_offset_mapping = {
            'forward': direction_arr[self.direction_mod_offset % len(self.direction_arr)],
            'backward': direction_arr[(self.direction_mod_offset + 1) % len(self.direction_arr)],
            'right': direction_arr[(self.direction_mod_offset + 2) % len(self.direction_arr)],
            'left': direction_arr[(self.direction_mod_offset + 3) % len(self.direction_arr)],
        }

        self.logging = logging
        self.log_arr = []

    def calculate_offset_mapping(self):
        self.quad_offset_mapping = {
            'forward': direction_arr[self.direction_mod_offset % len(self.direction_arr)],
            'backward': direction_arr[(self.direction_mod_offset + 1) % len(self.direction_arr)],
            'right': direction_arr[(self.direction_mod_offset + 2) % len(self.direction_arr)],
            'left': direction_arr[(self.direction_mod_offset + 3) % len(self.direction_arr)],
        }

    def turn_right(self):
        self.direction_mod_offset += 1
        self.calculate_offset_mapping()

    def turn_left(self):
        self.direction_mod_offset -= 1
        self.calculate_offset_mapping()

    def move_left(self):
        self.turn_left()
        self.move_forward()
        if self.logging:
            self.log_arr.append("left")

    def move_right(self):
        self.turn_right()
        self.move_forward()
        if self.logging:
            self.log_arr.append("right")

    def move_up(self):
        client.moveByVelocityAsync(0, 0, 1, 0.3).join()
        if self.logging:
            self.log_arr.append("up")

    def move_down(self):
        client.moveByVelocityAsync(0, 0, -1, 0.3)
        if self.logging:
            self.log_arr.append("down")

    def move_forward(self):
        quad_offset = self.quad_offset_mapping['forward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   self.velocity * quad_offset[2], 0.3).join()
        if self.logging:
            self.log_arr.append("forward")

    def move_backward(self):
        quad_offset = self.quad_offset_mapping['backward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   self.velocity * quad_offset[2], 0.3).join()
        if self.logging:
            self.log_arr.append("backward")

    def write_log(self):
        print('Writing logs...')
        with open('movement_log.txt') as f:
            res = '\n'.join(self.log_arr)
            f.write(res)
        print('Writing finished')


d = Drone()
while True:
    d.move_forward()
    value = randint(0, 3)
    if value == 0:
        for i in range(0, 3):
            d.turn_left()
    elif value == 1:
        for i in range(0, 3):
            d.turn_right()
    elif value == 2:
        for i in range(0, 3):
            d.move_down()
    else:
        for i in range(0, 3):
            d.move_up()
