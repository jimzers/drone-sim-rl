import airsim
import numpy as np
import cv2
from random import randint, choice
import json

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()

direction_mod_offset = 0

direction_arr = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]


class Drone:
    def __init__(self, logging=True):
        self.velocity = 5
        self.direction_mod_offset = 0
        self.direction_arr = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
        self.input_file_name = ("./src/input_commands.txt")
        self.output_file_name = ("./src/output_commands.txt")
        self.quad_offset_mapping = {
            'forward': direction_arr[self.direction_mod_offset % len(self.direction_arr)],
            'right': direction_arr[(self.direction_mod_offset + 1) % len(self.direction_arr)],
            'backward': direction_arr[(self.direction_mod_offset + 2) % len(self.direction_arr)],
            'left': direction_arr[(self.direction_mod_offset + 3) % len(self.direction_arr)],
        }

        self.movements = {
            'forward': self.move_forward,
            'backward': self.move_backward,
            'right': self.move_right,
            'left': self.move_left,
            # 'up': self.move_up,
            # 'down': self.move_down
        }

        self.logging = logging
        self.log_arr = []

    def calculate_offset_mapping(self):
        self.quad_offset_mapping = {
            'forward': direction_arr[self.direction_mod_offset % len(self.direction_arr)],
            'right': direction_arr[(self.direction_mod_offset + 1) % len(self.direction_arr)],
            'backward': direction_arr[(self.direction_mod_offset + 2) % len(self.direction_arr)],
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
        # if self.logging:
        #     self.log_arr.append("left")

    def move_right(self):
        self.turn_right()
        self.move_forward()
        # if self.logging:
        #     self.log_arr.append("right")

    def move_up(self):
        client.moveByVelocityAsync(0, 0, 1, 0.3).join()
        # if self.logging:
        #     self.log_arr.append("up")

    def move_down(self):
        client.moveByVelocityAsync(0, 0, -1, 0.3)
        # if self.logging:
        #     self.log_arr.append("down")

    def move_forward(self):
        quad_offset = self.quad_offset_mapping['forward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   self.velocity * quad_offset[2], 0.3).join()
        # if self.logging:
        #     self.log_arr.append("forward")

    def move_backward(self):
        quad_offset = self.quad_offset_mapping['backward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   self.velocity * quad_offset[2], 0.3).join()
        # if self.logging:
        #     self.log_arr.append("backward")

    def write_log(self, logfile='./src/movement_log.txt'):
        # TODO: parameterize logfile name
        print('Writing logs...')
        f = open(logfile, "w")
        for command in self.log_arr:
            f.write(command + "\n")
        print('Writing finished')

    def pick_movement(self, movement, rand_chance=0.1):
        rand_num = np.random.rand()
        move = self.movements[movement]
        move_str = movement
        if rand_num > 1.0 - rand_chance:
            # do random action now
            print("----MOVEMENT HAS CHANGED----")
            move_str = choice(list(self.movements.keys()))
            move = self.movements[move_str]
        # return the movement
        return move, move_str

    def record_pose(self):
        # print(client.getMultirotorState().kinematics_estimated)
        # print(client.getMultirotorState().kinematics_estimated.orientation)
        # print("THIS IS WHAT JSON MYDSFASDFSDA ==========================================================================================")
        # json_mylist = json.dumps(client.getMultirotorState().kinematics_estimated.orientation, separators=(',', ':'))
        # print(json_mylist)
        drone_pos = client.getMultirotorState().kinematics_estimated.orientation
        output = " | ".join(["x_val: " + str(drone_pos.x_val), "y_val: " + str(drone_pos.y_val), "z_val: " + str(drone_pos.z_val)])
        print(output)
        self.log_arr.append(output)
        return output



    def get_np_image(self, save_image=False, filename="curr_image.png"):
        responses = client.simGetImages([airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False)])
        response = responses[0]

        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(response.height, response.width, 3)

        # # original image is fliped vertically
        # img_rgb = np.flipud(img_rgb)

        if save_image:
            cv2.imwrite(filename, img_rgb)

        return img_rgb

    def move(self):

        input = open(self.input_file_name, "r")
        for command in input:
            cleaned_cmd = command.strip()
            if cleaned_cmd in self.movements:
                movement, move_str = self.pick_movement(cleaned_cmd)
                if move_str == 'forward':
                    for i in range(2):
                        movement()
                else:
                    movement()
                # for debug purposes
                self.log_arr.append(move_str)
                drone_pose = self.record_pose()
                self.get_np_image(True, "curr_image.png")
                airsim.time.sleep(1)
        input.close()
        self.write_log()
        print("finished")
d = Drone()
d.move()
# d.turn_right()
# d.move_forward()
# airsim.time.sleep(1)
# d.turn_left()
# d.move_forward()
# airsim.time.sleep(1)
# d.turn_left()
# d.move_forward()
# airsim.time.sleep(1)
# d.turn_left()
# d.move_forward()
# airsim.time.sleep(1)
# d.turn_left()
# d.move_forward()
# airsim.time.sleep(1)

# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
