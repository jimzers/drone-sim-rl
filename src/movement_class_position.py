import airsim
import numpy as np
import cv2
from random import randint, choice
import json

client = airsim.MultirotorClient()
direction_mod_offset = 0

direction_arr = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]


class Drone:
    def __init__(self, logging=True):
        self.velocity = 5

        self.direction_arr = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)]
        self.input_file_name = ("./src/input_commands.txt")
        self.output_file_name = ("./src/output_commands.txt")

        drone_pos = client.getMultirotorState().kinematics_estimated.position
        self.curr_pos = [drone_pos.x_val, drone_pos.y_val, drone_pos.z_val]
        print(self.curr_pos)

        """ How the direction mod works
        Depends on the remainder after division by 4:
        0: Facing forward
        1: Facing right
        2: Facing backwards
        3: Facing left
        """
        self.direction_mod_offset = 0

        self.movements = {
            'forward': self.move_forward,
            'backward': self.move_backward,
            'turn_right': self.turn_right,
            'turn_left': self.turn_left,
            # 'right': self.move_right,
            # 'left': self.move_left,
            # 'up': self.move_up,
            # 'down': self.move_down
        }

        self.logging = logging
        self.log_arr = []

    def turn_right(self):
        self.direction_mod_offset += 1
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        client.rotateToYawAsync(direction_num * 90).join()

    def turn_left(self):
        self.direction_mod_offset -= 1
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        client.rotateToYawAsync(direction_num * 90).join()

    def move_left(self, distance):
        self.turn_left()
        self.move_forward(distance)
        # if self.logging:
        #     self.log_arr.append("left")

    def move_right(self, distance):
        self.turn_right()
        self.move_forward(distance)
        # if self.logging:
        #     self.log_arr.append("right")

    def move_up(self, distance):
        client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2] + distance,
                                   self.velocity).join()
        self.curr_pos = [self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2] + distance]

    def move_down(self, distance):
        client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1], self.curr_pos[2] - distance,
                                   self.velocity).join()
        self.curr_pos = [self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2] - distance]

    def move_forward(self, distance):
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        if direction_num == 0:
            client.moveToPositionAsync(self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2],
                                       self.velocity).join()
            self.curr_pos = [self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2]]
        elif direction_num == 1:
            client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1] + distance, self.curr_pos[2], self.velocity).join()
            self.curr_pos = [self.curr_pos[0], self.curr_pos[1] + distance, self.curr_pos[2]]
        elif direction_num == 2:
            client.moveToPositionAsync(self.curr_pos[0] - distance, self.curr_pos[1], self.curr_pos[2], self.velocity).join()
            self.curr_pos = [self.curr_pos[0] - distance, self.curr_pos[1], self.curr_pos[2]]
        else:  # direction_num == 3
            client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1] - distance, self.curr_pos[2], self.velocity).join()
            self.curr_pos = [self.curr_pos[0], self.curr_pos[1] - distance, self.curr_pos[2]]

    def move_backward(self, distance):
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        if direction_num == 0:
            client.moveToPositionAsync(self.curr_pos[0] - distance, self.curr_pos[1], self.curr_pos[2],
                                       self.velocity).join()
            self.curr_pos = [self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2]]
        elif direction_num == 1:
            client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1] - distance, self.curr_pos[2],
                                       self.velocity).join()
            self.curr_pos = [self.curr_pos[0], self.curr_pos[1] + distance, self.curr_pos[2]]
        elif direction_num == 2:
            client.moveToPositionAsync(self.curr_pos[0] + distance, self.curr_pos[1], self.curr_pos[2],
                                       self.velocity).join()
            self.curr_pos = [self.curr_pos[0] - distance, self.curr_pos[1], self.curr_pos[2]]
        else:  # direction_num == 3
            client.moveToPositionAsync(self.curr_pos[0], self.curr_pos[1] + distance, self.curr_pos[2],
                                       self.velocity).join()
            self.curr_pos = [self.curr_pos[0], self.curr_pos[1] - distance, self.curr_pos[2]]

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
        drone_pos = client.getMultirotorState().kinematics_estimated.position
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

    def move(self, output_file, input_file, no_input=False):

        input = open(input_file, "r")
        for command in input:
            cleaned_cmd = command.strip()
            if cleaned_cmd in self.movements:
                movement, move_str = self.pick_movement(cleaned_cmd)
                if move_str == 'forward' or move_str == 'backward':
                    movement(2)
                else:
                    movement()
                # for debug purposes
                self.log_arr.append(move_str)
                drone_pose = self.record_pose()
                self.get_np_image(True, "curr_image.png")
                airsim.time.sleep(1)
        input.close()
        self.write_log(output_file)
        print("finished episode")

    def clear_logging_arr(self):
        self.log_arr = []

    def run_multiple(self, num_episodes=5, base_output_name="logs/output_command"):
        for i in range(num_episodes):
            client.reset()
            client.confirmConnection()
            client.enableApiControl(True)
            client.armDisarm(True)
            airsim.time.sleep(1)
            client.takeoffAsync().join()
            airsim.time.sleep(1)
            drone_pos = client.getMultirotorState().kinematics_estimated.position
            self.curr_pos = [drone_pos.x_val, drone_pos.y_val, 10]
            print("AAAA")
            print(self.curr_pos)
            output_filename = base_output_name + "{:02d}".format(i) + ".txt"
            self.move(output_filename, self.input_file_name)
            self.clear_logging_arr()

d = Drone()
d.run_multiple()
# d.move()


# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
