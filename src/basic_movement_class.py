import airsim
import numpy as np
import cv2
from random import randint, choice
import json

client = airsim.MultirotorClient()
direction_mod_offset = 0

direction_arr = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]


class Drone:
    """ Contains basic movement control for an Airsim drone """
    def __init__(self, logging=True):
        """
        Initializes the drone object with a specific velocity and the
        quad offset map to keep track of where the drone is facing
        """
        self.velocity = 4
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
        """
        Recalculates the quad offset map to keep
        track of the direction the drone is facing
        """
        self.quad_offset_mapping = {
            'forward': direction_arr[self.direction_mod_offset % len(self.direction_arr)],
            'right': direction_arr[(self.direction_mod_offset + 1) % len(self.direction_arr)],
            'backward': direction_arr[(self.direction_mod_offset + 2) % len(self.direction_arr)],
            'left': direction_arr[(self.direction_mod_offset + 3) % len(self.direction_arr)],
        }

    def turn_right(self):
        """ Turns the drone right  """
        self.direction_mod_offset += 1
        self.calculate_offset_mapping()
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        client.rotateToYawAsync(direction_num * 90).join()

    def turn_left(self):
        """ Turns the drone left  """
        self.direction_mod_offset -= 1
        self.calculate_offset_mapping()
        direction_num = self.direction_mod_offset % len(self.direction_arr)
        client.rotateToYawAsync(direction_num * 90).join()

    def move_left(self,distance):
        """
        Turns the drone left and then moves forward
        :param distance: The distance the drone will move left
        """
        self.turn_left()
        self.move_forward(distance)
        # self.log_arr.append("left")

    def move_right(self,distance):
        """
        Turns the drone right and then moves forward
        :param distance: The distance the drone will move right
        """
        self.turn_right()
        self.move_forward(distance)
        # self.log_arr.append("right")

    def move_up(self,distance):
        """
        Moves the drone up for a certain distance
        :param distance: the distance the drone moves up
        """
        client.moveByVelocityAsync(0, 0, 1, 0.3).join()
        # if self.logging:
        #     self.log_arr.append("up")

    def move_down(self):
        """
        Moves the drone down for a certain distance
        """
        client.moveByVelocityAsync(0, 0, -1, 0.3).join()
        # if self.logging:
        #     self.log_arr.append("down")

    def move_forward(self, distance):
        """
        Moves the drone forward for a certain distance
        :param distance: The distance the drone will move forward
        """
        quad_offset = self.quad_offset_mapping['forward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   0.15, distance/self.velocity).join()
        # if self.logging:
        #     self.log_arr.append("forward")

    def move_backward(self, distance):
        """
        Moves the drone backward for a certain distance
        :param distance: The distance the drone will move backward
        """
        quad_offset = self.quad_offset_mapping['backward']
        client.moveByVelocityAsync(self.velocity * quad_offset[0], self.velocity * quad_offset[1],
                                   0.15, distance/self.velocity).join()
        # if self.logging:
        #     self.log_arr.append("backward")

    def write_log(self, logfile='./src/movement_log.txt'):
        """
        Writes a log describing all the movements the drone performed
        :param logfile: The file the log will be written to
        """
        # TODO: parameterize logfile name
        print('Writing logs...')
        f = open(logfile, "w")
        for command in self.log_arr:
            f.write(command + "\n")
        print('Writing finished')

    def pick_movement(self, movement, rand_chance=0.1):
        """
        Has a probability to change the initial movement of the drone
        by 10%, otherwise continues to move in the same direction
        :param movement: the initial movement of the drone
        :param rand_chance: The chance the drone will change its movement
        :return: the new movement and the movement represented by a string
        """
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
        """
        Records the pose of the drone and adds the pose to the logging array
        :return: the pose in a formatted string
        """
        drone_pos = client.getMultirotorState().kinematics_estimated.position
        output = " | ".join(["x_val: " + str(drone_pos.x_val), "y_val: " + str(drone_pos.y_val), "z_val: " + str(drone_pos.z_val)])
        print(output)
        self.log_arr.append(output)
        return output



    def get_np_image(self, save_image=False, filename="curr_image.png"):
        """
        takes an image from the drone's camera in Unreal Engine.
        :param save_image: Whether or not to save the image
        :param filename: The file the image will be saved in
        :return: the numpy array representing the image
        """
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
        """
        Moves the drone using a file with movement commands. Adds in an additional
        10% noise that can potentially change the drone's movement.
        :param output_file: the file the logging file will write too
        :param input_file: the file with all the movements the drone will perform
        :param no_input: whether or not the drone should read from an input file
        """
        input = open(input_file, "r")
        for command in input:
            cleaned_cmd = command.strip()
            if cleaned_cmd in self.movements:
                movement, move_str = self.pick_movement(cleaned_cmd)
                if move_str == 'forward' or move_str == 'backward':
                    movement(4)
                else:
                    movement(2)
                # for debug purposes
                self.log_arr.append(move_str)
                drone_pose = self.record_pose()
                self.get_np_image(True, "curr_image.png")
                airsim.time.sleep(1)
        input.close()
        self.write_log(output_file)
        print("finished episode")

    def clear_logging_arr(self):
        """ Clears the logging array """
        self.log_arr = []

    def run_multiple(self, num_episodes=5, base_output_name="logs/output_command"):
        """
        Runs the movement method for multiple episodes
        :param num_episodes: the number of episodes to be performed
        :param base_output_name: the base file name for all the movements of the drone
        """
        for i in range(num_episodes):
            client.reset()
            client.confirmConnection()
            client.enableApiControl(True)
            client.armDisarm(True)
            airsim.time.sleep(1)
            client.takeoffAsync().join()
            output_filename = base_output_name + "{:02d}".format(i) + ".txt"
            self.move(output_filename, self.input_file_name)
            self.clear_logging_arr()

#test code for the class
d = Drone()
d.run_multiple(num_episodes = 1)


# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
