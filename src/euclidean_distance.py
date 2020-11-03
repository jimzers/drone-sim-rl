import airsim
import numpy as np
import cv2
import os
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()

#TODO
# Test more
# Create test environment
# Get dummy goal position

class Drone:

    def __init__(self):
        self.time_per_movement = 5
        self.curr_angle = 0
        drone_pos = client.getMultirotorState().kinematics_estimated.position
        self.goal_pos = [10500, 110]
        self.curr_pos = [drone_pos.x_val, drone_pos.y_val]
        self.speed = 6
        self.green_bonus = 10

    def rotate_by_angle(self, angle):
        self.curr_angle += angle
        print("Current angle: ", self.curr_angle)
        client.rotateToYawAsync(self.curr_angle, 1).join()
        airsim.time.sleep(2)

    def detect_red(self, np_img):
        img_hsv = cv2.cvtColor(np_img, cv2.COLOR_BGR2HSV)

        # lower mask (0-10)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join the masks
        mask = mask0 + mask1

        # or your HSV image, which I *believe* is what you want
        output_binary = img_hsv.copy()
        output_binary[np.where(mask == 0)] = 0

        cv2.imwrite("before.png", np_img)
        cv2.imwrite("image.png", output_binary)

        num_reds = np.count_nonzero(output_binary)
        return num_reds

    def detect_green(self, np_img):
        img_hsv = cv2.cvtColor(np_img, cv2.COLOR_BGR2HSV)

        # mask (40-70)
        lower_red = np.array([40, 50, 50])
        upper_red = np.array([70, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join the masks
        mask = mask0

        # or your HSV image, which I *believe* is what you want
        output_binary = img_hsv.copy()
        output_binary[np.where(mask == 0)] = 0

        cv2.imwrite("greenimage.png", output_binary)

        num_reds = np.count_nonzero(output_binary)
        return num_reds

    def crop_center_x(self, img, cropx=256):
        y, x, c = img.shape
        print("image shape")
        print(img.shape)
        startx = x // 2 - (cropx // 2)
        return img[:, startx:startx + cropx, :]

    def get_np_image(self):
        responses = client.simGetImages([airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False)])
        response = responses[0]

        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(response.height, response.width, 3)

        # # original image is fliped vertically
        # img_rgb = np.flipud(img_rgb)

        #cut_img = self.crop_center_x(img_rgb, cropx=80)

        return img_rgb

    def move_forward(self):
        airsim.time.sleep(1)
        client.moveByVelocityAsync(math.cos(math.radians(self.curr_angle)), math.sin(math.radians(self.curr_angle)), 0,
                                   self.time_per_movement, self.speed).join()
        airsim.time.sleep(1)

    def move_right(self):
        self.rotate_by_angle(90)
        airsim.time.sleep(1)
        self.move_forward()

    def move_left(self):
        self.rotate_by_angle(-90)
        airsim.time.sleep(1)
        self.move_forward()

    def find_reds(self, upper_threshold=10000):
        reds_in_directions = []
        # look left first
        self.rotate_by_angle(-90)
        print('FINDING REDS - START')
        for i in range(3):
            curr_img = self.get_np_image()
            curr_img = self.crop_center_x(curr_img)
            cv2.imwrite("bruh"+str(i)+".png", curr_img)

            reds = self.detect_red(curr_img)
            greens = self.detect_green(curr_img)
            print("reds: ", reds, " | greens: ", greens)
            total_score = reds - self.green_bonus * greens
            if total_score < upper_threshold:
                reds_in_directions.append(total_score)
            else:
                reds_in_directions.append(-1)
            if i != 2:
                self.rotate_by_angle(90)

        print("END FIND REDS ===============================")
        self.rotate_by_angle(-90)
        return reds_in_directions

    # make an array of forward left and right, each time it passes the threshold
    # should only
    def choose_direction(self):
        reds_in_directions = self.find_reds()
        print(reds_in_directions)
        curr_pos = np.array([client.getMultirotorState().kinematics_estimated.position.x_val, client.getMultirotorState().kinematics_estimated.position.y_val])
        # This will be a dictionary with key = direction and value = euclidean distance based on future point
        euclidean_distances = {}
        idx = 0
        for direc in reds_in_directions:
            if direc != -1:
                temp_angle = (idx * 90 - 90) + self.curr_angle
                future_pos = curr_pos + self.time_per_movement * self.speed * np.array([math.cos(math.radians(temp_angle)),
                                                                    math.sin(math.radians(temp_angle))])
                euclidean_distances[temp_angle] = self.euclidean_distance(future_pos)
            idx += 1
        #print("Euclidean distances for each direction: ", euclidean_distances)
        if len(euclidean_distances.values()) == 0:
            return 180
        min_distance = min(euclidean_distances.values())
        min_key = [key for key in euclidean_distances if euclidean_distances[key] == min_distance]
        #print("Angle it will move to: ", min_key)
        return min_key[0]-self.curr_angle

    def euclidean_distance(self, future_pos):
        # print("shapes")
        # print(self.goal_pos)
        # print(future_pos)
        # #print(future_pos.shape)
        # print("EUCLIDENA DISTA")
        # print((np.sum((np.array(self.goal_pos) - np.array(future_pos))**2))**0.5)
        # print(np.linalg.norm(np.array(self.goal_pos) - np.array(future_pos)))
        return np.linalg.norm(np.array(self.goal_pos) - np.array(future_pos))
        # return (np.sum((np.array(self.goal_pos) - np.array(future_pos))**2))**0.5

    def move(self):
        self.rotate_by_angle(self.choose_direction())
        self.move_forward()

d = Drone();
while not False:
    d.move()


client.armDisarm(False)
client.reset()
client.enableApiControl(False)
