from copy import deepcopy

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


def get_np_image():
    responses = client.simGetImages([airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # # original image is fliped vertically
    # img_rgb = np.flipud(img_rgb)

    return img_rgb


def detect_red(np_img):
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
    # cv2.imwrite("image.png", output_binary)

    num_reds = np.count_nonzero(output_binary)
    print(num_reds)
    return num_reds


def crop_center_x(img, cropx=80):
    y, x, c = img.shape
    startx = x // 2 - (cropx // 2)
    return img[:, startx:startx + cropx, :]


def split_image(img, num_split):
    img_list = []
    print("I am a vegetarian")
    # (144, 256, 3) is the size of each image
    img_shape = img.shape
    for i in range(num_split):
        seg_img = img[:, int(i * (img_shape[1] / num_split)):int((i + 1) * (img_shape[1] / num_split)), :]
        img_list.append(seg_img)
    return img_list


def find_least_red(img_list):
    red_values = []
    for img in img_list:
        red_values.append(detect_red(img))
    print("Red values:")
    print(red_values)
    min_red = min(red_values)
    index = red_values.index(min_red)
    print("Index of least red segmented image: ")
    print(index)
    return img_list[index], index
    #for i in range(len(img_list)):
        # print("segmented image: ")
        # print(img_list[i])
        # print(type(img_list[i]))
        #if detect_red(img_list[i]) < min:
            #min = detect_red(img_list[i])
            #index = i
    #return img_list[index], index


client.rotateToYawAsync(0).join()
curr_angle = 0

airsim.time.sleep(2)
num_splits = 3
for i in range(15):
    img_2 = get_np_image()
    img_list = split_image(img_2, num_splits)
    img, index = find_least_red(img_list)
    angle = index * (180 / num_splits) + (90 / num_splits) - 90
    print("Angle drone will rotate: ", angle)
    curr_angle += angle
    #client.rotateByYawRateAsync(angle, 1).join()
    client.rotateToYawAsync(curr_angle, 1).join()
    airsim.time.sleep(2)
    print("Current angle: ", curr_angle)
    client.moveByVelocityAsync(math.cos(math.radians(curr_angle)), math.sin(math.radians(curr_angle)), 0, 3).join()
    # client.moveByVelocityAsync(math.sin(270 - curr_angle), math.cos(270 - curr_angle), 0, 1).join()
    # client.moveByVelocityAsync(-math.cos(curr_angle+90), math.sin(curr_angle), 0, 1).join()
    cv2.imwrite("unseg.png", img_2)
    cv2.imwrite("seg.png", img)
# exit protocol
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
