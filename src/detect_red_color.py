import airsim
import numpy as np
import cv2
import os

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()


def get_np_image():
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # original image is fliped vertically
    img_rgb = np.flipud(img_rgb)

    return img_rgb


def process_image_v2(np_img):
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

    # technically don't have to do this if counting count_nonzero. in fact it adds more runtime.
    # remove when not debugging.
    output_binary[np.where(mask != 0)] = 1

    cv2.imwrite("before.png", np_img)
    cv2.imwrite("image.png", output_binary)

    num_reds = np.count_nonzero(output_binary)
    print(num_reds)

    if num_reds > 700:
        print("Red object detected!")
        return 1
    else:
        print("No red object in sight.")
        return 0


img = get_np_image()
# print(img)
# print(type(img))

res = process_image_v2(img)
print(res)

# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
