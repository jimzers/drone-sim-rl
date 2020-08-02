import airsim
import numpy as np
import cv2

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


def detect_red(np_img, pixel_thresh):
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
    # cv2.imwrite("image.png", output_binary)

    num_reds = np.count_nonzero(output_binary)
    print(num_reds)

    if num_reds > pixel_thresh:
        print("Red object detected!")
        return True
    else:
        print("No red object in sight.")
        return False


def get_reds(np_img):
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
    # cv2.imwrite("image.png", output_binary)

    num_reds = np.count_nonzero(output_binary)
    return num_reds


def crop_center_x(img, cropx=80):
    y, x, c = img.shape
    startx = x // 2 - (cropx // 2)
    return img[:, startx:startx + cropx, :]


vel = 5
quad_offset = (1, 0, 0)
print(quad_offset)
num_consecutive_stops = 0
while True:
    img = get_np_image()
    crop_img = crop_center_x(img, 150)
    cv2.imwrite("crop.png", crop_img)
    num_reds = get_reds(crop_img)
    print(num_reds)
    if num_reds < 16000:
        print("moving...")
        num_consecutive_stops = 0
        client.moveByVelocityAsync(vel * quad_offset[0], vel * quad_offset[1], vel * quad_offset[2], 0.3).join()
    else:
        print("threshold hit")
        num_consecutive_stops += 1
        airsim.time.sleep(0.3)
    if num_consecutive_stops > 25:
        break
    # airsim.time.sleep(0.02)

# TODO: move wrt drone's relational position

# exit protocol
client.armDisarm(False)
client.reset()

client.enableApiControl(False)
