import cv2
import numpy as np
from controller import Robot



P_COEFFICIENT = 0.1


robot = Robot()
timestep = int(robot.getBasicTimeStep())


camera = robot.getCamera('camera')
camera.enable(timestep)


motor_left = robot.getMotor('left wheel motor')
motor_right = robot.getMotor('right wheel motor')
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0)
motor_right.setVelocity(0)


def get_image_from_camera():
    
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)



while robot.step(timestep) != -1:
    img = get_image_from_camera()

 
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    largest_contour = max(contours, key=cv2.contourArea)
    largest_contour_center = cv2.moments(largest_contour)
    center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])

    error = camera.getWidth() / 2 - center_x

    
    motor_left.setVelocity(- error * P_COEFFICIENT)
    motor_right.setVelocity(error * P_COEFFICIENT)
