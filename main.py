# from numpy import exp, abs, angle
import time
# Import internal programs
# Import Internal Programs
import L2_speed_control as sc
import L2_inverse_kinematics as inv
import L2_vector as vec
import L1_lidar
import rcpy
import rcpy.motor as motor
# Import External programs
import time

print("loading libraries for color tracking...")
import cv2  # For image capture and processing
import argparse  # For fetching user arguments
import numpy as np  # Kernel

print("loading rcpy.")
import rcpy  # Import rcpy library
import rcpy.motor as motor  # Import rcpy motor module

print("finished loading libraries.")
#    Camera

camera_input = 0  # Define camera input. Default=0. 0=/dev/video0

size_w = 240  # Resized image width. This is the image width in pixels.
size_h = 160  # Resized image height. This is the image height in pixels.

#    Color Range, described in HSV

# pink for spray function (change to purple)
# spray1_min = 155     # Minimum H value
# spray2_min = 60     # Minimum S value
# spray3_min = 45    # Minimum V value

# spray1_max = 180     # Maximum H value
# spray2_max = 195     # Maximum S value
# spray3_max = 240    # Maximum V value

# green for lift
lift1_min = 30  # Minimum H value
lift2_min = 90  # Minimum S value
lift3_min = 130  # Minimum V value

lift1_max = 70  # Maximum H value
lift2_max = 190  # Maximum S value
lift3_max = 205  # Maximum V value

filter = 'HSV'  # Use HSV to describe pixel color values


def MotorUp(speed):
    motor.set(motor_up, speed)


def patrolling():
    blocker = 0
    initial = vec.getValid()
    print(initial)
    for Vec in initial:
        print(Vec)
        dist = Vec[0]
        angle = Vec[1]
        if dist <= 0.3 and dist > 0.005:
            blocker = blocker + 1
            print(blocker)

    if blocker > 0:
        print("false")  # something in the way, stop and wait
        myVelocities = np.array([0.0, 0])  # input your first pair
        myPhiDots = inv.convert(myVelocities)
        sc.driveOpenLoop(myPhiDots)
    else:
        print("True")  # nothing in front, move forward
        myVelocities = np.array([0.4, 0])  # input your first pair
        myPhiDots = inv.convert(myVelocities)
        sc.driveOpenLoop(myPhiDots)
        time.sleep(.5)  # input your duration (s)
        dis_moved = dis_moved + 0.2


def deliver():
    x = 0
    if __name__ == "__main__":
        while rcpy.get_state() != rcpy.EXITING:  # exit loop if rcpy not ready
            if rcpy.get_state() == rcpy.RUNNING:  # execute loop when rcpy is ready
                if x == 0:
                    print("motors.py: driving fwd")
                    # MotorL(0.6)                         # gentle speed for testing program. 0.3 PWM may not spin the wheels.
                    # MotorR(0.6)
                    MotorUp(-1)
                    time.sleep(5)  # run fwd for 4 seconds
                    MotorUp(0)
                    time.sleep(5)
                    print("motors.py: driving reverse")
                    # MotorL(-0.6)
                    # MotorR(-0.6)
                    MotorUp(1)
                    time.sleep(5)  # run reverse for 4 seconds
                    x = 1
                else:
                    MotorUp(0)


def main():
    motor_r = 2  # Right Motor assigned to #2
    motor_l = 1  # Left Motor assigned to #1
    motor_up = 4  # Lift motor assigned to 4
    camera = cv2.VideoCapture(camera_input)  # Define camera variable
    camera.set(3, size_w)  # Set width of images that will be retrived from camera
    camera.set(4, size_h)  # Set height of images that will be retrived from camera

    performFuncDist = 20  # close enough  - Mimumum pixel size of object in order to perform function
    # targetSprayFar = 6      # Too Far       - Minimum pixel size of object to track
    # targetSprayDist = 65    # Target Pixels - Target size of object to track
    # targetSprayClose = 70   # Too Close     - Maximum pixel size of object to track
    targetLiftFar = 50  # Too Far       - Minimum pixel size of object to track
    targetLiftDist = 130  # Target Pixels - Target size of object to track
    targetLiftClose = 170  # Too Close     - Maximum pixel size of object to track

    band = 200  # range of x considered to be centered

    x = 0  # will describe target location left to right
    y = 0  # will describe target location bottom to top

    radius = 0  # estimates the radius of the detected target
    width = 0  # estimates the width of the detected target
    duty_l = 0  # initialize motor with zero duty cycle
    duty_r = 0  # initialize motor with zero duty cycle

    print("initializing rcpy...")
    rcpy.set_state(rcpy.RUNNING)  # initialize rcpy
    print("finished initializing rcpy.")

    try:

        while rcpy.get_state() != rcpy.EXITING:

            if rcpy.get_state() == rcpy.RUNNING:
                cnts2 = 0
                scale_t = 0.8  # a scaling factor for speeds
                scale_d = 0.8  # a scaling factor for speeds

                motor_r = 2  # Right Motor assigned to #2
                motor_l = 1  # Left Motor assigned to #1
                motor_up = 4

                ret, image = camera.read()  # Get image from camera

                height, width, channels = image.shape  # Get size of image

                if not ret:
                    break

                if filter == 'RGB':  # If image mode is RGB switch to RGB mode
                    frame_to_thresh = image.copy()
                else:
                    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Otherwise continue reading in HSV
                # thresh
                # spray = cv2.inRange(frame_to_thresh, (spray1_min, spray2_min, spray3_min), (spray1_max, spray2_max, spray3_max))   # Find all pixels in color range
                lift = cv2.inRange(frame_to_thresh, (lift1_min, lift2_min, lift3_min),
                                   (lift1_max, lift2_max, lift3_max))  # Find all pixels in color range

                kernel = np.ones((5, 5), np.uint8)  # Set gaussian blur strength.
                # mask1 = cv2.morphologyEx(spray, cv2.MORPH_OPEN, kernel)     # Apply gaussian blur
                # mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)

                mask2 = cv2.morphologyEx(lift, cv2.MORPH_OPEN, kernel)  # Apply gaussian blur
                mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernel)

                # cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]     # Find closed shapes in image
                cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[
                    -2]  # Find closed shapes in image
                center = None  # Create variable to store point

                # For lifting functionality

                if len(cnts2) > 0:  # If more than 0 closed shapes exist

                    c = max(cnts2, key=cv2.contourArea)  # Get the properties of the largest rectangle
                    x, y, w, h = cv2.boundingRect(c)  # Get properties of rectangle shape
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(image, [box], 0, (0, 0, 255))
                    w = round(w, 2)  # Round width value to 2 decimals

                    x = int(x)  # Cast x value to an integer
                    M = cv2.moments(c)  # Gets area of circle contour
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # Get center x,y value of circle
                elif len(cnts2) == 1:
                    x, y, w, h = cv2.boundingRect(cnts2)  # Get properties of rectangle shape
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(image, [box], 0, (0, 0, 255))
                    w = round(w, 2)  # Round width value to 2 decimals

                    x = int(x)  # Cast x value to an integer
                    M = cv2.moments(c)  # Gets area of circle contour
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))  # Get center x,y value of circle

                if len(
                        cnts2) > 0:  # perform spray function; the width pixel is within the target pixel range go perform task
                    case = "close enough"
                    dir = "driving"
                    if x > ((width / 2) - (band / 2)) and x < ((width / 2) + (
                            band / 2)):  # If center point is centered;move to set distant based on pixel closeness

                        if w >= targetLiftDist:  # Too Close
                            case = "too close"
                            duty = -1 * ((w - targetLiftDist) / (targetLiftClose - targetLiftDist))
                            if duty_l < 0.05 and duty_l > -0.05:
                                case = "lifting"
                                X = 0
                                if X == 0:
                                    print("motors.py: driving fwd")
                                    motor.set(motor_l, 0)
                                    motor.set(motor_r, 0)
                                    motor.set(motor_up, -1)
                                    time.sleep(5)  # run fwd for 4 seconds
                                    motor.set(motor_up, 0)
                                    time.sleep(5)
                                    print("motors.py: driving reverse")
                                    motor.set(motor_up, 1)
                                    time.sleep(5)  # run reverse for 4 seconds
                                    X = 1
                                else:
                                    motor.set(motor_up, 0)
                        elif w < targetLiftDist:  # Too Far
                            case = "too far from target"

                            duty = 1 - ((w - targetLiftFar) / (targetLiftDist - targetLiftFar))
                            duty = scale_d * duty

                        duty_r = duty
                        duty_l = duty

                    else:
                        case = "turning"  # center on the object
                        duty_l = round((x - 0.5 * width) / (0.5 * width), 2)  # Duty Left
                        duty_l = duty_l * scale_t

                        duty_r = round((0.5 * width - x) / (0.5 * width), 2)  # Duty Right
                        duty_r = duty_r * scale_t

                    # Keep duty cycle within range

                    if duty_r > 1:
                        duty_r = 1

                    elif duty_r < -1:
                        duty_r = -1

                    if duty_l > 1:
                        duty_l = 1

                    elif duty_l < -1:
                        duty_l = -1

                    # Round duty cycles
                    duty_l = round(duty_l, 2)
                    duty_r = round(duty_r, 2)

                    # print(case, "\tradius: ", round(w,1), "\tx: ", round(x,0), "\t\tL: ", duty_l, "\tR: ", duty_r)

                    # Set motor duty cycles
                    motor.set(motor_l, duty_l)
                    motor.set(motor_r, duty_r)

                # return to main


                else:  # to far away; keep patrolling (ryans's code here)
                    print("patrolling")
                    blocker = 0
                    initial = vec.getValid()
                    print(initial)
                    for Vec in initial:
                        print(Vec)
                        dist = Vec[0]
                        angle = Vec[1]
                        if dist <= 0.5 and dist > 0.005:
                            blocker = blocker + 1
                            print(blocker)
                    if blocker > 10:  # wall, turn around
                        print("wall")  # something in the way, stop and wait
                        duty = 0.6
                        duty_r = duty
                        duty_l = -duty
                        motor.set(motor_l, duty_l)
                        motor.set(motor_r, duty_r)
                        time.sleep(3)
                    elif blocker < 10 and blocker > 3:  # person wait
                        print("person")  # something in the way, stop and wait
                        duty = 0.0
                    else:
                        print("True")  # nothing in front, move forward
                        duty = 0.8

                    duty_r = duty
                    duty_l = duty
                    motor.set(motor_l, duty_l)
                    motor.set(motor_r, duty_r)
                if len(cnts2) > 0:  # If more than 0 closed shapes exist
                    print(case, "\tradius: ", round(w, 1), "\tx: ", round(x, 0), "\t\tL: ", duty_l, "\tR: ", duty_r)
                # Set motor duty cycles
                motor.set(motor_l, duty_l)
                motor.set(motor_r, duty_r)

            elif rcpy.get_state() == rcpy.PAUSED:
                pass

    except KeyboardInterrupt:  # condition added to catch a "Ctrl-C" event and exit cleanly
        rcpy.set_state(rcpy.EXITING)
        pass

    finally:

        rcpy.set_state(rcpy.EXITING)
        print("Exiting Color Tracking.")


# exiting program will automatically clean up cape

if __name__ == '__main__':
    main()
