from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 1

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
controller = ControlFlow(control)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)


try:
    if challengeLevel == 0:
        robot.CONST_speed_control = 0.5
        control.start_keyboard_input()
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

    if challengeLevel == 1:
        def prevent_flip():
            quaternion = imu.checkImu()
            angles = imu.euler_from_quaternion(quaternion.orientation)
            pitch = angles[1] 
            #print(f"pitch: {pitch}")
            if pitch < -0.01 or 0.01 < pitch:
                print("STOP")
                control.stop_keyboard_input()
                control.send_cmd_vel(0.0,0.0)
                time.sleep(1.0)
                control.start_keyboard_input()
        def detect_wall(scan_distance, distance_threshold):
            scan = lidar.checkScan()
            dist_tuple = lidar.detect_obstacle_in_cone(scan, scan_distance, 0, 20)
            #dist = dist_tuple[0] * math.cos(math.radians(dist_tuple[1]))
            #print(f"shortest: {dist_tuple}, dist: {dist}")
            dist = dist_tuple[0]
            if ( 0.0 <= dist <= distance_threshold):
                #print(dist)
                #d = vt
                #t = d/v
                time_moveback = (distance_threshold - dist)/0.2 + 0.15
                print(f"MOVEBACK {time_moveback} SECONDS")
                control.stop_keyboard_input()
                #controller.reverse(0.1)
                control.send_cmd_vel(0.0,0.0)
                control.set_cmd_vel(-0.4,0.0,time_moveback)
                control.start_keyboard_input()
            #controller.make_move(atomic_time)
        scan_distance = 0.5
        distance_threshold = 0.15
        atomic_time = 0.1
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=atomic_time)
            time.sleep(atomic_time)
            detect_wall(scan_distance, distance_threshold)
            prevent_flip()
            #controller.reverse(0.1)
            #controller.make_move(atomic_time)

    if challengeLevel == 2:
        atomic_time = 0.1
        should_stop = False

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=atomic_time)
            time.sleep(atomic_time)
            detection = camera.ML_predict_stop_sign(camera.rosImg_to_cv2())

            if (detection[0] == True and (detection[4] - detection[2])/ (detection[3] - detection[1]) <= 1.2):
                print("STOP ACTIVATED")
                should_stop = True
            if (should_stop and detection[0] == False):
                print("STOPPING NOW")
                should_stop = False
                control.stop_keyboard_input()
                control.set_cmd_vel(0.0,0.0, 1.0)
                time.sleep(1.0)
                print("RESUME")
                control.start_keyboard_input()

            
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
