from TMMC_Wrapper import *
from TMMC_Wrapper.Control import ROBOTMODE
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 6

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = False

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel <= 0:
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
        controller = ControlFlow(control, camera)
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
                return False
            return True
        def detect_wall(scan_distance, distance_threshold):
            scan = lidar.checkScan()
            dist_tuple = lidar.detect_obstacle_in_cone(scan, scan_distance, 0, 5)
            dist = dist_tuple[0] * math.cos(math.radians(np.deg2rad(dist_tuple[1])))
            #print(f"shortest: {dist_tuple}, dist: {dist}")
            #dist = dist_tuple[0]
            print(dist)
            if ( 0.0 <= dist <= distance_threshold):
                control.send_cmd_vel(0.0,0.0)
                controller.reverse(0.2)

                #control.start_keyboard_input()
            controller.make_move(atomic_time)
        print("CHALLENGE 1")
        scan_distance = 0.9
        distance_threshold = 0.40
        atomic_time = 0.03
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=atomic_time)
            time.sleep(atomic_time)
            #time.sleep(atomic_time)
            image = camera.rosImg_to_cv2()
            ##print(image)
            poses = camera.estimate_apriltag_pose(image)
            print(poses)
            for pose in poses:
                #print(type(pose[0]))
                if pose[0] == 2:
                    print("yay")
                    controller.drive_to_tag(2)
                    controller.make_move(atomic_time)
            #print("ya")
            #if prevent_flip():
            #detect_wall(scan_distance, distance_threshold)
            #controller.reverse(0.1)
            #controller.make_move(atomic_time)

    if challengeLevel == 2:
        atomic_time = 0.1
        should_stop = False
        controller = ControlFlow(control, camera, imu)

        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=atomic_time)
            time.sleep(atomic_time)
            detection = camera.ML_predict_stop_sign(camera.rosImg_to_cv2())
            print(detection)
            if (detection[0] == True and (detection[4] - detection[2])/ (detection[3] - detection[1]) <= 1.2):
                print("STOP ACTIVATED")
                should_stop = True
            if (should_stop and detection[0] == False):
                print("STOPPING NOW")
                should_stop = False
                control.stop_keyboard_input()
                control.set_cmd_vel(-0.3,0.0, 0.1)
                time.sleep(1.0)
                print("RESUME")
                control.start_keyboard_input()
            
    if challengeLevel == 3:
        # create path
        # find 7->3->5->6
        edges = [
            (7, 3, 1.5, (45, -1)),
            (3, 5, 0.2, (135, -1)),
            (5, 6, 0.1, (90, -1)),
            (6, 7, 0.1, (90, -1))
        ]
        path = RobotPath()
        for edge in edges:
            path.add_edge(*edge)

        controller = ControlFlow(control, camera, imu, path=RobotPath, )
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            controller.make_move(0.1)
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        controller = ControlFlow(control, camera, imu, mode=ROBOTMODE.INIT)

        controller.search_for_tag(3, -1, 360)

        
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

            controller.make_move(0.1)

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5

    if challengeLevel == 6:
        print("TEST 6 OMG")
        controller = ControlFlow(control, camera, imu, mode=ROBOTMODE.INIT)
        controller.search_for_tag(5, -1, 360)
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            controller.make_move(0.1)
            

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
