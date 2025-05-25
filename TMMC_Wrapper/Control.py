from TMMC_Wrapper import Camera
from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import numpy as np
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener
from collections import deque
from .Robot import Robot

from enum import Enum

class Control:
    def __init__(self, robot : Robot):
        ''' Initializes the control object by storing the robot reference. '''
        self.robot = robot
        self.imu = IMU(self.robot)

    def set_faster_cmd_vel(self):
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration

    def set_cmd_vel(self, velocity_x : float, velocity_phi : float, duration : float):
        """ Sets up and initiates a timer-based process to continuously publish velocity commands for the specified duration. """
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.robot.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self.robot,self.cmd_vel_future)  

    def cmd_vel_timer_callback(self):
        ''' Acts as the timer callback that continuously publishes velocity commands until the duration expires. '''
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.robot.cmd_vel_publisher.publish(msg)
        
    def undock(self) -> Undock.Result:
        ''' Sends an asynchronous undock goal to the robot, waits for the action to complete, and returns the corresponding result. '''
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()
        
    def dock(self) -> Dock.Result:
        ''' Sends an asynchronous dock goal to the robot, waits for the action to complete, and returns the resulting outcome. '''
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()

    def rotate(self, angle : float, direction : int):
        ''' Continuously monitors the robot’s orientation using its IMU, issuing turning commands until the robot has rotated by the desired angle, then stops the rotation. '''
        q_initial = self.imu.checkImu().orientation
        _, _, yaw_start = self.imu.euler_from_quaternion(q_initial)
        yaw_start_deg = math.degrees(yaw_start)
        
        def minimal_angle_diff(start, current):
            diff = (current - start + 180) % 360 - 180
            return abs(diff)
        
        current_diff = 0.0
        
        while current_diff < abs(angle):
            q_current = self.imu.checkImu().orientation
            _, _, yaw_current = self.imu.euler_from_quaternion(q_current)
            yaw_current_deg = math.degrees(yaw_current)
            current_diff = minimal_angle_diff(yaw_start_deg, yaw_current_deg)
            
            self.send_cmd_vel(0.0, direction * 0.20)
            rclpy.spin_once(self.robot, timeout_sec=0.1)
        
        self.send_cmd_vel(0.0, 0.0)
        if self.robot.DEBUG:
            print("turn complete")

        print("turn complete")
        
    def send_cmd_vel(self, linear_x : float, angular_z : float):
        ''' Publishes a twist message to robot\'s command velocity topic using the provided linear and angular velocity values. '''
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.robot.cmd_vel_publisher.publish(msg)

    def stop_keyboard_input(self):
        ''' Disables the processing of keyboard inputs by setting the robot\'s input flag to false. '''
        self.robot.input = False

    def start_keyboard_input(self):
        ''' Enables keyboardinput processing by setting the robot\'s input flag to true. '''
        self.robot.input = True

    def start_keyboard_control(self):
        ''' Initiates a keyboard listener and a dedicated update thread that continuously checks pressed keys to translate them into movement commands. '''
        if self.robot.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            self.robot.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if self.robot.input:
                        self.move_forward()
                elif 's' in pressed_keys:
                    if self.robot.input:
                        self.move_backward()
                elif 'a' in pressed_keys:
                    if self.robot.input:
                        self.turn_left()
                elif 'd' in pressed_keys:
                    if self.robot.input:
                        self.turn_right()
                else:
                    # If no keys are pressed, stop the movement.
                    self.send_cmd_vel(0.0, 0.0)

            def key_control_loop():
                while not self.robot.stop_event.is_set() and rclpy.ok():
                    update_command()
                    time.sleep(0.05)

            def on_press(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.add(key_char)

            def on_release(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.discard(key_char)


            # Start the keyboard listener.
            self.robot.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.robot.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            self.robot.update_thread = threading.Thread(target=key_control_loop, daemon=True)
            self.robot.update_thread.start()
        else:
            print("Keyboard listener already running")

    def stop_keyboard_control(self):
        ''' Stops the keyboard listener and terminates the update thread for keyboard control. '''
        if self.robot.keyboard_listener is not None:
            self.robot.keyboard_listener.stop()
            self.robot.keyboard_listener = None
            if self.robot.DEBUG:
                print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

        if hasattr(self, 'stop_event'):
            self.robot.stop_event.set()
        if hasattr(self, 'update_thread'):
            self.robot.update_thread.join()

    def move_forward(self):
        ''' Sends a command to move the robot forward at a speed scaled by the robot\'s constant speed control factor. '''
        self.send_cmd_vel(self.robot.CONST_speed_control, 0.0)

    def move_backward(self):
        ''' Sends a command to move the robot backward at a speed scaled by the robot\'s constant speed control factor. '''
        self.send_cmd_vel(-self.robot.CONST_speed_control, 0.0)

    def turn_left(self):
        ''' Sends a command to turn the robot left at an angular velocity scaled by the robot\'s constant speed control factor. '''
        self.send_cmd_vel(0.0, self.robot.CONST_speed_control)

    def turn_right(self):
        ''' Sends a command to turn the robot right at an angular velocity scaled by the robot\'s constant speed control factor. '''
        self.send_cmd_vel(0.0, -self.robot.CONST_speed_control)

class ROBOTMODE:
    KEYBOARD=0
    REVERSING=1
    STOPPED=2
    DRIVETOTAG=3
    SEARCHFORTAG=4
    INIT=5 # check what tag is at the beginning

ROBOTMODE = Enum('ROBOTMODE', 
                 [('KEYBOARD', 0), 
                  ('REVERSING', 1), 
                  ('STOPPED', 2), 
                  ('DRIVETOTAG', 3), 
                  ('SEARCHFORTAG', 4),
                  ('INIT', 5)
                  ])

class ControlFlow():
    def __init__(self, control: Control, camera: Camera, imu: IMU, mode=ROBOTMODE.KEYBOARD):
        self.control = control
        self.imu = imu
        self.mode = mode
        self.default_mode = self.mode
        self.timeout = 1
        self.destination_tag = 1
        self.pose = None
        self.last_mode = None
        self.vel = 0

        # variables for SEARCHFORTAG
        self.desired_tag = None
        self.camera = camera
        self.degree = 0
        self.direction = 1
        self.ang_vel = 0.4
        self.dist = 10000
        self.ltime = time.time()

        # for movement (rotation)
        self.rotation_queue = deque()
        self.current_rotation = None # needs to be a tuple (target_degrees, direction, ang_vel)

    def make_move(self, atomic_time):
        self.handle_movement()
        if self.mode == ROBOTMODE.KEYBOARD:
            self.control.start_keyboard_input()
            if self.detect_stop_sign():
                self.stop(0.3)
        
        elif self.mode == ROBOTMODE.STOPPED:
            self.control.stop_keyboard_input()
            if self.timeout <= 0:
                self.mode = self.last_mode or self.default_mode
            self.timeout -= atomic_time

        elif self.mode == ROBOTMODE.REVERSING:
            self.control.stop_keyboard_input()
            self.control.move_backward()
            #self.timeout -= atomic_time
            #while self.timeout <= 0:
            #    self.timeout -= atomic_time
            time.sleep(self.timeout)
            self.mode = self.default_mode

        elif self.mode == ROBOTMODE.STOPPED:
            self.control.stop_keyboard_input()
            self.timeout -= atomic_time
            self.control.set_cmd_vel(0.0, 0.0, atomic_time)

            if self.timeout <= 0:
                self.mode = self.default_mode

        elif self.mode == ROBOTMODE.DRIVETOTAG:
            print("MODE DRIVE TO TAG")
            self.vel = 0.5
            self.control.stop_keyboard_input()

            tags = self.camera.estimate_apriltag_pose(self.camera.rosImg_to_cv2())
            for tag in tags:
                if tag[0] == self.destination_tag:
                    self.pose = tag
                    angle, distance = self._find_angle_and_distance(self.pose)
                    print("Saw tag at angle ", angle, "distance", distance)
                    angle_rotate = abs(float(angle))
                    direction = 1 if angle > 0 else -1
                    self.dist = distance
              
                    # if angle_rotate > 3:
                    #     print(f"ROTATING {angle_rotate} {direction}")
                    self.clear_rotations()
                    self.rotate(0.4, direction, 0.5)
                    break
            else:
                self.dist -= 0.05
                #self.clear_rotations()

            if self.dist <= 0.4:
                self.mode = ROBOTMODE.INIT

            #may need to add driving correction since velocity * time may not be real distance
            #velocity = 1.0
            #time_forward = distance/velocity
            #print(f"MOVING MOVING sleeping for {time_forward}")
            #self.control.set_cmd_vel(velocity, 0.0, time_forward)
            #time.sleep(time_forward)
        elif self.mode == ROBOTMODE.SEARCHFORTAG:
            self.vel = 0
            # Requirement: The desired tag must be set, the angle to turn and multiplier +- 1

            # Safety Check
            # print("Desired tag: ", self.desired_tag)
            
            # Get tag data from the camera so far
            tags = self.camera.estimate_apriltag_pose(self.camera.rosImg_to_cv2())
            for tag in tags:
                if tag[0] == self.desired_tag and -10 <= tag[2] <= 10:
                    #If the tag is found then no need to search more
                    self.drive_to_tag(self.desired_tag)

        elif self.mode == ROBOTMODE.INIT:
            self.vel = 0
            self.control.stop_keyboard_input()
            tags = self.camera.estimate_apriltag_pose(self.camera.rosImg_to_cv2())
            if tags:
                pass

                
    def handle_movement(self):
        def minimal_angle_diff(start, current):
            diff = (current - start + 180) % 360 - 180
            return abs(diff)

        rot = 0
        vel = self.vel

        if self.current_rotation:
            start_orientation, target, direction, angvel = self.current_rotation
            _, _, yaw_start = self.imu.euler_from_quaternion(start_orientation)
            yaw_start_deg = math.degrees(yaw_start)
            q_current = self.imu.checkImu().orientation
            _, _, yaw_current = self.imu.euler_from_quaternion(q_current)
            yaw_current_deg = math.degrees(yaw_current)
            current_diff = minimal_angle_diff(yaw_start_deg, yaw_current_deg)

            if current_diff >= abs(target):
                self.current_rotation = None
            else:
                rot = direction * angvel

        elif self.rotation_queue:
            start = self.imu.checkImu().orientation
            self.current_rotation = (start, *self.rotation_queue.popleft())
        print("Movement", vel, rot)
        self.control.send_cmd_vel(float(vel), float(rot))


    def reverse(self, timeout=1.0):
        print("REVERSING")
        self.mode = ROBOTMODE.REVERSING
        self.timeout = timeout
    
    def stop(self, timeout=0.3):
        self.last_mode = self.mode # store mode to begin after resuming
        self.mode = ROBOTMODE.STOPPED
        self.timeout = timeout

    def drive_to_tag(self, tag):
        #FOR FUTURE, NEED EGDE?
        self.destination_tag = tag
        self.mode = ROBOTMODE.DRIVETOTAG

    def stop(self, timeout=1.0):
        self.mode = ROBOTMODE.STOPPED
        self.timeout = timeout

    def search_for_tag(self, tag, direction, angle=90):
        self.rotate(angle, direction, 0.3)
        self.desired_tag = tag
        self.mode = ROBOTMODE.SEARCHFORTAG

    def _find_angle_and_distance(self, pose):
        _, range, bearing, elevation = pose
        distance = (range * np.cos(np.deg2rad(elevation)))/np.cos(np.deg2rad(bearing))
        return bearing, distance

    def detect_stop_sign(self):
        status, x1, y1, x2, y2 = self.camera.ML_predict_stop_sign(self.camera.rosImg_to_cv2())
        return status and (y2-y1)/(x2-x1) <= 1.2
    
    def rotate(self, degrees, direction, angvel=1):
        if self.current_rotation:
            self.rotation_queue.append((degrees, -direction, angvel))
        else:
            self.current_rotation = (self.imu.checkImu().orientation, degrees, -direction, angvel)

    def clear_rotations(self):
        self.current_rotation = None
        self.rotation_queue = deque()
        



