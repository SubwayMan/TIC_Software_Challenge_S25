import math
import rclpy
from geometry_msgs.msg import Quaternion
import numpy as np
from .Robot import Robot

class IMU:
    def __init__(self, robot : Robot):
        ''' Initializes the IMU object by storing a reference to the provided robot. '''
        self.robot = robot

    def checkImu(self) -> Quaternion:
        ''' Waits until an IMU message is received and returns that message. '''
        self.robot.imu_future = rclpy.Future()
        self.robot.spin_until_future_completed(self.robot.imu_future)
        return self.robot.last_imu_msg

    def rotation_angle(self, q : Quaternion) -> float:
        ''' Calculates the angle of rotation encoded in the quaternion. '''
        # w = cos(theta/2)
        w_clamped = max(-1.0, min(1.0, q.w))
        return 2 * math.acos(w_clamped)

    def conjugate_q(self, q : Quaternion) -> Quaternion:
        ''' Returns the conjugate of the given quaternion. '''
        return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)

    def quaternion_multiply(self, q1 : Quaternion, q2 : Quaternion) -> Quaternion:
        ''' Computes the product of two quaternions. q1 * q2.'''
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
        return Quaternion(
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2,
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

    def euler_from_quaternion(self, quaternion : Quaternion) -> tuple[float, float, float]:
        """ Convert the input quaternion into the corresponding Euler angles; roll, pitch, yaw. """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if yaw<0:
            yaw += math.pi*2

        return roll, pitch, yaw

    def has_rotation_occurred(self, orientation1 : Quaternion, orientation2 : Quaternion, desired_rotation_angle : float) -> bool:
        ''' Determines if the robot\'s orientation has rotated by the desired amount. '''
        q1_inv = self.conjugate_q(orientation1)
        q_rel = self.quaternion_multiply(orientation2, q1_inv)
        rotation_angle = self.rotation_angle(q_rel)

        if self.robot.DEBUG:
          print("current rotation: ", math.degrees(rotation_angle))

        #is desired angle met
        return math.isclose(rotation_angle, desired_rotation_angle, abs_tol=0.01)  # Adjust tolerance as needed

    def has_rotation_occurred_degrees(self, orientation1 : Quaternion, orientation2 : Quaternion, desired_rotation_angle : float, direction: int):
        if direction == "-1": #left
            rads = math.radians(desired_rotation_angle)
        else:
            rads = math.radians(-1 * desired_rotation_angle)
        return self.has_rotation_occurred(orientation1, orientation2, rads)
