#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

# Define a Joint class that holds name, min/max limits and the current angle.
class Joint:
    def __init__(self, name, min_angle, max_angle, current_angle=0.0):
        self.name = name
        self.min_angle = min_angle  # in radians
        self.max_angle = max_angle  # in radians
        self.current_angle = current_angle

    def set_angle(self, angle):
        if angle < self.min_angle or angle > self.max_angle:
            raise ValueError(f"{self.name} angle {angle:.2f} rad out of range "
                             f"({self.min_angle:.2f} to {self.max_angle:.2f} rad)")
        self.current_angle = angle

    def __str__(self):
        return (f"{self.name}: {math.degrees(self.current_angle):.2f}° "
                f"(Range: {math.degrees(self.min_angle):.2f}°-{math.degrees(self.max_angle):.2f}°)")

# Define a RobotArm class that aggregates a list of joints and can compute interpolation.
class RobotArm:
    def __init__(self, joints):
        self.joints = joints

    def get_current_angles(self):
        return [joint.current_angle for joint in self.joints]

    def set_target_angles(self, target_angles):
        if len(target_angles) != len(self.joints):
            raise ValueError("Number of target angles must equal the number of joints.")
        for joint, angle in zip(self.joints, target_angles):
            if angle < joint.min_angle or angle > joint.max_angle:
                raise ValueError(f"{joint.name} angle {angle:.2f} rad is out of range.")
        return target_angles

    def interpolate_angles(self, target_angles, steps=30):
        current_angles = np.array(self.get_current_angles())
        target_angles = np.array(target_angles)
        # For each joint, generate a sequence of intermediate angles.
        sequences = [np.linspace(current_angles[i], target_angles[i], steps)
                     for i in range(len(current_angles))]
        # Zip the sequences to form a list of tuples (one per step).
        movement_sequence = list(zip(*sequences))
        return movement_sequence

    def update_angles(self, angles):
        for joint, angle in zip(self.joints, angles):
            joint.current_angle = angle

# The ROS2 node that publishes joint states based on user input.
class RobotArmPublisher(Node):
    def __init__(self):
        super().__init__('robot_arm_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # The joint names must match those defined in your URDF.
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        # Create the robot arm with six joints and their limits.
        self.robot_arm = RobotArm([
            Joint("j1", -math.pi, math.pi, 0.0),
            Joint("j2", -math.pi/2, math.pi/2, 0.0),
            Joint("j3", -math.pi, math.pi, 0.0),
            Joint("j4", -math.pi/2, math.pi/2, 0.0),
            Joint("j5", -math.pi, math.pi, 0.0),
            Joint("j6", -math.pi, math.pi, 0.0)
        ])

    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.robot_arm.get_current_angles()
        self.publisher_.publish(msg)

    def move_to_target(self, target_angles, steps=30, delay=0.1):
        sequence = self.robot_arm.interpolate_angles(target_angles, steps)
        for angles in sequence:
            self.robot_arm.update_angles(angles)
            self.publish_joint_states()
            time.sleep(delay)
        # Ensure the current angles exactly match the target.
        self.robot_arm.update_angles(target_angles)
        self.publish_joint_states()

def main(args=None):
    rclpy.init(args=args)
    node = RobotArmPublisher()

    try:
        while rclpy.ok():
            target_angles = []
            # Prompt the user for each joint's target angle (in degrees).
            for i in range(6):
                inp = input(f"Enter target angle (in degrees) for joint {i+1} (or 'q' to quit): ")
                if inp.lower() == 'q':
                    print("Exiting.")
                    return
                try:
                    angle_deg = float(inp)
                    angle_rad = math.radians(angle_deg)
                    target_angles.append(angle_rad)
                except ValueError:
                    print("Invalid input; please enter a numeric value.")
                    break
            if len(target_angles) != 6:
                continue  # If not all joints were provided correctly, re-prompt.
            try:
                node.robot_arm.set_target_angles(target_angles)
                node.move_to_target(target_angles)
                print("Final joint states:")
                for joint in node.robot_arm.joints:
                    print(joint)
            except ValueError as e:
                print(e)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

