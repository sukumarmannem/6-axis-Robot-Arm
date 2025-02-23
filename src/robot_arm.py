import math
import time
import numpy as np
import matplotlib.pyplot as plt
class Joint:
    def __init__(self, name, min_angle, max_angle, current_angle=0.0):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = current_angle

    def set_angle(self, angle):
        if angle < self.min_angle or angle > self.max_angle:
            raise ValueError(f"Angle {angle:.2f} rad for {self.name} is out of range"
                             f" ({self.min_angle:.2f} to {self.max_angle:.2f} rad)")     
        self.current_angle = angle

    def __str__(self):
        return (f"{self.name}: {math.degrees(self.current_angle):.2f}°"
                f" (Range: {math.degrees(self.min_angle):.2f}°-{math.degrees(self.max_angle):.2f}°)")

class Robotarm:
    def __init__(self, joints, link_lengths):
        if len(joints) != len(link_lengths):
            raise ValueError("Number of joints must match number of link lengths")
        self.joints = joints
        self.link_lengths = link_lengths

    def get_current_angles(self):
        return [joint.current_angle for joint in self.joints]   
    
    def set_target_angles(self, target_angles):
        if len(target_angles) != len(self.joints):
            raise ValueError("Number of target angles must match number of joints")
        for joint, angle in zip(self.joints, target_angles):
            if angle < joint.min_angle or angle > joint.max_angle:
                raise ValueError(f"Angle {angle:.2f} rad for {joint.name} is out of range")
            return target_angles

    def interpolate_angles(self, target_angles, steps=30):
        current_angles = np.array(self.get_current_angles())
        target_angles = np.array(target_angles) 
        sequences = [np.linspace(current_angles[i], target_angles[i], steps) for i in range(len(current_angles))]
        movement_sequence = list(zip(*sequences))
        return movement_sequence
    def update_angles(self, angles):
        for joint, angle in zip(self.joints, angles):
            joint.current_angle = angle

    def forward_kinematics(self):
        positions = [(0, 0)]
        total_angle = 0   
        x, y = 0, 0
        for joint, link_length in zip(self.joints, self.link_lengths):
            total_angle += joint.current_angle
            x = positions[-1][0] + link_length * math.cos(total_angle)
            y = positions[-1][1] + link_length * math.sin(total_angle)
            positions.append((x, y))
        return positions

def visualize_robot(arm):
        positions = arm.forward_kinematics()
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        plt.clf()
        plt.plot(xs, ys, '-o', linewidth=4 ,markersize=8)
        total_length = sum(arm.link_lengths)
        plt.xlim(-total_length, total_length)
        plt.ylim(-total_length, total_length)   
        plt.title("Robot Arm")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid(True)
        plt.pause(0.05)

def main():
    joints = [
        Joint("Joint 1", -math.pi, math.pi, 0.0),
        Joint("Joint 2", -math.pi/2, math.pi/2, 0.0),
        Joint("Joint 3", -math.pi, math.pi, 0.0),
        Joint("Joint 4", -math.pi/2, math.pi/2, 0.0),
        Joint("Joint 5", -math.pi, math.pi, 0.0),
        Joint("Joint 6", -math.pi, math.pi, 0.0)
    ]
    link_lengths = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    arm = Robotarm(joints, link_lengths)   
    print("Initial joint angles:")
    plt.ion()
    fig = plt.figure()
    print("type 'q' to exit")
    while True:
        target_angles = []
        for i in range(6):
            user_input = input(f"Enter target angle for joint {i+1} in degrees: ")
            if user_input.lower() == 'q':
                print("Exiting.")
                return
            try:
                angle_deg = float(user_input)
            except ValueError:
                print("Invalid input. Please enter a number.")
                break
            angle_rad = math.radians(angle_deg)
            target_angles.append(angle_rad)
        
        if len(target_angles) != 6:
            continue
        
        try:
            arm.set_target_angles(target_angles)
            movement_sequence = arm.interpolate_angles(target_angles, steps=30)
            for angles in movement_sequence:
                arm.update_angles(angles)
                visualize_robot(arm)
                time.sleep(0.05)
            print("Final joint states (in degrees):")
            for joint in arm.joints:
                print(joint)
            
        except ValueError as e:
            print(e)

if __name__ == "__main__":
    main()
