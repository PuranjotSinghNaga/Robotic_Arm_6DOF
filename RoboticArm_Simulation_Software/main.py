import pybullet as p
import pybullet_data
import tkinter as tk
from tkinter import ttk
import numpy as np
import serial
import time
import math

# Serial Port Setup
uc_port = 'COM8'  # Change this to your Arduino's port
baud_rate = 9600

try:
    ser = serial.Serial(uc_port, baud_rate)
    time.sleep(1)  # Give time for the connection to establish
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)  # Exit the program if there's an error

# Initialize PyBullet simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load URDF from standard directory

# Load your robot URDF with the base fixed
robot_id = p.loadURDF("example.urdf", useFixedBase=True)

# Get number of joints in the robot
num_joints = p.getNumJoints(robot_id)

# Create a simple GUI using Tkinter
class JointControlApp:
    def __init__(self, root, angle_window):
        self.root = root
        self.angle_window = angle_window
        self.root.title("Robot Joint Control")
        self.sliders = []
        self.joint_angle_labels = []
        self.recorded_positions = []  # List to store recorded positions
        self.trajectory_in_progress = False  # Flag to check if a trajectory is being executed

        # Create sliders for each joint
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            joint_lower_limit = joint_info[8]
            joint_upper_limit = joint_info[9]

            # Create label and slider
            label = tk.Label(root, text=f"Joint {joint_index}: {joint_name}")
            label.pack()
            slider = tk.Scale(root, from_=math.degrees(joint_lower_limit), to=math.degrees(joint_upper_limit), orient=tk.HORIZONTAL, length=300, command=lambda val, j=joint_index: self.update_joint_position(j, val))
            slider.pack()
            self.sliders.append(slider)

            # Create label for joint angles in the second window
            joint_angle_label = tk.Label(angle_window, text=f"Joint {joint_index}: {joint_name} - Angle: 0.0°")
            joint_angle_label.pack()
            self.joint_angle_labels.append(joint_angle_label)

        # Add a "Record Position" button
        record_button = tk.Button(root, text="Record Position", command=self.record_position)
        record_button.pack()

        # Create a label for displaying recorded positions in the second window
        self.recorded_positions_label = tk.Label(angle_window, text="Recorded Positions: None")
        self.recorded_positions_label.pack()

        # Create a dropdown (combobox) to select recorded positions
        self.position_selector = ttk.Combobox(angle_window, state="readonly")
        self.position_selector.pack()

        # Add a button to move to selected recorded position
        move_button = tk.Button(angle_window, text="Move to Selected Position", command=self.move_to_selected_position)
        move_button.pack()

        # Add a button to send joint angles to Arduino
        send_button = tk.Button(root, text="Send Joint Angles to Arduino", command=self.send_joint_angles_to_arduino)
        send_button.pack()

    # Update the robot joint position in the PyBullet simulation
    def update_joint_position(self, joint_index, slider_value):
        # Convert degrees to radians for PyBullet
        target_position_rad = math.radians(float(slider_value))
        p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=target_position_rad)

    # Update the joint angles in the second window
    def update_joint_angles(self):
        for joint_index in range(num_joints):
            joint_state = p.getJointState(robot_id, joint_index)
            joint_angle_rad = joint_state[0]
            joint_angle_deg = math.degrees(joint_angle_rad)  # Convert to degrees
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')

            # Update the text in the angle label
            self.joint_angle_labels[joint_index].config(text=f"Joint {joint_index}: {joint_name} - Angle: {joint_angle_deg:.2f}°")

    # Record the current joint positions
    def record_position(self):
        current_positions = []
        for joint_index in range(num_joints):
            joint_state = p.getJointState(robot_id, joint_index)
            joint_angle_rad = joint_state[0]
            joint_angle_deg = math.degrees(joint_angle_rad)  # Convert to degrees
            current_positions.append(joint_angle_deg)

        self.recorded_positions.append(current_positions)
        print(f"Recorded position: {current_positions}")

        # Update the recorded positions label in the second window
        self.update_recorded_positions_label()

    # Update the recorded positions display in the second window
    def update_recorded_positions_label(self):
        positions_text = "\n".join([f"Position {i + 1}: {pos}" for i, pos in enumerate(self.recorded_positions)])
        if positions_text:
            self.recorded_positions_label.config(text=f"Recorded Positions:\n{positions_text}")
            # Update the combobox with new recorded positions
            self.position_selector['values'] = [f"Position {i + 1}" for i in range(len(self.recorded_positions))]
            self.position_selector.set("Select a Position")
        else:
            self.recorded_positions_label.config(text="Recorded Positions: None")

    # Move the robot to the selected recorded position
    def move_to_selected_position(self):
        selected_index = self.position_selector.current()
        if selected_index >= 0:
            selected_position = self.recorded_positions[selected_index]
            # Call trajectory execution function
            self.execute_trajectory(selected_position)

    # Plan a linear trajectory and move the robot smoothly
    def execute_trajectory(self, target_position, steps=100, time_step=0.01):
        # Get the current joint positions in degrees
        current_position_deg = [math.degrees(p.getJointState(robot_id, joint_index)[0]) for joint_index in range(num_joints)]

        # Generate linear trajectory for each joint
        for step in range(steps):
            interpolated_position = [
                np.linspace(current_pos, target_pos, steps)[step]
                for current_pos, target_pos in zip(current_position_deg, target_position)
            ]

            # Apply interpolated positions to each joint (convert back to radians)
            for joint_index in range(num_joints):
                target_position_rad = math.radians(interpolated_position[joint_index])
                p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=target_position_rad)

            # Step simulation and wait
            p.stepSimulation()
            self.root.after(int(time_step * 1000))  # Delay for smooth transition

    # Send joint angles to Arduino
    def send_joint_angles_to_arduino(self):
        joint_angles = [math.degrees(p.getJointState(robot_id, joint_index)[0]) for joint_index in range(num_joints)]  # Get angles in degrees
        command = ','.join(map(str, joint_angles))  # Create a comma-separated string of angles
        send_data(command)  # Send the command to the Arduino

# Function to send data to Arduino
# Function to send data to Arduino and read the echoed message
def send_data(data):
    """Send data to Arduino and read the echoed response."""
    try:
        # Ensure the data is encoded to bytes before sending
        ser.write(data.encode('utf-8') + b'\n')  # Send the data with a newline
        print(f'Sent: {data}')

        # Read the echoed response from Arduino
        if ser.in_waiting > 0:  # Check if there's data to read
            response = ser.readline()
            try:
                response = response.decode('utf-8').strip()  # Attempt to decode
                print(f'Echoed: {response}')  # Print the echoed message
            except UnicodeDecodeError:
                # If decoding fails, print the raw bytes instead
                print('Received non-UTF-8 data:', response)  # Handle decoding errors
    except Exception as e:
        print(f'Error sending data: {e}')


# Main loop
root = tk.Tk()
angle_window = tk.Toplevel(root)  # Create a second window for joint angles and recorded positions
angle_window.title("Joint Angles & Recorded Positions")

app = JointControlApp(root, angle_window)

def update_pybullet():
    p.stepSimulation()
    app.update_joint_angles()  # Update the joint angles in the second window
    root.after(50, update_pybullet)  # Run PyBullet simulation in the background

update_pybullet()
root.mainloop()

# Disconnect PyBullet after closing the window
p.disconnect()
ser.close()  # Close the serial connection
