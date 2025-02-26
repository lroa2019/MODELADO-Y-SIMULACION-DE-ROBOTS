import pybullet as p
import pybullet_data
import time
import csv
import numpy as np

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Cargar el plano base
planeId = p.loadURDF("plane.urdf")

# Cargar el robot Husky en el origen
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

# Cargar los elementos del escenario
rampId = p.loadURDF("urdf_models/ramp.urdf", [6.5, 0, 0], useFixedBase=True)
barrierId = p.loadURDF("urdf_models/barrier.urdf", [17, -1.5, -0.2], useFixedBase=True)
goalId = p.loadURDF("urdf_models/goal.urdf", [20, 0, 0], useFixedBase=True)

# Get the joint index (find "rotation_joint")
num_joints_husky = p.getNumJoints(huskyId)
for i in range(num_joints_husky):
    joint_info = p.getJointInfo(huskyId, i)
    print(f"Joint {joint_info[0]}: {joint_info[1].decode()}")  # Print joint names

# enable barrrier movement
p.setJointMotorControl2(barrierId, 1, p.VELOCITY_CONTROL, force=0)

wheel_joints = [2, 3, 4, 5]  
speeds = [5.0, 5.0, 5.0, 5.0]

p.setJointMotorControlArray(huskyId, wheel_joints, p.VELOCITY_CONTROL, targetVelocities=speeds)

p.setRealTimeSimulation(1)

csv_filename = "ESPAIDERMAN.csv" #MODIFICA EL NOMBRE

with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    
    # Write header
    writer.writerow(["Time", "X", "Y", "Z", "Linear Speed", 
                    "Wheel 1 Speed", "Wheel 2 Speed", "Wheel 3 Speed", "Wheel 4 Speed",
                    "Wheel 1 Torque", "Wheel 2 Torque", "Wheel 3 Torque", "Wheel 4 Torque"])

    start_time = time.time()
    stop = False
    try:
        while not stop:
            # Get current time
            current_time = time.time() - start_time

            # Get robot position and orientation
            position, orientation = p.getBasePositionAndOrientation(huskyId)
            if position[0] > 20.0:
                stop = True
                print(f"Data saved to {csv_filename}")
            # Get robot linear velocity (speed)
            linear_velocity, _ = p.getBaseVelocity(huskyId)
            robot_speed = np.linalg.norm(linear_velocity)

            # Get wheel speeds and torques
            wheel_speeds = []
            wheel_torques = []
            for wheel in wheel_joints:
                joint_state = p.getJointState(huskyId, wheel)
                wheel_speeds.append(joint_state[1])  # Joint velocity
                wheel_torques.append(joint_state[3])  # Applied torque

            # Save data to CSV
            writer.writerow([current_time, position[0], position[1], position[2], robot_speed] + 
                            wheel_speeds + wheel_torques)

            # Sleep to match real-time simulation speed
            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"Data saved to {csv_filename}")

# Finalizar simulación
p.disconnect()
