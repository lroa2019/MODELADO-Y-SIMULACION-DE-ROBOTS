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
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 0.0])

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

# Initial control parameters
target_speed = 2.0  # Target speed in m/s
minimum_torque = 25
K_speed = 5
K_pitch = 5
dt = 0.01  # Time step

wheel_joints = [2, 3, 4, 5]  

lateralFriction = 0.93
spinningFriction = 0.005
rollingFriction = 0.003
for wheel in wheel_joints:
  p.changeDynamics(huskyId, wheel, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)

p.setRealTimeSimulation(1)


csv_filename = "Fase4.csv"

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
      current_time = time.time() - start_time

      position, orientation = p.getBasePositionAndOrientation(huskyId)
      
      if position[0] > 20.0:
        stop = True
        print(f"Data saved to {csv_filename}")
        
      linear_velocity, _ = p.getBaseVelocity(huskyId)
      robot_speed = np.linalg.norm(linear_velocity)

      euler_angles = p.getEulerFromQuaternion(orientation)
      pitch = euler_angles[1]  # Se usa Pitch para saber la inclinacion del robot.
      
      speed_error = target_speed - robot_speed    
      
      torque = (K_speed * speed_error) + (K_pitch * pitch) + minimum_torque
      torque = max(0, torque)
      for wheel in wheel_joints:
        p.setJointMotorControl2(huskyId, wheel, p.TORQUE_CONTROL, force=torque)
        p.setJointMotorControl2(huskyId, wheel, p.VELOCITY_CONTROL, targetVelocity=11.2)

      #p.setJointMotorControlArray(huskyId, wheel_joints, p.VELOCITY_CONTROL, targetVelocities=[11.25]*4, forces=[torque]*4)

      wheel_speeds = []
      wheel_torques = []
      for wheel in wheel_joints:
        joint_state = p.getJointState(huskyId, wheel)
        wheel_speeds.append(joint_state[1])  
        wheel_torques.append(joint_state[3]) 

      
      writer.writerow([current_time, position[0], position[1], position[2], robot_speed] + 
                      wheel_speeds + wheel_torques)

      
      time.sleep(dt)

  except KeyboardInterrupt:
    print(f"Data saved to {csv_filename}")

# Finalizar simulación
p.disconnect()
