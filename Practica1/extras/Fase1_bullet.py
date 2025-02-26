import pybullet as p
import pybullet_data
import time

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

# Simulación durante 30 segundos
for _ in range(7200):  # 30 segundos a 240 FPS
    p.stepSimulation()
    time.sleep(1 / 240)

# Finalizar simulación
p.disconnect()
