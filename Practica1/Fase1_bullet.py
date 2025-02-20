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
rampId = p.loadURDF("urdf_models/ramp.urdf", useFixedBase=True)
barrierId = p.loadURDF("urdf_models/barrier.urdf", useFixedBase=True)
goalId = p.loadURDF("urdf_models/goal.urdf", useFixedBase=True)

# Simulación durante 30 segundos
for _ in range(7200):  # 30 segundos a 240 FPS
    p.stepSimulation()
    time.sleep(1 / 240)

# Finalizar simulación
p.disconnect()
