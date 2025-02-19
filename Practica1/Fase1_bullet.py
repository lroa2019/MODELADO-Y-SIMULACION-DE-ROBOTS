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
rampId = p.loadURDF("urdf_models/ramp.urdf", [10, 0, 0], useFixedBase=True)
barrierId = p.loadURDF("urdf_models/barrier.urdf", [17, 1, 0.5], useFixedBase=True)
goalId = p.loadURDF("urdf_models/goal.urdf", [20, 0, 0.05], useFixedBase=True)

# Simulación durante 10 segundos
for _ in range(24000):  # 10 segundos a 240 FPS
    p.stepSimulation()
    time.sleep(1 / 240)

# Finalizar simulación
p.disconnect()
