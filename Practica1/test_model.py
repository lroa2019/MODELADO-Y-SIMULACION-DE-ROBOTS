import pybullet as p
import pybullet_data
import time

# Inicializar PyBullet en modo GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Cargar el modelo URDF deseado (modifica el nombre de archivo)
#model_path = "urdf_models/ramp.urdf"  # Cambia a 'barrier.urdf', 'cylinder.urdf', 'goal.urdf'
#p.loadURDF(model_path, [0, 0, 0],  useFixedBase=True)

#rampId = p.loadURDF("urdf_models/ramp.urdf", [0, 0, 0],  useFixedBase=True)
#barrierId = p.loadURDF("urdf_models/barrier.urdf", [0, 0, 0],  useFixedBase=True)
#cylinderId = p.loadURDF("urdf_models/cylinder.urdf", [0, 0, 0],  useFixedBase=True)
#goalId = p.loadURDF("urdf_models/goal.urdf", [0, 0, 0],  useFixedBase=True)
goalId = p.loadURDF("urdf_models/goal.urdf", [20, 0, 0.05],  useFixedBase=True)

# Ejecutar la simulación y esperar para visualizar
for _ in range(24000):  # 10 segundos a 240 FPS
    p.stepSimulation()
    time.sleep(1 / 240)

# Finalizar la simulación
p.disconnect()
