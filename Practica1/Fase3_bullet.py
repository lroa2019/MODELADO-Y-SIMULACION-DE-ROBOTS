import pybullet as p
import pybullet_data
import time
import csv
import numpy as np

# Inicializar PyBullet en modo GUI (interfaz gráfica)
physicsClient = p.connect(p.GUI)

# Agregar la ruta de búsqueda para los modelos URDF predeterminados de PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Establecer la gravedad en la simulación (9.81 m/s² hacia abajo)
p.setGravity(0, 0, -9.81)

# Cargar el plano base para la simulación
planeId = p.loadURDF("plane.urdf")

# Cargar el robot Husky en la posición inicial (0,0,0.1)
huskyId = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

# Cargar los elementos del escenario con posiciones específicas
rampId = p.loadURDF("urdf_models/ramp.urdf", [6.5, 0, 0], useFixedBase=True)  # Rampa en (6.5,0,0)
barrierId = p.loadURDF("urdf_models/barrier.urdf", [17, -1.5, -0.2], useFixedBase=True)  # Barrera en (17,-1.5,-0.2)
goalId = p.loadURDF("urdf_models/goal.urdf", [20, 0, 0], useFixedBase=True)  # Objetivo en (20,0,0)

# Obtener y mostrar los nombres de las articulaciones del Husky
num_joints_husky = p.getNumJoints(huskyId)
for i in range(num_joints_husky):
    joint_info = p.getJointInfo(huskyId, i)
    print(f"Joint {joint_info[0]}: {joint_info[1].decode()}")  # Muestra el índice y el nombre de cada articulación

# Permitir que la barrera gire libremente (sin fuerza aplicada)
p.setJointMotorControl2(barrierId, 1, p.VELOCITY_CONTROL, force=0)

# Identificar las articulaciones de las ruedas del Husky
wheel_joints = [2, 3, 4, 5]  # Índices de las 4 ruedas
speeds = [11.0, 11.0, 11.0, 11.0]  # Velocidad objetivo para cada rueda

# Configurar el torque aplicado a las ruedas
force = 25
forces = [force] * len(wheel_joints)

# Ajustar la fricción de las ruedas para mejorar el control del Husky
lateralFriction = 0.93  # Fricción lateral para estabilidad en giros
spinningFriction = 0.005  # Fricción de rotación para evitar resistencia al giro
rollingFriction = 0.003  # Fricción de rodadura para mejorar la tracción
for wheel in wheel_joints:
    p.changeDynamics(huskyId, wheel, lateralFriction=lateralFriction, 
                     spinningFriction=spinningFriction, rollingFriction=rollingFriction)
    
    # Aplicar un torque constante a cada rueda para que el Husky avance
    p.setJointMotorControl2(huskyId, wheel, p.TORQUE_CONTROL, force=force)

# Establecer el control de velocidad para las ruedas
p.setJointMotorControlArray(huskyId, wheel_joints, p.VELOCITY_CONTROL, targetVelocities=speeds, forces=forces)

# Habilitar la simulación en tiempo real
p.setRealTimeSimulation(1)

# Archivo CSV donde se guardarán los datos del experimento
csv_filename = "Fase3.csv"

# Abrir el archivo CSV y escribir los datos de la simulación
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    
    # Escribir la cabecera del archivo CSV
    writer.writerow(["Time", "X", "Y", "Z", "Linear Speed", 
                    "Wheel 1 Speed", "Wheel 2 Speed", "Wheel 3 Speed", "Wheel 4 Speed",
                    "Wheel 1 Torque", "Wheel 2 Torque", "Wheel 3 Torque", "Wheel 4 Torque"])

    # Iniciar la simulación
    start_time = time.time()
    stop = False  # Variable para detener la simulación cuando el Husky llegue a la meta
    try:
        while not stop:
            
            # Obtener el tiempo actual desde el inicio de la simulación
            current_time = time.time() - start_time

            # Obtener la posición y orientación del Husky
            position, orientation = p.getBasePositionAndOrientation(huskyId)

            # Verificar si el Husky ha alcanzado la meta (posición X > 20 m)
            if position[0] > 20.0:
                stop = True  # Detener la simulación
                print(f"Data saved to {csv_filename}")  # Mensaje de confirmación

            # Obtener la velocidad lineal del Husky
            linear_velocity, _ = p.getBaseVelocity(huskyId)
            robot_speed = np.linalg.norm(linear_velocity)  # Calcular la magnitud de la velocidad

            # Obtener la velocidad y torque de cada rueda
            wheel_speeds = []
            wheel_torques = []
            for wheel in wheel_joints:
                joint_state = p.getJointState(huskyId, wheel)
                wheel_speeds.append(joint_state[1])  # Velocidad de la rueda
                wheel_torques.append(joint_state[3])  # Torque aplicado a la rueda

            # Guardar los datos en el archivo CSV
            writer.writerow([current_time, position[0], position[1], position[2], robot_speed] + 
                            wheel_speeds + wheel_torques)

            # Pequeña pausa para sincronizar la simulación
            time.sleep(0.1)

    except KeyboardInterrupt:
        # En caso de interrupción manual, guardar los datos antes de cerrar
        print(f"Data saved to {csv_filename}")

# Finalizar la simulación
p.disconnect()
