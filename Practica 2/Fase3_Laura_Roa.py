import pybullet as p
import pybullet_data
import time
import numpy as np
import csv
import math

# Variables globales
simulation_time = 0      # Tiempo acumulado de simulación
g_total = 0.0            # Acumulador de gasto total de torque

# Índices de joints según URDF
WHEELS   = [0, 1, 2, 3, 4, 5]        # Índices de las ruedas
Q1_JOINT = 7                         # Articulación SCARA 1 (RRTR)
Q2_JOINT = 9                         # Articulación SCARA 2
ARM_GRIP_JOINT = 11                 # Articulación lineal (T)

F1_JOINT = 13                        # Dedo 1 del gripper
F2_JOINT = 14                        # Dedo 2 del gripper

ARM_J    = [Q1_JOINT, Q2_JOINT, ARM_GRIP_JOINT]  # Joints de brazo (IK)
GRIPPER  = [F1_JOINT, F2_JOINT]                  # Joints del gripper
EEF_LINK = 12                                    # End-effector link

FULL_ARM = ARM_J + [EEF_LINK] + GRIPPER          # Todos los joints que se monitorean

# Avanza la simulación 1 paso
def _spinOnce() -> None:
  global simulation_time
  simulation_time += DT
  p.stepSimulation()
  time.sleep(DT)

# Normaliza ángulos en el rango [-pi, pi]
def _normalize_angle(angle):
  return (angle + math.pi) % (2*math.pi) - math.pi

# Control de velocidad de las ruedas del robot
def set_wheel_velocity(v_lin, v_ang, radius=0.05, half_axle=0.15, max_force=10):
  v_left  = (v_lin - v_ang*half_axle) / radius
  v_right = (v_lin + v_ang*half_axle) / radius
  for j in WHEELS[:3]:  # ruedas del lado izquierdo
      p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=-v_left, force=max_force)
  for j in WHEELS[3:]: # ruedas del lado derecho
      p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, targetVelocity=-v_right, force=max_force)

# Navegación básica hacia un punto (sin SLAM)
def go_to_xy(target_x, target_y, lin_speed=0.4, ang_speed=1.0, pos_tol=0.02, ang_tol=0.02):
  while True:
    (x, y, _), ori = p.getBasePositionAndOrientation(robot)
    dx, dy = target_x - x, target_y - y
    dist = math.hypot(dx, dy)
    
    if dist < pos_tol:  # Detener si estamos cerca
      break
    
    yaw = p.getEulerFromQuaternion(ori)[2]
    yaw_goal = math.atan2(-dx, dy) - yaw
    yaw_goal = _normalize_angle(yaw_goal)
    
    if abs(yaw_goal) > ang_tol:
      v_lin = 0.0
      v_ang = ang_speed if yaw_goal > 0 else -ang_speed
    else:
      v_lin = lin_speed
      v_ang = 0.0
    
    set_wheel_velocity(v_lin, v_ang)
    _spinOnce()
    
  set_wheel_velocity(0, 0)

# Espera a que el robot esté completamente detenido
def esperar_parada_plataforma(lin_thresh=0.01, ang_thresh=0.02, settle_steps=50):
  still = 0
  while still < settle_steps:
    lin_vel, ang_vel = p.getBaseVelocity(robot)
    lin_speed = math.hypot(lin_vel[0], lin_vel[1])
    ang_speed = abs(ang_vel[2]) 

    if lin_speed < lin_thresh and ang_speed < ang_thresh:
      still += 1
    else:
      still = 0

    _spinOnce()

# Registra en CSV las fuerzas de torque actuales del brazo
def register_data(joints_indices):
  global simulation_time
  global g_total
  
  total_torque = 0.0
  indices = 0
  for joint in joints_indices:
    _, _, _, joint_torque = p.getJointState(robot, joint)
    if abs(joint_torque) > 0:
      indices += 1
    total_torque += abs(joint_torque)  # Acumula torques absolutos
  g_total += total_torque
  writer.writerow(['{:.3f}'.format(simulation_time), indices, '{:.3f}'.format(total_torque)])

# Cálculo de IK para el SCARA
def ik_to(target_pos, target_orn=None, num_iters=1000, max_joint_speed=0.1):
  print(f"IK a posicion: {target_pos}, orientacion: {target_orn}")
  jts = p.calculateInverseKinematics(robot, EEF_LINK, 
                                      targetPosition=target_pos,
                                      targetOrientation=target_orn,
                                      solver=0,
                                      maxNumIterations=num_iters,
                                      residualThreshold=0.0001
                                    )
  return [jts[i] for i in [6, 7, 8, 9]]

# Mueve el brazo a una configuración deseada (posiciones conjuntas)
def servo_arm(qs, kp=0.1, max_force=700):
  p.setJointMotorControl2(robot, Q1_JOINT, p.POSITION_CONTROL, targetPosition=qs[0], positionGain=kp, force=max_force)
  p.setJointMotorControl2(robot, Q2_JOINT, p.POSITION_CONTROL, targetPosition=qs[1], positionGain=kp, force=max_force)
  p.setJointMotorControl2(robot, ARM_GRIP_JOINT, p.POSITION_CONTROL, targetPosition=qs[2], positionGain=kp, force=max_force)
  p.setJointMotorControl2(robot, EEF_LINK, p.POSITION_CONTROL, targetPosition=2.78, positionGain=kp, force=max_force)

# Control del gripper (abrir o cerrar)
def grip(close=True, kp=0.1, max_force=700):
  position = 0.0 if close else 0.5
  p.setJointMotorControl2(robot, F1_JOINT, p.POSITION_CONTROL, targetPosition=position, positionGain=kp, force=max_force)
  p.setJointMotorControl2(robot, F2_JOINT, p.POSITION_CONTROL, targetPosition=position, positionGain=kp, force=max_force)
  for _ in range(FPS):  # Ejecuta durante 1 segundo
    register_data(FULL_ARM)
    _spinOnce()

# Mueve el brazo al punto deseado en coordenadas (usando IK)
def move(real_world_coord, seconds=3):
  qs = ik_to(real_world_coord)
  servo_arm(qs)
  for _ in range(seconds * FPS):
    register_data(FULL_ARM)
    _spinOnce()

# ──────────────────────── SIMULACIÓN PRINCIPAL ──────────────────────── #

# Parámetros de simulación
FPS = 200
DT = 1/FPS

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

start_orientation = p.getQuaternionFromEuler([0, 0, 0])

# Cargar escenario
plane = p.loadURDF("plane.urdf", [0,0,0], start_orientation)
robot = p.loadURDF("models/spydy_gonzalez/urdf/spydy_gonzalez.urdf", [0.0, 0.0, 1], start_orientation)
cuby = p.loadURDF("models/cuby/urdf/cuby.urdf", [0.0, 4.0, 0.0], start_orientation)

# Información de joints para debug
num_joints_spydy = p.getNumJoints(robot)
print(f"Num-Joints: {num_joints_spydy}")
for i in range(num_joints_spydy):
    joint_info = p.getJointInfo(robot, i)
    print(f"Joint {joint_info[0]}: {joint_info[1].decode()}")

# Configurar límites dinámicos (si fuera necesario)
p.changeDynamics(robot, ARM_GRIP_JOINT, jointLowerLimit=-1.5, jointUpperLimit=1.5)
p.changeDynamics(robot, F1_JOINT, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(robot, F2_JOINT, jointLowerLimit=-0.5, jointUpperLimit=0.5)

# Crear archivo CSV para registrar datos
file = open("csv_results/Fase3_Laura_Roa.csv", mode="w", newline="")
writer = csv.writer(file)
writer.writerow(["Tiempo", "NumeroJoints", "G_parcial"])

# 1. Conducir hacia el cubo (hasta 1m antes)
go_to_xy(0.0, 1.0)
esperar_parada_plataforma()

# 2. Coger el cubo
move([-0.75, 4.5, 1.5], 1)   # acercamiento lateral
move([0, 4, 1.5], 1)         # encima del cubo
grip(False)                  # abrir gripper
move([0, 4, 0.5], 1)         # bajar al cubo

# 3. Cerrar gripper para agarrar
grip()  

# 4. Subir y mover al contenedor
move([0, 4, 2.75], 1)
move([-0.75, 3.5, 2.75], 1)
move([-0.75, 2.75, 2.75], 1)
move([0, 2, 2.75], 1)

# 5. Soltar cubo
grip(False)

# 6. Volver a posición de inicio
move([-0.75, 2.75, 2.75], 1)
move([-1.5, 3.75, 2.75], 1)
move([-2.25, 3.0, 2.75], 1)
move([-3, 3.75, 1.5], 1)

# Finalizar
file.close()
print(f"G total de fuerzas es:{g_total}")
p.disconnect()
