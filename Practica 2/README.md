# Práctica 2: Diseño 3D de un Robot en Blender

## **Fase 1: Modelado del Robot Rover en Blender**  

### **Objetivo**  
El primer paso de la práctica consiste en diseñar un modelo 3D del robot rover en Blender, siguiendo las especificaciones indicadas en el enunciado. Este modelo servirá como base para la simulación en PyBullet.  

### **Requisitos del Modelado**  
El rover debe cumplir con las siguientes restricciones de dimensiones:  
- **Altura**: Entre 1.5 m y 3 m  
- **Anchura**: Entre 1.7 m y 2.2 m  
- **Longitud**: Entre 3 m y 4 m  
- **Número de ruedas**: Entre 4 y 8  
- **Tracción**: Libre elección (puede ser 4x4, 6x6, 8x8, etc.)  

### **Archivos Generados**  
- `robot.blend` → Archivo con el modelo 3D en Blender.  
- `render_robot.jpg` → Imagen renderizada del rover.  
- `rover.urdf` → Archivo URDF para la simulación en PyBullet.  

Este modelo estará listo para la integración con el mecanismo "Pick and Place" en la siguiente fase.

## **Fase 2: Implementación del Mecanismo "Pick and Place"**  

### **Objetivo**  
En esta fase se añadirá un **brazo robótico SCARA** con un **gripper de dos o más dedos** al rover modelado en Blender. Este mecanismo permitirá recoger cubos de 4 kg desde el suelo y depositarlos en un compartimento del rover.  

### **Requisitos del Brazo Robótico**  
El brazo debe cumplir con las siguientes especificaciones:  
- **Estructura:** SCARA (Selective Compliance Articulated Robot Arm).  
- **Grados de libertad:** 4 (**RRTR**)  
  - **R:** Rotación en la base del brazo.  
  - **R:** Rotación en el primer eslabón.  
  - **T:** Traslación en el eje vertical para bajar el gripper.  
  - **R:** Rotación en la muñeca para orientar el objeto.  
- **Longitud de los brazos:** Cada segmento debe medir entre 1 y 2 metros.  
- **Gripper:** Puede ser de dos o más dedos, con capacidad de sujetar un cubo de 0.5 m de lado.  

### **Condiciones de Funcionamiento**  
- Los cubos siempre estarán en el suelo en la posición `(0,4,0)`.  
- El robot debe recoger al menos **tres cubos** en su compartimento sin perderlos.  
- No se permite que el brazo atraviese el chasis del robot. Se pueden utilizar checkpoints intermedios para evitar colisiones.  
- El brazo no debe moverse mientras el rover está en desplazamiento.  
- **Tiempo máximo:** 15 segundos para recoger y depositar cada cubo.  

### **Archivos Generados**  
- `pick_and_place.blend` → Archivo del brazo robótico en Blender.  
- `pick_and_place.urdf` → Archivo URDF para la simulación en PyBullet.  
- `render_pick_and_place.jpg` → Imagen renderizada del brazo en Blender.  

Este modelo estará listo para la **simulación del coste energético** en la siguiente fase.

## **Fase 3: Análisis del Coste Energético del "Pick and Place"**  

### **Objetivo**  
En esta fase se evaluará el **gasto energético** del mecanismo "Pick and Place" mediante una simulación en **PyBullet**. Se analizará la fuerza aplicada en cada articulación del brazo robótico mientras recoge y deposita los cubos en el compartimento del rover.  

### **Escenario de Simulación en PyBullet**  
- El **rover** comienza en la posición `(0,0,0)`.  
- El **cubo** está en `(0,4,0)` sobre el suelo.  
- El **robot sigue una secuencia específica**:  
  1. **Mover el brazo hacia la posición inicial.**  
  2. **Descender y cerrar el gripper para sujetar el cubo.**  
  3. **Levantar el cubo y trasladarlo al compartimento del rover.**  
  4. **Soltar el cubo y volver a la posición de reposo.**  

### **Medición del Gasto Energético `G-parcial`**  
- Se usará `getJointState()` para obtener la **fuerza aplicada en cada articulación**.  
- Se calculará el **gasto energético parcial** de cada joint en cada instante de tiempo.  
- Se registrará en un archivo CSV con el siguiente formato:  

  ```
  Tiempo, NúmeroJoints, G_parcial
  ```

- Se integrará `G-parcial` a lo largo del tiempo para obtener el **gasto total `G-total`**.  

### **Implementación en Python**  
- Se usará `pybullet` para controlar el brazo SCARA y obtener los datos de torque.  
- La simulación **NO usará `setRealTimeSimulation()`** para mantener la precisión.  
- Se ejecutará la simulación en **pasos de 0.005 segundos** para capturar los datos con alta resolución.  
- Se almacenarán los valores en un archivo **CSV** para su posterior análisis.  

### **Generación del Gráfico "Tiempo vs G-parcial"**  
- Se representará el **costo energético a lo largo del tiempo** en un gráfico.  
- El **título del gráfico** incluirá:  
  - El valor total del gasto energético `G-total`.  
  - La desviación estándar de `G-parcial`.  
- El gráfico se guardará como **PDF** para la entrega.  

### **Archivos Generados**  
- `Fase3_Nombre_Apellido.py` → Código Python para la simulación en PyBullet.  
- `Fase3_Nombre_Apellido.csv` → Archivo con los datos de gasto energético.  
- `Fase3_Nombre_Apellido.pdf` → Gráfico del coste energético en función del tiempo.  


```
python3 Fase3_Laura_Roa.py
```
