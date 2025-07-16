import math
import matplotlib.pyplot as plt
import numpy as np

# Datos del robot (obtenidos de la URL proporcionada)
# Fuente: https://acapovilla.github.io/robotica-2025/clases/10/tres.html
r = 0.07  # Radio de las ruedas [m]
b = 0.135  # Separación entre las ruedas [m]

# --- Función para simular y graficar el recorrido ---
def simular_y_graficar_recorrido(x_dot, theta_dot, duracion_total, titulo="Recorrido del Robot"):
    """
    Simula el recorrido del robot y lo grafica.
    x_dot: Velocidad lineal del robot [m/s]
    theta_dot: Velocidad angular del robot [rad/s]
    duracion_total: Duración total de la trayectoria [s]
    titulo: Título para el gráfico
    """
    dt = 0.01  # Paso de tiempo para la simulación [s]
    num_pasos = int(duracion_total / dt)

    # Listas para almacenar la pose del robot
    x_pos = [0.0]
    y_pos = [0.0]
    theta_orient = [0.0] # Orientación inicial del robot (apuntando a lo largo del eje X)

    # Simulación del movimiento (integración de la velocidad)
    for i in range(num_pasos):
        # Velocidades en el marco de referencia global
        # O_x_dot = x_dot * cos(theta) - y_dot * sin(theta)  (y_dot_local es 0 para nuestro modelo)
        # O_y_dot = x_dot * sin(theta) + y_dot * cos(theta)  (y_dot_local es 0 para nuestro modelo)
        # O_theta_dot = theta_dot

        # Para un robot diferencial, la velocidad lineal 'x_dot' es a lo largo de su eje x local.
        # Las ecuaciones de la imagen para la pose global son:
        # O_x_dot = cos(theta) * R_x_dot - sin(theta) * R_y_dot
        # O_y_dot = sin(theta) * R_x_dot + cos(theta) * R_y_dot
        # O_theta_dot = R_theta_dot
        # Donde R_x_dot es nuestra 'x_dot', y R_y_dot es 0.

        current_theta = theta_orient[-1]

        # Velocidades lineales en el marco global
        global_x_dot = x_dot * math.cos(current_theta)
        global_y_dot = x_dot * math.sin(current_theta)

        # Actualizar la pose
        new_x = x_pos[-1] + global_x_dot * dt
        new_y = y_pos[-1] + global_y_dot * dt
        new_theta = (current_theta + theta_dot * dt) # % (2 * math.pi) # Normalizar el ángulo si es necesario

        x_pos.append(new_x)
        y_pos.append(new_y)
        theta_orient.append(new_theta)

    # Graficar el recorrido
    plt.figure(figsize=(8, 6))
    plt.plot(x_pos, y_pos, label='Recorrido del Robot', color='blue')
    plt.scatter(x_pos[0], y_pos[0], color='green', marker='o', s=100, label='Inicio')
    plt.scatter(x_pos[-1], y_pos[-1], color='red', marker='x', s=100, label='Fin')

    # Añadir flechas para la orientación en algunos puntos
    for i in range(0, len(x_pos), max(1, len(x_pos) // 10)): # 10 flechas
        plt.arrow(x_pos[i], y_pos[i],
                  0.1 * math.cos(theta_orient[i]), 0.1 * math.sin(theta_orient[i]),
                  head_width=0.03, head_length=0.05, fc='gray', ec='gray', alpha=0.5)

    plt.title(titulo)
    plt.xlabel("Posición X [m]")
    plt.ylabel("Posición Y [m]")
    plt.grid(True)
    plt.axis('equal') # Asegura que las proporciones sean correctas
    plt.legend()
    plt.show()

# --- Cálculo para Trayectoria Recta de 1 metro en 10 segundos ---
print("--- Cálculo para Trayectoria Recta ---")

# Datos de la trayectoria
distancia_recta = 1.0  # metros
tiempo_recta = 10.0    # segundos

# Velocidad lineal del robot (en el eje x local)
x_dot_recta = distancia_recta / tiempo_recta  # [m/s]
theta_dot_recta = 0.0                      # [rad/s] (para movimiento recto)

# Usando el modelo cinemático inverso para calcular las velocidades angulares de las ruedas:
phi_R_dot_recta = (1 / r) * (x_dot_recta + (b / 2) * theta_dot_recta)  # [rad/s]
phi_L_dot_recta = (1 / r) * (x_dot_recta - (b / 2) * theta_dot_recta)  # [rad/s]

print(f"Velocidad lineal del robot (x_dot): {x_dot_recta:.4f} [m/s]")
print(f"Velocidad angular del robot (theta_dot): {theta_dot_recta:.4f} [rad/s]")
print(f"Velocidad angular rueda derecha (phi_R_dot): {phi_R_dot_recta:.4f} [rad/s]")
print(f"Velocidad angular rueda izquierda (phi_L_dot): {phi_L_dot_recta:.4f} [rad/s]")
print("\n")

# Graficar el recorrido recto
simular_y_graficar_recorrido(x_dot_recta, theta_dot_recta, tiempo_recta, "Recorrido Recto del Robot (1m en 10s)")


# --- Trayectoria circular con un radio de 0.5 metros en sentido horario en 20 segundos ---
print("--- Cálculo para Trayectoria Circular (sentido horario) ---")

# Datos de la trayectoria
radio_circular = 0.5  # metros
tiempo_circular = 20.0 # segundos

# Para completar una trayectoria circular, la velocidad lineal (x_dot) es la circunferencia dividida por el tiempo,
# y la velocidad angular (theta_dot) es 2*pi dividido por el tiempo.
# Se mueve en sentido horario, por lo tanto, la velocidad angular será negativa.
circunferencia = 2 * math.pi * radio_circular  # [m]
x_dot_circular = circunferencia / tiempo_circular  # [m/s]
theta_dot_circular = - (2 * math.pi / tiempo_circular) # [rad/s] (negativo para sentido horario)

# Usando el modelo cinemático inverso para calcular las velocidades angulares de las ruedas:
phi_R_dot_circular = (1 / r) * (x_dot_circular + (b / 2) * theta_dot_circular)  # [rad/s]
phi_L_dot_circular = (1 / r) * (x_dot_circular - (b / 2) * theta_dot_circular)  # [rad/s]

print(f"Velocidad lineal del robot (x_dot): {x_dot_circular:.4f} [m/s]")
print(f"Velocidad angular del robot (theta_dot): {theta_dot_circular:.4f} [rad/s]")
print(f"Velocidad angular rueda derecha (phi_R_dot): {phi_R_dot_circular:.4f} [rad/s]")
print(f"Velocidad angular rueda izquierda (phi_L_dot): {phi_L_dot_circular:.4f} [rad/s]")

# Graficar el recorrido circular
simular_y_graficar_recorrido(x_dot_circular, theta_dot_circular, tiempo_circular, "Recorrido Circular Horario del Robot (Radio 0.5m en 20s)")