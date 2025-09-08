import numpy as np
import matplotlib.pyplot as plt

# Clase que modela un robot diferencial y sus ecuaciones cinemáticas
class RobotDiferencial:
    def __init__(self, radio_rueda, separacion_ruedas):
        """
        Inicializa el robot diferencial con sus propiedades geométricas.
        Args:
            radio_rueda (float): Radio de las ruedas del robot (r).
            separacion_ruedas (float): Separación entre las ruedas del robot (b).
        """
        self.r = radio_rueda
        self.b = separacion_ruedas
        print(f"Robot diferencial inicializado con r = {self.r} m y b = {self.b} m")

    def cinematica_directa(self, phi_dot_R, phi_dot_L):
        """
        Cinemática directa: calcula la velocidad angular del robot (theta_dot) y el radio de giro (R)
        a partir de las velocidades angulares de las ruedas.
        Args:
            phi_dot_R (float): Velocidad angular de la rueda derecha (rad/s).
            phi_dot_L (float): Velocidad angular de la rueda izquierda (rad/s).
        Returns:
            tuple: (theta_dot, R). Si ambas ruedas giran igual, R es infinito (None).
        """
        if abs(phi_dot_R - phi_dot_L) < 1e-9: # Comparación con tolerancia para flotantes
            theta_dot = 0.0
            R = None # Infinito
        else:
            theta_dot = (self.r / self.b) * (phi_dot_R - phi_dot_L)
            R = (self.b / 2) * ((phi_dot_R + phi_dot_L) / (phi_dot_R - phi_dot_L))
        return theta_dot, R

    def cinematica_inversa(self, x_dot, theta_dot):
        """
        Cinemática inversa: calcula las velocidades angulares de las ruedas (phi_dot_R, phi_dot_L)
        a partir de la velocidad lineal y angular del robot.
        Args:
            x_dot (float): Velocidad lineal del robot (m/s).
            theta_dot (float): Velocidad angular del robot (rad/s).
        Returns:
            tuple: (phi_dot_R, phi_dot_L)
        """
        phi_dot_R = (1 / self.r) * (x_dot + (self.b / 2) * theta_dot)
        phi_dot_L = (1 / self.r) * (x_dot - (self.b / 2) * theta_dot)
        return phi_dot_R, phi_dot_L

    def velocidad_en_marco_local(self, phi_dot_R, phi_dot_L):
        """
        Calcula las velocidades del robot en su marco de referencia local (vx, vy, vtheta).
        vx: avance, vy: lateral (siempre 0 en diferencial), vtheta: giro.
        """
        vx = (self.r / 2) * (phi_dot_R + phi_dot_L)
        vy = 0.0 
        vtheta = (self.r / self.b) * (phi_dot_R - phi_dot_L)
        return np.array([vx, vy, vtheta])

    def velocidad_en_marco_inercial(self, phi_dot_R, phi_dot_L, theta_actual):
        """
        Transforma las velocidades del marco local al marco inercial (global),
        usando la orientación actual del robot (theta_actual).
        """
        local_velocities = self.velocidad_en_marco_local(phi_dot_R, phi_dot_L)
        vx_local = local_velocities[0]
        vy_local = local_velocities[1]
        vtheta_local = local_velocities[2]
        # Rotación de velocidades al marco global
        vx_global = vx_local * np.cos(theta_actual) - vy_local * np.sin(theta_actual)
        vy_global = vx_local * np.sin(theta_actual) + vy_local * np.cos(theta_actual)
        vtheta_global = vtheta_local 
        return np.array([vx_global, vy_global, vtheta_global])

    def simular_movimiento(self, punto_inicial, theta_inicial, duracion_total, velocidad_lineal, velocidad_angular, radio_curva=None):
        """
        Simula el movimiento del robot para un segmento de trayectoria dado,
        integrando la posición y orientación a lo largo del tiempo.
        Args:
            punto_inicial (list): [x_inicial, y_inicial]
            theta_inicial (float): Orientación inicial en radianes.
            duracion_total (float): Duración del segmento en segundos.
            velocidad_lineal (float): Velocidad lineal deseada (m/s).
            velocidad_angular (float): Velocidad angular deseada (rad/s).
            radio_curva (float, optional): Radio de la curva si es aplicable. Para impresión.
        Returns:
            tuple: (posiciones_x, posiciones_y, orientaciones_theta, punto_final, theta_final)
        """
        x_actual, y_actual = punto_inicial
        theta_actual = theta_inicial
        # Calcula velocidades de ruedas para el segmento
        phi_dot_R, phi_dot_L = self.cinematica_inversa(velocidad_lineal, velocidad_angular)
        # Número de pasos para integración (100 Hz)
        num_pasos = int(duracion_total * 100)
        if num_pasos == 0: num_pasos = 1
        tiempo_paso = duracion_total / num_pasos
        posiciones_x = [x_actual]
        posiciones_y = [y_actual]
        orientaciones_theta = [theta_actual]
        # Integración de la trayectoria
        for _ in range(num_pasos):
            inercial_velocities = self.velocidad_en_marco_inercial(phi_dot_R, phi_dot_L, theta_actual)
            vx_global, vy_global, vtheta_global = inercial_velocities
            x_actual += vx_global * tiempo_paso
            y_actual += vy_global * tiempo_paso
            theta_actual += vtheta_global * tiempo_paso
            posiciones_x.append(x_actual)
            posiciones_y.append(y_actual)
            orientaciones_theta.append(theta_actual)
        return posiciones_x, posiciones_y, orientaciones_theta, [x_actual, y_actual], theta_actual

# --- Función principal para la simulación de la trayectoria específica ---
def simular_trayectoria_completa(robot):
    """
    Simula una trayectoria compuesta por varios segmentos (giros y rectas),
    imprime los comandos necesarios y grafica el recorrido.
    """
    print("\n======== SIMULACIÓN DE TRAYECTORIA ESPECÍFICA ========")
    # Definición de los segmentos de la trayectoria
    segments = []
    # Segmento 1: Giro a la izquierda (180°)
    # La velocidad lineal necesaria para mantener un radio R está relacionada con la velocidad angular theta_dot
    # R = x_dot / theta_dot
    # x_dot = R * theta_dot
    # Para 180 grados (pi radianes) con radio de giro de 0.25m
    radio_giro_1 = 0.25 # metros (radio de giro especificado)
    angulo_giro_1 = np.pi # 180 grados
    duracion_giro_1 = 6.0 # segundos (aumentado para reducir velocidades angulares)
    # Calcular velocidad angular del robot para el giro
    theta_dot_giro_1 = angulo_giro_1 / duracion_giro_1
    # Calcular velocidad lineal del robot para el giro manteniendo el radio
    x_dot_giro_1 = radio_giro_1 * theta_dot_giro_1 # Se asume R como el radio de giro del centro del robot
    segments.append({
        'nombre': 'Giro a la Izquierda (180°)',
        'duracion': duracion_giro_1,
        'linear_x': x_dot_giro_1,
        'angular_z': theta_dot_giro_1,
        'color': 'red'
    })
    # Segmento 2: Recta 1m
    distancia_recta_1 = 1.0 # metros
    velocidad_recta_1 = 0.18 # m/s (reducida para respetar límite de 50 RPM)
    duracion_recta_1 = distancia_recta_1 / velocidad_recta_1
    segments.append({
        'nombre': 'Recta 1m',
        'duracion': duracion_recta_1,
        'linear_x': velocidad_recta_1,
        'angular_z': 0.0,
        'color': 'green'
    })
    # Segmento 3: Giro a la derecha (180°)
    radio_giro_2 = 0.25 # metros (radio de giro especificado)
    angulo_giro_2 = -np.pi # 180 grados a la derecha
    duracion_giro_2 = 6.0 # segundos (aumentado para reducir velocidades angulares)
    theta_dot_giro_2 = angulo_giro_2 / duracion_giro_2
    x_dot_giro_2 = radio_giro_2 * abs(theta_dot_giro_2) # Velocidad lineal siempre positiva
    segments.append({
        'nombre': 'Giro a la Derecha (180°)',
        'duracion': duracion_giro_2,
        'linear_x': x_dot_giro_2,
        'angular_z': theta_dot_giro_2,
        'color': 'blue'
    })
    # Segmento 4: Recta 1m final
    distancia_recta_2 = 1.0 # metros
    velocidad_recta_2 = 0.18 # m/s (reducida para respetar límite de 50 RPM)
    duracion_recta_2 = distancia_recta_2 / velocidad_recta_2
    segments.append({
        'nombre': 'Recta 1m Final',
        'duracion': duracion_recta_2,
        'linear_x': velocidad_recta_2,
        'angular_z': 0.0,
        'color': 'purple'
    })
    # --- Simulación y recolección de datos para graficar la trayectoria ---
    current_x, current_y = 0.0, 0.0
    current_theta = 0.0 # El robot inicia apuntando a lo largo del eje X
    all_x = [current_x]
    all_y = [current_y]
    all_theta = [current_theta]
    print(f"Punto inicial de la trayectoria: ({current_x:.2f}, {current_y:.2f}), Orientación inicial: {current_theta:.2f} rad")
    for i, seg in enumerate(segments):
        print(f"\n--- Ejecutando Segmento {i+1}: {seg['nombre']} ---")
        # Calcular las velocidades de las ruedas para este segmento
        phi_dot_R_seg, phi_dot_L_seg = robot.cinematica_inversa(seg['linear_x'], seg['angular_z'])
        print(f"  Velocidad lineal deseada: {seg['linear_x']:.4f} m/s")
        print(f"  Velocidad angular deseada: {seg['angular_z']:.4f} rad/s")
        print(f"  Duración del segmento: {seg['duracion']:.2f} s")
        print(f"  Phi_dot_R: {phi_dot_R_seg:.4f} rad/s, Phi_dot_L: {phi_dot_L_seg:.4f} rad/s")
        
        # Verificar límites de velocidad angular (50 RPM = 5.236 rad/s)
        max_phi_dot = 50.0 * (2 * np.pi / 60)  # 50 RPM en rad/s
        if abs(phi_dot_R_seg) > max_phi_dot or abs(phi_dot_L_seg) > max_phi_dot:
            print(f"  ⚠️  ADVERTENCIA: Velocidad angular excede el límite de 50 RPM ({max_phi_dot:.4f} rad/s)")
            print(f"     Rueda derecha: {abs(phi_dot_R_seg):.4f} rad/s ({abs(phi_dot_R_seg) * 60 / (2 * np.pi):.2f} RPM)")
            print(f"     Rueda izquierda: {abs(phi_dot_L_seg):.4f} rad/s ({abs(phi_dot_L_seg) * 60 / (2 * np.pi):.2f} RPM)")
        else:
            print(f"  ✓ Velocidades dentro del límite de 50 RPM")
            print(f"     Rueda derecha: {abs(phi_dot_R_seg) * 60 / (2 * np.pi):.2f} RPM")
            print(f"     Rueda izquierda: {abs(phi_dot_L_seg) * 60 / (2 * np.pi):.2f} RPM")
        # Simular el segmento
        seg_x, seg_y, seg_theta, final_point, final_theta = robot.simular_movimiento(
            [current_x, current_y], current_theta, seg['duracion'], seg['linear_x'], seg['angular_z']
        )
        # Actualizar la posición y orientación actual para el siguiente segmento
        current_x, current_y = final_point
        current_theta = final_theta
        # Almacenar la trayectoria del segmento (sin duplicar el punto inicial)
        all_x.extend(seg_x[1:])
        all_y.extend(seg_y[1:])
        all_theta.extend(seg_theta[1:])
    print("\n======== TRAYECTORIA COMPLETA SIMULADA ========")
    print(f"Punto final de la trayectoria: ({all_x[-1]:.4f}, {all_y[-1]:.4f})")
    print(f"Orientación final: {all_theta[-1]:.4f} rad")
    # --- Visualización de la trayectoria ---
    plt.figure(figsize=(10, 8))
    plt.plot(all_x, all_y, label='Trayectoria Completa', color='black', linewidth=2)
    plt.plot(all_x[0], all_y[0], 'go', markersize=8, label='Inicio')
    plt.plot(all_x[-1], all_y[-1], 'ro', markersize=8, label='Fin')
    # Marcar los puntos de transición entre segmentos
    temp_x, temp_y = [0.0], [0.0]
    temp_theta = 0.0
    for i, seg in enumerate(segments):
        _, _, _, next_point, next_theta = robot.simular_movimiento(
            [temp_x[-1], temp_y[-1]], temp_theta, seg['duracion'], seg['linear_x'], seg['angular_z']
        )
        if i < len(segments) - 1: # No marcar el final si es el último segmento
            plt.plot(next_point[0], next_point[1], 'cx', markersize=10, mew=2) # Punto de transición
        temp_x.append(next_point[0])
        temp_y.append(next_point[1])
        temp_theta = next_theta
    plt.xlabel('Posición X (metros)')
    plt.ylabel('Posición Y (metros)')
    plt.title('Trayectoria del Robot Diferencial')
    plt.grid(True)
    plt.axis('equal') # Escalas iguales en X e Y
    plt.legend()
    plt.show()

# --- Main execution ---
if __name__ == "__main__":
    # Propiedades del robot (ajustar según el robot real)
    radio_rueda_robot = 0.035  # metros
    separacion_ruedas_robot = 0.135  # metros
    robot = RobotDiferencial(radio_rueda_robot, separacion_ruedas_robot)
    # Simular y graficar el camino descrito
    simular_trayectoria_completa(robot)
    # --- Cálculos adicionales de ejercicios anteriores (después de la simulación) ---
    # 1. Trayectoria recta de 1[m] en 10 [s]
    print("\n======== CÁLCULOS: TRAYECTORIA RECTA DE 1m EN 10s ========")
    distancia_recta = 1.0 # metros
    tiempo_recta = 10.0 # segundos
    x_dot_recta = distancia_recta / tiempo_recta # m/s
    theta_dot_recta = 0.0 # rad/s (movimiento recto)
    phi_dot_R_recta, phi_dot_L_recta = robot.cinematica_inversa(x_dot_recta, theta_dot_recta)
    print(f"\nPara una trayectoria recta de {distancia_recta} m en {tiempo_recta} s:")
    print(f"Velocidad lineal requerida del robot (x_dot): {x_dot_recta:.4f} m/s")
    print(f"Velocidad angular requerida del robot (theta_dot): {theta_dot_recta:.4f} rad/s")
    print(f"Velocidad angular requerida rueda derecha (phi_dot_R): {phi_dot_R_recta:.4f} rad/s")
    print(f"Velocidad angular requerida rueda izquierda (phi_dot_L): {phi_dot_L_recta:.4f} rad/s")
    # 2. Trayectoria circular con un radio de 0.5 [m] en sentido horario en 20 [s]
    print("\n======== CÁLCULOS: TRAYECTORIA CIRCULAR (R=0.5m, HORARIO, 20s) ========")
    radio_circulo = 0.5 # metros
    tiempo_circulo = 20.0 # segundos
    # Para una trayectoria circular completa, la distancia lineal es la circunferencia
    distancia_lineal_circulo = 2 * np.pi * radio_circulo # metros
    x_dot_circulo = distancia_lineal_circulo / tiempo_circulo # m/s
    # Para un giro horario completo, el cambio angular es -2*pi radianes
    cambio_angular_circulo = -2 * np.pi # rad (horario)
    theta_dot_circulo = cambio_angular_circulo / tiempo_circulo # rad/s
    phi_dot_R_circulo, phi_dot_L_circulo = robot.cinematica_inversa(x_dot_circulo, theta_dot_circulo)
    print(f"\nPara una trayectoria circular de radio {radio_circulo} m (horario) en {tiempo_circulo} s:")
    print(f"Velocidad lineal requerida del robot (x_dot): {x_dot_circulo:.4f} m/s")
    print(f"Velocidad angular requerida del robot (theta_dot): {theta_dot_circulo:.4f} rad/s")
    print(f"Velocidad angular requerida rueda derecha (phi_dot_R): {phi_dot_R_circulo:.4f} rad/s")
    print(f"Velocidad angular requerida rueda izquierda (phi_dot_L): {phi_dot_L_circulo:.4f} rad/s")
    # Cálculos de velocidad máxima
    print("\n======== CÁLCULOS DE VELOCIDAD MÁXIMA (50 RPM) ========")
    max_motor_rpm = 50.0
    max_phi_dot_rad_s = max_motor_rpm * (2 * np.pi / 60)
    print(f"Velocidad angular máxima de los motores: {max_motor_rpm} RPM = {max_phi_dot_rad_s:.4f} rad/s")
    max_linear_velocity = robot.r * max_phi_dot_rad_s
    print(f"Velocidad lineal máxima del robot: {max_linear_velocity:.4f} m/s")
    max_angular_velocity = (2 * robot.r / robot.b) * max_phi_dot_rad_s
    print(f"Velocidad angular máxima del robot: {max_angular_velocity:.4f} rad/s")