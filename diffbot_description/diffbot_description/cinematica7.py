#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

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
        Calcula la velocidad del robot en su marco de referencia local.
        Args:
            phi_dot_R (float): Velocidad angular de la rueda derecha (rad/s).
            phi_dot_L (float): Velocidad angular de la rueda izquierda (rad/s).
        Returns:
            tuple: (x_dot_local, theta_dot_local)
        """
        x_dot_local = (self.r / 2) * (phi_dot_R + phi_dot_L)
        theta_dot_local = (self.r / self.b) * (phi_dot_R - phi_dot_L)
        return x_dot_local, theta_dot_local

    def velocidad_en_marco_inercial(self, phi_dot_R, phi_dot_L, theta_actual):
        """
        Calcula la velocidad del robot en el marco de referencia inercial (global).
        Args:
            phi_dot_R (float): Velocidad angular de la rueda derecha (rad/s).
            phi_dot_L (float): Velocidad angular de la rueda izquierda (rad/s).
            theta_actual (float): Orientación actual del robot (rad).
        Returns:
            tuple: (x_dot_global, y_dot_global, theta_dot_global)
        """
        x_dot_local, theta_dot_local = self.velocidad_en_marco_local(phi_dot_R, phi_dot_L)
        
        x_dot_global = x_dot_local * np.cos(theta_actual)
        y_dot_global = x_dot_local * np.sin(theta_actual)
        theta_dot_global = theta_dot_local
        
        return x_dot_global, y_dot_global, theta_dot_global

    def simular_movimiento(self, punto_inicial, theta_inicial, duracion_total, velocidad_lineal, velocidad_angular, radio_curva=None):
        """
        Simula el movimiento del robot durante un tiempo determinado.
        Args:
            punto_inicial (tuple): Posición inicial (x, y).
            theta_inicial (float): Orientación inicial (rad).
            duracion_total (float): Duración de la simulación (s).
            velocidad_lineal (float): Velocidad lineal constante (m/s).
            velocidad_angular (float): Velocidad angular constante (rad/s).
            radio_curva (float, optional): Radio de curvatura para referencia.
        Returns:
            tuple: (posiciones_x, posiciones_y, orientaciones, tiempos)
        """
        dt = 0.01  # Paso de tiempo
        num_pasos = int(duracion_total / dt)
        
        # Inicializar arrays
        posiciones_x = [punto_inicial[0]]
        posiciones_y = [punto_inicial[1]]
        orientaciones = [theta_inicial]
        tiempos = [0.0]
        
        # Calcular velocidades angulares de las ruedas
        phi_dot_R, phi_dot_L = self.cinematica_inversa(velocidad_lineal, velocidad_angular)
        
        # Simulación
        for i in range(num_pasos):
            theta_actual = orientaciones[-1]
            
            # Velocidades en marco global
            x_dot_global, y_dot_global, theta_dot_global = self.velocidad_en_marco_inercial(
                phi_dot_R, phi_dot_L, theta_actual
            )
            
            # Integración numérica (Euler)
            nueva_x = posiciones_x[-1] + x_dot_global * dt
            nueva_y = posiciones_y[-1] + y_dot_global * dt
            nueva_theta = orientaciones[-1] + theta_dot_global * dt
            nuevo_tiempo = tiempos[-1] + dt
            
            # Agregar a las listas
            posiciones_x.append(nueva_x)
            posiciones_y.append(nueva_y)
            orientaciones.append(nueva_theta)
            tiempos.append(nuevo_tiempo)
        
        return posiciones_x, posiciones_y, orientaciones, tiempos

class Cinematica7Node(Node):
    def __init__(self):
        super().__init__('cinematica7_node')
        self.get_logger().info('Nodo Cinematica7 iniciado')
        
        # Inicializar robot
        radio_rueda_robot = 0.035  # metros
        separacion_ruedas_robot = 0.135  # metros
        self.robot = RobotDiferencial(radio_rueda_robot, separacion_ruedas_robot)
        
        # Ejecutar simulaciones
        self.simular_trayectoria_recta()
        self.simular_trayectoria_circular()
        self.calculos_velocidad_maxima()

    def simular_trayectoria_recta(self):
        """
        Simula una trayectoria recta de 1 metro en 10 segundos.
        """
        self.get_logger().info("======== SIMULACIÓN: TRAYECTORIA RECTA ========")
        
        # Parámetros de la trayectoria recta
        distancia = 1.0  # metros
        tiempo_total = 10.0  # segundos
        velocidad_lineal = distancia / tiempo_total  # m/s
        velocidad_angular = 0.0  # rad/s (movimiento recto)
        
        # Punto inicial y orientación
        punto_inicial = (0.0, 0.0)
        theta_inicial = 0.0  # radianes
        
        self.get_logger().info(f"Distancia a recorrer: {distancia} m")
        self.get_logger().info(f"Tiempo total: {tiempo_total} s")
        self.get_logger().info(f"Velocidad lineal: {velocidad_lineal:.4f} m/s")
        self.get_logger().info(f"Velocidad angular: {velocidad_angular:.4f} rad/s")
        
        # Calcular velocidades angulares de las ruedas
        phi_dot_R, phi_dot_L = self.robot.cinematica_inversa(velocidad_lineal, velocidad_angular)
        self.get_logger().info(f"Velocidad angular rueda derecha: {phi_dot_R:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular rueda izquierda: {phi_dot_L:.4f} rad/s")
        
        # Simular movimiento
        pos_x, pos_y, orientaciones, tiempos = self.robot.simular_movimiento(
            punto_inicial, theta_inicial, tiempo_total, velocidad_lineal, velocidad_angular
        )
        
        # Graficar trayectoria
        plt.figure(figsize=(12, 8))
        
        # Subplot 1: Trayectoria en el plano XY
        plt.subplot(2, 2, 1)
        plt.plot(pos_x, pos_y, 'b-', linewidth=2, label='Trayectoria')
        plt.plot(pos_x[0], pos_y[0], 'go', markersize=8, label='Inicio')
        plt.plot(pos_x[-1], pos_y[-1], 'ro', markersize=8, label='Final')
        plt.xlabel('Posición X [m]')
        plt.ylabel('Posición Y [m]')
        plt.title('Trayectoria Recta en el Plano XY')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # Subplot 2: Posición X vs Tiempo
        plt.subplot(2, 2, 2)
        plt.plot(tiempos, pos_x, 'r-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Posición X [m]')
        plt.title('Posición X vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        # Subplot 3: Posición Y vs Tiempo
        plt.subplot(2, 2, 3)
        plt.plot(tiempos, pos_y, 'g-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Posición Y [m]')
        plt.title('Posición Y vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        # Subplot 4: Orientación vs Tiempo
        plt.subplot(2, 2, 4)
        orientaciones_grados = [np.degrees(theta) for theta in orientaciones]
        plt.plot(tiempos, orientaciones_grados, 'm-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Orientación [°]')
        plt.title('Orientación vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Resultados finales
        self.get_logger().info(f"Posición final: ({pos_x[-1]:.4f}, {pos_y[-1]:.4f}) m")
        self.get_logger().info(f"Orientación final: {orientaciones[-1]:.4f} rad ({np.degrees(orientaciones[-1]):.2f}°)")
        self.get_logger().info(f"Distancia real recorrida: {pos_x[-1]:.4f} m")

    def simular_trayectoria_circular(self):
        """
        Simula una trayectoria circular con radio de 0.5 metros en sentido horario durante 20 segundos.
        """
        self.get_logger().info("\n======== SIMULACIÓN: TRAYECTORIA CIRCULAR ========")
        
        # Parámetros de la trayectoria circular
        radio_circulo = 0.5  # metros
        tiempo_total = 20.0  # segundos
        
        # Calcular velocidades
        circunferencia = 2 * np.pi * radio_circulo
        velocidad_lineal = circunferencia / tiempo_total  # m/s
        velocidad_angular = -2 * np.pi / tiempo_total  # rad/s (negativo para sentido horario)
        
        # Punto inicial y orientación
        punto_inicial = (0.0, 0.0)
        theta_inicial = 0.0  # radianes
        
        self.get_logger().info(f"Radio del círculo: {radio_circulo} m")
        self.get_logger().info(f"Tiempo total: {tiempo_total} s")
        self.get_logger().info(f"Velocidad lineal: {velocidad_lineal:.4f} m/s")
        self.get_logger().info(f"Velocidad angular: {velocidad_angular:.4f} rad/s")
        
        # Calcular velocidades angulares de las ruedas
        phi_dot_R, phi_dot_L = self.robot.cinematica_inversa(velocidad_lineal, velocidad_angular)
        self.get_logger().info(f"Velocidad angular rueda derecha: {phi_dot_R:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular rueda izquierda: {phi_dot_L:.4f} rad/s")
        
        # Simular movimiento
        pos_x, pos_y, orientaciones, tiempos = self.robot.simular_movimiento(
            punto_inicial, theta_inicial, tiempo_total, velocidad_lineal, velocidad_angular, radio_circulo
        )
        
        # Graficar trayectoria
        plt.figure(figsize=(12, 8))
        
        # Subplot 1: Trayectoria en el plano XY
        plt.subplot(2, 2, 1)
        plt.plot(pos_x, pos_y, 'b-', linewidth=2, label='Trayectoria')
        plt.plot(pos_x[0], pos_y[0], 'go', markersize=8, label='Inicio')
        plt.plot(pos_x[-1], pos_y[-1], 'ro', markersize=8, label='Final')
        
        # Dibujar círculo teórico para comparación
        theta_teorico = np.linspace(0, 2*np.pi, 100)
        x_teorico = radio_circulo * np.cos(theta_teorico)
        y_teorico = -radio_circulo * np.sin(theta_teorico)  # Negativo para sentido horario
        y_teorico += radio_circulo  # Desplazar para que empiece en (0,0)
        plt.plot(x_teorico, y_teorico, 'r--', alpha=0.5, label='Círculo teórico')
        
        plt.xlabel('Posición X [m]')
        plt.ylabel('Posición Y [m]')
        plt.title('Trayectoria Circular en el Plano XY')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # Subplot 2: Posición X vs Tiempo
        plt.subplot(2, 2, 2)
        plt.plot(tiempos, pos_x, 'r-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Posición X [m]')
        plt.title('Posición X vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        # Subplot 3: Posición Y vs Tiempo
        plt.subplot(2, 2, 3)
        plt.plot(tiempos, pos_y, 'g-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Posición Y [m]')
        plt.title('Posición Y vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        # Subplot 4: Orientación vs Tiempo
        plt.subplot(2, 2, 4)
        orientaciones_grados = [np.degrees(theta) for theta in orientaciones]
        plt.plot(tiempos, orientaciones_grados, 'm-', linewidth=2)
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Orientación [°]')
        plt.title('Orientación vs Tiempo')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        # Resultados finales
        self.get_logger().info(f"Posición final: ({pos_x[-1]:.4f}, {pos_y[-1]:.4f}) m")
        self.get_logger().info(f"Orientación final: {orientaciones[-1]:.4f} rad ({np.degrees(orientaciones[-1]):.2f}°)")
        
        # Calcular radio real de la trayectoria
        centro_x = np.mean(pos_x)
        centro_y = np.mean(pos_y)
        radio_real = np.sqrt((pos_x[0] - centro_x)**2 + (pos_y[0] - centro_y)**2)
        self.get_logger().info(f"Radio real de la trayectoria: {radio_real:.4f} m")
        self.get_logger().info(f"Error en el radio: {abs(radio_real - radio_circulo):.4f} m")

    def calculos_velocidad_maxima(self):
        """
        Calcula las velocidades máximas del robot basándose en la velocidad máxima de los motores.
        """
        self.get_logger().info("\n======== CÁLCULOS DE VELOCIDAD MÁXIMA ========")
        
        # Velocidad máxima de los motores (50 RPM)
        max_motor_rpm = 50.0
        max_phi_dot_rad_s = max_motor_rpm * (2 * np.pi / 60)  # Conversión a rad/s
        
        self.get_logger().info(f"Velocidad angular máxima de los motores: {max_motor_rpm} RPM = {max_phi_dot_rad_s:.4f} rad/s")
        
        # Velocidad lineal máxima (ambas ruedas girando hacia adelante a máxima velocidad)
        max_linear_velocity = self.robot.r * max_phi_dot_rad_s
        self.get_logger().info(f"Velocidad lineal máxima del robot: {max_linear_velocity:.4f} m/s")
        
        # Velocidad angular máxima (una rueda hacia adelante, otra hacia atrás, ambas a máxima velocidad)
        max_angular_velocity = (2 * self.robot.r / self.robot.b) * max_phi_dot_rad_s
        self.get_logger().info(f"Velocidad angular máxima del robot: {max_angular_velocity:.4f} rad/s")
        
        # Radio mínimo de giro a máxima velocidad lineal
        min_turn_radius = max_linear_velocity / max_angular_velocity
        self.get_logger().info(f"Radio mínimo de giro: {min_turn_radius:.4f} m")
        
        # Ejemplos de combinaciones de velocidades
        self.get_logger().info("\n--- Ejemplos de combinaciones de velocidades ---")
        
        # Ejemplo 1: 75% velocidad lineal máxima
        v_linear_75 = 0.75 * max_linear_velocity
        max_angular_at_75_linear = (max_phi_dot_rad_s - v_linear_75/self.robot.r) * (2*self.robot.r/self.robot.b)
        self.get_logger().info(f"Al 75% de velocidad lineal máxima ({v_linear_75:.4f} m/s):")
        self.get_logger().info(f"  Velocidad angular máxima disponible: {max_angular_at_75_linear:.4f} rad/s")
        
        # Ejemplo 2: 50% velocidad lineal máxima
        v_linear_50 = 0.50 * max_linear_velocity
        max_angular_at_50_linear = (max_phi_dot_rad_s - v_linear_50/self.robot.r) * (2*self.robot.r/self.robot.b)
        self.get_logger().info(f"Al 50% de velocidad lineal máxima ({v_linear_50:.4f} m/s):")
        self.get_logger().info(f"  Velocidad angular máxima disponible: {max_angular_at_50_linear:.4f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = Cinematica7Node()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()