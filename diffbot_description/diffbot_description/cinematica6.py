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

class Cinematica6Node(Node):
    def __init__(self):
        super().__init__('cinematica6_node')
        self.get_logger().info('Nodo Cinematica6 iniciado')
        
        # Inicializar robot
        radio_rueda_robot = 0.035  # metros
        separacion_ruedas_robot = 0.135  # metros
        self.robot = RobotDiferencial(radio_rueda_robot, separacion_ruedas_robot)
        
        # Ejecutar simulaciones
        self.simular_trayectoria_completa()
        self.calculos_adicionales()

    def simular_trayectoria_completa(self):
        """
        Simula una trayectoria completa con diferentes segmentos.
        """
        self.get_logger().info("======== SIMULACIÓN DE TRAYECTORIA COMPLETA ========")
        
        # Configuración de la simulación
        punto_inicial = (0.0, 0.0)
        theta_inicial = 0.0  # Orientación inicial (radianes)
        
        # Segmento 1: Movimiento recto hacia adelante
        self.get_logger().info("\n--- Segmento 1: Movimiento Recto (2 segundos) ---")
        velocidad_lineal_1 = 0.2  # m/s
        velocidad_angular_1 = 0.0  # rad/s
        duracion_1 = 2.0  # segundos
        
        pos_x_1, pos_y_1, orient_1, tiempo_1 = self.robot.simular_movimiento(
            punto_inicial, theta_inicial, duracion_1, velocidad_lineal_1, velocidad_angular_1
        )
        
        # Segmento 2: Giro en el lugar (sentido antihorario)
        self.get_logger().info("--- Segmento 2: Giro en el Lugar (1.57 segundos) ---")
        punto_inicial_2 = (pos_x_1[-1], pos_y_1[-1])
        theta_inicial_2 = orient_1[-1]
        velocidad_lineal_2 = 0.0  # m/s
        velocidad_angular_2 = 1.0  # rad/s (antihorario)
        duracion_2 = 1.57  # segundos (aproximadamente π/2 radianes)
        
        pos_x_2, pos_y_2, orient_2, tiempo_2 = self.robot.simular_movimiento(
            punto_inicial_2, theta_inicial_2, duracion_2, velocidad_lineal_2, velocidad_angular_2
        )
        
        # Segmento 3: Movimiento recto hacia adelante (nueva dirección)
        self.get_logger().info("--- Segmento 3: Movimiento Recto en Nueva Dirección (2 segundos) ---")
        punto_inicial_3 = (pos_x_2[-1], pos_y_2[-1])
        theta_inicial_3 = orient_2[-1]
        velocidad_lineal_3 = 0.2  # m/s
        velocidad_angular_3 = 0.0  # rad/s
        duracion_3 = 2.0  # segundos
        
        pos_x_3, pos_y_3, orient_3, tiempo_3 = self.robot.simular_movimiento(
            punto_inicial_3, theta_inicial_3, duracion_3, velocidad_lineal_3, velocidad_angular_3
        )
        
        # Segmento 4: Movimiento circular
        self.get_logger().info("--- Segmento 4: Movimiento Circular (3.14 segundos) ---")
        punto_inicial_4 = (pos_x_3[-1], pos_y_3[-1])
        theta_inicial_4 = orient_3[-1]
        velocidad_lineal_4 = 0.15  # m/s
        velocidad_angular_4 = 0.5   # rad/s
        duracion_4 = 3.14  # segundos (aproximadamente π radianes)
        radio_curva_4 = velocidad_lineal_4 / velocidad_angular_4  # Radio de curvatura
        
        pos_x_4, pos_y_4, orient_4, tiempo_4 = self.robot.simular_movimiento(
            punto_inicial_4, theta_inicial_4, duracion_4, velocidad_lineal_4, velocidad_angular_4, radio_curva_4
        )
        
        # Combinar todos los segmentos para graficar
        pos_x_total = pos_x_1 + pos_x_2[1:] + pos_x_3[1:] + pos_x_4[1:]
        pos_y_total = pos_y_1 + pos_y_2[1:] + pos_y_3[1:] + pos_y_4[1:]
        
        # Graficar la trayectoria completa
        plt.figure(figsize=(12, 10))
        
        # Trayectoria completa
        plt.plot(pos_x_total, pos_y_total, 'b-', linewidth=2, label='Trayectoria Completa')
        
        # Segmentos individuales con colores diferentes
        plt.plot(pos_x_1, pos_y_1, 'g-', linewidth=3, alpha=0.7, label='Segmento 1: Recto')
        plt.plot(pos_x_2, pos_y_2, 'r-', linewidth=3, alpha=0.7, label='Segmento 2: Giro')
        plt.plot(pos_x_3, pos_y_3, 'm-', linewidth=3, alpha=0.7, label='Segmento 3: Recto')
        plt.plot(pos_x_4, pos_y_4, 'c-', linewidth=3, alpha=0.7, label='Segmento 4: Circular')
        
        # Puntos de inicio y fin
        plt.plot(pos_x_total[0], pos_y_total[0], 'go', markersize=10, label='Inicio')
        plt.plot(pos_x_total[-1], pos_y_total[-1], 'ro', markersize=10, label='Final')
        
        # Puntos de transición entre segmentos
        plt.plot(pos_x_1[-1], pos_y_1[-1], 'ks', markersize=8, label='Transiciones')
        plt.plot(pos_x_2[-1], pos_y_2[-1], 'ks', markersize=8)
        plt.plot(pos_x_3[-1], pos_y_3[-1], 'ks', markersize=8)
        
        # Configuración del gráfico
        plt.xlabel('Posición X [m]')
        plt.ylabel('Posición Y [m]')
        plt.title('Trayectoria Completa del Robot Diferencial')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        plt.tight_layout()
        plt.show()
        
        self.get_logger().info(f"Posición final: ({pos_x_total[-1]:.3f}, {pos_y_total[-1]:.3f}) m")
        self.get_logger().info(f"Orientación final: {orient_4[-1]:.3f} rad ({np.degrees(orient_4[-1]):.1f}°)")

    def calculos_adicionales(self):
        """Realiza cálculos adicionales como en el ejercicio original"""
        
        self.get_logger().info("\n======== CÁLCULOS: TRAYECTORIA RECTA DE 1m EN 10s ========")
        distancia_recta = 1.0 # metros
        tiempo_recta = 10.0 # segundos
        x_dot_recta = distancia_recta / tiempo_recta # m/s
        theta_dot_recta = 0.0 # rad/s (movimiento recto)
        phi_dot_R_recta, phi_dot_L_recta = self.robot.cinematica_inversa(x_dot_recta, theta_dot_recta)
        self.get_logger().info(f"\nPara una trayectoria recta de {distancia_recta} m en {tiempo_recta} s:")
        self.get_logger().info(f"Velocidad lineal requerida del robot (x_dot): {x_dot_recta:.4f} m/s")
        self.get_logger().info(f"Velocidad angular requerida del robot (theta_dot): {theta_dot_recta:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular requerida rueda derecha (phi_dot_R): {phi_dot_R_recta:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular requerida rueda izquierda (phi_dot_L): {phi_dot_L_recta:.4f} rad/s")
        
        self.get_logger().info("\n======== CÁLCULOS: TRAYECTORIA CIRCULAR (R=0.5m, HORARIO, 20s) ========")
        radio_circulo = 0.5 # metros
        tiempo_circulo = 20.0 # segundos
        
        distancia_lineal_circulo = 2 * np.pi * radio_circulo # metros
        x_dot_circulo = distancia_lineal_circulo / tiempo_circulo # m/s
        
        cambio_angular_circulo = -2 * np.pi # rad (horario)
        theta_dot_circulo = cambio_angular_circulo / tiempo_circulo # rad/s
        phi_dot_R_circulo, phi_dot_L_circulo = self.robot.cinematica_inversa(x_dot_circulo, theta_dot_circulo)
        self.get_logger().info(f"\nPara una trayectoria circular de radio {radio_circulo} m (horario) en {tiempo_circulo} s:")
        self.get_logger().info(f"Velocidad lineal requerida del robot (x_dot): {x_dot_circulo:.4f} m/s")
        self.get_logger().info(f"Velocidad angular requerida del robot (theta_dot): {theta_dot_circulo:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular requerida rueda derecha (phi_dot_R): {phi_dot_R_circulo:.4f} rad/s")
        self.get_logger().info(f"Velocidad angular requerida rueda izquierda (phi_dot_L): {phi_dot_L_circulo:.4f} rad/s")
        
        self.get_logger().info("\n======== CÁLCULOS DE VELOCIDAD MÁXIMA (50 RPM) ========")
        max_motor_rpm = 50.0
        max_phi_dot_rad_s = max_motor_rpm * (2 * np.pi / 60)
        self.get_logger().info(f"Velocidad angular máxima de los motores: {max_motor_rpm} RPM = {max_phi_dot_rad_s:.4f} rad/s")
        max_linear_velocity = self.robot.r * max_phi_dot_rad_s
        self.get_logger().info(f"Velocidad lineal máxima del robot: {max_linear_velocity:.4f} m/s")
        max_angular_velocity = (2 * self.robot.r / self.robot.b) * max_phi_dot_rad_s
        self.get_logger().info(f"Velocidad angular máxima del robot: {max_angular_velocity:.4f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = Cinematica6Node()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()