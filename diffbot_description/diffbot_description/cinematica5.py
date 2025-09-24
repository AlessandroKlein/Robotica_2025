#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import matplotlib.pyplot as plt
import numpy as np

class Cinematica5Node(Node):
    def __init__(self):
        super().__init__('cinematica5_node')
        self.get_logger().info('Nodo Cinematica5 iniciado')
        
        # Parámetros del robot diferencial
        # Datos del robot (obtenidos de la URL proporcionada)
        # Fuente: https://acapovilla.github.io/robotica-2025/clases/10/tres.html
        self.r = 0.035  # Radio de las ruedas [m]
        self.b = 0.135  # Separación entre las ruedas [m]
        
        # Ejecutar simulaciones
        self.ejecutar_simulaciones()

    def simular_y_graficar_recorrido(self, x_dot, theta_dot, duracion_total, titulo="Recorrido del Robot"):
        """
        Simula el recorrido del robot diferencial y lo grafica en 2D.
        Args:
            x_dot: Velocidad lineal del robot [m/s]
            theta_dot: Velocidad angular del robot [rad/s]
            duracion_total: Duración total de la trayectoria [s]
            titulo: Título para el gráfico
        """
        dt = 0.01  # Paso de tiempo para la simulación [s]
        num_pasos = int(duracion_total / dt)

        # Listas para almacenar la pose del robot en cada instante
        x_pos = [0.0]
        y_pos = [0.0]
        theta_orient = [0.0] # Orientación inicial del robot (apuntando a lo largo del eje X)

        # Simulación del movimiento (integración de la velocidad)
        for i in range(num_pasos):
            current_theta = theta_orient[-1]

            # Velocidades lineales en el marco global
            global_x_dot = x_dot * math.cos(current_theta)
            global_y_dot = x_dot * math.sin(current_theta)
            global_theta_dot = theta_dot

            # Integración numérica (Euler)
            new_x = x_pos[-1] + global_x_dot * dt
            new_y = y_pos[-1] + global_y_dot * dt
            new_theta = theta_orient[-1] + global_theta_dot * dt

            # Agregar a las listas
            x_pos.append(new_x)
            y_pos.append(new_y)
            theta_orient.append(new_theta)

        # Graficar la trayectoria
        plt.figure(figsize=(10, 8))
        plt.plot(x_pos, y_pos, 'b-', linewidth=2, label='Trayectoria del robot')
        plt.plot(x_pos[0], y_pos[0], 'go', markersize=8, label='Inicio')
        plt.plot(x_pos[-1], y_pos[-1], 'ro', markersize=8, label='Final')

        # Dibujar algunas flechas para mostrar la orientación
        num_arrows = 10
        step_arrows = len(x_pos) // num_arrows
        for i in range(0, len(x_pos), step_arrows):
            if i < len(x_pos):
                arrow_length = 0.05
                dx = arrow_length * math.cos(theta_orient[i])
                dy = arrow_length * math.sin(theta_orient[i])
                plt.arrow(x_pos[i], y_pos[i], dx, dy, head_width=0.02, head_length=0.01, fc='red', ec='red')

        plt.xlabel('Posición X [m]')
        plt.ylabel('Posición Y [m]')
        plt.title(titulo)
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        plt.tight_layout()
        plt.show()

    def ejecutar_simulaciones(self):
        """Ejecuta las simulaciones de trayectoria recta y circular"""
        
        self.get_logger().info("--- Cálculo para Trayectoria Recta ---")
        
        # Parámetros para trayectoria recta
        distancia_recta = 1.0  # metros
        tiempo_recta = 10.0    # segundos
        
        # Cálculo de velocidades para trayectoria recta
        x_dot_recta = distancia_recta / tiempo_recta  # [m/s]
        theta_dot_recta = 0.0                      # [rad/s] (para movimiento recto)
        
        # Cálculo de velocidades angulares de las ruedas (cinemática inversa)
        phi_R_dot_recta = (1 / self.r) * (x_dot_recta + (self.b / 2) * theta_dot_recta)  # [rad/s]
        phi_L_dot_recta = (1 / self.r) * (x_dot_recta - (self.b / 2) * theta_dot_recta)  # [rad/s]
        
        # Velocidades lineales de las ruedas
        v_R_recta = phi_R_dot_recta * self.r  # [m/s]
        v_L_recta = phi_L_dot_recta * self.r  # [m/s]
        
        self.get_logger().info(f"Velocidad lineal del robot (x_dot): {x_dot_recta:.4f} [m/s]")
        self.get_logger().info(f"Velocidad angular del robot (theta_dot): {theta_dot_recta:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad angular rueda derecha (phi_R_dot): {phi_R_dot_recta:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad angular rueda izquierda (phi_L_dot): {phi_L_dot_recta:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad lineal rueda derecha (v_R): {v_R_recta:.4f} [m/s]")
        self.get_logger().info(f"Velocidad lineal rueda izquierda (v_L): {v_L_recta:.4f} [m/s]")
        
        # Simular y graficar trayectoria recta
        self.simular_y_graficar_recorrido(x_dot_recta, theta_dot_recta, tiempo_recta, "Recorrido Recto del Robot (1m en 10s)")
        
        self.get_logger().info("--- Cálculo para Trayectoria Circular (sentido horario) ---")
        
        # Parámetros para trayectoria circular
        radio_circular = 0.5  # metros
        tiempo_circular = 20.0 # segundos
        
        # Cálculo de velocidades para trayectoria circular
        circunferencia = 2 * math.pi * radio_circular  # [m]
        x_dot_circular = circunferencia / tiempo_circular  # [m/s]
        theta_dot_circular = - (2 * math.pi / tiempo_circular) # [rad/s] (negativo para sentido horario)
        
        # Cálculo de velocidades angulares de las ruedas (cinemática inversa)
        phi_R_dot_circular = (1 / self.r) * (x_dot_circular + (self.b / 2) * theta_dot_circular)  # [rad/s]
        phi_L_dot_circular = (1 / self.r) * (x_dot_circular - (self.b / 2) * theta_dot_circular)  # [rad/s]
        
        # Velocidades lineales de las ruedas
        v_R_circular = phi_R_dot_circular * self.r  # [m/s]
        v_L_circular = phi_L_dot_circular * self.r  # [m/s]
        
        self.get_logger().info(f"Velocidad lineal del robot (x_dot): {x_dot_circular:.4f} [m/s]")
        self.get_logger().info(f"Velocidad angular del robot (theta_dot): {theta_dot_circular:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad angular rueda derecha (phi_R_dot): {phi_R_dot_circular:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad angular rueda izquierda (phi_L_dot): {phi_L_dot_circular:.4f} [rad/s]")
        self.get_logger().info(f"Velocidad lineal rueda derecha (v_R): {v_R_circular:.4f} [m/s]")
        self.get_logger().info(f"Velocidad lineal rueda izquierda (v_L): {v_L_circular:.4f} [m/s]")
        
        # Simular y graficar trayectoria circular
        self.simular_y_graficar_recorrido(x_dot_circular, theta_dot_circular, tiempo_circular, "Recorrido Circular Horario del Robot (Radio 0.5m en 20s)")

def main(args=None):
    rclpy.init(args=args)
    node = Cinematica5Node()
    
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()