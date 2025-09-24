#!/usr/bin/env python3
"""
Calculadora de parámetros de calibración de odometría.

Este script lee los datos de calibración recolectados y calcula los parámetros
de corrección según las fórmulas del laboratorio.

Autor: Sistema de calibración de odometría
Fecha: 2025
"""

import json
import math
import argparse
import sys


class CalibrationCalculator:
    def __init__(self, data_file, wheel_base=0.135, wheel_radius=0.035):
        """
        Inicializa la calculadora de calibración.
        
        Args:
            data_file: Archivo JSON con los datos de calibración
            wheel_base: Separación entre ruedas (b) en metros
            wheel_radius: Radio de las ruedas en metros
        """
        self.data_file = data_file
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.calibration_data = None
        
        # Cargar datos
        self.load_data()
    
    def load_data(self):
        """Carga los datos de calibración desde el archivo JSON."""
        try:
            with open(self.data_file, 'r') as f:
                self.calibration_data = json.load(f)
            print(f"Datos cargados desde: {self.data_file}")
        except FileNotFoundError:
            print(f"Error: No se encontró el archivo {self.data_file}")
            sys.exit(1)
        except json.JSONDecodeError:
            print(f"Error: El archivo {self.data_file} no es un JSON válido")
            sys.exit(1)
    
    def calculate_centroids(self):
        """Calcula los centroides de los errores para cada dirección."""
        # Errores CCW
        ccw_runs = self.calibration_data.get('ccw_runs', [])
        if not ccw_runs:
            print("Error: No hay datos de corridas CCW")
            return None, None, None, None
        
        ccw_errors_x = [run['error_x'] for run in ccw_runs]
        ccw_errors_y = [run['error_y'] for run in ccw_runs]
        ccw_mean_x = sum(ccw_errors_x) / len(ccw_errors_x)
        ccw_mean_y = sum(ccw_errors_y) / len(ccw_errors_y)
        
        # Errores CW
        cw_runs = self.calibration_data.get('cw_runs', [])
        if not cw_runs:
            print("Error: No hay datos de corridas CW")
            return None, None, None, None
        
        cw_errors_x = [run['error_x'] for run in cw_runs]
        cw_errors_y = [run['error_y'] for run in cw_runs]
        cw_mean_x = sum(cw_errors_x) / len(cw_errors_x)
        cw_mean_y = sum(cw_errors_y) / len(cw_errors_y)
        
        print("=== CENTROIDES CALCULADOS ===")
        print(f"CCW - Media εx: {ccw_mean_x:.6f} m")
        print(f"CCW - Media εy: {ccw_mean_y:.6f} m")
        print(f"CW - Media εx: {cw_mean_x:.6f} m")
        print(f"CW - Media εy: {cw_mean_y:.6f} m")
        
        return ccw_mean_x, ccw_mean_y, cw_mean_x, cw_mean_y
    
    def calculate_calibration_parameters(self):
        """Calcula los parámetros de calibración según las fórmulas del laboratorio."""
        # Obtener longitud del lado del cuadrado
        L = self.calibration_data['metadata']['side_length']
        print(f"\nLongitud del lado del cuadrado (L): {L} m")
        print(f"Separación entre ruedas nominal (b): {self.wheel_base} m")
        print(f"Radio de ruedas (r): {self.wheel_radius} m")
        
        # Calcular centroides
        ccw_mean_x, ccw_mean_y, cw_mean_x, cw_mean_y = self.calculate_centroids()
        
        if any(x is None for x in [ccw_mean_x, ccw_mean_y, cw_mean_x, cw_mean_y]):
            return None
        
        print("\n=== CÁLCULO DE PARÁMETROS ===")
        
        # Calcular α (alpha)
        # α = (CW_x̄ + CCW_x̄) / (-4L) o α = (CW_ȳ - CCW_ȳ) / (-4L)
        alpha_from_x = (cw_mean_x + ccw_mean_x) / (-4 * L)
        alpha_from_y = (cw_mean_y - ccw_mean_y) / (-4 * L)
        
        print(f"α (desde x): {alpha_from_x:.8f} rad")
        print(f"α (desde y): {alpha_from_y:.8f} rad")
        
        # Usar el promedio de ambos cálculos
        alpha = (alpha_from_x + alpha_from_y) / 2
        print(f"α (promedio): {alpha:.8f} rad")
        
        # Calcular E_b
        # E_b = (π/2) / (π/2 - α)
        E_b = (math.pi / 2) / (math.pi / 2 - alpha)
        print(f"E_b: {E_b:.8f}")
        
        # Calcular β (beta)
        # β = (CW_x̄ - CCW_x̄) / (-4L) o β = (CW_ȳ + CCW_ȳ) / (-4L)
        beta_from_x = (cw_mean_x - ccw_mean_x) / (-4 * L)
        beta_from_y = (cw_mean_y + ccw_mean_y) / (-4 * L)
        
        print(f"β (desde x): {beta_from_x:.8f} rad")
        print(f"β (desde y): {beta_from_y:.8f} rad")
        
        # Usar el promedio de ambos cálculos
        beta = (beta_from_x + beta_from_y) / 2
        print(f"β (promedio): {beta:.8f} rad")
        
        # Calcular R
        # R = (L/2) / sin(β/2)
        if abs(beta) < 1e-10:
            print("Advertencia: β es muy pequeño, usando aproximación lineal")
            R = L / abs(beta) if abs(beta) > 0 else float('inf')
        else:
            R = (L / 2) / math.sin(beta / 2)
        print(f"R: {R:.6f} m")
        
        # Calcular E_d
        # E_d = (R + b/2) / (R - b/2)
        if abs(R - self.wheel_base/2) < 1e-10:
            print("Error: R ≈ b/2, división por cero")
            return None
        
        E_d = (R + self.wheel_base/2) / (R - self.wheel_base/2)
        print(f"E_d: {E_d:.8f}")
        
        # Calcular coeficientes de corrección
        # c_L = 2 / (E_d + 1)
        # c_R = 2 / (1/E_d + 1)
        c_L = 2 / (E_d + 1)
        c_R = 2 / (1/E_d + 1)
        
        print(f"c_L: {c_L:.8f}")
        print(f"c_R: {c_R:.8f}")
        
        # Calcular separación corregida
        b_actual = E_b * self.wheel_base
        print(f"b_actual: {b_actual:.6f} m")
        
        # Resumen de parámetros
        parameters = {
            'alpha': alpha,
            'beta': beta,
            'E_b': E_b,
            'E_d': E_d,
            'c_L': c_L,
            'c_R': c_R,
            'b_nominal': self.wheel_base,
            'b_actual': b_actual,
            'wheel_radius': self.wheel_radius
        }
        
        return parameters
    
    def save_parameters(self, parameters, output_file='calibration_parameters.json'):
        """Guarda los parámetros calculados en un archivo JSON."""
        if parameters is None:
            print("Error: No se pudieron calcular los parámetros")
            return
        
        # Agregar metadatos
        output_data = {
            'metadata': {
                'source_file': self.data_file,
                'wheel_base_nominal': self.wheel_base,
                'wheel_radius': self.wheel_radius,
                'calculation_method': 'UMBmark_method'
            },
            'parameters': parameters
        }
        
        try:
            with open(output_file, 'w') as f:
                json.dump(output_data, f, indent=2)
            print(f"\nParámetros guardados en: {output_file}")
        except Exception as e:
            print(f"Error al guardar parámetros: {e}")
    
    def print_summary(self, parameters):
        """Imprime un resumen de los parámetros calculados."""
        if parameters is None:
            return
        
        print("\n" + "="*50)
        print("RESUMEN DE PARÁMETROS DE CALIBRACIÓN")
        print("="*50)
        print(f"Separación nominal (b):     {parameters['b_nominal']:.6f} m")
        print(f"Separación corregida:       {parameters['b_actual']:.6f} m")
        print(f"Factor de corrección E_b:   {parameters['E_b']:.8f}")
        print(f"Factor de corrección E_d:   {parameters['E_d']:.8f}")
        print(f"Coeficiente rueda izq (c_L): {parameters['c_L']:.8f}")
        print(f"Coeficiente rueda der (c_R): {parameters['c_R']:.8f}")
        print("="*50)
        
        print("\nPARA USAR EN ROS2 LAUNCH:")
        print(f"  - c_L: {parameters['c_L']:.8f}")
        print(f"  - c_R: {parameters['c_R']:.8f}")
        print(f"  - b_actual: {parameters['b_actual']:.6f}")


def main():
    parser = argparse.ArgumentParser(description='Calcula parámetros de calibración de odometría')
    parser.add_argument('data_file', help='Archivo JSON con datos de calibración')
    parser.add_argument('--wheel-base', type=float, default=0.135, 
                       help='Separación entre ruedas en metros (default: 0.135)')
    parser.add_argument('--wheel-radius', type=float, default=0.035,
                       help='Radio de las ruedas en metros (default: 0.035)')
    parser.add_argument('--output', '-o', default='calibration_parameters.json',
                       help='Archivo de salida para los parámetros (default: calibration_parameters.json)')
    
    args = parser.parse_args()
    
    # Crear calculadora
    calculator = CalibrationCalculator(args.data_file, args.wheel_base, args.wheel_radius)
    
    # Calcular parámetros
    parameters = calculator.calculate_calibration_parameters()
    
    if parameters:
        # Mostrar resumen
        calculator.print_summary(parameters)
        
        # Guardar parámetros
        calculator.save_parameters(parameters, args.output)
    else:
        print("Error: No se pudieron calcular los parámetros de calibración")
        sys.exit(1)


if __name__ == '__main__':
    main()