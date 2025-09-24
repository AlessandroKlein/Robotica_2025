from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# =========================================================
#   Archivo: example_calibrated_params.launch.py
#   Resumen:
#   Este archivo demuestra cómo usar el sistema de odometría calibrado
#   con parámetros de ejemplo calculados mediante el proceso de calibración.
#   
#   Para usar este archivo con tus propios parámetros de calibración:
#   1. Ejecuta el proceso de calibración completo usando:
#      - square_trajectory.py
#      - calibration_data_collector.py  
#      - calibration_calculator.py
#   2. Reemplaza los valores de ejemplo con los calculados
#   3. Lanza este archivo para usar el sistema calibrado
# =========================================================

def generate_launch_description():
    # Ejemplo de parámetros de calibración calculados
    # NOTA: Estos son valores de ejemplo. Reemplaza con los valores reales
    # obtenidos del proceso de calibración usando calibration_calculator.py
    
    # Parámetros de ejemplo (reemplazar con valores reales):
    example_c_L = "0.98"      # Coeficiente de corrección rueda izquierda
    example_c_R = "1.02"      # Coeficiente de corrección rueda derecha  
    example_b_actual = "0.138" # Separación corregida entre ruedas (m)
    
    # Incluir el launch file principal con parámetros de calibración
    calibrated_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("diffbot_gz"),
                "launch",
                "gzodom_calibrated.launch.py"
            ])
        ),
        launch_arguments={
            "name": "diffbot",
            "use_sim_time": "true",
            "wheel_radius": "0.035",
            "wheel_separation": "0.135",
            "c_L": example_c_L,
            "c_R": example_c_R,
            "b_actual": example_b_actual,
            "left_wheel_joint_name": "left_wheel_joint",
            "right_wheel_joint_name": "right_wheel_joint",
        }.items(),
    )

    return LaunchDescription([
        calibrated_launch,
    ])