
Crear Proyecto nuevo:
    cd ./src && ros2 pkg create --build-type ament_python clase_04 && cd ..

Istalar las librerias utilizadas:
    rosdep install -i --from-path src -y

Compilar:
    colcon build
    colcon build --packages-select clase_06


Agregar Libreria a Ros2:
    source install/setup.bash


____________________________________________________________________________________

Listar paquete con filtro:
    ros2 pkg executables | grep clase_04
    
    
Ejecutar el Proyecto:
    ros2 run clase_04 

Ver Nodos:
    ros2 node list

Ver La info de un nodo:
    ros2 node info "node_name"

Listar los topics:
    ros2 topic list

Informacion de un topic:
    ros2 topic info "topic_name"

Ver los datos de un topic:
    ros2 topic echo "topic_name"



____________________________________________________________________________________
Escaleas de mensajes:
    nodo.get_logger().info('Publique el mensaje "%s"' % msg.data)
    
    FATAL
    ERROR
    WARN
    INFO
    DEBUG
    
Tipos de mensajes de tiempo:
    once: Solo una ves
    self.get_logger().warn('Advertencia una sola vez', once=True)
    skip_first: No lo veo la primera vez
    throttle_duration_sec: La cantidad de veces que se puede enviar el mensaje por segundo (Cantidad de mensajes por segundo)
    self.get_logger().debug(f'Valor de la medicion {valor}', throttle_duration_sec=1)

____________________________________________________________________________________
Crear paquete archvio launch:
    mkdir ./launch && cd ./launch && touch {name}.launch.py 
    touch ./src/clase_06/launch/{name}.launch.py 
