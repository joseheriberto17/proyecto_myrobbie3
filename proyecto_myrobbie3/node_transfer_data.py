# librerias disponible
import rclpy
from math import sin,cos,atan,radians,degrees,pi
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# tipo de mensaje a publicar
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

# procesamiento de cadenas de texto
import re



# creacion del nodo
class Node_Transfer_Data(Node):

    def __init__(self):
        super().__init__('node_transfer_data')

        # creacion del topico a publicar
        # topico para publicar la velocidad de las ruedas
        self.publisher_String_cmd_vel = self.create_publisher(String, 'ros_sub_1', 10)

        # topico para publicar el valor de la distancia de cada sensor TOF
        self.publisher_Laser_Scan_1 = self.create_publisher(Range, 'sensor_tof_1/range', 10)
        self.publisher_Laser_Scan_2 = self.create_publisher(Range, 'sensor_tof_2/range', 10)
        self.publisher_Laser_Scan_3 = self.create_publisher(Range, 'sensor_tof_3/range', 10)
        self.publisher_Laser_Scan_4 = self.create_publisher(Range, 'sensor_tof_4/range', 10)
        self.publisher_Laser_Scan_5 = self.create_publisher(Range, 'sensor_tof_5/range', 10)
        self.publisher_Laser_Scan_6 = self.create_publisher(Range, 'sensor_tof_6/range', 10)

        #frecuencia del envia de datos hacia los topico publicados
        self.timer_pub = 0.1  # seconds
        self.timer = self.create_timer(self.timer_pub, self.pub_cmd_vel_callback)


        # creacion del topicos a subscribirse
        # obtiene la cadena que envia un mensaje del robot
        self.subscription_1 = self.create_subscription(String,'ros_pub_1',self.data_robot_callback,10)
        self.subscription_2 = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)

        # variable de la clase
        self.laser_array = [0.00]*6
        self.u_dir = 0
        self.w_dir = 0

        # radio de las ruedas
        self.r = 0.014 # mm

        # distancia entre las ruedas
        self.l = 0.093/2

        self.cadena = ''
        self.data_laser = list()


    # callback del publicador de la velocidad de la ruedas 
    def pub_cmd_vel_callback(self):
        # datos de los sensores TOF
        laser = Range()

        laser.max_range = 0.200 # m
        laser.min_range = 0.010 # m
        laser.radiation_type = 1 # infrarojo
        laser.field_of_view = radians(25) # radianes

        # datos de la velocidad de la ruedas
        cmd_vel=String()

        w1 = (self.u_dir/self.r) - (self.l/self.r)*self.w_dir # rad/s
        w2 = (self.u_dir/self.r) + (self.l/self.r)*self.w_dir # rad/s

        # porcentaje de velocidad
        w1_100 = int((w1/11.9380)*100) 
        w2_100 = int((w2/11.9380)*100) 

        suma = w1_100 + w2_100

        cmd_vel.data = f"q{w1_100:03d}e{w2_100:03d}s{suma:04d}"


        # publicacion de las distancia de cada sensor tof 
        laser.range = self.laser_array[0]
        laser.header.frame_id = 'sensor_tof_1'
        self.publisher_Laser_Scan_1.publish(laser)

        laser.range = self.laser_array[1]
        laser.header.frame_id = 'sensor_tof_2'
        self.publisher_Laser_Scan_2.publish(laser)

        laser.range = self.laser_array[2]
        laser.header.frame_id = 'sensor_tof_3'
        self.publisher_Laser_Scan_3.publish(laser)

        laser.range = self.laser_array[3]
        laser.header.frame_id = 'sensor_tof_4'
        self.publisher_Laser_Scan_4.publish(laser)

        laser.range = self.laser_array[4]
        laser.header.frame_id = 'sensor_tof_5'
        self.publisher_Laser_Scan_5.publish(laser)

        laser.range = self.laser_array[5]
        laser.header.frame_id = 'sensor_tof_6'
        self.publisher_Laser_Scan_6.publish(laser)

        # publicacion de la velocidad de las ruedas
        self.publisher_String_cmd_vel.publish(cmd_vel)

        self.get_logger().info( 'laser:"%s" '
                                'w1_100:"%s" '
                                'w2_100:"%s" '
                                'cadena:"%s" ' % 
                                ([i for i in self.laser_array],
                                w1_100,
                                w2_100,
                                self.cadena))
        self.cadena = ''


    def data_robot_callback(self,msg):
        # mensaje esperado
        # A1: 000,A2: 000,B1: 000.00,B2: 000.00,C1: 000,C2: 000,C3: 000,C4: 000,C5: 000,C6: 000,D1: 00000.0 ms
        # donde:
        # Ax = velocidad de la rueda deseada, 
        # Bx = velocidad de la rueda estimada
        # Cx = valor indivudual de la distancia de cada sensor TOF
        # Dx = tiempo de ejecucion de robot

        self.cadena = msg.data

        # extraes los enteros y flotantes del tramo de datos 
        pattern = r'A1: ([-+]?\d+),A2: ([-+]?\d+),B1: ([-+]?\d+),B2: ([-+]?\d+),C1: (\d+),C2: (\d+),C3: (\d+),C4: (\d+),C5: (\d+),C6: (\d+),D1: (\d+) s'
        matches = re.match(pattern, self.cadena)

        if matches:
            # tramo = list(map(lambda x: int(x) if '.' not in x else float(x), matches.groups()))
            tramo = list(map(int,matches.groups()))
            self.laser_array= [i/1000 for i in tramo[4:9+1]]
        else:
            self.laser_array = [0.00]*6

    # subscripciones a los topicos de los sensores TOF    
    def cmd_vel_callback(self,msg):
        self.u_dir = msg.linear.x 
        self.w_dir = msg.angular.z


            


def main(args=None):
    rclpy.init(args=args)

    nodo_publicador = Node_Transfer_Data()

    rclpy.spin(nodo_publicador)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nodo_publicador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()