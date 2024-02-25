import rclpy
from math import sin,cos,atan,radians,degrees
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# tipo de mensaje a publicar
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

# creacion del nodo
class Nodo_Myrobbie3(Node):

    def __init__(self):
        super().__init__('nodo_myrobbie3')

        # creacion del topico a publicar
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_pub = 0.1  # seconds
        self.timer = self.create_timer(self.timer_pub, self.pub_callback)

        # creacion de parametros
        PD_Kc = ParameterDescriptor(description='variable proporcional del PID')
        PD_Td = ParameterDescriptor(description='variable derivativa del PID')
        self.declare_parameter('Kc',0.015,PD_Kc)
        self.declare_parameter('Td',0.01,PD_Td)

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # self.timer_pub = 0.05  # seconds
        # self.timer = self.create_timer(self.timer_pub, self.topic_callback)

        # creacion del topicos a subscribirse
        self.subscription_1 = self.create_subscription(Range,'sensor_tof_1/range',self.tof_1_callback,10)
        self.subscription_2 = self.create_subscription(Range,'sensor_tof_2/range',self.tof_2_callback,10)
        self.subscription_3 = self.create_subscription(Range,'sensor_tof_3/range',self.tof_3_callback,10)
        self.subscription_4 = self.create_subscription(Range,'sensor_tof_4/range',self.tof_4_callback,10)
        self.subscription_5 = self.create_subscription(Range,'sensor_tof_5/range',self.tof_5_callback,10)
        self.subscription_6 = self.create_subscription(Range,'sensor_tof_6/range',self.tof_6_callback,10)

        self.range = [0,0,0,0,0,0]
        self.limit = [0,0,0,0,0,0]      

        # datos para el controlador
        self.u_k0 = 0 # u(k)
        self.u_k1 = 0 #u(k-1)
        self.e_k0 = 0 # e(k)
        self.e_k1 = 0 # e(k-1)
        self.e_k2 = 0 # e(k-2)

        self.Td = 0
        self.Kc = 0
        self.Ts = self.timer_pub # tiempo de muestreo


    # funciones secundarias para el nodo
    def medir_distancia(self,msg,min_range,max_range,count):
        if msg > min_range and msg < max_range:
            self.limit[count] = 0
            self.range[count] = int(msg*1000)
        else:
            self.limit[count] = 1
            self.range[count] = 600

    # callback del publicador cmd_vel 
    def pub_callback(self):
        msg = Twist()
        msg.linear.x = 0.08 # m/s
        
        # terminos del la estimacion angular alfa
        a = self.range[1]
        b = self.range[0]
        
        #estimacion del error 
        if (a == 600 or b == 600) or (a == 0 or b == 0):

            self.angular_z = 0.0   

            # datos para el controlador
            self.u_k0 = 0 # u(k)
            self.u_k1 = 0 #u(k-1)
            self.e_k0 = 0 # e(k)
            self.e_k1 = 0 # e(k-1)
            self.e_k2 = 0 # e(k-2)

            self.publisher_.publish(msg)
            self.get_logger().info('no hay muro')
            
        else:
            # entrada de nueva variable de parametros
            self.Td = self.get_parameter('Td').get_parameter_value().double_value # constate derivativa
            self.Kc = self.get_parameter('Kc').get_parameter_value().double_value # constante proporcional
            

            # angulo del carro alfa
            theta = radians(45)
            alfa = atan((a*cos(theta)-b)/(a*sin(theta))) + theta/2
            gamma = alfa-(theta/2)

            L = msg.linear.x*self.timer_pub
            diff_z = L*sin(alfa)*1000
            diff_y = b*cos(gamma) - 436*cos(theta/2)

            # estimacion de error
            self.e_k0 = -(diff_z + diff_y)

            # modelo del controlador PD
            self.u_k0 = self.Kc*((1+self.Td/self.Ts)*self.e_k0 - (2*(self.Td/self.Ts)+1)*self.e_k1 + (self.Td/self.Ts)*self.e_k2) + self.u_k1

            # salida de PID
            msg.angular.z = self.u_k0

            # desplazamiento de los registros
            self.u_k1 = self.u_k0
            
            self.e_k2 = self.e_k1
            self.e_k1 = self.e_k0

            # publicacion de datos 
            self.publisher_.publish(msg)
            self.get_logger().info('recibido: diff_y:"%s", '
                                         'diff_z:"%s", '
                                         'alfa:"%s", '
                                         'a:"%s", '
                                         'b:"%s", '
                                         'gamma:"%s", '
                                         'u_k:"%s", '
                                         'Kc:"%s", '
                                         'Td:"%s", ' % 
                                         (round(diff_y,4),
                                         round(diff_z,4),
                                         round(degrees(alfa),4),
                                         a,
                                         b,
                                         round(degrees(gamma),4),
                                         self.u_k0,
                                         self.Kc,
                                         self.Td))    

    # subscripciones a los topicos de los sensores TOF    
    def tof_1_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,0)
    def tof_2_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,1)
    def tof_3_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,2)
    def tof_4_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,3)
    def tof_5_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,4)
    def tof_6_callback(self,msg):
        Nodo_Myrobbie3.medir_distancia(self,msg.range,msg.min_range,msg.max_range,5)
         
            


def main(args=None):
    rclpy.init(args=args)

    nodo_publicador = Nodo_Myrobbie3()

    rclpy.spin(nodo_publicador)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nodo_publicador.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()