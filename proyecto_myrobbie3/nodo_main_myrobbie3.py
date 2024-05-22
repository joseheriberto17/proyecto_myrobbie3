import rclpy
from math import sin,cos,atan,radians,degrees,pi
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

        # creacion de parametros de la aplicacion
        PD_Kc = ParameterDescriptor(description='variable proporcional del PID')
        PD_Td = ParameterDescriptor(description='variable derivativa del PID')
        self.declare_parameter('Kc',0.01,PD_Kc)
        self.declare_parameter('Td',2.0,PD_Td)

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
        self.u_k1 = 0 # u(k-1)
        self.e_k0 = 0 # e(k)
        self.e_k1 = 0 # e(k-1)
        self.e_k2 = 0 # e(k-2)

        self.Td = 0
        self.Kc = 0
        self.Ts = self.timer_pub # tiempo de muestreo


    # funciones secundarias para el nodo

    # indica que si la distacia de un sensor supera el rango de medicion se deja como 200
    def medir_distancia(self,msg,min_range,max_range,count):
        if msg > min_range and msg < max_range:
            self.range[count] = int(msg*1000)
        else:
            self.range[count] = 200

    
    
     # callback del publicador cmd_vel 
    def pub_callback(self):
        
        # velocidad maxima de las ruedas del robot al 80% en rad/seg
        vel_wheel = 114*0.8*(2*pi/60) 

        # velocidad en u en el marco de robot 
        u_dir = 0.08 # m/s

        # radio de las ruedas
        r = 0.014 # mm

        # distancia entre las ruedas
        l = 0.093/2

        #limite del valor de salida del controlador PD
        limit_uk = -(u_dir/r - vel_wheel)*(r/l)

        msg = Twist()
        msg.linear.x = u_dir
        L = msg.linear.x*self.timer_pub
        
        # terminos del la estimacion angular alfa
        a = self.range[1:]
        b = self.range[:-1]

        alfa = [0]*5
        gamma = [0]*5

        # posiciones de los de angulo de cada direcion b de robot
        beta = [radians(-22.5),radians(22.5),radians(67.5),radians(112.5),radians(157.5)]

        # entrada de nueva variable de parametros
        self.Td = self.get_parameter('Td').get_parameter_value().double_value # constate derivativa
        self.Kc = self.get_parameter('Kc').get_parameter_value().double_value # constante proporcional


        for i in range(len(self.range)-1):
            theta = radians(45) 
            
            if (a[i] == 200 or b[i] == 200) or (a[i] == 0 or b[i] == 0):

                b[i] = 200
                gamma[i] = radians(200)
                alfa[i] = radians(200)
            else:             
                gamma[i] = atan((a[i]*cos(theta)-b[i])/(a[i]*sin(theta)))
                alfa[i] = (gamma[i]-beta[i])

        alfa_no_inf = [i for i in alfa if i < radians(200)]
        gamma_no_inf = [i for i in gamma if i < radians(200)]
        b_no_inf = [i for i in b if i < 200]
        
        if len(alfa_no_inf) != 0:
            prom_alfa = sum(alfa_no_inf)/len(alfa_no_inf)
            prom_gamma = sum(gamma_no_inf)/len(gamma_no_inf)
            prom_b = sum(b_no_inf)/len(b_no_inf)

            diff_z = -L*sin(prom_alfa)*1000
            diff_y =  (125 - prom_b*cos(prom_gamma))/10 

            # estimacion de error
            self.e_k0 = (diff_z + diff_y)

            # modelo del controlador PD para la orientacion angular del robot.
            self.u_k0 = self.Kc*((1+self.Td/self.Ts)*self.e_k0 - (2*(self.Td/self.Ts)+1)*self.e_k1 + (self.Td/self.Ts)*self.e_k2) + self.u_k1

            
            # limites de error del controlador
            if self.u_k0 >= limit_uk:
                self.u_k0 = limit_uk

            if self.u_k0 <= -limit_uk:
                self.u_k0 = -limit_uk

            # salida de PD
            msg.angular.z = self.u_k0

            # desplazamiento de los registros
            self.u_k1 = self.u_k0
            
            self.e_k2 = self.e_k1
            self.e_k1 = self.e_k0

        else:
            prom_alfa = radians(200)
            prom_gamma = radians(200)
            prom_b = 200

            diff_z = 200
            diff_y = 200

            # datos para el controlador
            self.u_k0 = 0.0 # u(k)
            self.u_k1 = 0.0 # u(k-1)
            self.e_k0 = 0.0 # e(k)
            self.e_k1 = 0.0 # e(k-1)
            self.e_k2 = 0.0 # e(k-2)
   
        self.publisher_.publish(msg)
        
        # Publicacion de las varialbles en la consola.
        self.get_logger().info('recibido: '
                                'prom_alfa:"%+3.3f" '
                                'uk:"%+2.3f" '
                                'diff_y:"%+2.3f" '
                                'diff_z:"%+2.3f" '
                                'Kc:"%+2.3f" '
                                'Td:"%+2.3f" ' % 
                                (round(degrees(prom_alfa),3),
                                round(self.u_k0,3),
                                round(diff_y,3),
                                round(diff_z,3),
                                round(self.Kc,3),
                                round(self.Td,3)))

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