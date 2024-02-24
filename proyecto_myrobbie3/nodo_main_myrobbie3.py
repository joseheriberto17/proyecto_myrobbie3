import rclpy
from math import sin,cos,atan,radians,degrees
from rclpy.node import Node

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

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # self.timer_pub = 0.05  # seconds
        # self.timer = self.create_timer(self.timer_pub, self.topic_callback)

        self.subscription_1 = self.create_subscription(Twist,'cmd_vel',self.cmd_callback,10)

        # creacion del topicos a subscribirse
        self.subscription_1 = self.create_subscription(Range,'sensor_tof_1/range',self.tof_1_callback,10)
        self.subscription_2 = self.create_subscription(Range,'sensor_tof_2/range',self.tof_2_callback,10)
        self.subscription_3 = self.create_subscription(Range,'sensor_tof_3/range',self.tof_3_callback,10)
        self.subscription_4 = self.create_subscription(Range,'sensor_tof_4/range',self.tof_4_callback,10)
        self.subscription_5 = self.create_subscription(Range,'sensor_tof_5/range',self.tof_5_callback,10)
        self.subscription_6 = self.create_subscription(Range,'sensor_tof_6/range',self.tof_6_callback,10)

        self.i = 0
        self.set_y = 0
        self.error = 0
        self.linear_x = 0
        self.angular_z = 0

        self.range = [1,1,1,1,1,1]
        self.limit = [0,0,0,0,0,0]

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
        
        # msg.linear.x = 0.2 # m/s
        
        # terminos del la estimacion angular alfa
        a = self.range[1]
        b = self.range[0]
        

        #error 
        if a == 600 or b == 600:
            alfa = 500
            gamma = 500
            diff_y = 500
            diff_z = 500
            self.error = diff_z + diff_y


            
        else:
            theta = radians(45)
            
            L = self.linear_x*self.timer_pub

            # angulo del carro alfa
            alfa = atan((a*cos(theta)-b)/(a*sin(theta))) + theta/2

            gamma = alfa-(theta/2)
 
            diff_y = b*cos(gamma) - 436*cos(theta/2) # 
            diff_z = L*sin(alfa)*1000
            self.error = diff_z + diff_y
        # self.publisher_.publish(msg)
        self.get_logger().info('recibido: diff_y:"%s", '
                                         'diff_z:"%s", '
                                         'alfa:"%s", '
                                         'a:"%s", '
                                         'b:"%s", '
                                         'gamma:"%s", ' % 
                                         (round(diff_y,4),
                                         round(diff_z,4),
                                         round(degrees(alfa),4),
                                         a,
                                         b,
                                         round(degrees(gamma),4)))
        
    #subscricion al cmd_ve
    def cmd_callback(self,msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

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