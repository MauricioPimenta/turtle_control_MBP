# Contém o nó principal do sistema de controle
# Este nó deve:
# - Subscrever ao tópico de pose do turtlesim
# - Subscrever ao tópico '/goal', onde são publicadas mensagens do tipo 'geometry_msgs/msg/Pose2D'
# - Publicar no tópico de comando de velocidade do turtlesim
#
# A Classe que define seu nó de controle deve conter os seguintes métodos:
#
# __init__: utilize a rotina de inicialização para chamar as rotinas de inicialização compartimentadas a seguir:
#
#	- init_publisher: inicializa o publisher. Siga o tutorial básico do ROS2 para criar um timer e um callback para o publisher.
#	- init_subscriber: inicialize seus subscribers, ligando-os com os callbacks adequados.
#	- init_variables: inicialize aqui as variáveis que serão utilizadas. Ex: self.x, self.x_error, self.x_goal, self.k_omega, etc...
#	- pose_callbacks: callback para receber a pose do turtlesim.
#	- goal_callbacks: callback para receber a posição do objetivo.
#	- pub_callback: método principal do nó, implementado como callback do publisher. Aqui você deverá computar o erro, implementar o controle e publicar a mensagem de velocidade para o turtlesim.
#
#

import math as m

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class TurtleControl(Node):

	def __init__(self):
		super().__init__('turtle_control')
		self.get_logger().info('Initializing turtle_control node...')

		self.init_variables()
		self.init_subscribers()
		self.init_publishers()
		


	def init_subscribers(self):
		self.sub_topic_turtle_Pose2D_name = 'turtle1/pose'
		self.sub_topic_goal_name = 'goal'

		self.get_logger().info('Initializing subscribers...')
		self.get_logger().info('Initializing Pose2D subscriber: ' + self.sub_topic_turtle_Pose2D_name)
		self.pose_subscriber_ = self.create_subscription(Pose, self.sub_topic_turtle_Pose2D_name, self.pose_callback, 10)

		self.get_logger().info('Initializing goal subscriber: ' + self.sub_topic_goal_name)
		self.goal_subscriber_ = self.create_subscription(Pose2D, self.sub_topic_goal_name, self.goal_callback, 10)


	def init_publishers(self):
		self.get_logger().info('Initializing publisher for turtle1/cmd_vel...')
		
		self.pub_topic_name = 'turtle1/cmd_vel'
		self.publisher_ = self.create_publisher(Twist, self.pub_topic_name, 10)
		
		# create a timer to call the publisher callback at a fixed rate
		self.timer_period = 0.1
		self.timer = self.create_timer(self.timer_period, self.pub_callback)


	def init_variables(self):
		self.x = 0.0
		self.x_error = 0.0
		self.y = 0.0
		self.y_error = 0.0
		self.theta = 0.0
		self.theta_error = 0.0
		
		self.x_goal = 0.0
		self.y_goal = 0.0
		self.theta_goal = 0.0
		
		self.v = 0.0
		self.omega = 0.0
		self.k_omega = 0.2
		self.vmax = 0.2


	def pose_callback(self, msg):
		self.x = msg.x
		self.y = msg.y
		self.theta = msg.theta
		self.get_logger().warning('\n\nPose received: x=%f, y=%f, theta=%f\n' % (self.x, self.y, self.theta))
  
  
	def goal_callback(self, msg):
		self.x_goal = msg.x
		self.y_goal = msg.y
		self.theta_goal = msg.theta
		self.get_logger().warning('\n\nGoal received: x=%f, y=%f, theta=%f\n' % (self.x_goal, self.y_goal, self.theta_goal))

	
	# pub_callback: método principal do nó, implementado como callback do publisher.
	# Aqui você deverá computar o erro, implementar o controle e publicar a mensagem 
	# de velocidade para o turtlesim.
	def pub_callback(self):
		# Computar o erro
		self.x_error = self.x_goal - self.x
		self.y_error = self.y_goal - self.y
		
		self.pos_error = m.sqrt(self.x_error**2 + self.y_error**2)
		self.theta_error = m.atan2(self.y_error,self.x_error) - self.theta
		# self.theta_error = self.theta_goal - self.theta
		
		self.get_logger().error('\n\n Errors: \n\nx_error: %f, y_error: %f, theta_error: %f\n' % (self.x_error, self.y_error, self.theta_error))

		


		# Implementar o controle
		self.omega = self.k_omega * self.theta_error
		self.v = self.vmax * m.tanh((self.x_error**2 + self.y_error**2)**0.5)

		# Publicar a mensagem de velocidade para o turtlesim
		msg = Twist()
		msg.linear.x = self.v
		msg.linear.y = 0.0
		msg.linear.z = 0.0
		msg.angular.x = 0.0
		msg.angular.y = 0.0
		msg.angular.z = self.omega

		self.get_logger().warning('Publishing: "%s"' % msg)
		self.publisher_.publish(msg)


def main(args=None):
	rclpy.init(args=args)
	print(args)
 
	# if args[0] == 'debug':
	# 	rclpy.logging.set_logger_level(level=rclpy.logging.LoggingSeverity.DEBUG)

	turtle_control = TurtleControl()

	rclpy.spin(turtle_control)

	# Destroy the node explicitly
	turtle_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
