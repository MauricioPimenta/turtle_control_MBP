import math as m
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.qos
import tf_transformations


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from turtlesim.msg import Pose as TurtlePose




class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.get_logger().info('Initializing position_monitor node...')
        self.init_variables()
        self.init_subscribers()
        self.init_publishers()
    
    
    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.RobotPositions = []
        
        # define an array of goals - [x, y]
        turtlex0 = 5.544445
        turtley0 = 5.544445
        offset = 2.0
        self.goals = [[turtlex0 + offset, turtley0 + 0.0],
                      [turtlex0 + offset, turtley0 + offset],
                      [turtlex0 + 0.0, turtley0 + offset],
                      [turtlex0 - offset, turtley0 + offset],
                      [turtlex0 - offset, turtley0 + 0.0],
                      [turtlex0 - offset, turtley0 - offset],
                      [turtlex0 + 0.0, turtley0 - offset],
                      [turtlex0 + offset, turtley0 - offset]]
        self.currentGoal = 0
        self.min_distance_to_goal = 0.1
        self.goalMsg = None
        
        self.publishingFrequency = 5.0 # Hz
        
    
    def init_subscribers(self):
        self.sub_topic_turtle_Pose2D_name = 'turtle1/pose'
        self.sub_topic_goal_name = 'goal'
  
        # self.odomSubscriber = self.create_subscription(Odometry, self.sub_topic_goal_name, self.getOdomCallback(), 10)
        self.turtlePose_subscriber_ = self.create_subscription(TurtlePose, self.sub_topic_turtle_Pose2D_name, self.getTurtlePoseCallback, 10)
        
        
    def init_publishers(self):
        
        self.goal_Publisher_ = self.create_publisher(Pose, 'goal', 10)
        timer_frequency = self.publishingFrequency
        timer_period = 1.0 / timer_frequency
        
        self.timer = self.create_timer(timer_period, self.publisherCallback)
    
    
    # Get the pose from the turtlesim
    def getTurtlePoseCallback(self, msg: TurtlePose):
        self.get_logger().info('Getting pose from /turtle1/pose')
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        
        self.get_logger().warning('\n\nPose received: x=%f, y=%f, theta=%f\n' % (self.x, self.y, self.theta))
        
        self.checkDistanceToGoal()
        
    
    # # get odometry data from robot
    # def getOdomCallback(self, msg: Pose):
    #     self.get_logger().info('Getting pose from /odom')
        
    #     # copy the received message and store in a vector of all received messages
    #     self.lastOdometryHeader = msg.header
    #     self.lastOdometryPoseWithCovariance = msg.pose
    #     self.lastOdometryTwistWithCovariance = msg.twist
        
    #     # store the last position and orientation of the robot
    #     self.x = msg.pose.pose.position.x
    #     self.y = msg.pose.pose.position.y
    #     quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #     [self.roll, self.pitch, self.yaw] = tf_transformations.euler_from_quaternion(quat)
    #     self.theta = self.yaw
        
    #     self.checkDistanceToGoal()
        
    
    def publisherCallback(self):
        if self.goalMsg == None:
            self.goalMsg = Pose()
            self.goalMsg.position.x = self.goals[self.currentGoal][0]
            self.goalMsg.position.y = self.goals[self.currentGoal][1]
        
        self.get_logger().info('Publishing goal...')
        
        self.goal_Publisher_.publish(self.goalMsg)
        
        
    def checkDistanceToGoal(self):
        x_goal = self.goals[self.currentGoal][0]
        y_goal = self.goals[self.currentGoal][1]
        
        distance = m.sqrt((x_goal - self.x)**2 + (y_goal - self.y)**2)
        if distance < self.min_distance_to_goal:
            self.get_logger().info('Goal reached!')
            self.currentGoal += 1
            if self.currentGoal >= len(self.goals):
                self.currentGoal = 0
            
            # update the goal message with the next goal
            self.goalMsg = Pose()
            self.goalMsg.position.x = self.goals[self.currentGoal][0]
            self.goalMsg.position.y = self.goals[self.currentGoal][1]
            
            
def main(args=None):
    rclpy.init(args=args)
    print(args)
 
    # if args[0] == 'debug':
    # 	rclpy.logging.set_logger_level(level=rclpy.logging.LoggingSeverity.DEBUG)

    robot_monitor = PositionMonitor()
    rclpy.spin(robot_monitor)

    # Destroy the node explicitly
    robot_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
