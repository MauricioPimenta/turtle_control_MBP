import math as m
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.qos
import tf_transformations as tft


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
        turtlex0 = 0
        turtley0 = 0
        offsetx = 2.0
        offsety = 0.5
        # self.goals = [[turtlex0 + offsetx, turtley0 + 0.0],
        #               [turtlex0 + offsetx, turtley0 + offsety],
        #               [turtlex0 + 0.0, turtley0 + offsety],
        #               [turtlex0 - offsetx, turtley0 + offsety],
        #               [turtlex0 - offsetx, turtley0 + 0.0],
        #               [turtlex0 - offsetx, turtley0 - offsety],
        #               [turtlex0 + 0.0, turtley0 - offsety],
        #               [turtlex0 + offsetx, turtley0 - offsety]]
        self.goals = [[2.0 , -0.5],
                      [2.0 , 0.5],
                      [-2.0, 0.5],
                      [-2.0, -0.5]]
        self.currentGoal = 0
        self.min_distance_to_goal = 0.1
        self.goalMsg = None
        
        self.publishingFrequency = 5.0 # Hz
        
    
    def init_subscribers(self):
        self.sub_topic_turtle_Pose2D_name = 'odom'
        
  
        # self.odomSubscriber = self.create_subscription(Odometry, self.sub_topic_goal_name, self.getOdomCallback(), 10)
        self.turtlePose_subscriber_ = self.create_subscription(Odometry, self.sub_topic_turtle_Pose2D_name, self.getOdomPoseCallback, 10)
        
        
    def init_publishers(self):
        self.topic_goal_name = 'goal'
        self.turtle_pose_name = 'turtle1/pose'
        
        self.goal_Publisher_ = self.create_publisher(Pose, self.topic_goal_name, 10)
        self.turtle_pose_Publisher_ = self.create_publisher(TurtlePose, self.turtle_pose_name, 10)
        
        timer_frequency = self.publishingFrequency
        timer_period = 1.0 / timer_frequency
        
        self.timer = self.create_timer(timer_period, self.publisherCallback)
    
    
    # Get the pose from the Odometry and publishes as the TurtlePose
    def getOdomPoseCallback(self, msg: Odometry):
        self.get_logger().info('Getting pose from ' + self.sub_topic_turtle_Pose2D_name)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.row, self.pitch, self.yaw = tft.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
        self.theta = self.yaw
        self.get_logger().warning('\n\nPose received: x=%f, y=%f, theta=%f\n' % (self.x, self.y, self.theta))
        
        turtlepose = TurtlePose()
        turtlepose.x = self.x
        turtlepose.y = self.y
        turtlepose.theta = self.theta
        
        self.turtle_pose_Publisher_.publish(turtlepose)
        
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
