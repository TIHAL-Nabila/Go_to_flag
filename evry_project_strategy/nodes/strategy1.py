#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from evry_project_plugins.srv import DistanceToFlag
import math

class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        # Subscribers
        rospy.Subscriber(self.robot_name + "/sensor/sonar_front", Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom", Odometry, self.callbackPose)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher(self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor"""
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle"""
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot"""
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose."""
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


def run_demo():
    """Main loop for robust strategy with obstacle avoidance."""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    rospy.loginfo(f"Robot {robot_name} is starting...")

    # PID Parameters for goal tracking
    Kp = 1.2
    Ki = 0.05
    Kd = 0.6
    d_des = 0.1  # Acceptable distance threshold to the flag
    dt = 0.05

    # Obstacle avoidance parameters
    obstacle_distance_threshold = 2.0  # Avoid objects within 1 meter
    avoidance_speed = 0.5
    avoidance_angle = math.pi / 2
    avoidance_turn_time = 1.0

    # Initial speed and timing strategy
    rospy.sleep(3 * int(robot_name[-1]))  # Staggered start based on robot ID

    # Get initial distance to the flag
    distance = float(robot.getDistanceToFlag())

    # Calculate flag position relative to the robot's initial pose
    flag_x = robot.x + distance * math.cos(robot.yaw)
    flag_y = robot.y + distance * math.sin(robot.yaw)

    while not rospy.is_shutdown():
        # Get distance to the flag and obstacle distance
        distance = float(robot.getDistanceToFlag())
        obstacle_distance = float(robot.get_sonar())  # Assuming get_sonar() provides distance to the closest obstacle

        rospy.loginfo(f"{robot_name} distance to flag: {distance}")
        rospy.loginfo(f"{robot_name} obstacle distance: {obstacle_distance}")

        # Obstacle avoidance logic
        if obstacle_distance < obstacle_distance_threshold:
            rospy.logwarn(f"{robot_name} is avoiding an obstacle!")
            robot.set_speed_angle(avoidance_speed, avoidance_angle)  # Turn to avoid obstacle
            rospy.sleep(avoidance_turn_time)
            continue  # Skip goal tracking for this iteration

        # Stop the robot if it reaches the flag
        if distance <= d_des:
            robot.set_speed_angle(0, 0)
            rospy.loginfo(f"{robot_name} has reached its flag.")
            break

        # Calculate angle to the flag
        flag_angle = math.atan2(flag_y - robot.y, flag_x - robot.x)
        angle_error = flag_angle - robot.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize to [-π, π]

        # PID control for angular velocity
        angle = Kp * angle_error + Ki * (angle_error * dt) + Kd * (angle_error / dt)

        # PID control for linear velocity
        err1 = distance - d_des
        vt = Kp * err1 + Ki * (err1 * dt) + Kd * (err1 / dt)

        # Set speed and maintain direction
        robot.set_speed_angle(vt, angle)
        rospy.sleep(dt)


if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()