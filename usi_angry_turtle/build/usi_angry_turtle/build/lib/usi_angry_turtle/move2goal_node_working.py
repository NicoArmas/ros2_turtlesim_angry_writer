from turtle import pen
from typing import Set
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
from math import pow, atan2, sqrt
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from usi_angry_turtle.pid import PID


class Move2GoalNode(Node):
    def __init__(self, goal_pose, tolerance):
        # Creates a node with name 'move2goal'
        super().__init__('move2goal')

        # Create attributes to store the goal and current poses and tolerance
        self.goal_pose = goal_pose
        self.tolerance = tolerance
        self.current_pose = None

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create a subscriber to the topic '/turtle1/pose', which will call self.pose_callback every 
        # time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.lin_pid = PID(0.3,0, 0.1)
        self.ang_pid = PID(0.3,0, 0.1)
        
        self.pen_off = True

        self.prev_goal = Pose()
    
    def pen_drawing(self, drawing):
        req = SetPen.Request()
        req._b, req._g, req._r = 255, 255, 255
        req._width = 20
        req._off = 1
        
        if drawing:
            print("I'm drawing")
            req._off = 0
        else:
            print("I'm not dwrawing")

        self.pen.call_async(req)

        self.pen_off = req._off

    def start_moving(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(0.2, self.move_callback)
        
        # Create a Future object which will be marked as "completed" once the turtle reaches the goal
        self.done_future = Future()
        
        return self.done_future
        
    def pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)
        print(f'X: {self.current_pose.x} Y: {self.current_pose.y}')
        
    def move_callback(self):
        """Callback called periodically by the timer to publish a new command."""
        
        if self.current_pose is None:
            # Wait until we receive the current pose of the turtle for the first time
            return
        
        if self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance:
            # We still haven't reached the goal pose. Use a proportional controller to compute velocities
            # that will move the turtle towards the goal (https://en.wikipedia.org/wiki/Proportional_control)
        
            # Twist represents 3D linear and angular velocities, in turtlesim we only care about 2 dimensions:
            # linear velocity along the x-axis (forward) and angular velocity along the z-axis (yaw angle)\
            #print('Sono dentro')
                
            
            cmd_vel = Twist() 

            cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose)


            if self.pen_off:
                cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)

                #print(f'Ang {cmd_vel.angular.z}, Lin {cmd_vel.linear.x}')
                
                if abs(cmd_vel.angular.z) > 0.05:
                    cmd_vel.linear.x = 0.0
                
                if abs(cmd_vel.linear.x) > 0:
                    cmd_vel.angular.z = 0.0
            
            else:
                
                
                cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose, constant=2.5)
                
                #print(f'Ang {cmd_vel.angular.z}, Lin {cmd_vel.linear.x}')


                if abs(cmd_vel.angular.z) > 0.03 and \
                (self.prev_goal.x == self.goal_pose.x or self.prev_goal.y == self.goal_pose.y):
                    #print('Sto esagerando')
                    cmd_vel.linear.x = 0.0
                else:
                    const = 1.5
                    # if (self.prev_goal.x == self.goal_pose.x or self.prev_goal.y == self.goal_pose.y):
                    #     const = 3
                    cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose, constant = const)
                    
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            self.get_logger().info("Goal reached, shutting down...")
            
            # Stop the turtle
            cmd_vel = Twist() 
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.vel_publisher.publish(cmd_vel)
            
            # Mark the future as completed, which will shutdown the node
            self.done_future.set_result(True)

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        st_angle = atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)
        #st_angle = st_angle % (2 * math.pi)
        
        
        if st_angle - self.current_pose.theta > math.pi:
            st_angle = -(st_angle)
        elif st_angle - self.current_pose.theta < -(math.pi):
            st_angle = (st_angle + (2 * math.pi))


        return st_angle

    def angular_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        ang_vel =  constant * (self.steering_angle(goal_pose, current_pose) - current_pose.theta)
        return  ang_vel







def main():

    rclpy.init(args=sys.argv)
    done = Future()
    tolerance = 0.05
    goal = Pose()

    node = Move2GoalNode(goal, tolerance)

    u_letter = [(1,7.5), (1,4.5), (2,3.5), (3,4.5), (3,7.5)]
    # s_letter = [(6,7), (5,7), (4.4, 6.8), (4.2, 6.6),
    #             (4,6), (4.2, 5.4), (4.4, 5.2), (5,5), 
    #             (5.6, 4.8), (5.8, 4.6), (6,4), 
    #             (5.8, 3.4), (5.6, 3.2), (5,3), (4,3)]
    s_letter = [(7,7.5), (6,7.5), (5.99,5.5), (6.15, 5.5), (6.2,3.5), (4.8,3.5)]
    i_letter = [(9.5,3.5), (9.5,7.5)]
    letters = [u_letter, s_letter, i_letter]
    
    req = SetPen.Request()
    req._b, req._g, req._r = 255, 255, 255
    req._width = 20
    
    node.pen_drawing(False)

    for letter in letters:
        node.pen_drawing(False)
        
        for x,y in letter:
            
            print(x,y)
            goal = Pose()
            goal.x = float(x)
            goal.y = float(y)

            node.goal_pose = goal
            
            done = node.start_moving()
            rclpy.spin_until_future_complete(node, done)
            node.prev_goal = goal
            node.pen_drawing(True)
            #print(f'Current theta (rad): {node.current_pose.theta}')
            #print(f'Current theta (deg): {math.degrees(node.current_pose.theta)}')

if __name__ == '__main__':
    
    main()

