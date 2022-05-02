from turtle import pen
from typing import Set
import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
from math import dist, pow, atan2, sqrt
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn, Kill
    

class Move2GoalNode(Node):
    def __init__(self, goal_pose, tolerance):
        # Creates a node with name 'move2goal'
        super().__init__('move2goal')

        # Create attributes to store the goal and current poses and tolerance
        self.goal_pose = goal_pose
        self.tolerance = tolerance
        self.current_pose = None
        self.offender_pose = None
        self.offender_vel = None
        
        self.saved_pose = Pose()
        self.saved_goal = Pose()

        # Create a publisher for the topic '/turtle1/cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create a subscriber to the topic '/turtle1/pose', which will call self.pose_callback every 
        # time a message of type Pose is received
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.pen2 = self.create_client(SetPen, '/turtle2/set_pen')

        self.off_killer = self.create_client(Kill, 'kill')

        self.spawner = self.create_client(Spawn, 'spawn')
        self.offender_pose_sub = self.create_subscription(Pose, '/turtle2/pose', self.off_pose_callback, 10)
        self.offender_vel_sub = self.create_subscription(Twist, '/turtle2/cmd_vel', self.off_vel_callback, 10)
        
        self.pen_off = True

        self.prev_goal = Pose()

        self.k1 = 0.3
        self.k2 = 2.0 

        self.curr_state = "WRITING"
        self.prev_state = None

        self.warping = None
        self.returned = None

        self.off_killed = True


    
    def set_state(self, des_state):

        self.prev_state = self.curr_state
        self.curr_state = des_state



    def spawn_offender(self):
        sp_req = Spawn.Request()
        sp_req._x, sp_req._y = 10.5,10.5
        sp_req._theta = 0.0
        sp_req._name = "turtle2"

        self.spawner.call_async(sp_req)
        self.offender_pose = Pose()
        self.offender_pose.x = sp_req._x
        self.offender_pose.y = sp_req._y
        self.offender_pose.theta = sp_req._theta

        self.off_killed = False
    
    def pen_drawing(self, drawing):
        req = SetPen.Request()
        req._b, req._g, req._r = 255, 255, 255
        req._width = 20
        req._off = 1
        
        if drawing:
            req._off = 0


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

    def off_pose_callback(self, msg):
        """Callback called every time a new Pose message is received by the subscriber."""
        self.offender_pose = msg
        self.offender_pose.x = round(self.offender_pose.x, 4)
        self.offender_pose.y = round(self.offender_pose.y, 4)

        if self.offender_pose is not None:
            self.off_killed = False

    def off_vel_callback(self, msg):
        """Callback called every time a new Twist message is received by the subscriber."""
        self.offender_vel = msg

    def predict_off_pose(self, dist_off, const=1.5):
        
        target_pose = Pose()

        m = const * (dist_off * self.offender_vel.linear.x)
        angle = self.offender_pose.theta

        target_pose.x = self.offender_pose.x + (m * math.cos(angle))
        target_pose.y = self.offender_pose.y + (m * math.cos(angle))
        
        return target_pose

    def rotate_to(self, s_theta):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 2 * (s_theta - self.current_pose.theta)
        self.vel_publisher.publish(cmd_vel)

    def kill_offender(self):
        req = Kill.Request()
        req._name = "turtle2"

        self.off_killer.call_async(req)
        
        self.off_killed = True

        
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

            if not(self.off_killed):
                dist_off = self.euclidean_distance(self.current_pose, self.offender_pose)

            if self.curr_state == "WRITING" or self.curr_state == "ANGRY":
                
                if not self.off_killed:
                    if dist_off < self.k2 and dist_off >= self.k1:
                        if self.curr_state == "WRITING":
                            self.saved_pose = self.current_pose
                            self.saved_goal = self.goal_pose
                        self.set_state("ANGRY")
                        self.goal_pose = self.predict_off_pose(dist_off)

                    if self.curr_state == "ANGRY":
                        self.goal_pose = self.predict_off_pose(dist_off)
                        self.pen_drawing(False)

                        if dist_off < self.k1:
                            self.set_state("RETURNING")
                            self.returned = False
                            self.kill_offender()
                            self.goal_pose = self.saved_pose

            elif self.curr_state == "RETURNING":

                if self.returned:
                    self.set_state("WRITING")
                    self.returned = None  
                    self.goal_pose = self.saved_goal
                   
                    if not(self.warping):
                        self.pen_drawing(True)
                
                else:
                    self.goal_pose = self.saved_pose



            cmd_vel = Twist() 

            cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose)


            if self.pen_off:
                cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)

                
                if abs(cmd_vel.angular.z) > 0.05:
                    cmd_vel.linear.x = 0.0
                
                if abs(cmd_vel.linear.x) > 0:
                    cmd_vel.angular.z = 0.0
            
            else:
                
                
                cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose, constant=2.5)
                


                if abs(cmd_vel.angular.z) > 0.03 and \
                (self.prev_goal.x == self.goal_pose.x or self.prev_goal.y == self.goal_pose.y):
                    cmd_vel.linear.x = 0.0
                else:
                    const = 1.5

                    cmd_vel.linear.x = self.linear_vel(self.goal_pose, self.current_pose, constant = const)
                    
            
            # Publish the command
            self.vel_publisher.publish(cmd_vel)
        else:
            
            if self.curr_state == "RETURNING":
                cmd_vel = Twist() 
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.vel_publisher.publish(cmd_vel)
                
                if (abs(self.saved_pose.theta - self.current_pose.theta) > 0.002):
                    self.rotate_to(self.saved_pose.theta)
                else:
                    self.returned = True

                    self.set_state("WRITING")
                    self.goal_pose = self.saved_goal
                    if not(self.warping):
                        self.pen_drawing(True)

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
        
        
        if st_angle - self.current_pose.theta > math.pi:
            st_angle = -(st_angle)
        elif st_angle - self.current_pose.theta < -(math.pi):
            st_angle = (st_angle + (2 * math.pi))


        return st_angle

    def angular_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        ang_vel =  constant * (self.steering_angle(goal_pose, current_pose) - current_pose.theta)
        return  ang_vel


    def write(self, letters):

        req = SetPen.Request()
        req._b, req._g, req._r = 255, 255, 255
        req._width = 0
        req._off = 1

        self.pen2.call_async(req)

        for i, letter in enumerate(letters):
            self.warping = True
            self.pen2.call_async(req)
            self.pen_drawing(False)
            for j, (x,y) in enumerate(letter):
                self.pen2.call_async(req)
                if (i==0 and j==0):
                    self.warping = True
                    self.pen_drawing(False)
                goal = Pose()
                goal.x = float(x)
                goal.y = float(y)

                self.goal_pose = goal
                
                done = self.start_moving()
                rclpy.spin_until_future_complete(self, done)
                self.prev_goal = goal
                self.warping = False
                self.pen_drawing(True)



def main():

    rclpy.init(args=sys.argv)
    done = Future()
    tolerance = 0.05
    goal = Pose()

    node = Move2GoalNode(goal, tolerance)

    u_letter = [(1,7.5), (1,4.5), (2,3.5), (3,4.5), (3,7.5)]
    s_letter = [(7,7.5), (6,7.5), (5.99,5.5), (6.15, 5.5), (6.2,3.5), (4.8,3.5)]
    i_letter = [(9.5,3.5), (9.5,7.5)]
    letters = [u_letter, s_letter, i_letter]
    
    
    
    node.pen_drawing(False)

    req2 = SetPen.Request()
    req2._b, req2._g, req2._r = 255, 255, 255
    req2._width = 0

    node.pen2.call_async(req2)

    while True:

        node.write(letters)

    

if __name__ == '__main__':
    
    main()

