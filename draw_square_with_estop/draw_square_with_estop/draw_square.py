
import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class DrawSquare(Node):
    def __init__(self):
        super().__init__('draw_square_with_estop')
        self.e_stop = Event()
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

    def handle_estop(self, msg):
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def run_loop(self):
        # pause to make sure publisher and subscribers are ready
        sleep(2)
        for _ in range(4):
            if not self.e_stop.is_set():
                print("turning left")
                self.turn_left()
            if not self.e_stop.is_set():
                print("driving forward")
                self.drive_forward(1.0)
        print('done with run loop')

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def turn_left(self):
        angular_vel = 0.3
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=angular_vel)
        sleep(math.pi / angular_vel)
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        forward_vel = 0.1
        if not self.e_stop.is_set():
            self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
