#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_package.srv import MoveForward
from Motion import Motion

class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server')
        self.srv = self.create_service(MoveForward, 'move_forward', self.move_forward_callback)
        self.motion = Motion()

    def move_forward_callback(self, request, response):
        #self.motion.moveForward(request.forward, request.time)
        #self.motion.waitForMovementCompletion(request.forward)
        print("request", request)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    motion_server = MotionServer()

    rclpy.spin(motion_server)

    motion_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
