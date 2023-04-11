#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from functools import partial
from std_msgs.msg import Bool
from robot_interfaces.msg import Position
from robot_interfaces.srv import CmdPositionService


class IANode(Node):
    action_name = None
    action_param = None

    def __init__(self):
        super().__init__('ia_node')

        self.actions_dict = []
        self.parse_strat()

        # Node
        self.number_timer_ = self.create_timer(0.1, self.master_callback)
        self.get_logger().info("IA Node is awake ! (⌐■_■) 𝘴𝘶𝘱 𝘣𝘳𝘢 ?")

    def master_callback(self):
        self.doCurrentAction()

    def callback_waiting_tirette(self, msg, param):
        if msg.data == bool(param):
            self.update_current_action_status('done')
            self.destroy_subscription(self.subscriber_)  # Unsubscribe from the topic

    def doCurrentAction(self, ):
        current_action = self.actions_dict[0]
        action = current_action['action']
        self.action_name, self.action_param = list(action.items())[0]

        self.get_logger().info(f"[Current Action] {current_action['action']} ({current_action['status']})")

        if current_action['status'] == "pending":
            getattr(self, self.action_name)(self.action_param)  # Call the method using self.action_name
            self.update_current_action_status('ongoing')

    # ACTIONS METHODS
    ''' 
    name: waiting_tirette
    desc: Waiting ther user to insert the tirette     
    '''

    def waiting_tirette(self, param):
        self.subscriber_ = self.create_subscription(
            Bool,
            "tirette_topic",
            lambda msg: self.callback_waiting_tirette(msg, param), 1)

    ''' 
    name: grab
    param: 
    desc: got to (x,y,r) and start to grab the stuff
    '''

    def grab(self, param):
        print("TODO Doing action grab with param:", param)

    ''' 
    name: goto
    param: 
    desc: got to (x,y,r)
    '''

    def goto(self, param):
        print("Doing action goto with param:", param)
        '''
        coordinate = [int(num) for num in param.split(',')]
        self.goto_publisher_ = self.create_publisher(CmdPositionService, "cmd_position_service", 10)

        position = Position(
            x=coordinate[0],
            y=coordinate[1],
            r=coordinate[2])
        self.goto_publisher_.publish(position)
        '''
        client = self.create_client(CmdPositionService, "cmd_position_service")
        while not client.wait_for_service(0.25):
            self.get_logger().warn("Waiting for Server to finsh goto...")
        request = CmdPositionService.Request()
        request.x = 0
        request.y = 0
        request.r = 90
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_goto))

        print("Publish:", request, "to cmd_position_service")

    def callback_goto(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Action Done ! {response}")
                self.update_current_action_status('done')
            else:
                self.get_logger().info(f"Something went wrong with response: {response}")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def update_current_action_status(self, status):
        if status == "done":
            self.actions_dict.pop(0)
            self.get_logger().info(f"[Action Done!] {self.action_name}")
        else:
            self.actions_dict[0]['status'] = status

    def parse_strat(self):
        with open('/home/edog/ros2_ws/src/ia_package/resource/strat.json') as file:
            config = json.load(file)

        self.get_logger().info(f"[Loading Strategy] {config['name']} ({config['description']})")
        self.get_logger().info(f"[Start] Color: {config['color']} | StartPos:({config['startingPos']})")

        for strat in config['strategy']:
            for action in strat['actions']:
                self.actions_dict.append(
                    {
                        'action': action,
                        'status': 'pending',
                    }
                )


def main(args=None):
    rclpy.init(args=args)
    ia_node = IANode()
    rclpy.spin(ia_node)
    ia_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
