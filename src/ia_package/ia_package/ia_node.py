#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


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
        print("Doing action grab with param:", param)

    def update_current_action_status(self, status):
        if status == "done":
            self.actions_dict.pop(0)
            self.get_logger().info(f"[Action Done!] {self.action_name}")
        else:
            self.actions_dict[0]['status'] = status

    def parse_strat(self):
        # with open('../resource/strat.json') as file:
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
