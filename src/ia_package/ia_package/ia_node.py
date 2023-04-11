#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class IANode(Node):
    def __init__(self):
        super().__init__('ia_node')
        self.actions_dict = []
        self.parse_strat()

        # Node
        self.number_timer_ = self.create_timer(0.1, self.master_callback)
        self.get_logger().info("IA Node is awake ! (⌐■_■) 𝘴𝘶𝘱 𝘣𝘳𝘢 ?")

    def master_callback(self):
        self.doCurrentAction()

    def callback_waiting_tirette(self, msg):
        if msg.data:
            self.get_logger().info(f"msg from node {msg.data}")
            self.update_current_action_status('done')

    def doCurrentAction(self, ):
        current_action = self.actions_dict[0]
        action = current_action['action']
        self.get_logger().info(f"[Current Action] {current_action['action']} ({current_action['status']})")

        if current_action['status'] == "pending":
            action_name, action_param = list(action.items())[0]
            getattr(self, action_name)(action_param)  # Call the method using action_name
            self.update_current_action_status('ongoing')

    def waiting_tirette(self, param):
        print(f"Inside waiting_tirette function with parameter: {param}")

        # Tirette Subscriber
        self.subscriber_ = self.create_subscription(Bool, "tirette_topic", self.callback_waiting_tirette, 1)

    def update_current_action_status(self, status):
        if status == "done":
            self.actions_dict.pop(0)
        else:
            self.actions_dict[0]['status'] = status

    def parse_strat(self):
        #with open('../resource/strat.json') as file:
        with open('/home/edog/ros2_ws/install/ia_package/share/ia_package/strat.json') as file:
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
