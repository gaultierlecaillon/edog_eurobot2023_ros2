#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from functools import partial
from std_msgs.msg import Bool
from robot_interfaces.msg import Position
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import BoolBool


class IANode(Node):
    action_name = None
    action_param = None
    current_action_already_printed = False

    def __init__(self):
        super().__init__('ia_node')
        self.actions_dict = []
        self.load_strategy_from_file()

        self.number_timer_ = self.create_timer(0.1, self.master_callback)
        self.get_logger().info("\033[38;5;208mIA Node is running!\n\n\t\t\t (⌐■_■) 𝘴𝘶𝘱 𝘣𝘳𝘢 ?\033[0m\n")

    def master_callback(self):
        self.execute_current_action()

    def callback_waiting_tirette(self, msg, param):
        if msg.data == cast_str_bool(param):
            self.update_current_action_status('done')
            self.destroy_subscription(self.subscriber_)  # Unsubscribe from the topic

    def execute_current_action(self):
        if len(self.actions_dict) > 0:
            current_action = self.actions_dict[0]
            action = current_action['action']
            self.action_name, self.action_param = list(action.items())[0]

            if not self.current_action_already_printed:
                self.get_logger().info(
                    f"[Current Action] {self.action_name} {self.action_param} ({current_action['status']})")
                self.current_action_already_printed = True

            if current_action['status'] == "pending":
                getattr(self, self.action_name)(self.action_param)
                self.update_current_action_status('ongoing')
        else:
            self.get_logger().info(f"[Match done] No more actions to exec)")
            rclpy.shutdown()

    def waiting_tirette(self, param):
        self.subscriber_ = self.create_subscription(
            Bool,
            "tirette_topic",
            lambda msg: self.callback_waiting_tirette(msg, param), 1)

    def grab(self, param):
        self.get_logger().info(f"TODO: Performing grab action with param: {param}")

    def calibrate(self, param):
        self.get_logger().info(f"[Exec Action] calibrate with param: {param}")
        client = self.create_client(BoolBool, "cmd_calibration_service")
        while not client.wait_for_service(0.25):
            self.get_logger().warn("Waiting for Server to be available...")

        request = BoolBool.Request()
        request.data = True
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to cmd_calibration_service")

    def goto(self, param):
        self.get_logger().info(f"[Exec Action] goto with param: {param}")

        client = self.create_client(CmdPositionService, "cmd_position_service")
        while not client.wait_for_service(0.25):
            self.get_logger().warn("Waiting for Server to be available...")

        request = CmdPositionService.Request()
        # todo
        request.x = 0
        request.y = 0
        request.r = 90
        future = client.call_async(request)

        future.add_done_callback(
            partial(self.callback_current_action))

        self.get_logger().info(f"[Publish] {request} to cmd_position_service")

    def callback_current_action(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[Current Action] Done ! {response}")
                self.update_current_action_status('done')
            else:
                self.get_logger().info(f"Something went wrong with response: {response}")

        except Exception as e:

            self.get_logger().error("Service call failed %r" % (e,))

    def update_current_action_status(self, status):
        if status == "done":
            self.actions_dict.pop(0)
        else:
            self.actions_dict[0]['status'] = status
        self.current_action_already_printed = False

    def load_strategy_from_file(self):
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


def cast_str_bool(var):
    return var == 'True'


def main(args=None):
    rclpy.init(args=args)
    ia_node = IANode()
    rclpy.spin(ia_node)
    ia_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
