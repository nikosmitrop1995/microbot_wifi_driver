from ament_index_python.packages import get_package_share_path
import rclpy
from rclpy.node import Node
import subprocess


class Compile(Node):

    def __init__(self):
        super().__init__('compile')
        self.run_cmd()

    def run_cmd(self):
        # Use subprocess to run the command
        dir = get_package_share_path('microbot_wifi_motor_driver')
        # print(dir)
        args = 'cd ' + str(dir) + " && pio run"
        subprocess.call(args, shell=True)


def main(args=None):
    rclpy.init(args=args)
    Compile()


if __name__ == '__main__':
    main()
