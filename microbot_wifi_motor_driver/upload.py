from ament_index_python.packages import get_package_share_path
import rclpy
from rclpy.node import Node
import subprocess


class Upload(Node):

    def __init__(self):
        super().__init__('upload_node')
        self.run_cmd()

    def run_cmd(self):
        # Use subprocess to run the command
        dir = get_package_share_path('microbot_wifi_motor_driver')
        # print(dir)
        args = 'cd ' + str(dir) + " && pio run -t upload"
        subprocess.call(args, shell=True)


def main(args=None):
    rclpy.init(args=args)
    Upload()


if __name__ == '__main__':
    main()
