import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from odrive_ros2.OdriveRos2 import OdriveRos2
from odrive_ros2.odrive_interface import ODriveInterfaceAPI

class OdriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')
        self.declare_parameter('simulation_mode', False,
                               ParameterDescriptor(description='Simulation mode'))

        self.declare_parameter('publish_joint_angles', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED ! Weather to publish joint angles'))
        self.declare_parameter('publish_temperatures', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))

        self.declare_parameter('axis_for_right', 0,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED! if right calibrates first, this should be 0, else 1')) #
        self.declare_parameter('wheel_track', 0.285,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED! m, distance between wheel centres'))
        self.declare_parameter('tyre_circumference', 0.341,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED! used to translate velocity commands in m/s into motor rpm'))

        self.declare_parameter('connect_on_startup', False,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('calibrate_on_startup', False,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('engage_on_startup', False,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))

        self.declare_parameter('use_preroll', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))

        self.declare_parameter('publish_current', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('publish_raw_odom', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))

        self.declare_parameter('publish_odom', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('publish_odom_tf', False,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('odom_topic', "odom",
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('odom_frame', "odom",
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('base_frame', "base_link",
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))
        self.declare_parameter('odom_calc_hz', 50.0,
                               ParameterDescriptor(description='The frequency of the odom calculations and other high rate calculations'))
        self.odom_calc_hz    = self.get_parameter('odom_calc_hz').get_parameter_value().double_value

        self.timer_recalc_odom = self.create_timer(1/self.odom_calc_hz, self.recalculate_odom)
        self.timer_manage_hardware = self.create_timer(1, self.manage_hardware)

        self.controller = OdriveRos2(self)
        self.driver = ODriveInterfaceAPI(logger = self.get_logger())
        self.driver.connect()
        print('Odrive ROS2 driver initialized')

    def shutdown(self):
        self.driver.disconnect()
        self.controller.shutdown()

    def recalculate_odom(self):
        self.controller.tick()

    def manage_hardware(self):
        pass

def main(args=None):
    print('Starting Odrive ROS2 driver.')
    rclpy.init(args=args)
    node = OdriveNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
