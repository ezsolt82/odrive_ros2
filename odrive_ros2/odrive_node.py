import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist

from odrive_ros2.OdriveRos2 import OdriveRos2
from odrive_ros2.odrive_interface import ODriveInterfaceAPI

class OdriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')
        #self.command_queue = Queue.Queue(maxsize=5)

        self.init_params()
        self.controller = OdriveRos2(self)
        self.driver = ODriveInterfaceAPI(logger = self.get_logger())
        self.driver.connect()
        # Const to calculate from meter per sec to round per sec
        self.m_s_to_rps = 1/self.tyre_circumference
        self.driver.engage()

        self.timer_recalc_odom = self.create_timer(1/self.odom_calc_hz, self.fast_timer)
        self.timer_manage_hardware = self.create_timer(1, self.slow_timer)
        self.vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        print('Odrive ROS2 driver initialized')

    def init_params(self):
        self.declare_parameter('simulation_mode', False,
                               ParameterDescriptor(description='Simulation mode'))

        self.declare_parameter('publish_joint_angles', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED ! Weather to publish joint angles'))
        self.declare_parameter('publish_temperatures', True,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED!'))

        self.declare_parameter('axis_for_right', 0,
                               ParameterDescriptor(description='NOT YET IMPLEMENTED! if right calibrates first, this should be 0, else 1')) #
        self.declare_parameter('wheel_track', 0.285,
                               ParameterDescriptor(description='Distance between wheel centres in meter'))
        self.declare_parameter('tyre_circumference', 0.341,
                               ParameterDescriptor(description='Used to translate velocity commands in m/s into motor rpm'))

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

        self.sim_mode             = self.get_parameter('simulation_mode').get_parameter_value().bool_value
        self.publish_joint_angles = self.get_parameter('publish_joint_angles').get_parameter_value().bool_value
        self.publish_temperatures = self.get_parameter('publish_temperatures').get_parameter_value().bool_value

        self.axis_for_right       = self.get_parameter('axis_for_right').get_parameter_value().double_value
        self.wheel_track          = self.get_parameter('wheel_track').get_parameter_value().double_value
        self.tyre_circumference   = self.get_parameter('tyre_circumference').get_parameter_value().double_value

        self.connect_on_startup   = self.get_parameter('connect_on_startup').get_parameter_value().bool_value
        self.calibrate_on_startup = self.get_parameter('calibrate_on_startup').get_parameter_value().bool_value
        self.engage_on_startup    = self.get_parameter('engage_on_startup').get_parameter_value().bool_value

        self.has_preroll          = self.get_parameter('use_preroll').get_parameter_value().bool_value

        self.publish_current      = self.get_parameter('publish_current').get_parameter_value().bool_value
        self.publish_raw_odom     = self.get_parameter('publish_raw_odom').get_parameter_value().bool_value

        self.publish_odom         = self.get_parameter('publish_odom').get_parameter_value().bool_value
        self.publish_tf           = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value
        self.odom_topic           = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame           = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame           = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_calc_hz         = self.get_parameter('odom_calc_hz').get_parameter_value().double_value


    def cmd_vel_callback(self, msg):
        left_linear_val, right_linear_val = self.convert_linear_angular_to_l_r_linear(msg.linear.x, msg.angular.z)
        print(f"Driving motors: Left: {left_linear_val:.2f} Right: {right_linear_val:.2f}")
        # The driver gets the values as round per second.
        self.driver.drive(left_linear_val, right_linear_val)

    def convert_linear_angular_to_l_r_linear(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0)
        print("angular_to_linear = ccw * (self.wheel_track/2.0)")
        print(f"{angular_to_linear:.2f} = {ccw:.2f} * ({self.wheel_track:.2f} /2.0)")
        left_linear_val  = (forward - angular_to_linear) * self.m_s_to_rps
        print("left_linear_val  = int((forward - angular_to_linear) * self.m_s_to_rps)")
        print(f"{left_linear_val:.2f}  = int(( {forward:.2f} - {angular_to_linear:.2f}) * {self.m_s_to_rps:.2f})")
        right_linear_val = (forward + angular_to_linear) * self.m_s_to_rps

        return left_linear_val, right_linear_val


    def shutdown(self):
        self.driver.release()
        self.driver.disconnect()
        self.controller.shutdown()

    def fast_timer(self):
        self.controller.tick()
        # if not self.command_queue.empty():


    def slow_timer(self):
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
