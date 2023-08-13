import rclpy
from rclpy.node import Node
from odrive_ros2.OdriveRos2 import OdriveRos2

class OdriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')
        self.declare_parameter('simulation_mode', False)

        self.declare_parameter('publish_joint_angles', True) # if self.sim_mode else False
        self.declare_parameter('publish_temperatures', True)

        self.declare_parameter('~axis_for_right', 0) # if right calibrates first, this should be 0, else 1
        self.declare_parameter('~wheel_track', 0.285) # m, distance between wheel centres
        self.declare_parameter('~tyre_circumference', 0.341) # used to translate velocity commands in m/s into motor rpm

        self.declare_parameter('~connect_on_startup', False)
        self.declare_parameter('~calibrate_on_startup', False)
        self.declare_parameter('~engage_on_startup', False)

        self.declare_parameter('~use_preroll', True)

        self.declare_parameter('~publish_current', True)
        self.declare_parameter('~publish_raw_odom', True)

        self.declare_parameter('~publish_odom', True)
        self.declare_parameter('~publish_odom_tf', False)
        self.declare_parameter('~odom_topic', "odom")
        self.declare_parameter('~odom_frame', "odom")
        self.declare_parameter('~base_frame', "base_link")
        self.declare_parameter('~odom_calc_hz', 10)


        self.timer = self.create_timer(1, self.timer_callback)

        self.controller = OdriveRos2(self)
        print('Odrive ROS2 driver initialized')

    def timer_callback(self):
        self.controller.tick()

def main(args=None):
    print('Starting Odrive ROS2 driver.')
    rclpy.init(args=args)
    node = OdriveNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
