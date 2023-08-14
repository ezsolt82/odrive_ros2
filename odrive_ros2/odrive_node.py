import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32

from odrive_ros2.odrive_interface import ODriveInterfaceAPI

class OdriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')
        #self.command_queue = Queue.Queue(maxsize=5)

        self.init_params()
        self.driver = ODriveInterfaceAPI(logger = self.get_logger())
        self.driver.connect()
        # Const to calculate from meter per sec to round per sec
        self.m_s_to_rps = 1/self.tyre_circumference
        self.driver.engage()

        self.timer_recalc_odom = self.create_timer(1/self.odom_calc_hz, self.fast_timer)
        self.timer_manage_hardware = self.create_timer(1, self.slow_timer)
        self.vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.raw_odom_publisher_encoder_left  = self.create_publisher(Float32, 'left/raw_odom/encoder'  , self.odom_calc_hz)
        self.raw_odom_publisher_encoder_right = self.create_publisher(Float32, 'right/raw_odom/encoder' , self.odom_calc_hz)
        self.raw_odom_publisher_vel_left      = self.create_publisher(Float32, 'left/raw_odom/velocity' , self.odom_calc_hz)
        self.raw_odom_publisher_vel_right     = self.create_publisher(Float32, 'right/raw_odom/velocity', self.odom_calc_hz)
        self.odom_publisher                   = self.create_publisher(Odometry, self.odom_topic         , self.odom_calc_hz)


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

        self.declare_parameter('use_preroll', False,
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
        self.declare_parameter('odom_calc_hz', 50,
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
        self.odom_calc_hz         = self.get_parameter('odom_calc_hz').get_parameter_value().integer_value

        # setup message
        self.odom_msg = Odometry()
        #print(dir(self.odom_msg))
        self.odom_msg.header.frame_id = self.odom_frame
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
        self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
        self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
        self.odom_msg.twist.twist.angular.x = 0.0 # or roll
        self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
        self.odom_msg.twist.twist.angular.z = 0.0

        self.new_pos_l = 0
        self.new_pos_r = 0


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

    def fast_timer(self):
        time_now = self.get_clock().now()
        self.update_odometry(time_now)

    def slow_timer(self):
        pass

    def update_odometry(self, time_now):
        self.driver.update_time(time_now.nanoseconds/1e9)
        self.vel_l = -self.driver.left_vel_estimate()   # units: encoder counts/s
        self.vel_r = self.driver.right_vel_estimate() # neg is forward for right

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        self.new_pos_l = -self.driver.left_pos()      # units: encoder counts
        self.new_pos_r = self.driver.right_pos()      # sign!

        # for temperatures
        self.temp_v_l = self.driver.left_temperature()
        self.temp_v_r = self.driver.right_temperature()
        # for current
        self.current_l = self.driver.left_current()
        self.current_r = self.driver.right_current()
        # voltage
        self.bus_voltage = self.driver.bus_voltage()

        if self.publish_raw_odom:
            f = Float32()
            f.data = self.new_pos_l
            self.raw_odom_publisher_encoder_left.publish(f)

            f = Float32()
            f.data = self.new_pos_r
            self.raw_odom_publisher_encoder_right.publish(f)

            f = Float32()
            f.data =  self.vel_l
            self.raw_odom_publisher_vel_left.publish(f)

            f = Float32()
            f.data =  self.vel_r
            self.raw_odom_publisher_vel_right.publish(f)

        return


        ## Old from here ##

        now = time_now
        self.odom_msg.header.stamp = now
        self.tf_msg.header.stamp = now

        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference

        # if odometry updates disabled, just return the old position and zero twist.
        if not self.odometry_update_enabled:
            self.odom_msg.twist.twist.linear.x = 0.
            self.odom_msg.twist.twist.angular.z = 0.

            # but update the old encoder positions, so when we restart updates
            # it will start by giving zero change from the old position.
            self.old_pos_l = self.new_pos_l
            self.old_pos_r = self.new_pos_r

            self.odom_publisher.publish(self.odom_msg)
            if self.publish_tf:
                self.tf_publisher.sendTransform(self.tf_msg)

            return

        # Twist/velocity: calculated from motor values only
        s = tyre_circumference * (self.vel_l+self.vel_r) / (2.0*self.encoder_cpr)
        w = tyre_circumference * (self.vel_r-self.vel_l) / (wheel_track * self.encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w

        #rospy.loginfo("vel_l: % 2.2f  vel_r: % 2.2f  vel_l: % 2.2f  vel_r: % 2.2f  x: % 2.2f  th: % 2.2f  pos_l: % 5.1f pos_r: % 5.1f " % (
        #                vel_l, -vel_r,
        #                vel_l/encoder_cpr, vel_r/encoder_cpr, self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z,
        #                self.driver.left_axis.encoder.pos_cpr, self.driver.right_axis.encoder.pos_cpr))

        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        # Check for overflow. Assume we can't move more than half a circumference in a single timestep.
        half_cpr = self.encoder_cpr/2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr

        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value

        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/wheel_track # works for small angles

        xd = math.cos(th)*d
        yd = -math.sin(th)*d

        # elapsed time = event.last_real, event.current_real
        #elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed

        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)

        #rospy.loginfo("dl_m: % 2.2f  dr_m: % 2.2f  d: % 2.2f  th: % 2.2f  xd: % 2.2f  yd: % 2.2f  x: % 5.1f y: % 5.1f  th: % 5.1f" % (
        #                delta_pos_l_m, delta_pos_r_m,
        #                d, th, xd, yd,
        #                self.x, self.y, self.theta
        #                ))

        # fill odom message and publish

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2

        #rospy.loginfo("theta: % 2.2f  z_m: % 2.2f  w_m: % 2.2f  q[2]: % 2.2f  q[3]: % 2.2f (q[0]: %2.2f  q[1]: %2.2f)" % (
        #                        self.theta,
        #                        math.sin(self.theta)/2, math.cos(self.theta)/2,
        #                        q[2],q[3],q[0],q[1]
        #                        ))

        #self.odom_msg.pose.covariance
         # x y z
         # x y z

        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]


        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)


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
