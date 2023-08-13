

class OdriveRos2:
    last_speed = 0.0
    driver = None
    prerolling = False

    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    encoder_counts_per_rev = None
    m_s_to_value = 1.0
    axis_for_right = 0
    encoder_cpr = 4096

    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False

    publish_joint_angles = True
    # Simulation mode
    # When enabled, output simulated odometry and joint angles (TODO: do joint angles anyway from ?)
    sim_mode = False

    def __init__(self, node):
        self.node = node
        self.sim_mode             = self.node.get_parameter('simulation_mode').get_parameter_value().bool_value
        self.publish_joint_angles = self.node.get_parameter('publish_joint_angles').get_parameter_value().bool_value
        self.publish_temperatures = self.node.get_parameter('publish_temperatures').get_parameter_value().bool_value

        self.axis_for_right       = self.node.get_parameter('axis_for_right').get_parameter_value().double_value
        self.wheel_track          = self.node.get_parameter('wheel_track').get_parameter_value().double_value
        self.tyre_circumference   = self.node.get_parameter('tyre_circumference').get_parameter_value().double_value

        self.connect_on_startup   = self.node.get_parameter('connect_on_startup').get_parameter_value().bool_value
        self.calibrate_on_startup = self.node.get_parameter('calibrate_on_startup').get_parameter_value().bool_value
        self.engage_on_startup    = self.node.get_parameter('engage_on_startup').get_parameter_value().bool_value

        self.has_preroll     = self.node.get_parameter('use_preroll').get_parameter_value().bool_value

        self.publish_current = self.node.get_parameter('publish_current').get_parameter_value().bool_value
        self.publish_raw_odom =self.node.get_parameter('publish_raw_odom').get_parameter_value().bool_value

        self.publish_odom    = self.node.get_parameter('publish_odom').get_parameter_value().bool_value
        self.publish_tf      = self.node.get_parameter('publish_odom_tf').get_parameter_value().bool_value
        self.odom_topic      = self.node.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame      = self.node.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame      = self.node.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_calc_hz    = self.node.get_parameter('odom_calc_hz').get_parameter_value().double_value

    def tick(self):
        # print('Tick')
        pass

    def shutdown():
        print("Shutting down.")