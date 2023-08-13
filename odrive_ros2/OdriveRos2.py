

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
        print(self.sim_mode)

        self.publish_joint_angles = get_param('publish_joint_angles', True) # if self.sim_mode else False
        self.publish_temperatures = get_param('publish_temperatures', True)

        self.axis_for_right = float(get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(get_param('~wheel_track', 0.285)) # m, distance between wheel centres
        self.tyre_circumference = float(get_param('~tyre_circumference', 0.341)) # used to translate velocity commands in m/s into motor rpm

        self.connect_on_startup   = get_param('~connect_on_startup', False)
        self.calibrate_on_startup = get_param('~calibrate_on_startup', False)
        self.engage_on_startup    = get_param('~engage_on_startup', False)

        self.has_preroll     = get_param('~use_preroll', True)

        self.publish_current = get_param('~publish_current', True)
        self.publish_raw_odom =get_param('~publish_raw_odom', True)

        self.publish_odom    = get_param('~publish_odom', True)
        self.publish_tf      = get_param('~publish_odom_tf', False)
        self.odom_topic      = get_param('~odom_topic', "odom")
        self.odom_frame      = get_param('~odom_frame', "odom")
        self.base_frame      = get_param('~base_frame', "base_link")
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)

        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)

        rospy.Service('calibrate_motors',         std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',            std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',           std_srvs.srv.Trigger, self.release_motor)

        # odometry update, disable during preroll, whenever wheels off ground
        self.odometry_update_enabled = True
        rospy.Service('enable_odometry_updates', std_srvs.srv.SetBool, self.enable_odometry_update_service)

        self.status_pub = rospy.Publisher('status', std_msgs.msg.String, latch=True, queue_size=2)
        self.status = "disconnected"
        self.status_pub.publish(self.status)

        self.command_queue = Queue.Queue(maxsize=5)
        self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)

        self.publish_diagnostics = True
        if self.publish_diagnostics:
            self.diagnostic_updater = diagnostic_updater.Updater()
            self.diagnostic_updater.setHardwareID("Not connected, unknown")
            self.diagnostic_updater.add("ODrive Diagnostics", self.pub_diagnostics)

        if self.publish_temperatures:
            self.temperature_publisher_left  = rospy.Publisher('left/temperature', Float64, queue_size=2)
            self.temperature_publisher_right = rospy.Publisher('right/temperature', Float64, queue_size=2)

        self.i2t_error_latch = False
        if self.publish_current:
            #self.current_loop_count = 0
            #self.left_current_accumulator  = 0.0
            #self.right_current_accumulator = 0.0
            self.current_publisher_left  = rospy.Publisher('left/current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('right/current', Float64, queue_size=2)
            self.i2t_publisher_left  = rospy.Publisher('left/i2t', Float64, queue_size=2)
            self.i2t_publisher_right = rospy.Publisher('right/i2t', Float64, queue_size=2)

            rospy.logdebug("ODrive will publish motor currents.")

            self.i2t_resume_threshold  = get_param('~i2t_resume_threshold',  222)
            self.i2t_warning_threshold = get_param('~i2t_warning_threshold', 333)
            self.i2t_error_threshold   = get_param('~i2t_error_threshold',   666)

        self.last_cmd_vel_time = rospy.Time.now()

        if self.publish_raw_odom:
            self.raw_odom_publisher_encoder_left  = rospy.Publisher('left/raw_odom/encoder',   Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_encoder_right = rospy.Publisher('right/raw_odom/encoder',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_left      = rospy.Publisher('left/raw_odom/velocity',  Int32, queue_size=2) if self.publish_raw_odom else None
            self.raw_odom_publisher_vel_right     = rospy.Publisher('right/raw_odom/velocity', Int32, queue_size=2) if self.publish_raw_odom else None

        if self.publish_odom:
            rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)
            self.old_pos_l = 0
            self.old_pos_r = 0

            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, tcp_nodelay=True, queue_size=2)
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

            # store current location to be updated.
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0

            # setup transform
            self.tf_publisher = tf2_ros.TransformBroadcaster()
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.odom_frame
            self.tf_msg.child_frame_id  = self.base_frame
            self.tf_msg.transform.translation.x = 0.0
            self.tf_msg.transform.translation.y = 0.0
            self.tf_msg.transform.translation.z = 0.0
            self.tf_msg.transform.rotation.x = 0.0
            self.tf_msg.transform.rotation.y = 0.0
            self.tf_msg.transform.rotation.w = 0.0
            self.tf_msg.transform.rotation.z = 1.0

        if self.publish_joint_angles:
            self.joint_state_publisher = rospy.Publisher('/odrive/joint_states', JointState, queue_size=2)

            jsm = JointState()
            self.joint_state_msg = jsm
            #jsm.name.resize(2)
            #jsm.position.resize(2)
            jsm.name = ['joint_left_wheel','joint_right_wheel']
            jsm.position = [0.0, 0.0]
