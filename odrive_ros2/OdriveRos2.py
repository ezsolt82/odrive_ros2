

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

    def tick(self):
        # print('Tick')
        pass

    def shutdown():
        print("Shutting down.")