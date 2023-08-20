odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.config.pos_gain=4
odrv0.axis0.controller.config.vel_integrator_gain=8
odrv0.axis0.controller.config.vel_limit = 10

odrv0.axis0.controller.start_anticogging_calibration()
odrv0.axis0.controller.config.anticogging.calib_anticogging
dump_errors(odrv0)

odrv0.axis0.controller.config.anticogging.pre_calibrated = True
odrv0.axis0.controller.config.pos_gain=2
odrv0.axis0.controller.config.vel_integrator_gain=0.1

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.save_configuration()
odrv0.reboot()
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 0.3
