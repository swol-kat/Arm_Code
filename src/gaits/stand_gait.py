import gait_parent

class stand_gait(gait_parent):
    def __init__(self, arm_dict, controller, imu):
        super(stand_gait, self).__init__(arm_dict, controller, imu)
    
    def run_loop(self, time):
        self.arm_dict['front_left'].set_tip_force_limit(100, 100, 100)
        self.arm_dict['front_left'].send_to_pos(np.array([[0],[0],[-10]]))

        self.arm_dict['front_right'].set_tip_force_limit(100, 100, 100)
        self.arm_dict['front_right'].send_to_pos(np.array([[0],[0],[-10]]))

        self.arm_dict['back_left'].set_tip_force_limit(100, 100, 100)
        self.arm_dict['back_left'].send_to_pos(np.array([[0],[0],[-10]]))

        self.arm_dict['back_right'].set_tip_force_limit(100, 100, 100)
        self.arm_dict['back_right'].send_to_pos(np.array([[0],[0],[-10]]))

        return