
class gait_parent:
    def __init__(self, arm_dict, controller, imu):
        # things happen here
        self.arm_dict = arm_dict
        self.controller = controller
        self.imu = imu
    
    def run_loop(self, time):
        #override this. just takes in current time and updates arm positions
        pass