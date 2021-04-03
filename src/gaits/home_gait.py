import gait_parent

class home_gait(gait_parent):
    def __init__(self, arm_dict, controller, imu):
        super(home_gait, self).__init__(arm_dict, controller, imu)
        self.state = 'start'
        
    
    def is_complete(self):
        return self.state == 'complete'

    def run_loop(self, time):
        if self.state == 'start':
            #tell all arms to start calibrating
        
        if self.state == 'wait_calibrate':
            #then do homing

        if self.state == 'wait_home':
            #then enable arms
        
        if self.state == 'wait_enable':
            #and done
        
        if self.state == 'complete':
            #just idle
        return