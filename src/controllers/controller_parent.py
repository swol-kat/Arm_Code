class controller_parent:
    def __init__(self):
        # things happen here
        pass
    
    def get_variables(self):
        # override this. Should be a dict containing:
        #   commanded velocity (x, y, z)
        #   commanded rotation velocity (about x, y, z)
        #   commanded ride height (z)
        out_dict = {'vel_x':0.0, 'vel_y':0.0, 'vel_z':0.0, 'rot_x':0.0, 'rot_y':0.0, 'rot_z':0.0, 'ride_height':0.0}
        return out_dict