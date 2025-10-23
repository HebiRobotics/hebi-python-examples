import os, yaml
import numpy as np

# ------------------------------------------------------------------------------
# Controller Mappings
# ------------------------------------------------------------------------------

# Map from mobile IO feedback object to controllable states
class IgorControllerWrapper(object):
    def __init__(self):
        pass

    def arm_vel_x(self, fbk):
        return fbk[0].io.a.get_float(2)

    def arm_vel_y(self, fbk):
        return fbk[0].io.a.get_float(1)

    def stance_height(self, fbk):
        return fbk[0].io.a.get_float(3)

    def wrist_vel(self, fbk):
        return fbk[0].io.a.get_float(6)

    def chassis_yaw(self, fbk):
        return fbk[0].io.a.get_float(7)

    def chassis_vel(self, fbk):
        return fbk[0].io.a.get_float(8)

    def exit_idle_mode(self, fbk):
        return fbk[0].io.b.get_int(3)

    def quit(self, fbk):
        return fbk[0].io.b.get_int(1)

    def balance_controller_toggle(self, fbk):
        return fbk[0].io.b.get_int(2)

    def soft_shutdown(self, fbk):
        return fbk[0].io.b.get_int(4)

    def lower_arm(self, fbk):
        return fbk[0].io.b.get_int(8)

    def raise_arm(self, fbk):
        return fbk[0].io.b.get_int(6)

    def i_term_adjust(self, fbk):
        tmp = fbk[0].io.a.get_float(5)
        # If this is called without the relevant slider on the screen, use the -1 value indicating no I term
        if np.isnan(tmp):
            return -1
        return tmp

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------

class Igor2Config(object):
    """
    Used when starting up Igor II.
    """
    def __init__(self, filename):
        resource_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resources'))
        with open(filename) as config_file:
            config = yaml.safe_load(config_file)
            self.module_names = config['names']
            self.family = config['families']
            self.gains_xml = os.path.join(resource_path, config['gains'])
            self.command_lifetime = config['command_lifetime']
            self.feedback_frequency = config['feedback_frequency']
            
            self._mobile_io_name = config['mobile_io_name']
            self._mobile_io_family = config['mobile_io_family']
            self._mobile_io_frequency = config['mobile_io_frequency']

            self.lean_p = config['lean_p']
            self.lean_i = config['lean_i']
            self.lean_d = config['lean_d']
            self.max_wheel_velocity = config['max_wheel_velocity']
            self.velocity_p = config['velocity_p']
            self.velocity_i = config['velocity_i']
            self.velocity_d = config['velocity_d']

            self.enable_logging = config['enable_logging']
    
        self.controller_mapping = IgorControllerWrapper()

    # Return a mobile IO object (blocking until it is found)
    def get_controller(self):
        import hebi
        from time import sleep
        lookup = hebi.Lookup()
        mio = hebi.util.create_mobile_io(lookup, self._mobile_io_family, self._mobile_io_name)
        while mio is None:
            print(f'Could not find mobileIO on network with\nfamily: {self._mobile_io_family}\nname: {self._mobile_io_name}')
            sleep(1)
            mio = hebi.util.create_mobile_io(lookup, self._mobile_io_family, self._mobile_io_name)
        mio._group.feedback_frequency = self._mobile_io_frequency
        return mio