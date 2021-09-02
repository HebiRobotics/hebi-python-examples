import hebi
from time import sleep


class MobileIO():

    num_buttons = 8
    num_sliders = 8
    current_button_state = [0, 0, 0, 0, 0, 0, 0, 0]
    current_slider_state = [0, 0, 0, 0, 0, 0, 0, 0]

    btn_states = ["", "", "", "", "", "", "", ""]

    def __init__(self, family="HEBI", name="Mobile IO"):
        # Create some stuff
        self.lookup = hebi.Lookup()
        sleep(2)
        print(self.lookup)
        self.grp = self.lookup.get_group_from_names([family], [name])
        print(self.grp)
        self.cmd = hebi.GroupCommand(self.grp.size)
        self.fbk = hebi.GroupFeedback(self.grp.size)
        self.prev_fbk = self.fbk

    def setSnap(self, slider_num, snap_to):
        self.cmd.io.a.set_float(slider_num, snap_to)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setAxisValue(self, slider_num, value):
        self.cmd.io.f.set_float(slider_num, value)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setButtonMode(self, button_num, mode):
        self.cmd.io.b.set_int(button_num, mode)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setButtonOutput(self, button_num, output):
        self.cmd.io.e.set_int(button_num, output)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setLedColor(self, color):
        self.cmd.led.color = color
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setText(self, message):
        self.cmd.append_log = message
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def clearText(self):  # Bugged needs to be looked at
        self.cmd.clear_log()
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def setVibrate(self):
        self.cmd.effort(1)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    def getState(self):
        self.grp.get_next_feedback(reuse_fbk=self.fbk)
        for i in range(self.num_buttons):
            self.current_button_state[i] = int(self.fbk.io.b.get_int(i + 1))
        for i in range(self.num_sliders):
            if self.fbk.io.a.has_int(i + 1):
                self.current_slider_state[i] = int(self.fbk.io.a.get_int(i + 1))
            elif self.fbk.io.a.has_float(i + 1):
                self.current_slider_state[i] = float(self.fbk.io.a.get_float(i + 1))

        return (list(self.current_button_state), list(self.current_slider_state))

    def getDiff(self, prev, curr):
        for i in range(self.num_buttons):
            if (prev[0][i] == 0) & (curr[0][i] == 0):
                self.btn_states[i] = "off"
            elif (prev[0][i] == 1) & (curr[0][i] == 0):
                self.btn_states[i] = "falling"
            elif (prev[0][i] == 0) & (curr[0][i] == 1):
                self.btn_states[i] = "rising"
            elif (prev[0][i] == 1) & (curr[0][i] == 1):
                self.btn_states[i] = "on"
        return self.btn_states


# have grp, fbk, cmd
# set func: set cmd -> send_w_ack
# get func: get next fbk -> pull out vars you need
