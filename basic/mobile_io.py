import hebi
from time import sleep, time


class MobileIO():

    num_buttons = 8
    num_sliders = 8
    current_button_state = [0, 0, 0, 0, 0, 0, 0, 0]
    current_slider_state = [0, 0, 0, 0, 0, 0, 0, 0]

    btn_states = ["", "", "", "", "", "", "", ""]

    def __init__(self, family="HEBI", name="Mobile IO"):
        # lookup modules on network
        self.lookup = hebi.Lookup()
        # wait 2 seconds for list to populate
        sleep(2)
        print(self.lookup)
        self.grp = self.lookup.get_group_from_names([family], [name])
        print(self.grp)
        self.cmd = hebi.GroupCommand(self.grp.size)
        self.fbk = hebi.GroupFeedback(self.grp.size)
        self.prev_fbk = self.fbk

    # set the snap position on a slider
    def setSnap(self, slider_num, snap_to):
        self.cmd.io.a.set_float(slider_num, snap_to)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # set slider to a value
    def setAxisValue(self, slider_num, value):
        self.cmd.io.f.set_float(slider_num, value)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # set button mode (momentary/toggle)
    def setButtonMode(self, button_num, mode):
        self.cmd.io.b.set_int(button_num, mode)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # set light around button (on/off)
    def setButtonOutput(self, button_num, output):
        self.cmd.io.e.set_int(button_num, output)
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # set edge led color
    def setLedColor(self, color):
        self.cmd.led.color = color
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # send text
    def setText(self, message):
        self.cmd.append_log = message
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # clear text (currently does not work)
    def clearText(self):
        self.cmd.clear_log = True  # bugged needs to be looked at
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # send a buzz to the mobile device (only works on iphones)
    def sendVibrate(self):
        self.cmd.effort = 1
        return self.grp.send_command_with_acknowledgement(self.cmd)

    # returns the current state of all buttons and sliders
    def getState(self):
        self.grp.get_next_feedback(reuse_fbk=self.fbk)
        for i in range(self.num_buttons):
            self.current_button_state[i] = int(self.fbk.io.b.get_int(i + 1))
        for i in range(self.num_sliders):
            if self.fbk.io.a.has_int(i + 1):
                self.current_slider_state[i] = int(self.fbk.io.a.get_int(i + 1))
            elif self.fbk.io.a.has_float(i + 1):
                self.current_slider_state[i] = float(self.fbk.io.a.get_float(i + 1))

        return (self.current_button_state, self.current_slider_state)

    # returns the change in button positions (currently breaks if called in same step as getState())
    def getDiff(self):
        self.prev_fbk = list(self.current_button_state)
        self.getState()
        # compares current button states to previous button states
        for i in range(self.num_buttons):
            if (self.prev_fbk[i] == 0) & (self.current_button_state[i] == 0):
                self.btn_states[i] = "off"
            elif (self.prev_fbk[i] == 1) & (self.current_button_state[i] == 0):
                self.btn_states[i] = "falling"
            elif (self.prev_fbk[i] == 0) & (self.current_button_state[i] == 1):
                self.btn_states[i] = "rising"
            elif (self.prev_fbk[i] == 1) & (self.current_button_state[i] == 1):
                self.btn_states[i] = "on"
        return self.btn_states
