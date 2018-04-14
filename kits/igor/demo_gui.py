import wx
import wx.grid as gridlib
from pubsub import pub

import hebi
import numpy as np

class Form(wx.Frame):

  def __init__(self, igor):
    self._igor = igor
    self._group_command = hebi.GroupCommand(igor.group.size)

    wx.Frame.__init__(self, parent=None, title="Igor II Data")
    panel = wx.Panel(self)

    vbox = wx.BoxSizer(wx.VERTICAL)
    vbox_cmd = wx.BoxSizer(wx.VERTICAL)
    hbox_l_leg = wx.BoxSizer(wx.HORIZONTAL)
    hbox_r_leg = wx.BoxSizer(wx.HORIZONTAL)
    hbox_l_arm = wx.BoxSizer(wx.HORIZONTAL)
    hbox_r_arm = wx.BoxSizer(wx.HORIZONTAL)
    hbox_wheel = wx.BoxSizer(wx.HORIZONTAL)

    # Labels
    l_leg_st = wx.StaticText(panel, label='Left Leg')
    r_leg_st = wx.StaticText(panel, label='Right Leg')
    l_arm_st = wx.StaticText(panel, label='Left Arm')
    r_arm_st = wx.StaticText(panel, label='Right Arm')
    wheel_st = wx.StaticText(panel, label='Wheels')

    # Grids
    l_leg_grid = gridlib.Grid(panel)
    r_leg_grid = gridlib.Grid(panel)
    l_arm_grid = gridlib.Grid(panel)
    r_arm_grid = gridlib.Grid(panel)
    wheel_grid = gridlib.Grid(panel)

    # Create size
    l_leg_grid.CreateGrid(3, 2)
    r_leg_grid.CreateGrid(3, 2)
    l_arm_grid.CreateGrid(3, 4)
    r_arm_grid.CreateGrid(3, 4)
    wheel_grid.CreateGrid(1, 2)

    # Set rows for all
    l_leg_grid.SetRowLabelValue(0, "Position")
    r_leg_grid.SetRowLabelValue(0, "Position")
    l_arm_grid.SetRowLabelValue(0, "Position")
    r_arm_grid.SetRowLabelValue(0, "Position")
    wheel_grid.SetRowLabelValue(0, "Velocity")
    l_leg_grid.SetRowLabelValue(1, "Velocity")
    r_leg_grid.SetRowLabelValue(1, "Velocity")
    l_arm_grid.SetRowLabelValue(1, "Velocity")
    r_arm_grid.SetRowLabelValue(1, "Velocity")
    l_leg_grid.SetRowLabelValue(2, "Effort")
    r_leg_grid.SetRowLabelValue(2, "Effort")
    l_arm_grid.SetRowLabelValue(2, "Effort")
    r_arm_grid.SetRowLabelValue(2, "Effort")

    # Set leg cols
    l_leg_grid.SetColLabelValue(0, "Hip")
    l_leg_grid.SetColLabelValue(1, "Knee")
    r_leg_grid.SetColLabelValue(0, "Hip")
    r_leg_grid.SetColLabelValue(1, "Knee")

    # Set arm cols
    l_arm_grid.SetColLabelValue(0, "Base")
    r_arm_grid.SetColLabelValue(0, "Base")
    l_arm_grid.SetColLabelValue(1, "Shoulder")
    r_arm_grid.SetColLabelValue(1, "Shoulder")
    l_arm_grid.SetColLabelValue(2, "Elbow")
    r_arm_grid.SetColLabelValue(2, "Elbow")
    l_arm_grid.SetColLabelValue(3, "Wrist")
    r_arm_grid.SetColLabelValue(3, "Wrist")

    # Wheel cols
    wheel_grid.SetColLabelValue(0, "Left")
    wheel_grid.SetColLabelValue(1, "Right")

    self._l_leg_grid = l_leg_grid
    self._r_leg_grid = r_leg_grid
    self._l_arm_grid = l_arm_grid
    self._r_arm_grid = r_arm_grid
    self._wheel_grid = wheel_grid

    # Data
    self._l_leg_pos = np.empty(2, dtype=float)
    self._l_leg_vel = np.empty(2, dtype=float)
    self._l_leg_eff = np.empty(2, dtype=float)
    self._r_leg_pos = np.empty(2, dtype=float)
    self._r_leg_vel = np.empty(2, dtype=float)
    self._r_leg_eff = np.empty(2, dtype=float)
    self._l_arm_pos = np.empty(4, dtype=float)
    self._l_arm_vel = np.empty(4, dtype=float)
    self._l_arm_eff = np.empty(4, dtype=float)
    self._r_arm_pos = np.empty(4, dtype=float)
    self._r_arm_vel = np.empty(4, dtype=float)
    self._r_arm_eff = np.empty(4, dtype=float)

    self._l_wheel_vel = 0.0
    self._r_wheel_vel = 0.0

    hbox_l_leg.Add(l_leg_st, flag=wx.RIGHT, border=8)
    hbox_l_leg.Add(l_leg_grid, proportion=1)

    hbox_r_leg.Add(r_leg_st, flag=wx.RIGHT, border=8)
    hbox_r_leg.Add(r_leg_grid, proportion=1)

    hbox_l_arm.Add(l_arm_st, flag=wx.RIGHT, border=8)
    hbox_l_arm.Add(l_arm_grid, proportion=1)

    hbox_r_arm.Add(r_arm_st, flag=wx.RIGHT, border=8)
    hbox_r_arm.Add(r_arm_grid, proportion=1)

    hbox_wheel.Add(wheel_st, flag=wx.RIGHT, border=8)
    hbox_wheel.Add(wheel_grid, proportion=1)

    vbox_cmd.Add(hbox_l_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_r_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_l_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_r_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_wheel, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=10)
    vbox_cmd.Add((-1, 10))

    vbox_cmd.Fit(self) # change to vbox
    panel.SetSizer(vbox_cmd) # change to vbox
    pub.subscribe(self._update_container, "update")
    panel.Layout()

  def _update_container(self):
    igor = self._igor
    group_command = self._group_command

    l_leg = igor.left_leg
    r_leg = igor.right_leg
    l_arm = igor.left_arm
    r_arm = igor.right_arm

    l_leg_i = l_leg.group_indices
    r_leg_i = r_leg.group_indices
    l_arm_i = l_arm.group_indices
    r_arm_i = r_arm.group_indices

    pos = group_command.position
    vel = group_command.velocity
    eff = group_command.effort

    self._l_leg_pos[:] = pos[l_leg_i]
    self._l_leg_vel[:] = vel[l_leg_i]
    self._l_leg_eff[:] = eff[l_leg_i]
    self._r_leg_pos[:] = pos[r_leg_i]
    self._r_leg_vel[:] = vel[r_leg_i]
    self._r_leg_eff[:] = eff[r_leg_i]
    self._l_arm_pos[:] = pos[l_arm_i]
    self._l_arm_vel[:] = vel[l_arm_i]
    self._l_arm_eff[:] = eff[l_arm_i]
    self._r_arm_pos[:] = pos[r_arm_i]
    self._r_arm_vel[:] = vel[r_arm_i]
    self._r_arm_eff[:] = eff[r_arm_i]

    self._l_wheel_vel = vel[0]
    self._r_wheel_vel = vel[1]

    for i in range(2):
      self._l_leg_grid.SetCellValue(0, i, str(self._l_leg_pos[i]))
      self._l_leg_grid.SetCellValue(1, i, str(self._l_leg_vel[i]))
      self._l_leg_grid.SetCellValue(2, i, str(self._l_leg_eff[i]))

      self._r_leg_grid.SetCellValue(0, i, str(self._r_leg_pos[i]))
      self._r_leg_grid.SetCellValue(1, i, str(self._r_leg_vel[i]))
      self._r_leg_grid.SetCellValue(2, i, str(self._r_leg_eff[i]))

      self._l_arm_grid.SetCellValue(0, i, str(self._l_arm_pos[i]))
      self._l_arm_grid.SetCellValue(1, i, str(self._l_arm_vel[i]))
      self._l_arm_grid.SetCellValue(2, i, str(self._l_arm_eff[i]))

      self._r_arm_grid.SetCellValue(0, i, str(self._r_arm_pos[i]))
      self._r_arm_grid.SetCellValue(1, i, str(self._r_arm_vel[i]))
      self._r_arm_grid.SetCellValue(2, i, str(self._r_arm_eff[i]))

    for i in range(2, 4):
      self._l_arm_grid.SetCellValue(0, i, str(self._l_arm_pos[i]))
      self._l_arm_grid.SetCellValue(1, i, str(self._l_arm_vel[i]))
      self._l_arm_grid.SetCellValue(2, i, str(self._l_arm_eff[i]))

      self._r_arm_grid.SetCellValue(0, i, str(self._r_arm_pos[i]))
      self._r_arm_grid.SetCellValue(1, i, str(self._r_arm_vel[i]))
      self._r_arm_grid.SetCellValue(2, i, str(self._r_arm_eff[i]))

    self._wheel_grid.SetCellValue(0, 0, str(self._l_wheel_vel))
    self._wheel_grid.SetCellValue(0, 1, str(self._r_wheel_vel))

  def request_update(self):
    self._group_command.position = self._igor._group_command.position
    self._group_command.velocity = self._igor._group_command.velocity
    self._group_command.effort = self._igor._group_command.effort
    wx.CallAfter(pub.sendMessage, "update")


global _gui_frame

def _start_gui(igor):
  app = wx.App()
  frame = Form(igor)
  frame.Show()

  global _gui_frame
  _gui_frame = frame

  app.MainLoop()


def request_update():
  _gui_frame.request_update()