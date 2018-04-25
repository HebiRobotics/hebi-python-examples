import wx
import wx.grid as gridlib
from pubsub import pub

from time import sleep, time
from threading import Lock, Thread

import hebi
import numpy as np
import math


def f2str(val):
  if abs(val) < 1e-3:
    return '~0'
  else:
    return '{0:.2f}'.format(val)


def do_blend(err, bg_color, err_color, cutoff=0.02, max_err=0.5):
  err = abs(err)
  if math.isnan(err):
    return bg_color
  elif err > cutoff:
    err = min(err, max_err)
    err_range = max_err-cutoff
    err_diff = err-cutoff
    pct = err_diff/err_range
    r = wx.Colour()
    r.SetRGBA(wx.Colour.AlphaBlend(err_color.GetRGBA(), bg_color.GetRGBA(), pct))
    return r
  else:
    return bg_color


class Form(wx.Frame):

  RED = wx.Colour(255, 0, 0, 255)

  def _blend(self, val):
    return do_blend(val, self._default_color, Form.RED)

  def _setup_fbk_frame(self, panel, font):
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
    l_leg_st.SetFont(font)
    r_leg_st.SetFont(font)
    l_arm_st.SetFont(font)
    r_arm_st.SetFont(font)
    wheel_st.SetFont(font)

    # Grids
    l_leg_grid = gridlib.Grid(panel)
    r_leg_grid = gridlib.Grid(panel)
    l_arm_grid = gridlib.Grid(panel)
    r_arm_grid = gridlib.Grid(panel)
    wheel_grid = gridlib.Grid(panel)

    # Create size
    l_leg_grid.CreateGrid(6, 2)
    r_leg_grid.CreateGrid(6, 2)
    l_arm_grid.CreateGrid(6, 4)
    r_arm_grid.CreateGrid(6, 4)
    wheel_grid.CreateGrid(4, 2)

    # Set rows for all
    l_leg_grid.SetRowLabelValue(0, "Position")
    l_leg_grid.SetRowLabelValue(1, "PosERROR")
    r_leg_grid.SetRowLabelValue(0, "Position")
    r_leg_grid.SetRowLabelValue(1, "PosERROR")
    l_arm_grid.SetRowLabelValue(0, "Position")
    l_arm_grid.SetRowLabelValue(1, "PosERROR")
    r_arm_grid.SetRowLabelValue(0, "Position")
    r_arm_grid.SetRowLabelValue(1, "PosERROR")

    wheel_grid.SetRowLabelValue(0, "Velocity")
    wheel_grid.SetRowLabelValue(1, "VelERROR")
    wheel_grid.SetRowLabelValue(2, "Effort")
    wheel_grid.SetRowLabelValue(3, "EffERROR")

    l_leg_grid.SetRowLabelValue(2, "Velocity")
    l_leg_grid.SetRowLabelValue(3, "VelERROR")
    r_leg_grid.SetRowLabelValue(2, "Velocity")
    r_leg_grid.SetRowLabelValue(3, "VelERROR")
    l_arm_grid.SetRowLabelValue(2, "Velocity")
    l_arm_grid.SetRowLabelValue(3, "VelERROR")
    r_arm_grid.SetRowLabelValue(2, "Velocity")
    r_arm_grid.SetRowLabelValue(3, "VelERROR")
    l_leg_grid.SetRowLabelValue(4, "Effort")
    l_leg_grid.SetRowLabelValue(5, "EffERROR")
    r_leg_grid.SetRowLabelValue(4, "Effort")
    r_leg_grid.SetRowLabelValue(5, "EffERROR")
    l_arm_grid.SetRowLabelValue(4, "Effort")
    l_arm_grid.SetRowLabelValue(5, "EffERROR")
    r_arm_grid.SetRowLabelValue(4, "Effort")
    r_arm_grid.SetRowLabelValue(5, "EffERROR")

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

    # ----
    # Data

    hbox_l_leg.Add(l_leg_st, flag=wx.RIGHT, border=5)
    hbox_l_leg.Add(l_leg_grid, flag=wx.EXPAND, proportion=1)

    hbox_r_leg.Add(r_leg_st, flag=wx.RIGHT, border=5)
    hbox_r_leg.Add(r_leg_grid, flag=wx.EXPAND, proportion=1)

    hbox_l_arm.Add(l_arm_st, flag=wx.RIGHT, border=5)
    hbox_l_arm.Add(l_arm_grid, flag=wx.EXPAND, proportion=1)

    hbox_r_arm.Add(r_arm_st, flag=wx.RIGHT, border=5)
    hbox_r_arm.Add(r_arm_grid, flag=wx.EXPAND, proportion=1)

    hbox_wheel.Add(wheel_st, flag=wx.RIGHT, border=5)
    hbox_wheel.Add(wheel_grid, flag=wx.EXPAND, proportion=1)

    # Add all to parent sizer
    vbox_cmd.Add(wx.StaticText(panel, label='Current Feedback Data'), flag=wx.ALIGN_CENTER)
    vbox_cmd.Add((-1, 2))
    vbox_cmd.Add(wx.StaticLine(panel, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL), flag=wx.EXPAND|wx.LEFT|wx.RIGHT, border=10)
    vbox_cmd.Add((-1, 2))
    vbox_cmd.Add(hbox_l_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=5)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_r_leg, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=5)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_l_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=5)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_r_arm, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=5)
    vbox_cmd.Add((-1, 10))
    vbox_cmd.Add(hbox_wheel, flag=wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP, border=5)
    vbox_cmd.Add((-1, 10))
    self._vbox_cmd = vbox_cmd

  def _setup_igor_frame(self, panel, font):
    vbox_igr = wx.BoxSizer(wx.VERTICAL)
    vbox_igr.Add(wx.StaticText(panel, label='Current Igor State'), flag=wx.ALIGN_CENTER)
    vbox_igr.Add((-1, 2))
    vbox_igr.Add(wx.StaticLine(panel, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_HORIZONTAL),
                 flag=wx.EXPAND | wx.LEFT | wx.RIGHT, border=10)
    vbox_igr.Add((-1, 2))

    hbox_roll_angle = wx.BoxSizer(wx.HORIZONTAL)
    hbox_pitch_angle = wx.BoxSizer(wx.HORIZONTAL)
    hbox_height_com = wx.BoxSizer(wx.HORIZONTAL)
    hbox_fbk_lean_angle = wx.BoxSizer(wx.HORIZONTAL)
    hbox_fbk_lean_angle_velocity = wx.BoxSizer(wx.HORIZONTAL)
    hbox_lean_angle_error = wx.BoxSizer(wx.HORIZONTAL)
    hbox_lean_angle_error_cumulative = wx.BoxSizer(wx.HORIZONTAL)
    hbox_chassis_velocity_error = wx.BoxSizer(wx.HORIZONTAL)
    hbox_chassis_velocity_error_cumulative = wx.BoxSizer(wx.HORIZONTAL)
    hbox_calculated_lean_angle = wx.BoxSizer(wx.HORIZONTAL)
    hbox_calculated_directional_velocity = wx.BoxSizer(wx.HORIZONTAL)
    hbox_calculated_yaw_velocity = wx.BoxSizer(wx.HORIZONTAL)
    hbox_calculated_knee_velocity = wx.BoxSizer(wx.HORIZONTAL)
    hbox_lean_ff = wx.BoxSizer(wx.HORIZONTAL)
    hbox_vel_ff = wx.BoxSizer(wx.HORIZONTAL)

    roll_angle_st = wx.StaticText(panel,                 label='Roll Angle                 : ')
    pitch_angle_st = wx.StaticText(panel,                label='Pitch Angle                : ')
    height_com_st = wx.StaticText(panel,                 label='Height CoM                 : ')
    fbk_lean_angle_st = wx.StaticText(panel,             label='Fbk Lean Angle             : ')
    fbk_lean_angle_vel_st = wx.StaticText(panel,         label='Fbk Lean Angle Vel         : ')
    lean_angle_err_st = wx.StaticText(panel,             label='Lean Angle Err             : ')
    lean_angle_err_cumulative_st = wx.StaticText(panel,  label='Lean Angle Err Cumulative  : ')
    chassis_vel_err_st = wx.StaticText(panel,            label='Chassis Vel Err            : ')
    chassis_vel_err_cumulative_st = wx.StaticText(panel, label='Chassis Vel Err Cumulative : ')
    calc_lean_angle = wx.StaticText(panel,               label='Calculated Lean Angle      : ')
    calc_directional_vel = wx.StaticText(panel,          label='Calculated Directional Vel : ')
    calc_yaw_vel = wx.StaticText(panel,                  label='Calculated Yaw Vel         : ')
    calc_knee_vel = wx.StaticText(panel,                 label='Calculated Knee Vel        : ')
    calc_lean_feed_fwd = wx.StaticText(panel,            label='Lean FeedForward           : ')
    calc_feed_fwd = wx.StaticText(panel,                 label='Velocity FeedForward       : ')

    self._roll_angle_st_val = roll_angle_st_val = wx.StaticText(panel, label='')
    self._pitch_angle_st_val = pitch_angle_st_val = wx.StaticText(panel, label='')
    self._height_com_st_val = height_com_st_val = wx.StaticText(panel, label='')
    self._fbk_lean_angle_st_val = fbk_lean_angle_st_val = wx.StaticText(panel, label='')
    self._fbk_lean_angle_vel_st_val = fbk_lean_angle_vel_st_val = wx.StaticText(panel, label='')
    self._lean_angle_err_st_val = lean_angle_err_st_val = wx.StaticText(panel, label='')
    self._lean_angle_err_cumulative_st_val = lean_angle_err_cumulative_st_val = wx.StaticText(panel, label='')
    self._chassis_vel_err_st_val = chassis_vel_err_st_val = wx.StaticText(panel, label='')
    self._chassis_vel_err_cumulative_st_val = chassis_vel_err_cumulative_st_val = wx.StaticText(panel, label='')
    self._calc_lean_angle_st_val = calc_lean_angle_st_val = wx.StaticText(panel, label='')
    self._calc_directional_vel_st_val = calc_directional_vel_st_val = wx.StaticText(panel, label='')
    self._calc_yaw_vel_st_val = calc_yaw_vel_st_val = wx.StaticText(panel, label='')
    self._calc_knee_vel_st_val = calc_knee_vel_st_val = wx.StaticText(panel, label='')
    self._calc_lean_feed_fwd_st_val = calc_lean_feed_fwd_st_val = wx.StaticText(panel, label='')
    self._calc_feed_fwd_st_val = calc_feed_fwd_st_val = wx.StaticText(panel, label='')

    roll_angle_st.SetFont(font)
    roll_angle_st_val.SetFont(font)
    pitch_angle_st.SetFont(font)
    pitch_angle_st_val.SetFont(font)
    height_com_st.SetFont(font)
    height_com_st_val.SetFont(font)
    fbk_lean_angle_st.SetFont(font)
    fbk_lean_angle_st_val.SetFont(font)
    fbk_lean_angle_vel_st.SetFont(font)
    fbk_lean_angle_vel_st_val.SetFont(font)
    lean_angle_err_st.SetFont(font)
    lean_angle_err_st_val.SetFont(font)
    lean_angle_err_cumulative_st.SetFont(font)
    lean_angle_err_cumulative_st_val.SetFont(font)
    chassis_vel_err_st.SetFont(font)
    chassis_vel_err_st_val.SetFont(font)
    chassis_vel_err_cumulative_st.SetFont(font)
    chassis_vel_err_cumulative_st_val.SetFont(font)
    calc_lean_angle.SetFont(font)
    calc_lean_angle_st_val.SetFont(font)
    calc_directional_vel.SetFont(font)
    calc_directional_vel_st_val.SetFont(font)
    calc_yaw_vel.SetFont(font)
    calc_yaw_vel_st_val.SetFont(font)
    calc_knee_vel.SetFont(font)
    calc_knee_vel_st_val.SetFont(font)
    calc_lean_feed_fwd.SetFont(font)
    calc_lean_feed_fwd_st_val.SetFont(font)
    calc_feed_fwd.SetFont(font)
    calc_feed_fwd_st_val.SetFont(font)

    hbox_roll_angle.Add(roll_angle_st)
    hbox_roll_angle.Add(roll_angle_st_val)
    hbox_pitch_angle.Add(pitch_angle_st)
    hbox_pitch_angle.Add(pitch_angle_st_val)
    hbox_height_com.Add(height_com_st)
    hbox_height_com.Add(height_com_st_val)
    hbox_fbk_lean_angle.Add(fbk_lean_angle_st)
    hbox_fbk_lean_angle.Add(fbk_lean_angle_st_val)
    hbox_fbk_lean_angle_velocity.Add(fbk_lean_angle_vel_st)
    hbox_fbk_lean_angle_velocity.Add(fbk_lean_angle_vel_st_val)
    hbox_lean_angle_error.Add(lean_angle_err_st)
    hbox_lean_angle_error.Add(lean_angle_err_st_val)
    hbox_lean_angle_error_cumulative.Add(lean_angle_err_cumulative_st)
    hbox_lean_angle_error_cumulative.Add(lean_angle_err_cumulative_st_val)
    hbox_chassis_velocity_error.Add(chassis_vel_err_st)
    hbox_chassis_velocity_error.Add(chassis_vel_err_st_val)
    hbox_chassis_velocity_error_cumulative.Add(chassis_vel_err_cumulative_st)
    hbox_chassis_velocity_error_cumulative.Add(chassis_vel_err_cumulative_st_val)
    hbox_calculated_lean_angle.Add(calc_lean_angle)
    hbox_calculated_lean_angle.Add(calc_lean_angle_st_val)
    hbox_calculated_directional_velocity.Add(calc_directional_vel)
    hbox_calculated_directional_velocity.Add(calc_directional_vel_st_val)
    hbox_calculated_yaw_velocity.Add(calc_yaw_vel)
    hbox_calculated_yaw_velocity.Add(calc_yaw_vel_st_val)
    hbox_calculated_knee_velocity.Add(calc_knee_vel)
    hbox_calculated_knee_velocity.Add(calc_knee_vel_st_val)
    hbox_lean_ff.Add(calc_lean_feed_fwd)
    hbox_lean_ff.Add(calc_lean_feed_fwd_st_val)
    hbox_vel_ff.Add(calc_feed_fwd)
    hbox_vel_ff.Add(calc_feed_fwd_st_val)

    vbox_igr.Add(hbox_roll_angle)
    vbox_igr.Add(hbox_pitch_angle)
    vbox_igr.Add(hbox_height_com)
    vbox_igr.Add((-1, 3))
    vbox_igr.Add(hbox_fbk_lean_angle)
    vbox_igr.Add(hbox_fbk_lean_angle_velocity)
    vbox_igr.Add((-1, 3))
    vbox_igr.Add(hbox_lean_angle_error)
    vbox_igr.Add(hbox_lean_angle_error_cumulative)
    vbox_igr.Add((-1, 3))
    vbox_igr.Add(hbox_chassis_velocity_error)
    vbox_igr.Add(hbox_chassis_velocity_error_cumulative)
    vbox_igr.Add((-1, 3))
    vbox_igr.Add(hbox_lean_ff)
    vbox_igr.Add(hbox_vel_ff)
    vbox_igr.Add((-1, 3))
    vbox_igr.Add(hbox_calculated_lean_angle)
    vbox_igr.Add(hbox_calculated_directional_velocity)
    vbox_igr.Add(hbox_calculated_yaw_velocity)
    vbox_igr.Add(hbox_calculated_knee_velocity)

    self._vbox_igr = vbox_igr

  def update_fbk(self, fbk):
    self._fbk_lock.acquire()
    np.copyto(self._positions_fbk, fbk.position)
    np.copyto(self._position_cmds_fbk, fbk.position_command)
    np.subtract(self._position_cmds_fbk, self._positions_fbk, out=self._position_fbk_err)

    np.copyto(self._velocities_fbk, fbk.velocity)
    np.copyto(self._velocity_cmds_fbk, fbk.velocity_command)
    np.subtract(self._velocity_cmds_fbk, self._velocities_fbk, out=self._velocity_fbk_err)

    np.copyto(self._efforts_fbk, fbk.effort)
    np.copyto(self._effort_cmds_fbk, fbk.effort_command)
    np.subtract(self._effort_cmds_fbk, self._efforts_fbk, out=self._effort_fbk_err)

    np.copyto(self._gyros_fbk, fbk.gyro)
    np.copyto(self._orientations_fbk, fbk.orientation)
    self._fbk_lock.release()

  def __init__(self, igor):
    self._igor = igor
    self._group_command = hebi.GroupCommand(igor.group.size)

    # -------------
    # Feedback Data
    self._last_fbk_time = 0.0
    self._fbk_lock = Lock()

    self._positions_fbk = np.empty(14, dtype=float)
    self._position_cmds_fbk = np.empty(14, dtype=float)
    self._position_fbk_err = np.empty(14, dtype=float)

    self._velocities_fbk = np.empty(14, dtype=float)
    self._velocity_cmds_fbk = np.empty(14, dtype=float)
    self._velocity_fbk_err = np.empty(14, dtype=float)

    self._efforts_fbk = np.empty(14, dtype=float)
    self._effort_cmds_fbk = np.empty(14, dtype=float)
    self._effort_fbk_err = np.empty(14, dtype=float)

    self._gyros_fbk = np.empty((14, 3), dtype=float)
    self._orientations_fbk = np.empty((14, 4), dtype=float)

    wx.Frame.__init__(self, parent=None, title="Igor II Data")
    c = self.GetBackgroundColour()
    #c.Set(c.Red, c.Green, c.Blue, 255)
    self._default_color = c
    self.SetAutoLayout(False)
    panel = wx.Panel(self)
    self.SetAutoLayout(False)
    panel.SetAutoLayout(False)

    # -----------------
    # Set up components
    font = wx.Font(8, wx.TELETYPE, wx.NORMAL, wx.NORMAL)
    self._setup_fbk_frame(panel, font)
    self._setup_igor_frame(panel, font)

    #
    vbox = wx.BoxSizer(wx.HORIZONTAL)
    vbox.Add(self._vbox_cmd, flag=wx.ALIGN_LEFT)
    vbox.Add(2, -1)
    vbox.Add(wx.StaticLine(panel, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.LI_VERTICAL),
                 flag=wx.EXPAND | wx.BOTTOM | wx.TOP, border=5)
    vbox.Add(2, -1)
    vbox.Add(self._vbox_igr)

    vbox.Fit(self)
    panel.SetSizer(vbox)
    pub.subscribe(self._update_container, "update")
    panel.Layout()

  def _update_container(self):
    igor = self._igor

    self._roll_angle_st_val.SetLabel(f2str(igor.roll_angle))
    self._pitch_angle_st_val.SetLabel(f2str(igor.pitch_angle))
    self._height_com_st_val.SetLabel(f2str(igor.height_com))
    self._fbk_lean_angle_st_val.SetLabel(f2str(igor.feedback_lean_angle))
    self._fbk_lean_angle_vel_st_val.SetLabel(f2str(igor.feedback_lean_angle_velocity))
    self._lean_angle_err_st_val.SetLabel(f2str(igor.chassis.lean_angle_error))
    self._lean_angle_err_cumulative_st_val.SetLabel(f2str(igor.chassis.lean_angle_error_cumulative))
    self._chassis_vel_err_st_val.SetLabel(f2str(igor.chassis.velocity_error))
    self._chassis_vel_err_cumulative_st_val.SetLabel(f2str(igor.chassis.velocity_error_cumulative))
    self._calc_lean_feed_fwd_st_val.SetLabel(f2str(igor.chassis.lean_feedforward))
    self._calc_feed_fwd_st_val.SetLabel(f2str(igor.chassis.velocity_feedforward))
    self._calc_lean_angle_st_val.SetLabel(f2str(igor.chassis.calculated_lean_angle))
    self._calc_directional_vel_st_val.SetLabel(f2str(igor.chassis.calculated_directional_velocity))
    self._calc_yaw_vel_st_val.SetLabel(f2str(igor.chassis.calculated_yaw_velocity))
    self._calc_knee_vel_st_val.SetLabel(f2str(igor.chassis.calculated_knee_velocity))

    l_leg = igor.left_leg
    r_leg = igor.right_leg
    l_arm = igor.left_arm
    r_arm = igor.right_arm

    l_leg_i = l_leg.group_indices
    r_leg_i = r_leg.group_indices
    l_arm_i = l_arm.group_indices
    r_arm_i = r_arm.group_indices

    pos = self._positions_fbk
    pos_cmd = self._position_cmds_fbk
    pos_err = self._position_fbk_err
    vel = self._velocities_fbk
    vel_cmd = self._velocity_cmds_fbk
    vel_err = self._velocity_fbk_err
    eff = self._effort_cmds_fbk
    eff_cmd = self._effort_cmds_fbk
    eff_err = self._effort_fbk_err
    gyro = self._gyros_fbk
    orientation = self._orientations_fbk

    self._fbk_lock.acquire()

    for i in range(2):

      # -----------------------------------------
      # Left Leg
      idx = l_leg_i[i]
      self._l_leg_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._l_leg_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._l_leg_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._l_leg_grid.SetCellValue(1, i, f2str(err))
      self._l_leg_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._l_leg_grid.SetCellValue(3, i, f2str(err))
      self._l_leg_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._l_leg_grid.SetCellValue(5, i, f2str(err))
      self._l_leg_grid.SetCellBackgroundColour(5, i, c)

      # -----------------------------------------
      # Right Leg
      idx = r_leg_i[i]
      self._r_leg_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._r_leg_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._r_leg_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._r_leg_grid.SetCellValue(1, i, f2str(err))
      self._r_leg_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._r_leg_grid.SetCellValue(3, i, f2str(err))
      self._r_leg_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._r_leg_grid.SetCellValue(5, i, f2str(err))
      self._r_leg_grid.SetCellBackgroundColour(5, i, c)

      # -----------------------------------------
      # Left Arm
      idx = l_arm_i[i]
      self._l_arm_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._l_arm_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._l_arm_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(1, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(3, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(5, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(5, i, c)

      # -----------------------------------------
      # Right Arm
      idx = r_arm_i[i]
      self._r_arm_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._r_arm_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._r_arm_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(1, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(3, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(5, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(5, i, c)

    for i in range(2, 4):
      # -----------------------------------------
      # Left Arm
      idx = l_arm_i[i]
      self._l_arm_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._l_arm_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._l_arm_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(1, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(3, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._l_arm_grid.SetCellValue(5, i, f2str(err))
      self._l_arm_grid.SetCellBackgroundColour(5, i, c)

      # -----------------------------------------
      # Right Arm
      idx = r_arm_i[i]
      self._r_arm_grid.SetCellValue(0, i, f2str(pos[idx]))
      self._r_arm_grid.SetCellValue(2, i, f2str(vel[idx]))
      self._r_arm_grid.SetCellValue(4, i, f2str(eff[idx]))

      err = pos_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(1, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(1, i, c)

      err = vel_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(3, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(3, i, c)

      err = eff_err[idx]
      c = self._blend(err)
      self._r_arm_grid.SetCellValue(5, i, f2str(err))
      self._r_arm_grid.SetCellBackgroundColour(5, i, c)

    self._wheel_grid.SetCellValue(0, 0, f2str(vel[0]))
    self._wheel_grid.SetCellValue(0, 1, f2str(vel[1]))

    err = vel_err[0]
    c = self._blend(err)
    self._wheel_grid.SetCellValue(1, 0, f2str(err))
    self._wheel_grid.SetCellBackgroundColour(1, 0, c)

    err = vel_err[1]
    c = self._blend(err)
    self._wheel_grid.SetCellValue(1, 1, f2str(err))
    self._wheel_grid.SetCellBackgroundColour(1, 1, c)

    self._wheel_grid.SetCellValue(2, 0, f2str(eff[0]))
    self._wheel_grid.SetCellValue(2, 1, f2str(eff[1]))

    err = eff_err[0]
    c = self._blend(err)
    self._wheel_grid.SetCellValue(3, 0, f2str(err))
    self._wheel_grid.SetCellBackgroundColour(3, 0, c)

    err = eff_err[1]
    c = self._blend(err)
    self._wheel_grid.SetCellValue(3, 1, f2str(err))
    self._wheel_grid.SetCellBackgroundColour(3, 1, c)

    self._fbk_lock.release()

  def request_update(self):
    now = time()
    dt = now - self._last_fbk_time
    if dt > 0.05:
      # Limit to <20Hz
      self._last_fbk_time = now
      self.update_fbk(self._igor._group_feedback)
      wx.CallAfter(pub.sendMessage, "update")


global _gui_frame
_gui_frame = None

def _start_gui(igor):
  app = wx.App()
  frame = Form(igor)
  frame.Show()

  global _gui_frame
  _gui_frame = frame

  app.MainLoop()


def request_update():
  if not _gui_frame is None:
    _gui_frame.request_update()
