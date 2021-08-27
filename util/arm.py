import hebi
import numpy as np
import os

from .math_utils import gravity_from_quaternion


# Where the HRDF, safety params and gains files are for the arms
_arm_resource_local_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'kits', 'arm'))

# The shoulder joint (effort) compensation for most kits which has a gas spring
default_shoulder_joint_comp = -7.0


# ------------------------------------------------------------------------------
# Arm and arm component definitions
# ------------------------------------------------------------------------------


class ArmParams(object):
    __slots__ = ["_robot_name", "_gains", "_has_gripper", "_gripper_open_effort",
                 "_gripper_close_effort", "_gripper_gains", "_effort_offset",
                 "_ik_seed_pos", "_gravity_vec"]

    def __init__(self, robot_name, effort_offset, ik_seed_pos, has_gripper=False, gripper_open_effort=None, gripper_close_effort=None, gripper_gains=None):
        self._robot_name = robot_name

        gains_path = os.path.join(_arm_resource_local_dir, "gains", robot_name + "_gains.xml")
        gains = hebi.GroupCommand(len(ik_seed_pos))
        try:
            gains.read_gains(gains_path)
        except Exception as e:
            import sys
            sys.stderr.write("Could not load gains from file {0}\n".format(gains_path))
            raise e

        self._gains = gains
        self._has_gripper = has_gripper
        self._gripper_open_effort = gripper_open_effort
        self._gripper_close_effort = gripper_close_effort
        self._gripper_gains = gripper_gains
        self._effort_offset = np.asarray(effort_offset)
        self._ik_seed_pos = ik_seed_pos
        self._gravity_vec = np.zeros(3, dtype=np.float32)

    @property
    def robot_name(self):
        return self._robot_name

    @property
    def gains(self):
        return self._gains

    @property
    def has_gripper(self):
        return self._has_gripper

    @property
    def gripper_open_effort(self):
        return self._gripper_open_effort

    @property
    def gripper_close_effort(self):
        return self._gripper_close_effort

    @property
    def gripper_gains(self):
        return self._gripper_gains

    @property
    def effort_offset(self):
        return self._effort_offset

    @property
    def ik_seed_pos(self):
        return self._ik_seed_pos

    @property
    def gravity_vec(self):
        return self._gravity_vec

    @property
    def local_dir(self):
        return _arm_resource_local_dir

    def update_gravity(self, fbk):
        gravity_from_quaternion(fbk.orientation[0], output=self._gravity_vec)


__arm_setup_params_dict = {
    '6-DoF + gripper': (ArmParams("6-DoF_arm",
                                  effort_offset=[0, default_shoulder_joint_comp, 0, 0, 0, 0],
                                  ik_seed_pos=[0.01, 1.0, 2.5, 1.5, -1.5, 0.01],
                                  has_gripper=True, gripper_open_effort=1, gripper_close_effort=-5,
                                  gripper_gains="gripper_spool_gains"),
                        ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]),

    '6-DoF': (ArmParams("6-DoF_arm",
                        effort_offset=[0, default_shoulder_joint_comp, 0, 0, 0, 0],
                        ik_seed_pos=[0.01, 1.0, 2.5, 1.5, -1.5, 0.01]),
              ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"]),

    '5-DoF + gripper': (ArmParams("5-DoF_arm",
                                  effort_offset=[0, default_shoulder_joint_comp, 0, 0, 0],
                                  ik_seed_pos=[0.01, 1.0, 2.5, 1.5, -1.5],
                                  has_gripper=True, gripper_open_effort=1, gripper_close_effort=-5,
                                  gripper_gains="gripper_spool_gains"),
                        ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2"]),

    '5-DoF': (ArmParams("5-DoF_arm",
                        effort_offset=[0, default_shoulder_joint_comp, 0, 0, 0],
                        ik_seed_pos=[0.01, 1.0, 2.5, 1.5, -1.5]),
              ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2"]),

    '4-DoF': (ArmParams("4-DoF_arm",
                        effort_offset=[0, default_shoulder_joint_comp, 0, 0],
                        ik_seed_pos=[0.01, 1.0, 2.5, 1.5]),
              ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1"]),

    '4-DoF SCARA': (ArmParams("4-DoF_arm_scara",
                              effort_offset=[0, default_shoulder_joint_comp, 0, 0],
                              ik_seed_pos=[0.01, 1.0, 2.5, 1.5]),
                    ["J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1"]),

    '3-DoF': (ArmParams("3-DoF_arm",
                        effort_offset=[0, default_shoulder_joint_comp, 0],
                        ik_seed_pos=[0.01, 1.0, 2.5]),
              ["J1_base", "J2_shoulder", "J3_elbow"])
}


# ------------------------------------------------------------------------------
# Public API
# ------------------------------------------------------------------------------


def setup_arm_params(name, family, has_gas_spring=False, lookup=None):
    """
    :param name:  This argument currently supports the following names
                    * '6-DoF + gripper'
                    * '6-DoF'
                    * '5-DoF + gripper'
                    * '5-DoF'
                    * '4-DoF
                    * '4-DoF SCARA'
                    * '3-DoF'

    :param family:  The family name of the modules which should be selected.

    :param has_gas_spring:  Specifies whether there is a gas spring supporting the shoulder joint
                            of the arm to provide extra payload.
    """

    if name not in __arm_setup_params_dict:
        valid_vals = __arm_setup_params_dict.keys()
        print('Invalid `name` input {0}'.format(name))
        print('Valid names include: {0}'.format(valid_vals))
        raise KeyError('Invalid name')

    arm_setup_params = __arm_setup_params_dict[name]
    params = arm_setup_params[0]
    group_names = arm_setup_params[1]
    kin = hebi.robot_model.import_from_hrdf(os.path.join(_arm_resource_local_dir, "hrdf", params.robot_name + ".hrdf"))

    if lookup is None:
        lookup = hebi.Lookup()
        from time import sleep
        sleep(2)

    group = lookup.get_group_from_names([family], group_names)
    if group is None:
        raise RuntimeError("Could not find group")

    if not group.feedback_frequency > 0.0:
        group.send_feedback_request()

    params.update_gravity(group.get_next_feedback())
    return group, kin, params
