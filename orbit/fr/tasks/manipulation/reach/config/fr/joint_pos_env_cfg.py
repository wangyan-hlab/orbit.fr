# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.orbit.utils import configclass

import orbit.fr.tasks.manipulation.reach.mdp as mdp
from orbit.fr.tasks.manipulation.reach.reach_env_cfg import ReachEnvCfg

##
# Pre-defined configs
##
from orbit.fr.assets.robots.fr5 import FR5_CFG


##
# Environment configuration
##


@configclass
class FR5ReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to fr5
        FR5_CFG.init_state.rot = (0.0, 0.0, 0.0, 1.0)
        self.scene.robot = FR5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override events
        self.events.reset_robot_joints.params["position_range"] = (0.75, 1.25)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["FR5_link6"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["FR5_link6"]
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "FR5_link6"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)


@configclass
class FR5ReachEnvCfg_PLAY(FR5ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
