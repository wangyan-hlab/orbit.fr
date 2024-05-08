# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.orbit.assets import RigidObjectCfg
from omni.isaac.orbit.sensors import FrameTransformerCfg, CameraCfg
from omni.isaac.orbit.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.orbit.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from omni.isaac.orbit.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

import orbit.fr.tasks.manipulation.lift.mdp as mdp
from orbit.fr.tasks.manipulation.lift.lift_env_cfg import LiftEnvCfg

##
# Pre-defined configs
##
from omni.isaac.orbit.markers.config import FRAME_MARKER_CFG  # isort: skip
# from omni.isaac.orbit_assets.franka import FRANKA_PANDA_CFG  # isort: skip
from orbit.fr.assets.robots.fr5 import FR5_WSG50_CFG
import omni.isaac.orbit.sim as sim_utils

@configclass
class FR5CubeLiftEnvCfg(LiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set FR5 as robot
        FR5_WSG50_CFG.init_state.joint_pos["shoulder_pan_joint"] = 0
        FR5_WSG50_CFG.init_state.joint_pos["shoulder_lift_joint"] = -100 / 180 * 3.1415
        FR5_WSG50_CFG.init_state.joint_pos["elbow_joint"] = 70 / 180 * 3.1415
        FR5_WSG50_CFG.init_state.joint_pos["wrist_1_joint"] = -60 / 180 * 3.1415
        FR5_WSG50_CFG.init_state.joint_pos["wrist_2_joint"] = -1.5708
        FR5_WSG50_CFG.init_state.joint_pos["wrist_3_joint"] = 0
        self.scene.robot = FR5_WSG50_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (fr5)
        self.actions.body_joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_.*_joint"],
            scale=0.5,
            use_default_offset=True
        )
        self.actions.finger_joint_pos = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=[".*_driver_joint"],
            open_command_expr={".*_driver_joint*": 0.054},
            close_command_expr={".*_driver_joint*": 0.0},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "wsg50_hand"

        # Set the camera attached to the robot's wrist
        self.scene.camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wsg50_hand/wrist_cam",
            update_period=0.1,
            height=960,
            width=1280,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=15.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
            ),
            offset=CameraCfg.OffsetCfg(pos=(-0.033, -0.065, 0.054), rot=(0.0, 0.0, 0.0, 1.0), convention="ros"),
        )

        # Set Cube as object
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.2], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=RigidBodyPropertiesCfg(
                    rigid_body_enabled=True,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/FR5_base_link",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/wsg50_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
            ],
        )


@configclass
class FR5CubeLiftEnvCfg_PLAY(FR5CubeLiftEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
