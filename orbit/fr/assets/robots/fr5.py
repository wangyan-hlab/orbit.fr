"""Configuration for the FR5 robots

Author: WY
Email: wangyan@frtech.fr
"""

import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.actuators import ImplicitActuatorCfg
from omni.isaac.orbit.assets.articulation import ArticulationCfg
from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR
import getpass
USER = getpass.getuser()

##
# Configuration
##

FR5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/{USER}/orbit.fr/isaacsim_usd/FR5_V6/usd/FR5_V6/FR5_V6.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 1.57,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)
"""Configuration of FR5 arm without a gripper"""


FR5_WSG50_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/{USER}/orbit.fr/isaacsim_usd/FR5_V6/usd/FR5_V6_WSG50/FR5_V6_WSG50.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 1.57,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*_driver_joint"],
            velocity_limit=100.0,
            effort_limit=2.0,
            stiffness=1.2,
            damping=0.01,
        ),
    },
)
"""Configuration of FR5 arm without a wsg50 gripper"""


FR5_RTQ85_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/{USER}/orbit.fr/isaacsim_usd/FR5_V6/usd/FR5_V6_RTQ85/FR5_V6_RTQ85.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 1.57,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*", "elbow_.*", "wrist_.*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["robotiq_85_.*"],
            velocity_limit=100.0,
            effort_limit=2.0,
            stiffness=1.2,
            damping=0.01,
        ),
    },
)
"""Configuration of FR5 arm without a rtq85 gripper"""


FR5_DHPGI80_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/{USER}/orbit.fr/isaacsim_usd/FR5_V6/usd/FR5_V6_DHPGI80/FR5_V6_DHPGI80.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 1.57,
            "shoulder_lift_joint": -1.57,
            "elbow_joint": 1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*", "elbow_.*", "wrist_.*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=[".*_driver_joint"],
            velocity_limit=100.0,
            effort_limit=2.0,
            stiffness=1.2,
            damping=0.01,
        ),
    },
)
"""Configuration of FR5 arm without a dhpgi80 gripper"""


# RIZON_CFG = ArticulationCfg(
#     spawn=sim_utils.UsdFileCfg(
#         usd_path=f"omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Flexiv/Rizon4/flexiv_rizon4.usd",
#         activate_contact_sensors=False,
#         rigid_props=sim_utils.RigidBodyPropertiesCfg(
#             disable_gravity=False,
#             max_depenetration_velocity=5.0,
#         ),
#         articulation_props=sim_utils.ArticulationRootPropertiesCfg(
#             enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0,
#         ),
#         # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
#     ),
#     init_state=ArticulationCfg.InitialStateCfg(
#         joint_pos={
#             "joint1": 1.57,
#             "joint2": -1.57,
#             "joint3": 1.57,
#             "joint4": 0.0,
#             "joint5": 0.0,
#             "joint6": 0.0,
#             "joint7": 0.0,
#         },
#     ),
#     actuators={
#         "arm": ImplicitActuatorCfg(
#             joint_names_expr=[".*"],
#             velocity_limit=100.0,
#             effort_limit=87.0,
#             stiffness=800.0,
#             damping=40.0,
#         ),
#     },
# )
# """Configuration of Flexiv Rizon arm without a gripper"""
