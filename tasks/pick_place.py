from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np

class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "ur5e_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        gripper = ParallelGripper(
            end_effector_prim_path="/World/Robotiq_2F_140_physics_edit/_F_Body",
            joint_prim_names=["body_f1_l", "body_f1_r"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([14.0, 14.0]),
            action_deltas=np.array([-0.45, -0.45]) )
        manipulator = SingleManipulator(prim_path="/World/ur5e/base_link",
                                        name="ur5e_robot",
                                        end_effector_prim_name="_F_Body",
                                        gripper=gripper)
        joints_default_positions = np.zeros(14)
        joints_default_positions[1] = -1.57 -0.3
        joints_default_positions[2] =  0.3
        joints_default_positions[6] = 0.0
        joints_default_positions[7] = 0.0
        manipulator.set_joints_default_state(positions=joints_default_positions)

        return manipulator

