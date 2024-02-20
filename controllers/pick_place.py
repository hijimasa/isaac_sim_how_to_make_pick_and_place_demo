import omni.isaac.manipulators.controllers as manipulators_controllers
from omni.isaac.manipulators.grippers import ParallelGripper
from .rmpflow import RMPFlowController
from omni.isaac.core.articulations import Articulation

class PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: Articulation,
        events_dt=None
    ) -> None:
        if events_dt is None:
            #These values needs to be tuned in general, you checkout each event in execution and slow it down or speed
            #it up depends on how smooth the movments are
            events_dt = [0.005, 0.002, 1, 0.05, 0.0008, 0.005, 0.0008, 0.1, 0.0008, 0.008]
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            events_dt=events_dt
        )
        return
