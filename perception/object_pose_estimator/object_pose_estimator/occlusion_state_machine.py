"""State machine that classifies the current tracking situation.

States
------
TRACKING           : object fully visible; use Kalman predict + update   (D1)
PARTIALLY_OCCLUDED : object partially visible; Kalman update with scaled R (D2)
FULLY_OCCLUDED     : object not visible; Kalman predict only              (D3)
GRASPED            : gripper has closed; track via TF2 end-effector       (D3*)
"""

from enum import Enum, auto


class TrackingState(Enum):
    TRACKING           = auto()
    PARTIALLY_OCCLUDED = auto()
    FULLY_OCCLUDED     = auto()
    GRASPED            = auto()


class OcclusionStateMachine:
    def __init__(self,
                 partial_threshold: float = 0.20,
                 full_threshold: float = 0.75):
        self.state = TrackingState.TRACKING
        self.partial_threshold = partial_threshold
        self.full_threshold = full_threshold

    def update(self,
               occlusion_ratio: float,
               detection_succeeded: bool,
               gripper_closed: bool) -> TrackingState:
        """Compute and store the next state given current observations.

        Parameters
        ----------
        occlusion_ratio     fraction of bounding-box area covered by arm links
        detection_succeeded True when YOLO found the object this frame
        gripper_closed      True when both finger joints are below grasp threshold
        """
        if gripper_closed:
            self.state = TrackingState.GRASPED
            return self.state

        # Once grasped, only gripper opening releases the object
        if self.state == TrackingState.GRASPED:
            # gripper_closed is False here → gripper opened → object released
            self.state = TrackingState.TRACKING
            return self.state

        # Normal visibility transitions
        if not detection_succeeded or occlusion_ratio >= self.full_threshold:
            self.state = TrackingState.FULLY_OCCLUDED
        elif occlusion_ratio >= self.partial_threshold:
            self.state = TrackingState.PARTIALLY_OCCLUDED
        else:
            self.state = TrackingState.TRACKING

        return self.state
