import numpy as np

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class RidgebackDualPanda(ManipulatorModel):
    """
    Dual Panda Arms mounted on Clearpath Robotics Ridgeback Mobile Base.

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right", "left"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/ridgeback_dual_panda/robot.xml"), idn=idn)

    @property
    def default_base(self):
        return "RidgebackSimpleBase"

    @property
    def default_gripper(self):
        return {"right": "PandaGripper", "left": "PandaGripper"}

    @property
    def default_controller_config(self):
        return {"right": "joint_position", "left": "joint_position"}

    @property
    def init_qpos(self):
        # Seven joints per arm, ordered right then left as in the MJCF.
        panda_qpos = [0, np.pi / 16, 0, -5 * np.pi / 6, 0, np.pi - 0.2, np.pi / 4]
        return np.tile(panda_qpos, 2)

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.48

    @property
    def arm_type(self):
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand", "left": "left_hand"}
