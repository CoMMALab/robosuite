"""Ridgeback geometry with robosuite's planar omnidirectional base control."""

import numpy as np

from robosuite.models.bases.mobile_base_model import MobileBaseModel
from robosuite.utils.mjcf_utils import xml_path_completion


class RidgebackSimpleBase(MobileBaseModel):
    """Fixed rocker and wheels; motion comes from X, Y, and yaw joints.

    Chassis, wheel, and support transforms match RidgebackMobileBase at
    zero joint positions. The containing environment supplies the floor.
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("bases/ridgeback_simple_base.xml"), idn=idn)

    @property
    def top_offset(self):
        return np.array((0, 0, 0))

    @property
    def horizontal_radius(self):
        return 0.48
