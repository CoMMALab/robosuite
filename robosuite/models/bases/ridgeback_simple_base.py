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
    def bottom_offset(self):
        # The standalone MJCF places base_link 0.0262 m above the floor.
        # This property is a vector FROM base_link TO the floor, not the
        # positive world-space spawn height stored in the XML. robosuite
        # subtracts it in set_base_xpos() after attaching the base model.
        return np.array((0, 0, -0.0262))

    @property
    def horizontal_radius(self):
        return 0.48
