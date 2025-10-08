#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation

from ..config import RobotConfig


@RobotConfig.register_subclass("piper_follower")
@dataclass
class PIPERFollowerConfig(RobotConfig):
    # Port to connect to the arm
    port: str = "can0"

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # cameras
    cameras: dict[str, RealSenseCameraConfig] = field(default_factory=dict)
    # cameras: dict[str, RealSenseCameraConfig] = field(
    #     default_factory={
    #         "cam_1": RealSenseCameraConfig(
    #             serial_number_or_name="130322273975",
    #             fps=15,
    #             width=640,
    #             height=480,
    #             color_mode=ColorMode.RGB,
    #             use_depth=True,
    #             rotation=Cv2Rotation.NO_ROTATION
    #         ),
    #         "right_cam": RealSenseCameraConfig(
    #             serial_number_or_name="130322272857",
    #             fps=15,
    #             width=640,
    #             height=480,
    #             color_mode=ColorMode.RGB,
    #             use_depth=True,
    #             rotation=Cv2Rotation.NO_ROTATION
    #         )
    #     }
    # )
    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False
