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

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper_follower import PIPERFollowerConfig
from piper_sdk import *

logger = logging.getLogger(__name__)


class PIPERFollower(Robot):
    """
    PIPER Follower Arm.
    """

    config_class = PIPERFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PIPERFollowerConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self.can_port = config.port
        self.piper = C_PiperInterface(can_name=self.can_port)   
        self.piper.ConnectPort()
        
        # 0xAD sets the robot arm to high follow mode
        # self.piper.MotionCtrl_2(0x01, 0x01, 100, 0xAD)
        # self.piper.EnableArm()
        # reset arm
        # self.piper.MotionCtrl_1(0x02,0,0)
        # self.piper.MotionCtrl_2(0x01, 0x01, 50)
        # self.piper.EnableArm(7)
        # self.piper.GripperCtrl(0,1000,0x01, 0)
        self.cameras = make_cameras_from_configs(config.cameras)
        
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "gripper.pos": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return all(cam.is_connected for cam in self.cameras.values())
        # return self._is_connected and all(cam.is_connected for cam in self.cameras.values())

    # def initiate_position(self):
    #     count = 0
    #     factor = 57295.7795
    #     position = [0,0,0,0,0,0,0]
    #     while True:
    #         count  = count + 1
    #         # print(count)
    #         if(count == 0):
    #             print("1-----------")
    #             position = [0,0,0,0,0,0,0]
    #         elif(count == 300):
    #             print("2-----------")
    #             position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
    #         elif(count == 600):
    #             print("1-----------")
    #             position = [0,0,0,0,0,0,0]
    #         elif(count > 600):
    #             break
    #         joint_0 = round(position[0]*factor)
    #         joint_1 = round(position[1]*factor)
    #         joint_2 = round(position[2]*factor)
    #         joint_3 = round(position[3]*factor)
    #         joint_4 = round(position[4]*factor)
    #         joint_5 = round(position[5]*factor)
    #         joint_6 = round(position[6]*1000*1000)
    #         self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
    #         self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
    #         self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    #         print(self.piper.GetArmStatus())
    #         print(position)
    #         time.sleep(0.005)
    #     print("initiate position finished!")

    def startup_robot(self, piper:C_PiperInterface):
        '''
        enable robot and check enable status, try 5s, if enable timeout, exit program
        '''
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (enable_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_flag = piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            print("enable status:",enable_flag)
            piper.EnableArm(7)
            piper.GripperCtrl(0,1000,0x01, 0)
            print("--------------------")
            # check if timeout
            if elapsed_time > timeout:
                print("enable timeout....")
                elapsed_time_flag = True
                enable_flag = False
                break
            time.sleep(1)
            pass
        if not elapsed_time_flag:
            return enable_flag
        else:
            print("enable timeout, exit program")
            raise RuntimeError("Failed to enable robot motors within timeout period")

    def connect(self) -> None:
        self._is_connected = self.startup_robot(self.piper)
        print("Robot connected:", self.is_connected)
        for name in self.cameras:
            print(f"Connecting camera: {name}")
            self.cameras[name].connect()
            # self._is_connected = self.is_connected and self.cameras[name].is_connected

        if not self.is_connected:
            print("Could not connect to the cameras, check that all cameras are plugged-in.")
            raise ConnectionError()
        else:
            logger.info(f"{self} connected successfully.")

        # self.move_to_home()

        # Used when connecting to robot
    def move_to_home(self) -> None:
        count = 0
        while True:
            if(count == 0):
                print("1-----------")
                action = [0.07,0,0.22,0,0.08,0,0]
            elif(count == 300):
                print("2-----------")
                action = [0.15,0.0,0.35,0.08,0.08,0.075,0.0] # 0.08 is maximum gripper position
            elif(count == 600):
                print("3-----------")
                action = [0.200337, 0.020786, 0.289284, 0.179831, 0.010918, 0.173467, 0.0]
            count += 1
            before_write_t = time.perf_counter()
            state = self.get_state()
            state = state["state"]
            state[3:6] = self.euler_filter.rectify(state[3:6])
            self.send_action(action)
            self.rate.sleep(time.perf_counter() - before_write_t)
            if count > 800:
                break

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # self.calibration is not empty here
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        master_joint_0:float = round((self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444, 8)
        master_joint_1:float = round((self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444, 8)
        master_joint_2:float = round((self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444, 8)
        master_joint_3:float = round((self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444, 8)
        master_joint_4:float = round((self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444, 8)
        master_joint_5:float = round((self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444, 8)
        master_joint_6:float = round(self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000, 8)
        action_dict = {
            "joint_1.pos": master_joint_0,
            "joint_2.pos": master_joint_1,
            "joint_3.pos": master_joint_2,
            "joint_4.pos": master_joint_3,
            "joint_5.pos": master_joint_4,
            "joint_6.pos": master_joint_5,
            "gripper.pos": master_joint_6
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        return action_dict
    
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        follower_joint_0:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444, 8)
        follower_joint_1:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444, 8)
        follower_joint_2:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444, 8)
        follower_joint_3:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444, 8)
        follower_joint_4:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444, 8)
        follower_joint_5:float = round((self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444, 8)
        follower_joint_6:float = round(self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000, 8)
        obs_dict = {
            "joint_1.pos": follower_joint_0,
            "joint_2.pos": follower_joint_1,
            "joint_3.pos": follower_joint_2,
            "joint_4.pos": follower_joint_3,
            "joint_5.pos": follower_joint_4,
            "joint_6.pos": follower_joint_5,
            "gripper.pos": follower_joint_6
        }
        # obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        # print(f"obs_dict: {obs_dict}")
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Extract joint positions from action dict
        factor = 57295.7795  # 1000*180/pi
        joint_0 = round(action["joint_1.pos"] * factor)
        joint_1 = round(action["joint_2.pos"] * factor)
        joint_2 = round(action["joint_3.pos"] * factor)
        joint_3 = round(action["joint_4.pos"] * factor)
        joint_4 = round(action["joint_5.pos"] * factor)
        joint_5 = round(action["joint_6.pos"] * factor)
        joint_6 = round(action["gripper.pos"] * 1000 * 1000)

        # Send commands to the robot via Piper SDK
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

        # Optionally, you can return the actual action sent
        time.sleep(0.5)  # Small delay to ensure command is processed
        return {
            "joint_1.pos": action["joint_1.pos"],
            "joint_2.pos": action["joint_2.pos"],
            "joint_3.pos": action["joint_3.pos"],
            "joint_4.pos": action["joint_4.pos"],
            "joint_5.pos": action["joint_5.pos"],
            "joint_6.pos": action["joint_6.pos"],
            "gripper.pos": action["gripper.pos"]
        }

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        for cam in self.cameras.values():
            cam.disconnect()

        self._is_connected = False
        logger.info(f"{self} disconnected.")
