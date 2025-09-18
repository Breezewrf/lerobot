#!/usr/bin/env python3
# -*-coding:utf8-*-
# 注意demo无法直接运行，需要pip安装sdk后才能运行
"""
    This code is a client for GR00T.
"""
import time
from piper_sdk import *
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.robots.piper_follower.config_piper_follower import PIPERFollowerConfig
from lerobot.robots.piper_follower.piper_follower import PIPERFollower as PIPERLeader
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
import time
import json_numpy
import numpy as np
import requests

json_numpy.patch()

TASK_DESCRIPTION = "Put the tape in the paper cup"

leader_arm_config = PIPERFollowerConfig
leader_arm_config.port = "can0"  # Example port, adjust as necessary
leader_arm_config.cameras = {
    "cam_1": RealSenseCameraConfig(
        serial_number_or_name="130322273975",
        fps=15,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
        use_depth=True,
        rotation=Cv2Rotation.NO_ROTATION
    ),
    "right_camera1": RealSenseCameraConfig(
        serial_number_or_name="130322272857",
        fps=15,
        width=640,
        height=480,
        color_mode=ColorMode.RGB,
        use_depth=True,
        rotation=Cv2Rotation.NO_ROTATION
    )
}

leader_arm = PIPERLeader(leader_arm_config)

leader_arm.connect()

def get_observation():
    """
    Get observation from the Piper robot.
    """
    obs = leader_arm.get_observation()
    frame = {'observation.state': obs, 'observation.images.cam_1': obs['cam_1']}

    # Extract arm and gripper state
    obs_state_values = [
        frame['observation.state']['joint_1.pos'],
        frame['observation.state']['joint_2.pos'],
        frame['observation.state']['joint_3.pos'],
        frame['observation.state']['joint_4.pos'],
        frame['observation.state']['joint_5.pos'],
        frame['observation.state']['joint_6.pos'],
        frame['observation.state']['gripper.pos']
    ]
    obs_arm_state_array = np.array(obs_state_values[:6]).reshape(1, 6)
    obs_gripper_state = np.array([obs_state_values[6]]).reshape(1, 1)

    # Ensure cam_1 image shape is (1, 480, 640, 3)
    cam_1_img = frame['observation.state']['cam_1']
    if cam_1_img.ndim == 3 and cam_1_img.shape == (480, 640, 3):
        cam_1_img = cam_1_img[np.newaxis, ...]
    elif cam_1_img.ndim == 4 and cam_1_img.shape[0] == 1:
        pass  # already correct
    else:
        raise ValueError(f"Unexpected cam_1 image shape: {cam_1_img.shape}")
    
    cam_right_img = frame['observation.state']['right_camera1']
    if cam_right_img.ndim == 3 and cam_right_img.shape == (480, 640, 3):
        cam_right_img = cam_right_img[np.newaxis, ...]
    elif cam_right_img.ndim == 4 and cam_right_img.shape[0] == 1:
        pass  # already correct
    else:
        raise ValueError(f"Unexpected right_camera1 image shape: {cam_right_img.shape}")

    new_frame = {
        'state.single_arm': obs_arm_state_array,
        'state.gripper': obs_gripper_state,
        'video.cam_1': cam_1_img,
        'video.cam_right': cam_right_img,
        'annotation.human.task_description': [TASK_DESCRIPTION],
    }
    for k, v in new_frame.items():
        if isinstance(v, np.ndarray):
            print(f"{k}: shape={v.shape}")
        elif isinstance(v, list):
            print(f"{k}: list of length {len(v)}")
        elif isinstance(v, dict):
            print(f"{k}: dict with keys {list(v.keys())}")
        elif hasattr(v, 'shape'):
            print(f"{k}: shape={v.shape}")
        else:
            print(f"{k}: type={type(v)}")
    return new_frame
    

def initiate_position():
    count = 0
    factor = 57295.7795
    position = [0,0,0,0,0,0,0]
    while True:
        count  = count + 1
        # print(count)
        if(count == 0):
            print("1-----------")
            position = [0,0,0,0,0,0,0]
        elif(count == 300):
            print("2-----------")
            position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
        elif(count == 600):
            print("1-----------")
            position = [0,0,0,0,0,0,0]
        elif(count > 600):
            break
        joint_0 = round(position[0]*factor)
        joint_1 = round(position[1]*factor)
        joint_2 = round(position[2]*factor)
        joint_3 = round(position[3]*factor)
        joint_4 = round(position[4]*factor)
        joint_5 = round(position[5]*factor)
        joint_6 = round(position[6]*1000*1000)
        piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        print(piper.GetArmStatus())
        print(position)
        time.sleep(0.005)

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    piper.GripperCtrl(0,1000,0x01, 0)
    factor = 57295.7795 #1000*180/3.1415926
    initiate_position()

    while True:
        t = time.time()
        obs = get_observation()
        print("obs:", obs)
        response = requests.post(
            # "http://0.0.0.0:5555/act",
            "http://192.168.10.165:5555/act",
            # "http://159.223.171.199:44989/act",   # Bore tunnel
            json={"observation": obs},
        )
        print(f"used time {time.time() - t}")
        action = response.json()
        print("action:", action.keys())
        print(action['action.single_arm'].shape, action['action.gripper'].shape)
        # time.sleep(0.2)  # Adjust sleep time as necessary
        # Loop through the 16 controls and send them sequentially
        for i in range(16):
            joint_values = action['action.single_arm'][i]
            gripper_value = action['action.gripper'][i]
            joint_0 = round(joint_values[0] * factor)
            joint_1 = round(joint_values[1] * factor)
            joint_2 = round(joint_values[2] * factor)
            joint_3 = round(joint_values[3] * factor)
            joint_4 = round(joint_values[4] * factor)
            joint_5 = round(joint_values[5] * factor)
            joint_6 = round(gripper_value * 1000 * 1000)
            piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
            piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
            piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
            print(f"Step {i}: ArmStatus={piper.GetArmStatus()} Joint={joint_values} Gripper={gripper_value}")
            
            time.sleep(0.05)  # Adjust sleep time as necessary
        
    